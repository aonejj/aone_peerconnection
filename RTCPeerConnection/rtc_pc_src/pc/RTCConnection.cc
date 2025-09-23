//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : peerconnection lite version for rtc media server.
//
//////////////////////////////////////////////////////////////////////////
#include "absl/types/optional.h"
#include "absl/strings/match.h"


#include "api/uma_metrics.h"
#include "api/transport/enums.h"
#include "api/crypto/crypto_options.h"
#include "api/audio_codecs/audio_format.h"
#include "api/audio_codecs/opus/audio_encoder_opus_config.h"
#include "api/jsep_ice_candidate.h"
#include "api/video/video_codec_constants.h"
#include "media/base/h264_profile_level_id.h"
#include "media/base/rtp_data_engine.h"
#include "media/engine/payload_type_mapper.h"
#include "p2p/base/default_ice_transport_factory.h"
#include "p2p/base/basic_async_resolver_factory.h"
#include "p2p/client/basic_port_allocator.h"
#include "pc/ice_server_parsing.h"
#include "rtc_base/logging.h"
#include "rtc_base/checks.h"
#include "rtc_base/bind.h"
#include "rtc_base/synchronization/sequence_checker.h"
#include "rtc_base/helpers.h"
#include "rtc_base/network_constants.h"
#include "rtc_base/task_utils/pending_task_safety_flag.h"
#include "rtc_base/strings/audio_format_to_string.h"
#include "system_wrappers/include/metrics.h"

#include "../../src_update/media/base/_media_engine.h"
#include "RTCSdpOfferAnswerHandler.h"
#include "RTCRtpSender.h"
#include "RTCConnection.h"

#define __USING_VP8__
#define __USING_H264__


namespace webrtc {

uint32_t ConvertIceTransportTypeToCandidateFilter(
	PeerConnectionInterfaceDefs::IceTransportsType type) {
	switch (type) {
	case PeerConnectionInterfaceDefs::kNone:
		return cricket::CF_NONE;
	case PeerConnectionInterfaceDefs::kRelay:
		return cricket::CF_RELAY;
	case PeerConnectionInterfaceDefs::kNoHost:
		return (cricket::CF_ALL & ~cricket::CF_HOST);
	case PeerConnectionInterfaceDefs::kAll:
		return cricket::CF_ALL;
	default:
		RTC_NOTREACHED();
	}
	return cricket::CF_NONE;
}

absl::optional<int> RTCConfigurationToIceConfigOptionalInt(
	int rtc_configuration_parameter) {
	if (rtc_configuration_parameter ==
		PeerConnectionInterfaceDefs::RTCConfiguration::kUndefined) {
		return absl::nullopt;
	}
	return rtc_configuration_parameter;
}

// ICE config hardcoding
cricket::IceConfig ParseIceConfig(const PeerConnectionInterfaceDefs::RTCConfiguration &rtcConfigure) {
	cricket::ContinualGatheringPolicy gathering_policy;
	switch (rtcConfigure.continual_gathering_policy) {
	case PeerConnectionInterfaceDefs::ContinualGatheringPolicy::GATHER_ONCE:
		gathering_policy = cricket::ContinualGatheringPolicy::GATHER_ONCE;
		break;
	case PeerConnectionInterfaceDefs::ContinualGatheringPolicy::GATHER_CONTINUALLY:
		gathering_policy = cricket::ContinualGatheringPolicy::GATHER_CONTINUALLY;
		break;
	default:
		RTC_NOTREACHED();
		gathering_policy = cricket::ContinualGatheringPolicy::GATHER_ONCE;
	}


	cricket::IceConfig ice_config;
	ice_config.receiving_timeout = RTCConfigurationToIceConfigOptionalInt(rtcConfigure.ice_connection_receiving_timeout);
	ice_config.prioritize_most_likely_candidate_pairs = 
		rtcConfigure.prioritize_most_likely_ice_candidate_pairs;
	ice_config.backup_connection_ping_interval = 
		RTCConfigurationToIceConfigOptionalInt(rtcConfigure.ice_backup_candidate_pair_ping_interval);
	ice_config.continual_gathering_policy = gathering_policy;
	ice_config.presume_writable_when_fully_relayed = 
		rtcConfigure.presume_writable_when_fully_relayed;
	ice_config.surface_ice_candidates_on_ice_transport_type_changed = 
		rtcConfigure.surface_ice_candidates_on_ice_transport_type_changed;
	ice_config.ice_check_interval_strong_connectivity = 
		rtcConfigure.ice_check_interval_strong_connectivity;
	ice_config.ice_check_interval_weak_connectivity = 
		rtcConfigure.ice_check_interval_weak_connectivity;
	ice_config.ice_check_min_interval = 
		rtcConfigure.ice_check_min_interval;
	ice_config.ice_unwritable_timeout = 
		rtcConfigure.ice_unwritable_timeout;
	ice_config.ice_unwritable_min_checks = 
		rtcConfigure.ice_unwritable_min_checks;
	ice_config.ice_inactive_timeout = 
		rtcConfigure.ice_inactive_timeout;
	ice_config.stun_keepalive_interval = 
		rtcConfigure.stun_candidate_keepalive_interval;
	ice_config.network_preference =
		rtcConfigure.network_preference;

	return ice_config;
}

bool HasRtcpMuxEnabled(const cricket::ContentInfo* content) {
	return content->media_description()->rtcp_mux();
}


rtc::scoped_refptr<RTCConnection> RTCConnection::Create(ThreadsInfoInterface *threadsInfo, 
														std::unique_ptr<RTCCall> call,
														TaskQueueFactory *task_queue_factory,
														PeerConnectionInterfaceDefs::RTCConfiguration &rtcConfigure,
														RTCConnectionObserver *observer,
														rtc::RTCThreadManagerInterface *rtc_thread_manager
														) {
	bool is_unified_plan =
		rtcConfigure.sdp_semantics == SdpSemantics::kUnifiedPlan;

	rtc::scoped_refptr<RTCConnection> connection(
		rtc::make_ref_counted<RTCConnection>(
		threadsInfo, std::move(call), 
		task_queue_factory,
		observer, 
		is_unified_plan,
		rtc_thread_manager
		));

	RTCError init_error = connection->_initialize(rtcConfigure);
	if (!init_error.ok()) {
		RTC_LOG(LS_VERBOSE) << "rtc connection initialize fail";
		return nullptr;
	}

	return connection;
}

RTCConnection::RTCConnection(
	ThreadsInfoInterface *threadsInfo,
	std::unique_ptr<RTCCall> call,
	TaskQueueFactory *task_queue_factory,
	RTCConnectionObserver *observer, 
	bool is_unified_plan,
	rtc::RTCThreadManagerInterface *rtc_thread_manager
	)
 : _signaling_thread(threadsInfo->signaling_thread()),
   _worker_thread(threadsInfo->worker_thread()),
   _network_thread(threadsInfo->network_thread()),
   call_(std::move(call)),
   call_ptr_(call_.get()),
   task_queue_factory_(task_queue_factory),
   _weak_factory(this),
   _observer(observer),
   is_unified_plan_(is_unified_plan),
   _message_handler(signaling_thread()),
   data_channel_controller_(this	, rtc_thread_manager),
   _rtc_thread_manager(rtc_thread_manager)
{
	sctp_factory_ = std::make_unique<cricket::SctpTransportFactory>(_network_thread);
}

RTCConnection::~RTCConnection() {
	RTC_LOG(LS_VERBOSE) << "RTCConnection::~RTCConnection";
	RTC_DCHECK_RUN_ON(signaling_thread());

	if (_sdp_handler) {
		_sdp_handler->PrepareForShutdown();
	}

	if (rtp_manager()) {
		for (const auto& transceiver : rtp_manager()->transceivers()->List()) {
			transceiver->StopInternal();
		}
	}

	if (_sdp_handler) {
		_sdp_handler->DestroyAllChannels();
		_sdp_handler->ResetSessionDescFactory();
	}

	_transport_controller.reset();
	// port_allocator_ lives on the network thread and should be destroyed there.
	network_thread()->Invoke<void>(RTC_FROM_HERE, [this] {
		RTC_DCHECK_RUN_ON(network_thread());
		_allocator.reset();
	});

	worker_thread()->Invoke<void>(RTC_FROM_HERE, [this] {
		RTC_DCHECK_RUN_ON(worker_thread());
		call_safety_.reset();
		call_.reset();
	});
}

void RTCConnection::CreateOffer(CreateSessionDescriptionObserver *observer,
								const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions& options) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	_sdp_handler->CreateOffer(observer, options);
}

void RTCConnection::CreateAnswer(CreateSessionDescriptionObserver *observer, 
	const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions& options) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	_sdp_handler->CreateAnswer(observer, options);
}

void RTCConnection::SetLocalDescription(SetSessionDescriptionObserver *observer, SessionDescriptionInterface *desc_ptr) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	_sdp_handler->SetLocalDescription(observer, desc_ptr);
}

void RTCConnection::SetLocalDescription(rtc::scoped_refptr<SetSessionDescriptionObserver> observer,
	std::unique_ptr<SessionDescriptionInterface> desc) {
	RTC_DCHECK_RUN_ON(signaling_thread());

	_sdp_handler->SetLocalDescription(observer, std::move(desc));
}

void RTCConnection::SetRemoteDescription(SetSessionDescriptionObserver *observer,
	SessionDescriptionInterface *desc_ptr) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	_sdp_handler->SetRemoteDescription(observer, desc_ptr);
}

void RTCConnection::SetRemoteDescription(rtc::scoped_refptr<SetSessionDescriptionObserver> observer,
	std::unique_ptr<SessionDescriptionInterface> desc) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	_sdp_handler->SetRemoteDescription(observer, std::move(desc));
}

bool RTCConnection::AddIceCandidate(
	const IceCandidateInterface* ice_candidate) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	return _sdp_handler->AddIceCandidate(ice_candidate);
}

void RTCConnection::AddIceCandidate(
	std::unique_ptr<IceCandidateInterface> candidate,
	std::function<void(RTCError)> callback) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	_sdp_handler->AddIceCandidate(std::move(candidate), callback);
}

bool RTCConnection::RemoveIceCandidates(
	const std::vector<cricket::Candidate>& candidates) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	return _sdp_handler->RemoveIceCandidates(candidates);
}

CryptoOptions RTCConnection::GetCryptoOptions() {
	RTC_DCHECK_RUN_ON(signaling_thread());
	return _rtcConfigure.crypto_options.has_value()
		? *_rtcConfigure.crypto_options
		: CryptoOptions::NoGcm();
}

//////////////////////////////////////////////////////////////////////////
// for test rtp extension header.....
std::vector<RtpHeaderExtensionCapability> RTCConnection::GetAudioRtpHeaderExtensions() const {
	std::vector<RtpHeaderExtensionCapability> result;
	int id = 1;
	for (const auto& uri :
	{	RtpExtension::kTimestampOffsetUri,
		RtpExtension::kAudioLevelUri,
		RtpExtension::kAbsSendTimeUri,
		RtpExtension::kTransportSequenceNumberUri,
		RtpExtension::kMidUri, 
		RtpExtension::kRidUri,
		RtpExtension::kRepairedRidUri 
	} ) {
		result.emplace_back(uri, id++, RtpTransceiverDirection::kSendRecv);
	}
	return result;
}

std::vector<RtpHeaderExtensionCapability> RTCConnection::GetVideoRtpHeaderExtensions() const {
	std::vector<RtpHeaderExtensionCapability> result;
	int id = 1;
	for (const auto& uri :
	{	RtpExtension::kTimestampOffsetUri,
		RtpExtension::kAbsSendTimeUri,
		RtpExtension::kVideoRotationUri,
		RtpExtension::kTransportSequenceNumberUri,
		RtpExtension::kPlayoutDelayUri,
		RtpExtension::kVideoContentTypeUri,
		RtpExtension::kVideoTimingUri,
		RtpExtension::kColorSpaceUri, 
		RtpExtension::kMidUri,
		RtpExtension::kRidUri, 
		RtpExtension::kRepairedRidUri 
	}) {
		result.emplace_back(uri, id++, RtpTransceiverDirection::kSendRecv);
	}

	return result;
}
//////////////////////////////////////////////////////////////////////////

// Returns false if bundle is enabled and rtcp_mux is disabled.
bool RTCConnection::ValidateBundleSettings(const cricket::SessionDescription* desc) {
	bool bundle_enabled = desc->HasGroup(cricket::GROUP_TYPE_BUNDLE);
	if (!bundle_enabled)
		return true;

	const cricket::ContentGroup* bundle_group =
		desc->GetGroupByName(cricket::GROUP_TYPE_BUNDLE);
	RTC_DCHECK(bundle_group != NULL);

	const cricket::ContentInfos& contents = desc->contents();
	for (cricket::ContentInfos::const_iterator citer = contents.begin();
		citer != contents.end(); ++citer) {
		const cricket::ContentInfo* content = (&*citer);
		RTC_DCHECK(content != NULL);
		if (bundle_group->HasContentName(content->name) && !content->rejected &&
			content->type == cricket::MediaProtocolType::kRtp) {
			if (!HasRtcpMuxEnabled(content))
				return false;
		}
	}
	// RTCP-MUX is enabled in all the contents.
	return true;
}


cricket::ChannelInterface* RTCConnection::GetChannel(const std::string& content_name) {
	for (const auto& transceiver : rtp_manager()->transceivers()->List()) {
		cricket::ChannelInterface* channel = transceiver->internal()->channel();
		if (channel && channel->content_name() == content_name) {
			return channel;
		}
	}
	
	if (rtp_data_channel() &&
		rtp_data_channel()->content_name() == content_name) {
		return rtp_data_channel();
	}
	return nullptr;
}

bool RTCConnection::SrtpRequired() const {
	return (_dtls_enabled ||
		_sdp_handler->webrtc_session_desc_factory()->SdesPolicy() ==
		cricket::SEC_REQUIRED);
}

void RTCConnection::OnSentPacket_w(const rtc::SentPacket& sent_packet) {
	RTC_DCHECK_RUN_ON(worker_thread());
	call_->OnSentPacket(sent_packet);
}

bool RTCConnection::SetupDataChannelTransport_n(const std::string& mid) {
	DataChannelTransportInterface* transport =
		_transport_controller->GetDataChannelTransport(mid);
	if (!transport) {
		RTC_LOG(LS_ERROR)
			<< "Data channel transport is not available for data channels, mid="
			<< mid;
		return false;
	}
	RTC_LOG(LS_INFO) << "Setting up data channel transport for mid=" << mid;

	data_channel_controller_.set_data_channel_transport(transport);
	data_channel_controller_.SetupDataChannelTransport_n();
	sctp_mid_n_ = mid;

	// Note: setting the data sink and checking initial state must be done last,
	// after setting up the data channel.  Setting the data sink may trigger
	// callbacks to PeerConnection which require the transport to be completely
	// set up (eg. OnReadyToSend()).
	transport->SetDataSink(&data_channel_controller_);
	return true;
}

void RTCConnection::TeardownDataChannelTransport_n() {
	if (!sctp_mid_n_ && !data_channel_controller_.data_channel_transport()) {
		return;
	}
	RTC_LOG(LS_INFO) << "[" << this <<"] Tearing down data channel transport for mid="
		<< *sctp_mid_n_;

	// |sctp_mid_| may still be active through an SCTP transport.  If not, unset
	// it.
	sctp_mid_n_.reset();
	data_channel_controller_.TeardownDataChannelTransport_n();
}

PeerConnectionInterfaceDefs::IceConnectionState
RTCConnection::ice_connection_state() {
	RTC_DCHECK_RUN_ON(signaling_thread());
	return ice_connection_state_;
}

RTCErrorOr<rtc::scoped_refptr<RTCRtpSenderInterface>> RTCConnection::AddTrack(
								rtc::scoped_refptr<MediaStreamTrackInterface> track,
								const std::vector<std::string>& stream_ids) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	if (!track) {
		LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_PARAMETER, "Track is null.");
	}

	if (!(track->kind() == MediaStreamTrackInterface::kAudioKind ||
		track->kind() == MediaStreamTrackInterface::kVideoKind)) {
		LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_PARAMETER,
			"Track has invalid kind: " + track->kind());
	}

	if (IsClosed()) {
		LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_STATE,
			"PeerConnection is closed.");
	}

	if (rtp_manager()->FindSenderForTrack(track)) {
		LOG_AND_RETURN_ERROR(
			RTCErrorType::INVALID_PARAMETER,
			"Sender already exists for track " + track->id() + ".");
	}
	auto sender_or_error = rtp_manager()->AddTrack(track, stream_ids);
	if (sender_or_error.ok()) {
		_sdp_handler->UpdateNegotiationNeeded();
	}

	return sender_or_error;
}

RTCError RTCConnection::RemoveTrack(rtc::scoped_refptr<RTCRtpSenderInterface> sender) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	if (!sender) {
		LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_PARAMETER, "Sender is null.");
	}
	if (IsClosed()) {
		LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_STATE,
			"PeerConnection is closed.");
	}
	if (IsUnifiedPlan()) {
		auto transceiver = FindTransceiverBySender(sender);
		if (!transceiver || !sender->track()) {
			return RTCError::OK();
		}
		sender->SetTrack(nullptr);
		if (transceiver->direction() == RtpTransceiverDirection::kSendRecv) {
			transceiver->internal()->set_direction(
				RtpTransceiverDirection::kRecvOnly);
		}
		else if (transceiver->direction() == RtpTransceiverDirection::kSendOnly) {
			transceiver->internal()->set_direction(
				RtpTransceiverDirection::kInactive);
		}
	}
	else {
		bool removed;
		if (sender->media_type() == cricket::MEDIA_TYPE_AUDIO) {
			removed = rtp_manager()->GetAudioTransceiver()->internal()->RemoveSender(
				sender);
		}
		else {
			RTC_DCHECK_EQ(cricket::MEDIA_TYPE_VIDEO, sender->media_type());
			removed = rtp_manager()->GetVideoTransceiver()->internal()->RemoveSender(
				sender);
		}
		if (!removed) {
			LOG_AND_RETURN_ERROR(
				RTCErrorType::INVALID_PARAMETER,
				"Couldn't find sender " + sender->id() + " to remove.");
		}
	}

	_sdp_handler->UpdateNegotiationNeeded();

	return RTCError::OK();
}


RTCErrorOr<rtc::scoped_refptr<RTCRtpTransceiverInterface>> RTCConnection::AddTransceiver(
	rtc::scoped_refptr<MediaStreamTrackInterface> track) {
	return AddTransceiver(track, RTCRtpTransceiverInit());
}

RTCErrorOr<rtc::scoped_refptr<RTCRtpTransceiverInterface>> RTCConnection::AddTransceiver(
	rtc::scoped_refptr<MediaStreamTrackInterface> track,
	const RTCRtpTransceiverInit& init) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	RTC_CHECK(IsUnifiedPlan())
		<< "AddTransceiver is only available with Unified Plan SdpSemantics";
	if (!track) {
		LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_PARAMETER, "track is null");
	}
	cricket::MediaType media_type;
	if (track->kind() == MediaStreamTrackInterface::kAudioKind) {
		media_type = cricket::MEDIA_TYPE_AUDIO;
	}
	else if (track->kind() == MediaStreamTrackInterface::kVideoKind) {
		media_type = cricket::MEDIA_TYPE_VIDEO;
	}
	else {
		LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_PARAMETER,
			"Track kind is not audio or video");
	}
	return AddTransceiver(media_type, track, init);
}

RTCErrorOr<rtc::scoped_refptr<RTCRtpTransceiverInterface>> RTCConnection::AddTransceiver(
	cricket::MediaType media_type) {
	return AddTransceiver(media_type, RTCRtpTransceiverInit());
}

RTCErrorOr<rtc::scoped_refptr<RTCRtpTransceiverInterface>> RTCConnection::AddTransceiver(
	cricket::MediaType media_type,
	const RTCRtpTransceiverInit& init) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	RTC_CHECK(IsUnifiedPlan())
		<< "AddTransceiver is only available with Unified Plan SdpSemantics";
	if (!(media_type == cricket::MEDIA_TYPE_AUDIO ||
		media_type == cricket::MEDIA_TYPE_VIDEO)) {
		LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_PARAMETER,
			"media type is not audio or video");
	}
	return AddTransceiver(media_type, nullptr, init);
}

RTCErrorOr<rtc::scoped_refptr<RTCRtpTransceiverInterface>> RTCConnection::AddTransceiver(
	cricket::MediaType media_type,
	rtc::scoped_refptr<MediaStreamTrackInterface> track,
	const RTCRtpTransceiverInit& init,
	bool update_negotiation_needed) {
	
	RTC_DCHECK_RUN_ON(signaling_thread());
	RTC_DCHECK((media_type == cricket::MEDIA_TYPE_AUDIO ||
		media_type == cricket::MEDIA_TYPE_VIDEO));
	if (track) {
		RTC_DCHECK_EQ(media_type,
			(track->kind() == MediaStreamTrackInterface::kAudioKind
			? cricket::MEDIA_TYPE_AUDIO
			: cricket::MEDIA_TYPE_VIDEO));
	}

	size_t num_rids = absl::c_count_if(init.send_encodings,
		[](const RtpEncodingParameters& encoding) {
		return !encoding.rid.empty();
	});
	if (num_rids > 0 && num_rids != init.send_encodings.size()) {
		LOG_AND_RETURN_ERROR(
			RTCErrorType::INVALID_PARAMETER,
			"RIDs must be provided for either all or none of the send encodings.");
	}

	if (num_rids > 0 && absl::c_any_of(init.send_encodings,
		[](const RtpEncodingParameters& encoding) {
		return !IsLegalRsidName(encoding.rid);
	})) {
		LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_PARAMETER,
			"Invalid RID value provided.");
	}

	if (absl::c_any_of(init.send_encodings,
		[](const RtpEncodingParameters& encoding) {
		return encoding.ssrc.has_value();
	})) {
		LOG_AND_RETURN_ERROR(
			RTCErrorType::UNSUPPORTED_PARAMETER,
			"Attempted to set an unimplemented parameter of RtpParameters.");
	}

	RtpParameters parameters;
	parameters.encodings = init.send_encodings;

	// Encodings are dropped from the tail if too many are provided.
	if (parameters.encodings.size() > kMaxSimulcastStreams) {
		parameters.encodings.erase(
			parameters.encodings.begin() + kMaxSimulcastStreams,
			parameters.encodings.end());
	}

	// Single RID should be removed.
	if (parameters.encodings.size() == 1 &&
		!parameters.encodings[0].rid.empty()) {
		RTC_LOG(LS_INFO) << "Removing RID: " << parameters.encodings[0].rid << ".";
		parameters.encodings[0].rid.clear();
	}

	// If RIDs were not provided, they are generated for simulcast scenario.
	if (parameters.encodings.size() > 1 && num_rids == 0) {
		rtc::UniqueStringGenerator rid_generator;
		for (RtpEncodingParameters& encoding : parameters.encodings) {
			encoding.rid = rid_generator();
		}
	}

	if (UnimplementedRtpParameterHasValue(parameters)) {
		LOG_AND_RETURN_ERROR(
			RTCErrorType::UNSUPPORTED_PARAMETER,
			"Attempted to set an unimplemented parameter of RtpParameters.");
	}

	auto result = cricket::CheckRtpParametersValues(parameters);
	if (!result.ok()) {
		LOG_AND_RETURN_ERROR(result.type(), result.message());
	}

	RTC_LOG(LS_INFO) << "Adding " << cricket::MediaTypeToString(media_type)
		<< " transceiver in response to a call to AddTransceiver.";
	// Set the sender ID equal to the track ID if the track is specified unless
	// that sender ID is already in use.

	std::string sender_id = (track && !rtp_manager()->FindSenderById(track->id())
		? track->id()
		: rtc::CreateRandomUuid());

	auto sender = rtp_manager()->CreateSender(
		media_type, sender_id, track, init.stream_ids, parameters.encodings);
	auto receiver =
		rtp_manager()->CreateReceiver(media_type, rtc::CreateRandomUuid());
	auto transceiver = rtp_manager()->CreateAndAddTransceiver(sender, receiver);
	transceiver->internal()->set_direction(init.direction);

	if (update_negotiation_needed) {
		_sdp_handler->UpdateNegotiationNeeded();	
	}

	return rtc::scoped_refptr<RTCRtpTransceiverInterface>(transceiver);
}

std::vector<rtc::scoped_refptr<RTCRtpSenderInterface>> RTCConnection::GetSenders() const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	std::vector<rtc::scoped_refptr<RTCRtpSenderInterface>> ret;

	for (const auto& sender : rtp_manager()->GetSendersInternal()) {
		ret.push_back(sender);
	}

	return ret;
}

std::vector<rtc::scoped_refptr<RTCRtpReceiverInterface>> RTCConnection::GetReceivers() const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	std::vector<rtc::scoped_refptr<RTCRtpReceiverInterface>> ret;

	for (const auto& receiver : rtp_manager()->GetReceiversInternal()) {
		ret.push_back(receiver);
	}

	return ret;
}

std::vector<rtc::scoped_refptr<RTCRtpTransceiverInterface>> RTCConnection::GetTransceivers() const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	RTC_CHECK(IsUnifiedPlan())
		<< "GetTransceivers is only supported with Unified Plan SdpSemantics.";
	std::vector<rtc::scoped_refptr<RTCRtpTransceiverInterface>> all_transceivers;
	
	for (const auto& transceiver : rtp_manager()->transceivers()->List()) {
		all_transceivers.push_back(transceiver);
	}

	return all_transceivers;
}


rtc::scoped_refptr<DataChannelInterface> RTCConnection::CreateDataChannel(
					const std::string& label,
					const DataChannelInit* config) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	RTC_LOG(LS_VERBOSE) << "RTCConnection::CreateDataChannel";

	bool first_datachannel = !data_channel_controller_.HasDataChannels();

	std::unique_ptr<InternalDataChannelInit> internal_config;
	if (config) {
		internal_config.reset(new InternalDataChannelInit(*config));
	}
	rtc::scoped_refptr<DataChannelInterface> channel(
		data_channel_controller_.InternalCreateDataChannelWithProxy(
		label, internal_config.get()));
	if (!channel.get()) {
		return nullptr;
	}

	// Trigger the onRenegotiationNeeded event for every new RTP DataChannel, or
	// the first SCTP DataChannel.
	if (data_channel_type() == cricket::DCT_RTP || first_datachannel) {
		_sdp_handler->UpdateNegotiationNeeded();
	}
	return channel;
}

void RTCConnection::GenerateKeyFrame() {
	RTC_DCHECK_RUN_ON(signaling_thread());
	if (call_ != nullptr) {
		call_->GenerateKeyFrame();
	}
}

void RTCConnection::GenerateKeyFramePli(uint32_t ssrc) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	if (call_ != nullptr) {
		call_->GenerateKeyFramePli(ssrc);
	}
}

const SessionDescriptionInterface* RTCConnection::local_description() const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	return _sdp_handler->local_description();
}

const SessionDescriptionInterface* RTCConnection::remote_description() const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	return _sdp_handler->remote_description();
}

const SessionDescriptionInterface* RTCConnection::current_local_description() const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	return _sdp_handler->current_local_description();
}

const SessionDescriptionInterface* RTCConnection::current_remote_description()const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	return _sdp_handler->current_remote_description();
}

const SessionDescriptionInterface* RTCConnection::pending_local_description() const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	return _sdp_handler->pending_local_description();
}

const SessionDescriptionInterface* RTCConnection::pending_remote_description() const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	return _sdp_handler->pending_remote_description();
}

void RTCConnection::Close() {
	RTC_DCHECK_RUN_ON(signaling_thread());

	if (IsClosed()) {
		return;
	}

	ice_connection_state_ = PeerConnectionInterfaceDefs::kIceConnectionClosed;
	Observer()->OnIceConnectionChange(ice_connection_state_);
	standardized_ice_connection_state_ =
		PeerConnectionInterfaceDefs::IceConnectionState::kIceConnectionClosed;

	connection_state_ = PeerConnectionInterfaceDefs::PeerConnectionState::kClosed;
	Observer()->OnConnectionChange(connection_state_);

	_sdp_handler->Close();

	for (const auto& transceiver : rtp_manager()->transceivers()->List()) {
		transceiver->internal()->SetRTCConnectionClosed();
		if (!transceiver->stopped())
			transceiver->StopInternal();
	}

	_sdp_handler->DestroyAllChannels();

	_sdp_handler->ResetSessionDescFactory();
	_transport_controller.reset();
	rtp_manager_->Close();

	network_thread()->Invoke<void>(
		RTC_FROM_HERE, [this] { _allocator->DiscardCandidatePool(); });

	worker_thread()->Invoke<void>(RTC_FROM_HERE, [this] {
		RTC_DCHECK_RUN_ON(worker_thread());
		call_safety_.reset();
		call_.reset();
	});

	_observer = nullptr;
}

bool RTCConnection::IsClosed() const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	return _sdp_handler->signaling_state() == PeerConnectionInterfaceDefs::kClosed;
}

RTCCall::Stats RTCConnection::GetCallStats() {
	if (!worker_thread()->IsCurrent()) {
		return worker_thread()->Invoke<RTCCall::Stats>(
			RTC_FROM_HERE, rtc::Bind(&RTCConnection::GetCallStats, this));
	}
	RTC_DCHECK_RUN_ON(worker_thread());
	rtc::Thread::ScopedDisallowBlockingCalls no_blocking_calls(_rtc_thread_manager);
	if (call_) {
		return call_->GetStats();
	}
	else {
		return RTCCall::Stats();
	}
}

bool RTCConnection::GetSctpSslRole(rtc::SSLRole* role) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	if (!local_description() || !remote_description()) {
		RTC_LOG(LS_VERBOSE)
			<< "Local and Remote descriptions must be applied to get the "
			"SSL Role of the SCTP transport.";
		return false;
	}
	if (!data_channel_controller_.data_channel_transport()) {
		RTC_LOG(LS_INFO) << "Non-rejected SCTP m= section is needed to get the "
			"SSL Role of the SCTP transport.";
		return false;
	}

	absl::optional<rtc::SSLRole> dtls_role;
	if (sctp_mid_s_) {
		dtls_role = _transport_controller->GetDtlsRole(*sctp_mid_s_);
		if (!dtls_role && _sdp_handler->is_caller().has_value()) {
			dtls_role =
				*_sdp_handler->is_caller() ? rtc::SSL_SERVER : rtc::SSL_CLIENT;
		}
		*role = *dtls_role;
		return true;
	}
	return false;
}

void RTCConnection::OnSctpDataChannelClosed(DataChannelInterface* channel) {
	data_channel_controller_.OnSctpDataChannelClosed(
		static_cast<SctpDataChannel*>(channel));
}

bool RTCConnection::ShouldFireNegotiationNeededEvent(uint32_t event_id) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	return _sdp_handler->ShouldFireNegotiationNeededEvent(event_id);
}

SctpDataChannel* RTCConnection::FindDataChannelBySid(int sid) const {
	return data_channel_controller_.FindDataChannelBySid(sid);
}

std::vector<DataChannelStats> RTCConnection::GetDataChannelStats() const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	return data_channel_controller_.GetDataChannelStats();
}

absl::optional<std::string> RTCConnection::sctp_transport_name() const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	if (sctp_mid_s_ && _transport_controller) {
		auto dtls_transport = _transport_controller->GetDtlsTransport(*sctp_mid_s_);
		if (dtls_transport) {
			return dtls_transport->transport_name();
		}
		return absl::optional<std::string>();
	}
	return absl::optional<std::string>();
}

absl::optional<std::string> RTCConnection::GetDataMid() const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	switch (data_channel_type()) {
	case cricket::DCT_RTP:
		if (!data_channel_controller_.rtp_data_channel()) {
			return absl::nullopt;
		}
		return data_channel_controller_.rtp_data_channel()->content_name();
	case cricket::DCT_SCTP:
		return sctp_mid_s_;
	default:
		return absl::nullopt;
	}
}

cricket::DataChannelType RTCConnection::data_channel_type() const {
	return data_channel_controller_.data_channel_type();
}

void RTCConnection::SetIceConnectionState(PeerConnectionInterfaceDefs::IceConnectionState new_state) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	if (ice_connection_state_ == new_state) {
		return;
	}

	// After transitioning to "closed", ignore any additional states from
	// TransportController (such as "disconnected").
	if (IsClosed()) {
		return;
	}

	RTC_LOG(LS_INFO) <<"[" << this <<"] Changing IceConnectionState " << ice_connection_state_
		<< " => " << new_state;
	RTC_DCHECK(ice_connection_state_ !=
		PeerConnectionInterfaceDefs::kIceConnectionClosed);

	ice_connection_state_ = new_state;

	Observer()->OnIceConnectionChange(ice_connection_state_);	
}

RTCConnectionObserver* RTCConnection::Observer() const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	RTC_DCHECK(_observer);
	return _observer;
}

RTCError RTCConnection::_initialize(
	PeerConnectionInterfaceDefs::RTCConfiguration &rtcConfigure) {
	RTC_DCHECK_RUN_ON(signaling_thread());

	cricket::ServerAddresses stun_servers;
	std::vector<cricket::RelayServerConfig> turn_servers;

	RTCErrorType parse_error =
		ParseIceServers(rtcConfigure.servers, &stun_servers, &turn_servers);
	if (parse_error != RTCErrorType::NONE) {
		return RTCError(parse_error, "ICE server parse failed");
	}

	_create_codecs_info();

	std::unique_ptr<cricket::MediaEngineInterface> media_engine = 
		cricket::CreateMediaEngine(
		task_queue_factory_,
		_recv_audio_codecs, _send_audio_codecs,
		_recv_video_codecs, _send_video_codecs);

	// channel manager
	_channel_manager = std::make_unique<cricket::RTCChannelManager>(
		std::move(media_engine), 
		std::make_unique<cricket::RtpDataEngine>(),	
		worker_thread(), network_thread(),
		_rtc_thread_manager
		);

	_channel_manager->SetVideoRtxEnabled(true);
	_channel_manager->Init();


	// cert generateor
	std::unique_ptr<rtc::RTCCertificateGeneratorInterface> cert_generator = 
		std::make_unique<rtc::RTCCertificateGenerator>(signaling_thread(), network_thread());

	// default network manager
	_default_network_manager = std::make_unique<rtc::BasicNetworkManager>(nullptr, _rtc_thread_manager	);

	// packet socket factory
	_packet_socket_factory = std::make_unique<rtc::BasicPacketSocketFactory>(network_thread(), _rtc_thread_manager);

	// allocator
	_allocator = std::make_unique<cricket::BasicPortAllocator>(_default_network_manager.get(),
		_rtc_thread_manager,
		_packet_socket_factory.get(), nullptr);

	// resolver factory
	_async_resolver_factory = std::make_unique<BasicAsyncResolverFactory>();

	// ice transport factory
	_ice_transport_factory = std::make_unique<DefaultIceTransportFactory>(_rtc_thread_manager);
	
	_allocator->SetNetworkIgnoreMask(rtc::AdapterType::ADAPTER_TYPE_LOOPBACK);

	const auto pa_result = network_thread()->Invoke<InitializePortAllocatorResult>(
		RTC_FROM_HERE,
		rtc::Bind(&RTCConnection::_initialize_port_allocator_n, this,
		stun_servers, turn_servers, rtcConfigure));

	// Add the turn logging id to all turn servers
	for (cricket::RelayServerConfig& turn_server : turn_servers) {
		turn_server.turn_logging_id = rtcConfigure.turn_logging_id;
	}

	// Send information about IPv4/IPv6 status.
	PeerConnectionAddressFamilyCounter address_family;
	if (pa_result.enable_ipv6) {
		address_family = kPeerConnection_IPv6;
	}
	else {
		address_family = kPeerConnection_IPv4;
	}

	// RFC 3264: The numeric value of the session id and version in the
	// o line MUST be representable with a "64 bit signed integer".
	// Due to this constraint session id |session_id_| is max limited to
	// LLONG_MAX.
	_session_id = rtc::ToString(rtc::CreateRandomId64() & LLONG_MAX);
	JsepTransportController::Config config;
	config.redetermine_role_on_ice_restart = 
		rtcConfigure.redetermine_role_on_ice_restart;
	config.ssl_max_version = rtc::SSL_PROTOCOL_DTLS_12;
	config.disable_encryption = false;
	config.bundle_policy = rtcConfigure.bundle_policy;
	config.rtcp_mux_policy = rtcConfigure.rtcp_mux_policy;
	config.crypto_options = rtcConfigure.crypto_options.has_value()
							? *rtcConfigure.crypto_options
							: CryptoOptions::NoGcm();
	config.transport_observer = this;
	config.active_reset_srtp_params = rtcConfigure.active_reset_srtp_params;
	config.rtcp_handler = InitializeRtcpCallback();

	_dtls_enabled = (cert_generator != nullptr || !rtcConfigure.certificates.empty());
	if (rtcConfigure.enable_dtls_srtp) {
		_dtls_enabled = *(rtcConfigure.enable_dtls_srtp);
	}

	if (rtcConfigure.enable_rtp_data_channel) {
		data_channel_controller_.set_data_channel_type(cricket::DCT_RTP);
	}
	else {
		if (_dtls_enabled) {
			data_channel_controller_.set_data_channel_type(cricket::DCT_SCTP);
			config.sctp_factory = sctp_factory_.get();
		}
	}

	config.ice_transport_factory = _ice_transport_factory.get();

	config.on_dtls_handshake_error_ =
		[weak_ptr = _weak_factory.GetWeakPtr()](rtc::SSLHandshakeError s) {
		if (weak_ptr) {
			weak_ptr->OnTransportControllerDtlsHandshakeError(s);
		}
	};

	config._rtc_thread_manager = _rtc_thread_manager;

	_transport_controller.reset(new JsepTransportController(
		signaling_thread(), network_thread(), _allocator.get(),
		_async_resolver_factory.get(), config));

	// The following RTC_DCHECKs are added by looking at the caller thread.
	// If this is incorrect there might not be test failures
	// due to lack of unit tests which trigger these scenarios.
	// TODO(bugs.webrtc.org/12160): Remove above comments.
	// callbacks for signaling_thread.
	_transport_controller->SubscribeIceConnectionState(
		[this](cricket::IceConnectionState s) {
		RTC_DCHECK_RUN_ON(signaling_thread());
		OnTransportControllerConnectionState(s);
	});
	_transport_controller->SubscribeConnectionState(
		[this](PeerConnectionInterfaceDefs::PeerConnectionState s) {
		RTC_DCHECK_RUN_ON(signaling_thread());
		SetConnectionState(s);
	});
	_transport_controller->SubscribeStandardizedIceConnectionState(
		[this](PeerConnectionInterfaceDefs::IceConnectionState s) {
		RTC_DCHECK_RUN_ON(signaling_thread());
		SetStandardizedIceConnectionState(s);
	});
	_transport_controller->SubscribeIceGatheringState(
		[this](cricket::IceGatheringState s) {
		RTC_DCHECK_RUN_ON(signaling_thread());
		OnTransportControllerGatheringState(s);
	});
	_transport_controller->SubscribeIceCandidateGathered(
		[this](const std::string& transport,
		const std::vector<cricket::Candidate>& candidates) {
		RTC_DCHECK_RUN_ON(signaling_thread());
		OnTransportControllerCandidatesGathered(transport, candidates);
	});
	_transport_controller->SubscribeIceCandidateError(
		[this](const cricket::IceCandidateErrorEvent& event) {
		RTC_DCHECK_RUN_ON(signaling_thread());
		OnTransportControllerCandidateError(event);
	});
	_transport_controller->SubscribeIceCandidatesRemoved(
		[this](const std::vector<cricket::Candidate>& c) {
		RTC_DCHECK_RUN_ON(signaling_thread());
		OnTransportControllerCandidatesRemoved(c);
	});
	_transport_controller->SubscribeIceCandidatePairChanged(
		[this](const cricket::CandidatePairChangeEvent& event) {
		RTC_DCHECK_RUN_ON(signaling_thread());
		OnTransportControllerCandidateChanged(event);
	});

	if (!rtcConfigure.crypto_options.has_value()) {
		rtcConfigure.crypto_options = CryptoOptions::NoGcm();
	}

	_rtcConfigure = rtcConfigure;

	_transport_controller->SetIceConfig(ParseIceConfig(rtcConfigure));

	_sdp_handler = RTCSdpOfferAnswerHandler::Create(this, rtcConfigure, 
		std::move(cert_generator)	, _rtc_thread_manager);

	
	rtp_manager_ = std::make_unique<RTCRtpTransmissionManager>(
		IsUnifiedPlan(), signaling_thread(), worker_thread(), channel_manager(),
		_observer, [this]() {
		RTC_DCHECK_RUN_ON(signaling_thread());
		_sdp_handler->UpdateNegotiationNeeded();
		}, _rtc_thread_manager	);
	
	// only PlanB
	if (!IsUnifiedPlan()) {
		rtp_manager()->transceivers()->Add(
			RTCRtpTransceiverProxyWithInternal<RTCRtpTransceiver>::Create(
			signaling_thread(), rtc::make_ref_counted<RTCRtpTransceiver>(cricket::MEDIA_TYPE_AUDIO, _rtc_thread_manager)));
		rtp_manager()->transceivers()->Add(
			RTCRtpTransceiverProxyWithInternal<RTCRtpTransceiver>::Create(
			signaling_thread(), rtc::make_ref_counted<RTCRtpTransceiver>(cricket::MEDIA_TYPE_VIDEO, _rtc_thread_manager)));
	}

	return RTCError::OK();
}

RTCConnection::InitializePortAllocatorResult RTCConnection::_initialize_port_allocator_n(
	const cricket::ServerAddresses& stun_servers,
	const std::vector<cricket::RelayServerConfig>& turn_servers,
	const PeerConnectionInterfaceDefs::RTCConfiguration &rtcConfigure) {
	RTC_DCHECK_RUN_ON(network_thread());

	RTC_LOG(LS_VERBOSE) << "RTCConnection::_initialize_port_allocator_n";

	_allocator->Initialize();

	int port_allocator_flags = _allocator->flags();
	port_allocator_flags |= cricket::PORTALLOCATOR_ENABLE_SHARED_SOCKET |
							cricket::PORTALLOCATOR_ENABLE_IPV6 |
							cricket::PORTALLOCATOR_ENABLE_IPV6_ON_WIFI;

	if (rtcConfigure.tcp_candidate_policy == PeerConnectionInterfaceDefs::kTcpCandidatePolicyDisabled) {
		port_allocator_flags |= cricket::PORTALLOCATOR_DISABLE_TCP;
	}

	if (rtcConfigure.candidate_network_policy ==
		PeerConnectionInterfaceDefs::kCandidateNetworkPolicyLowCost) {
		port_allocator_flags |= cricket::PORTALLOCATOR_DISABLE_COSTLY_NETWORKS;
		RTC_LOG(LS_INFO) << "Do not gather candidates on high-cost networks";
	}

	_allocator->set_flags(port_allocator_flags);
	// No step delay is used while allocating ports.
	_allocator->set_step_delay(cricket::kMinimumStepDelay);
	_allocator->SetCandidateFilter(ConvertIceTransportTypeToCandidateFilter(rtcConfigure.type));
	_allocator->set_max_ipv6_networks(rtcConfigure.max_ipv6_networks);

	auto turn_servers_copy = turn_servers;
	for (auto& turn_server : turn_servers_copy) {
		turn_server.tls_cert_verifier = _tls_cert_verifier.get();
	}

	// Call this last since it may create pooled allocator sessions using the
	// properties set above.
	RTC_LOG(LS_VERBOSE) << "RTCConnection::_initialize_port_allocator_n _allocator->SetConfiguration+++";
	_allocator->SetConfiguration(
		stun_servers, std::move(turn_servers_copy),
		rtcConfigure.ice_candidate_pool_size,
		rtcConfigure.GetTurnPortPrunePolicy(),
		rtcConfigure.turn_customizer, 
		rtcConfigure.stun_candidate_keepalive_interval);
	RTC_LOG(LS_VERBOSE) << "RTCConnection::_initialize_port_allocator_n _allocator->SetConfiguration---";

	InitializePortAllocatorResult res;
	res.enable_ipv6 = port_allocator_flags & cricket::PORTALLOCATOR_ENABLE_IPV6;

	RTC_LOG(LS_VERBOSE) << "_initialize_port_allocator_n---";
	return res;
}

rtc::scoped_refptr<RTCRtpTransceiverProxyWithInternal<RTCRtpTransceiver>>
RTCConnection::FindTransceiverBySender(rtc::scoped_refptr<RTCRtpSenderInterface> sender) {
	return rtp_manager()->transceivers()->FindBySender(sender);
}

bool RTCConnection::OnTransportChanged(const std::string& mid,
									   RtpTransportInternal* rtp_transport,
									   rtc::scoped_refptr<DtlsTransport> dtls_transport,
									   DataChannelTransportInterface* data_channel_transport) {
	RTC_DCHECK_RUN_ON(network_thread());
	bool ret = true;
	auto base_channel = GetChannel(mid);
	if (base_channel) {
		ret = base_channel->SetRtpTransport(rtp_transport);
	}
	if (mid == sctp_mid_n_) {
		data_channel_controller_.OnTransportChanged(data_channel_transport);
	}
	return ret;
}

void RTCConnection::OnTransportControllerDtlsHandshakeError(rtc::SSLHandshakeError error) {

}

void RTCConnection::OnTransportControllerConnectionState(cricket::IceConnectionState state) {
	switch (state) {
	case cricket::kIceConnectionConnecting:
		// If the current state is Connected or Completed, then there were
		// writable channels but now there are not, so the next state must
		// be Disconnected.
		// kIceConnectionConnecting is currently used as the default,
		// un-connected state by the TransportController, so its only use is
		// detecting disconnections.
		if (ice_connection_state_ ==
			PeerConnectionInterfaceDefs::kIceConnectionConnected ||
			ice_connection_state_ ==
			PeerConnectionInterfaceDefs::kIceConnectionCompleted) {
			SetIceConnectionState(
				PeerConnectionInterfaceDefs::kIceConnectionDisconnected);
		}
		break;
	case cricket::kIceConnectionFailed:
		SetIceConnectionState(PeerConnectionInterfaceDefs::kIceConnectionFailed);
		break;
	case cricket::kIceConnectionConnected:
		RTC_LOG(LS_INFO) << "[" << this <<"] Changing to ICE connected state because "
			"all transports are writable.";
		SetIceConnectionState(PeerConnectionInterfaceDefs::kIceConnectionConnected);
		break;
	case cricket::kIceConnectionCompleted:
		RTC_LOG(LS_INFO) << "Changing to ICE completed state because "
			"all transports are complete.";
		if (ice_connection_state_ !=
			PeerConnectionInterfaceDefs::kIceConnectionConnected) {
			// If jumping directly from "checking" to "connected",
			// signal "connected" first.
			SetIceConnectionState(PeerConnectionInterfaceDefs::kIceConnectionConnected);
		}
		SetIceConnectionState(PeerConnectionInterfaceDefs::kIceConnectionCompleted);
		break;
	default:
		RTC_NOTREACHED();
	}
}

void RTCConnection::SetConnectionState(
	PeerConnectionInterfaceDefs::PeerConnectionState new_state) {
	if (connection_state_ == new_state)
		return;
	if (IsClosed())
		return;
	connection_state_ = new_state;
	Observer()->OnConnectionChange(new_state);	
}

void RTCConnection::SetStandardizedIceConnectionState(
	PeerConnectionInterfaceDefs::IceConnectionState new_state) {

	if (standardized_ice_connection_state_ == new_state) {
		return;
	}

	if (IsClosed()) {
		return;
	}

	standardized_ice_connection_state_ = new_state;
	
	Observer()->OnStandardizedIceConnectionChange(new_state);	
}

void RTCConnection::OnTransportControllerGatheringState(cricket::IceGatheringState state) {
	RTC_DCHECK(signaling_thread()->IsCurrent());
	if (state == cricket::kIceGatheringGathering) {
		OnIceGatheringChange(PeerConnectionInterfaceDefs::kIceGatheringGathering);
	}
	else if (state == cricket::kIceGatheringComplete) {
		OnIceGatheringChange(PeerConnectionInterfaceDefs::kIceGatheringComplete);
	}
	else if (state == cricket::kIceGatheringNew) {
		OnIceGatheringChange(PeerConnectionInterfaceDefs::kIceGatheringNew);
	}
	else {
		RTC_LOG(LS_ERROR) << "Unknown state received: " << state;
		RTC_NOTREACHED();
	}
}

void RTCConnection::OnIceGatheringChange(PeerConnectionInterfaceDefs::IceGatheringState new_state) {
	if (IsClosed()) {
		return;
	}
	ice_gathering_state_ = new_state;
	Observer()->OnIceGatheringChange(ice_gathering_state_);	
}

void RTCConnection::OnTransportControllerCandidatesGathered(
	const std::string& transport_name,
	const std::vector<cricket::Candidate>& candidates) {


	int sdp_mline_index;
	if (!GetLocalCandidateMediaIndex(transport_name, &sdp_mline_index)) {
		RTC_LOG(LS_ERROR)
			<< "OnTransportControllerCandidatesGathered: content name "
			<< transport_name << " not found";
		return;
	}

	for (cricket::Candidates::const_iterator citer = candidates.begin();
		citer != candidates.end(); ++citer) {
		// Use transport_name as the candidate media id.
		std::unique_ptr<JsepIceCandidate> candidate(
			new JsepIceCandidate(transport_name, sdp_mline_index, *citer));
		_sdp_handler->AddLocalIceCandidate(candidate.get());
		OnIceCandidate(std::move(candidate));
	}
}

void RTCConnection::OnIceCandidate(std::unique_ptr<IceCandidateInterface> candidate) {
	if (IsClosed()) {
		return;
	}
	
	Observer()->OnIceCandidate(candidate.get());
}

void RTCConnection::OnTransportControllerCandidateError(
	const cricket::IceCandidateErrorEvent& event) {
	OnIceCandidateError(event.address, event.port, event.url, event.error_code,
		event.error_text);
}

void RTCConnection::OnIceCandidateError(const std::string& address,
	int port,
	const std::string& url,
	int error_code,
	const std::string& error_text) {

	if (IsClosed()) {
		return;
	}
	Observer()->OnIceCandidateError(address, port, url, error_code, error_text);		
	// Leftover not to break wpt test during migration to the new API.
	Observer()->OnIceCandidateError(address + ":", url, error_code, error_text);		
}


void RTCConnection::OnTransportControllerCandidatesRemoved(
	const std::vector<cricket::Candidate>& candidates) {
	// Sanity check.
	for (const cricket::Candidate& candidate : candidates) {
		if (candidate.transport_name().empty()) {
			RTC_LOG(LS_ERROR) << "OnTransportControllerCandidatesRemoved: "
				"empty content name in candidate "
				<< candidate.ToString();
			return;
		}
	}
	OnIceCandidatesRemoved(candidates);
}

void RTCConnection::OnIceCandidatesRemoved(const std::vector<cricket::Candidate>& candidates) {
	if (IsClosed()) {
		return;
	}
	Observer()->OnIceCandidatesRemoved(candidates);	
}

void RTCConnection::OnTransportControllerCandidateChanged(
	const cricket::CandidatePairChangeEvent& event) {
	OnSelectedCandidatePairChanged(event);
}

void RTCConnection::OnSelectedCandidatePairChanged(
	const cricket::CandidatePairChangeEvent& event) {
	if (IsClosed()) {
		return;
	}

	if (event.selected_candidate_pair.local_candidate().type() ==
		cricket::LOCAL_PORT_TYPE &&
		event.selected_candidate_pair.remote_candidate().type() ==
		cricket::LOCAL_PORT_TYPE) {
	}

	Observer()->OnIceSelectedCandidatePairChanged(event);
}

std::function<void(const rtc::CopyOnWriteBuffer& packet,
	int64_t packet_time_us)>
RTCConnection::InitializeRtcpCallback() {
	RTC_DCHECK_RUN_ON(signaling_thread());

	auto flag =
		worker_thread()->Invoke<rtc::scoped_refptr<PendingTaskSafetyFlag>>(
		RTC_FROM_HERE, [this] {
		RTC_DCHECK_RUN_ON(worker_thread());
		if (!call_) {
			return rtc::scoped_refptr<PendingTaskSafetyFlag>();
		}
		if (!call_safety_)
			call_safety_.reset(new ScopedTaskSafety());
		return call_safety_->flag();
	});

	if (!flag) {
		return [](const rtc::CopyOnWriteBuffer&, int64_t) {};
	}

	return[this, flag = std::move(flag)](const rtc::CopyOnWriteBuffer& packet,
		int64_t packet_time_us) {
		RTC_DCHECK_RUN_ON(network_thread());
		// TODO(bugs.webrtc.org/11993): We should actually be delivering this call
		// directly to the Call class somehow directly on the network thread and not
		// incur this hop here. The DeliverPacket() method will eventually just have
		// to hop back over to the network thread.
		worker_thread()->PostTask(SafeTask(flag, [this, packet,
			packet_time_us] {
			RTC_DCHECK_RUN_ON(worker_thread());
			call_->Receiver()->DeliverPacket(MediaType::ANY, packet, packet_time_us);
		}));
	};
}


//////////////////////////////////////////////////////////////////////////
// for test and to move other file... 
void RTCConnection::_create_codecs_info() {
	_create_audio_encoder_codecs_info();
	_create_audio_decoder_codecs_info();
	_create_video_encoder_codecs_info();
	_create_video_decoder_codecs_info();
}

void RTCConnection::_create_audio_encoder_codecs_info() {
	// only opus enc
	const int kRtpTimestampRateHz = 48000;
	const int kDefaultMaxPlaybackRate = 48000;
	const int kOpusBitrateFbBps = 32000;

	std::vector<AudioCodecSpec> specs;
	const SdpAudioFormat fmt = { "opus",
		kRtpTimestampRateHz,
		2,
		{ { "minptime", "10" }, { "useinbandfec", "1" } } };

	// opus enc config 
	AudioEncoderOpusConfig config;
	config.num_channels = 1;		
	config.frame_size_ms = AudioEncoderOpusConfig::kDefaultFrameSizeMs;
	config.max_playback_rate_hz = kDefaultMaxPlaybackRate;
	config.fec_enabled = true;
	config.dtx_enabled = false;		
	config.cbr_enabled = false;		
	config.bitrate_bps = kOpusBitrateFbBps * config.num_channels;
	config.application = config.num_channels == 1
		? AudioEncoderOpusConfig::ApplicationMode::kVoip
		: AudioEncoderOpusConfig::ApplicationMode::kAudio;	

	RTC_DCHECK(config.IsOk());

	AudioCodecInfo info(config.sample_rate_hz, config.num_channels,
		*config.bitrate_bps,
		AudioEncoderOpusConfig::kMinBitrateBps,
		AudioEncoderOpusConfig::kMaxBitrateBps);
	info.allow_comfort_noise = false;
	info.supports_network_adaption = true;

	specs.push_back({ fmt, info });

	cricket::PayloadTypeMapper mapper;


	// Only generate CN payload types for these clockrates:
	std::map<int, bool, std::greater<int>> generate_cn = {
		{ 8000, false }, { 16000, false }, { 32000, false } };
	// Only generate telephone-event payload types for these clockrates:
	std::map<int, bool, std::greater<int>> generate_dtmf = {
		{ 8000, false }, { 16000, false }, { 32000, false }, { 48000, false } };

	auto map_format = [&mapper](const SdpAudioFormat& format,
		std::vector<cricket::AudioCodec>* out) {
		absl::optional<cricket::AudioCodec> opt_codec = mapper.ToAudioCodec(format);
		if (opt_codec) {
			if (out) {
				out->push_back(*opt_codec);
			}
		}
		else {
			RTC_LOG(LS_ERROR) << "Unable to assign payload type to format: "
				<< rtc::ToString(format);
		}

		return opt_codec;
	};

	for (const auto& spec : specs) {
		// We need to do some extra stuff before adding the main codecs to out.
		absl::optional<cricket::AudioCodec> opt_codec = map_format(spec.format, nullptr);
		if (opt_codec) {
			cricket::AudioCodec& codec = *opt_codec;
			if (spec.info.supports_network_adaption) {
				codec.AddFeedbackParam(
					cricket::FeedbackParam(cricket::kRtcpFbParamTransportCc, cricket::kParamValueEmpty));
			}

			if (spec.info.allow_comfort_noise) {
				// Generate a CN entry if the decoder allows it and we support the
				// clockrate.
				auto cn = generate_cn.find(spec.format.clockrate_hz);
				if (cn != generate_cn.end()) {
					cn->second = true;
				}
			}

			// Generate a telephone-event entry if we support the clockrate.
			auto dtmf = generate_dtmf.find(spec.format.clockrate_hz);
			if (dtmf != generate_dtmf.end()) {
				dtmf->second = true;
			}

			_send_audio_codecs.push_back(codec);
		}
	}

	// Add CN codecs after "proper" audio codecs.
	for (const auto& cn : generate_cn) {
		if (cn.second) {
			map_format({ cricket::kCnCodecName, cn.first, 1 }, &_send_audio_codecs);
		}
	}

	// Add telephone-event codecs last.
	for (const auto& dtmf : generate_dtmf) {
		if (dtmf.second) {
			map_format({ cricket::kDtmfCodecName, dtmf.first, 1 }, &_send_audio_codecs);
		}
	}
}

void RTCConnection::_create_audio_decoder_codecs_info() {
	// only opus dec
	std::vector<AudioCodecSpec> specs;
	AudioCodecInfo opus_info{ 48000, 1, 64000, 6000, 510000 };
	opus_info.allow_comfort_noise = false;
	opus_info.supports_network_adaption = true;
	SdpAudioFormat opus_format(
	{ "opus", 48000, 2, { { "minptime", "10" }, { "useinbandfec", "1" } } });
	specs.push_back({ std::move(opus_format), opus_info });

	cricket::PayloadTypeMapper mapper;


	// Only generate CN payload types for these clockrates:
	std::map<int, bool, std::greater<int>> generate_cn = {
		{ 8000, false }, { 16000, false }, { 32000, false } };
	// Only generate telephone-event payload types for these clockrates:
	std::map<int, bool, std::greater<int>> generate_dtmf = {
		{ 8000, false }, { 16000, false }, { 32000, false }, { 48000, false } };

	auto map_format = [&mapper](const SdpAudioFormat& format,
		std::vector<cricket::AudioCodec>* out) {
		absl::optional<cricket::AudioCodec> opt_codec = mapper.ToAudioCodec(format);
		if (opt_codec) {
			if (out) {
				RTC_LOG(LS_VERBOSE) << "_create_audio_decoder_codecs_info map_format  push codec ";
				RTC_LOG(LS_VERBOSE) << opt_codec->ToString();
				out->push_back(*opt_codec);
			}
		}
		else {
			RTC_LOG(LS_ERROR) << "Unable to assign payload type to format: "
				<< rtc::ToString(format);
		}

		return opt_codec;
	};

	for (const auto& spec : specs) {
		// We need to do some extra stuff before adding the main codecs to out.
		absl::optional<cricket::AudioCodec> opt_codec = map_format(spec.format, nullptr);
		if (opt_codec) {
			cricket::AudioCodec& codec = *opt_codec;
			if (spec.info.supports_network_adaption) {
				codec.AddFeedbackParam(
					cricket::FeedbackParam(cricket::kRtcpFbParamTransportCc, cricket::kParamValueEmpty));
			}

			if (spec.info.allow_comfort_noise) {
				// Generate a CN entry if the decoder allows it and we support the
				// clockrate.
				auto cn = generate_cn.find(spec.format.clockrate_hz);
				if (cn != generate_cn.end()) {
					cn->second = true;
				}
			}

			// Generate a telephone-event entry if we support the clockrate.
			auto dtmf = generate_dtmf.find(spec.format.clockrate_hz);
			if (dtmf != generate_dtmf.end()) {
				dtmf->second = true;
			}

			RTC_LOG(LS_VERBOSE) << "_create_audio_decoder_codecs_info push codec ";
			RTC_LOG(LS_VERBOSE) << codec.ToString();

			_recv_audio_codecs.push_back(codec);
		}
	}

	// Add CN codecs after "proper" audio codecs.
	for (const auto& cn : generate_cn) {
		if (cn.second) {
			map_format({ cricket::kCnCodecName, cn.first, 1 }, &_recv_audio_codecs);
		}
	}

	// Add telephone-event codecs last.
	for (const auto& dtmf : generate_dtmf) {
		if (dtmf.second) {
			map_format({ cricket::kDtmfCodecName, dtmf.first, 1 }, &_recv_audio_codecs);
		}
	}
}

void RTCConnection::_create_video_encoder_codecs_info() {
	std::vector<SdpVideoFormat> supported_codecs;

#ifdef __USING_VP8__
	supported_codecs.push_back(SdpVideoFormat(cricket::kVp8CodecName));
#endif

#ifdef __USING_H264__
	for (const webrtc::SdpVideoFormat& format : _supported_H264_codecs())
		supported_codecs.push_back(format);
#endif

	static const int kFirstDynamicPayloadTypeLowerRange = 35;
	static const int kLastDynamicPayloadTypeLowerRange = 65;

	static const int kFirstDynamicPayloadTypeUpperRange = 96;
	static const int kLastDynamicPayloadTypeUpperRange = 127;
	int payload_type_upper = kFirstDynamicPayloadTypeUpperRange;
	int payload_type_lower = kFirstDynamicPayloadTypeLowerRange;

	for (const webrtc::SdpVideoFormat& format : supported_codecs) {
		cricket::VideoCodec codec(format);
		bool isCodecValidForLowerRange =
			absl::EqualsIgnoreCase(codec.name, cricket::kFlexfecCodecName);
		if (!isCodecValidForLowerRange) {
			codec.id = payload_type_upper++;
		}
		else {
			codec.id = payload_type_lower++;
		}
		_add_video_default_feedback_params(&codec);
		_send_video_codecs.push_back(codec);

		if (payload_type_upper > kLastDynamicPayloadTypeUpperRange) {
			RTC_LOG(LS_ERROR)
				<< "Out of dynamic payload types [96,127], skipping the rest.";
			// TODO(https://bugs.chromium.org/p/webrtc/issues/detail?id=12194):
			// continue in lower range.
			break;
		}
		if (payload_type_lower > kLastDynamicPayloadTypeLowerRange) {
			// TODO(https://bugs.chromium.org/p/webrtc/issues/detail?id=12248):
			// return an error.
			RTC_LOG(LS_ERROR)
				<< "Out of dynamic payload types [35,65], skipping the rest.";
			break;
		}

		// Add associated RTX codec for non-FEC codecs.
		if (!absl::EqualsIgnoreCase(codec.name, cricket::kUlpfecCodecName) &&
			!absl::EqualsIgnoreCase(codec.name, cricket::kFlexfecCodecName)) {
			if (!isCodecValidForLowerRange) {
				_send_video_codecs.push_back(
					cricket::VideoCodec::CreateRtxCodec(payload_type_upper++, codec.id));
			}
			else {
				_send_video_codecs.push_back(
					cricket::VideoCodec::CreateRtxCodec(payload_type_lower++, codec.id));
			}

			if (payload_type_upper > kLastDynamicPayloadTypeUpperRange) {
				RTC_LOG(LS_ERROR)
					<< "Out of dynamic payload types [96,127], skipping rtx.";
				// TODO(https://bugs.chromium.org/p/webrtc/issues/detail?id=12194):
				// continue in lower range.
				break;
			}
			if (payload_type_lower > kLastDynamicPayloadTypeLowerRange) {
				// TODO(https://bugs.chromium.org/p/webrtc/issues/detail?id=12248):
				// return an error.
				RTC_LOG(LS_ERROR)
					<< "Out of dynamic payload types [35,65], skipping rtx.";
				break;
			}
		}
	}
}

void RTCConnection::_create_video_decoder_codecs_info() {
	std::vector<SdpVideoFormat> supported_codecs;
#ifdef __USING_VP8__
	supported_codecs.push_back(SdpVideoFormat(cricket::kVp8CodecName));
#endif

#ifdef __USING_H264__
	for (const webrtc::SdpVideoFormat& format : _supported_H264_codecs())
		supported_codecs.push_back(format);
#endif


	cricket::AddH264ConstrainedBaselineProfileToSupportedFormats(&supported_codecs);

	// Due to interoperability issues with old Chrome/WebRTC versions only use
	// the lower range for new codecs.
	static const int kFirstDynamicPayloadTypeLowerRange = 35;
	static const int kLastDynamicPayloadTypeLowerRange = 65;

	static const int kFirstDynamicPayloadTypeUpperRange = 96;
	static const int kLastDynamicPayloadTypeUpperRange = 127;
	int payload_type_upper = kFirstDynamicPayloadTypeUpperRange;
	int payload_type_lower = kFirstDynamicPayloadTypeLowerRange;

	for (const webrtc::SdpVideoFormat& format : supported_codecs) {
		cricket::VideoCodec codec(format);
		bool isCodecValidForLowerRange =
			absl::EqualsIgnoreCase(codec.name, cricket::kFlexfecCodecName);
		if (!isCodecValidForLowerRange) {
			codec.id = payload_type_upper++;
		}
		else {
			codec.id = payload_type_lower++;
		}
		_add_video_default_feedback_params(&codec);
		_recv_video_codecs.push_back(codec);

		if (payload_type_upper > kLastDynamicPayloadTypeUpperRange) {
			RTC_LOG(LS_ERROR)
				<< "Out of dynamic payload types [96,127], skipping the rest.";
			// TODO(https://bugs.chromium.org/p/webrtc/issues/detail?id=12194):
			// continue in lower range.
			break;
		}
		if (payload_type_lower > kLastDynamicPayloadTypeLowerRange) {
			// TODO(https://bugs.chromium.org/p/webrtc/issues/detail?id=12248):
			// return an error.
			RTC_LOG(LS_ERROR)
				<< "Out of dynamic payload types [35,65], skipping the rest.";
			break;
		}


		// Add associated RTX codec for non-FEC codecs.
		if (!absl::EqualsIgnoreCase(codec.name, cricket::kUlpfecCodecName) &&
			!absl::EqualsIgnoreCase(codec.name, cricket::kFlexfecCodecName)) {
			if (!isCodecValidForLowerRange) {
				_recv_video_codecs.push_back(
					cricket::VideoCodec::CreateRtxCodec(payload_type_upper++, codec.id));
			}
			else {
				_recv_video_codecs.push_back(
					cricket::VideoCodec::CreateRtxCodec(payload_type_lower++, codec.id));
			}

			if (payload_type_upper > kLastDynamicPayloadTypeUpperRange) {
				RTC_LOG(LS_ERROR)
					<< "Out of dynamic payload types [96,127], skipping rtx.";
				// TODO(https://bugs.chromium.org/p/webrtc/issues/detail?id=12194):
				// continue in lower range.
				break;
			}
			if (payload_type_lower > kLastDynamicPayloadTypeLowerRange) {
				// TODO(https://bugs.chromium.org/p/webrtc/issues/detail?id=12248):
				// return an error.
				RTC_LOG(LS_ERROR)
					<< "Out of dynamic payload types [35,65], skipping rtx.";
				break;
			}
		}
	}
}

std::vector<SdpVideoFormat> RTCConnection::_supported_H264_codecs() {
	return{
		_create_h264_format(H264::kProfileConstrainedBaseline, H264::kLevel3_1, "1") };
}

SdpVideoFormat RTCConnection::_create_h264_format(H264::Profile profile, H264::Level level, const std::string& packetization_mode) {
	const absl::optional<std::string> profile_string =
		H264::ProfileLevelIdToString(H264::ProfileLevelId(profile, level));
	RTC_CHECK(profile_string);
	return SdpVideoFormat(
		cricket::kH264CodecName,
		{ { cricket::kH264FmtpProfileLevelId, *profile_string },
		{ cricket::kH264FmtpLevelAsymmetryAllowed, "1" },
		{ cricket::kH264FmtpPacketizationMode, packetization_mode } });
}

void RTCConnection::_add_video_default_feedback_params(cricket::VideoCodec* codec) {
	// Don't add any feedback params for RED and ULPFEC.
	if (codec->name == cricket::kRedCodecName || codec->name == cricket::kUlpfecCodecName)
		return;
	codec->AddFeedbackParam(cricket::FeedbackParam(cricket::kRtcpFbParamRemb, cricket::kParamValueEmpty));
	codec->AddFeedbackParam(
		cricket::FeedbackParam(cricket::kRtcpFbParamTransportCc, cricket::kParamValueEmpty));
	// Don't add any more feedback params for FLEXFEC.
	if (codec->name == cricket::kFlexfecCodecName)
		return;
	codec->AddFeedbackParam(cricket::FeedbackParam(cricket::kRtcpFbParamCcm, cricket::kRtcpFbCcmParamFir));
	codec->AddFeedbackParam(cricket::FeedbackParam(cricket::kRtcpFbParamNack, cricket::kParamValueEmpty));
	codec->AddFeedbackParam(cricket::FeedbackParam(cricket::kRtcpFbParamNack, cricket::kRtcpFbNackParamPli));
}
//////////////////////////////////////////////////////////////////////////

bool RTCConnection::GetLocalCandidateMediaIndex(const std::string& content_name,
												int* sdp_mline_index) {
	if (!local_description() || !sdp_mline_index) {
		return false;
	}

	bool content_found = false;
	const cricket::ContentInfos& contents = local_description()->description()->contents();
	for (size_t index = 0; index < contents.size(); ++index) {
		if (contents[index].name == content_name) {
			*sdp_mline_index = static_cast<int>(index);
			content_found = true;
			break;
		}
	}
	return content_found;
}


void RTCConnection::BaseChannelConnectSentPacketSignal(cricket::RTCBaseChannel *channel) {
	channel->SignalSentPacket().connect(this, &RTCConnection::OnSentPacket_w);
}

void RTCConnection::SctpDataChannelConnectClosedSignal(SctpDataChannel *channel) {
	channel->SignalClosed.connect(this, &RTCConnection::OnSctpDataChannelClosed);
}

}