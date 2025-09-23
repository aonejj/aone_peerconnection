//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : Replace SdpOfferAnswerHandler to RTCSdpOfferAnswerHandler class for rtc media server.
//
//////////////////////////////////////////////////////////////////////////

#include "../rtc_pc_src/_deprecate_defines.h"

#include "api/uma_metrics.h"
#include "api/media_stream_proxy.h"
#include "pc/rtp_media_utils.h"
#include "pc/media_session.h"
#include "pc/media_stream.h"
#include "pc/simulcast_description.h"
#include "p2p/base/transport_description.h"
#include "rtc_base/helpers.h"
#include "rtc_base/bind.h"
#include "system_wrappers/include/metrics.h"

#include "RTCSdpOfferAnswerHandler.h"

using cricket::ContentInfo;
using cricket::ContentInfos;
using cricket::MediaContentDescription;
using cricket::MediaProtocolType;
using cricket::RidDescription;
using cricket::RidDirection;
using cricket::SessionDescription;
using cricket::SimulcastDescription;
using cricket::SimulcastLayer;
using cricket::SimulcastLayerList;
using cricket::StreamParams;
using cricket::TransportInfo;

using cricket::LOCAL_PORT_TYPE;
using cricket::PRFLX_PORT_TYPE;
using cricket::RELAY_PORT_TYPE;
using cricket::STUN_PORT_TYPE;

namespace webrtc {

// Error messages
const char kInvalidSdp[] = "Invalid session description.";
const char kInvalidCandidates[] = "Description contains invalid candidates.";
const char kSessionError[] = "Session error code: ";
const char kSessionErrorDesc[] = "Session error description: ";
const char kSdpWithoutSdesCrypto[] = "Called with SDP without SDES crypto.";
const char kSdpWithoutDtlsFingerprint[] = "Called with SDP without DTLS fingerprint.";
const char kSdpWithoutIceUfragPwd[] =
"Called with SDP without ice-ufrag and ice-pwd.";
const char kBundleWithoutRtcpMux[] =
"rtcp-mux must be enabled when BUNDLE "
"is enabled.";
const char kMlineMismatchInAnswer[] =
"The order of m-lines in answer doesn't match order in offer. Rejecting "
"answer."; 
const char kMlineMismatchInSubsequentOffer[] =
"The order of m-lines in subsequent offer doesn't match order from "
"previous offer/answer.";
const char kSimulcastVersionApplyLocalDescription[] =
"MediaServer.RTCConnection.Simulcast.ApplyLocalDescription";
const char kSimulcastVersionApplyRemoteDescription[] =
"MediaServer.RTCConnection.Simulcast.ApplyRemoteDescription";

const char kDefaultStreamId[] = "default";

static const char kDefaultAudioSenderId[] = "defaulta0";
static const char kDefaultVideoSenderId[] = "defaultv0";



// Wraps a CreateSessionDescriptionObserver and an OperationsChain operation
// complete callback. When the observer is invoked, the wrapped observer is
// invoked followed by invoking the completion callback.
class CreateSessionDescriptionObserverOperationWrapper
	: public CreateSessionDescriptionObserver {
public:
	CreateSessionDescriptionObserverOperationWrapper(
		rtc::scoped_refptr<CreateSessionDescriptionObserver> observer,
		std::function<void()> operation_complete_callback)
		: observer_(std::move(observer)),
		operation_complete_callback_(std::move(operation_complete_callback)) {
		RTC_DCHECK(observer_);
	}
	~CreateSessionDescriptionObserverOperationWrapper() override {
#if RTC_DCHECK_IS_ON
		RTC_DCHECK(was_called_);
#endif
	}

	void OnSuccess(SessionDescriptionInterface* desc) override {
#if RTC_DCHECK_IS_ON
		RTC_DCHECK(!was_called_);
		was_called_ = true;
#endif  // RTC_DCHECK_IS_ON
		// Completing the operation before invoking the observer allows the observer
		// to execute SetLocalDescription() without delay.
		operation_complete_callback_();
		observer_->OnSuccess(desc);	
	}

	void OnFailure(RTCError error) override {
#if RTC_DCHECK_IS_ON
		RTC_DCHECK(!was_called_);
		was_called_ = true;
#endif  // RTC_DCHECK_IS_ON
		operation_complete_callback_();
		observer_->OnFailure(std::move(error));
	}

private:
#if RTC_DCHECK_IS_ON
	bool was_called_ = false;
#endif  // RTC_DCHECK_IS_ON
	rtc::scoped_refptr<CreateSessionDescriptionObserver> observer_;
	std::function<void()> operation_complete_callback_;
};


bool IsValidOfferToReceiveMedia(int value) {
	typedef PeerConnectionInterfaceDefs::RTCOfferAnswerOptions Options;
	return (value >= Options::kUndefined) &&
		(value <= Options::kMaxOfferToReceiveMedia);
}

bool ValidateOfferAnswerOptions(
	const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions& rtc_options) {
	return IsValidOfferToReceiveMedia(rtc_options.offer_to_receive_audio) &&
		IsValidOfferToReceiveMedia(rtc_options.offer_to_receive_video);
}

// From |rtc_options|, fill parts of |session_options| shared by all generated
// m= sectionss (in other words, nothing that involves a map/array).
void ExtractSharedMediaSessionOptions(
	const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions& rtc_options,
	cricket::MediaSessionOptions* session_options) {
	session_options->vad_enabled = rtc_options.voice_activity_detection;
	session_options->bundle_enabled = rtc_options.use_rtp_mux;
	session_options->raw_packetization_for_video =
		rtc_options.raw_packetization_for_video;
}

// The length of RTCP CNAMEs.
static const int kRtcpCnameLength = 16;

// Generate a RTCP CNAME when a PeerConnection is created.
std::string GenerateRtcpCname() {
	std::string cname;
	if (!rtc::CreateRandomString(kRtcpCnameLength, &cname)) {
		RTC_LOG(LS_ERROR) << "Failed to generate CNAME.";
		RTC_NOTREACHED();
	}
	return cname;
}

std::string GetSignalingStateString(
	PeerConnectionInterfaceDefs::SignalingState state) {
	switch (state) {
	case PeerConnectionInterfaceDefs::kStable:
		return "stable";
	case PeerConnectionInterfaceDefs::kHaveLocalOffer:
		return "have-local-offer";
	case PeerConnectionInterfaceDefs::kHaveLocalPrAnswer:
		return "have-local-pranswer";
	case PeerConnectionInterfaceDefs::kHaveRemoteOffer:
		return "have-remote-offer";
	case PeerConnectionInterfaceDefs::kHaveRemotePrAnswer:
		return "have-remote-pranswer";
	case PeerConnectionInterfaceDefs::kClosed:
		return "closed";
	}
	RTC_NOTREACHED();
	return "";
}

static RTCError ValidateMids(const cricket::SessionDescription& description) {
	std::set<std::string> mids;
	for (const ContentInfo& content : description.contents()) {
		if (content.name.empty()) {
			LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_PARAMETER,
				"A media section is missing a MID attribute.");
		}
		if (!mids.insert(content.name).second) {
			LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_PARAMETER,
				"Duplicate a=mid value '" + content.name + "'.");
		}
	}
	return RTCError::OK();
}

std::string GetStreamIdsString(rtc::ArrayView<const std::string> stream_ids) {
	std::string output = "streams=[";
	const char* separator = "";
	for (const auto& stream_id : stream_ids) {
		output.append(separator).append(stream_id);
		separator = ", ";
	}
	output.append("]");
	return output;
}

void NoteKeyProtocolAndMedia(KeyExchangeProtocolType protocol_type,
	cricket::MediaType media_type) {
	// Array of structs needed to map {KeyExchangeProtocolType,
	// cricket::MediaType} to KeyExchangeProtocolMedia without using std::map in
	// order to avoid -Wglobal-constructors and -Wexit-time-destructors.
	static constexpr struct {
		KeyExchangeProtocolType protocol_type;
		cricket::MediaType media_type;
		KeyExchangeProtocolMedia protocol_media;
	} kEnumCounterKeyProtocolMediaMap[] = {
		{ kEnumCounterKeyProtocolDtls, cricket::MEDIA_TYPE_AUDIO,
		kEnumCounterKeyProtocolMediaTypeDtlsAudio },
		{ kEnumCounterKeyProtocolDtls, cricket::MEDIA_TYPE_VIDEO,
		kEnumCounterKeyProtocolMediaTypeDtlsVideo },
		{ kEnumCounterKeyProtocolDtls, cricket::MEDIA_TYPE_DATA,
		kEnumCounterKeyProtocolMediaTypeDtlsData },
		{ kEnumCounterKeyProtocolSdes, cricket::MEDIA_TYPE_AUDIO,
		kEnumCounterKeyProtocolMediaTypeSdesAudio },
		{ kEnumCounterKeyProtocolSdes, cricket::MEDIA_TYPE_VIDEO,
		kEnumCounterKeyProtocolMediaTypeSdesVideo },
		{ kEnumCounterKeyProtocolSdes, cricket::MEDIA_TYPE_DATA,
		kEnumCounterKeyProtocolMediaTypeSdesData },
	};
}

RTCError VerifyCrypto(const cricket::SessionDescription* desc, bool dtls_enabled) {
	const cricket::ContentGroup* bundle =
		desc->GetGroupByName(cricket::GROUP_TYPE_BUNDLE);
	for (const cricket::ContentInfo& content_info : desc->contents()) {
		if (content_info.rejected) {
			continue;
		}
		// Note what media is used with each crypto protocol, for all sections.
		NoteKeyProtocolAndMedia(dtls_enabled ? webrtc::kEnumCounterKeyProtocolDtls
			: webrtc::kEnumCounterKeyProtocolSdes,
			content_info.media_description()->type());
		const std::string& mid = content_info.name;
		if (bundle && bundle->HasContentName(mid) &&
			mid != *(bundle->FirstContentName())) {
			// This isn't the first media section in the BUNDLE group, so it's not
			// required to have crypto attributes, since only the crypto attributes
			// from the first section actually get used.
			continue;
		}

		// If the content isn't rejected or bundled into another m= section, crypto
		// must be present.
		const cricket::MediaContentDescription* media = content_info.media_description();
		const cricket::TransportInfo* tinfo = desc->GetTransportInfoByName(mid);
		if (!media || !tinfo) {
			// Something is not right.
			LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_PARAMETER, kInvalidSdp);
		}
		if (dtls_enabled) {
			if (!tinfo->description.identity_fingerprint) {
				RTC_LOG(LS_WARNING)
					<< "Session description must have DTLS fingerprint if "
					"DTLS enabled.";
				return RTCError(RTCErrorType::INVALID_PARAMETER,
					kSdpWithoutDtlsFingerprint);
			}
		}
		else {
			if (media->cryptos().empty()) {
				RTC_LOG(LS_WARNING)
					<< "Session description must have SDES when DTLS disabled.";
				return RTCError(RTCErrorType::INVALID_PARAMETER, kSdpWithoutSdesCrypto);
			}
		}
	}
	return RTCError::OK();
}

const cricket::ContentInfo* FindTransceiverMSection(
	RTCRtpTransceiverProxyWithInternal<RTCRtpTransceiver>* transceiver,
	const SessionDescriptionInterface* session_description) {
	return transceiver->mid()
		? session_description->description()->GetContentByName(
		*transceiver->mid())
		: nullptr;
}

bool VerifyIceUfragPwdPresent(const cricket::SessionDescription* desc) {
	const cricket::ContentGroup* bundle =
		desc->GetGroupByName(cricket::GROUP_TYPE_BUNDLE);
	for (const cricket::ContentInfo& content_info : desc->contents()) {
		if (content_info.rejected) {
			continue;
		}
		const std::string& mid = content_info.name;
		if (bundle && bundle->HasContentName(mid) &&
			mid != *(bundle->FirstContentName())) {
			// This isn't the first media section in the BUNDLE group, so it's not
			// required to have ufrag/password, since only the ufrag/password from
			// the first section actually get used.
			continue;
		}

		// If the content isn't rejected or bundled into another m= section,
		// ice-ufrag and ice-pwd must be present.
		const cricket::TransportInfo* tinfo = desc->GetTransportInfoByName(mid);
		if (!tinfo) {
			// Something is not right.
			RTC_LOG(LS_ERROR) << kInvalidSdp;
			return false;
		}
		if (tinfo->description.ice_ufrag.empty() ||
			tinfo->description.ice_pwd.empty()) {
			RTC_LOG(LS_ERROR) << "Session description must have ice ufrag and pwd.";
			return false;
		}
	}
	return true;
}

bool MediaSectionsHaveSameCount(const cricket::SessionDescription& desc1,
	const cricket::SessionDescription& desc2) {
	return desc1.contents().size() == desc2.contents().size();
}

bool IsMediaSectionBeingRecycled(SdpType type,
	const cricket::ContentInfo& content,
	const cricket::ContentInfo* old_content_one,
	const cricket::ContentInfo* old_content_two) {
	return type == SdpType::kOffer && !content.rejected &&
		((old_content_one && old_content_one->rejected) ||
		(old_content_two && old_content_two->rejected));
}

bool MediaSectionsInSameOrder(const cricket::SessionDescription& current_desc,
	const cricket::SessionDescription* secondary_current_desc,
	const cricket::SessionDescription& new_desc,
	const SdpType type) {
	if (current_desc.contents().size() > new_desc.contents().size()) {
		return false;
	}

	for (size_t i = 0; i < current_desc.contents().size(); ++i) {
		const cricket::ContentInfo* secondary_content_info = nullptr;
		if (secondary_current_desc &&
			i < secondary_current_desc->contents().size()) {
			secondary_content_info = &secondary_current_desc->contents()[i];
		}
		if (IsMediaSectionBeingRecycled(type, new_desc.contents()[i],
			&current_desc.contents()[i],
			secondary_content_info)) {
			// For new offer descriptions, if the media section can be recycled, it's
			// valid for the MID and media type to change.
			continue;
		}
		if (new_desc.contents()[i].name != current_desc.contents()[i].name) {
			return false;
		}
		const cricket::MediaContentDescription* new_desc_mdesc =
			new_desc.contents()[i].media_description();
		const cricket::MediaContentDescription* current_desc_mdesc =
			current_desc.contents()[i].media_description();
		if (new_desc_mdesc->type() != current_desc_mdesc->type()) {
			return false;
		}
	}
	return true;
}

std::string GetSetDescriptionErrorMessage(cricket::ContentSource source,
	SdpType type,
	const RTCError& error) {
	rtc::StringBuilder oss;
	oss << "Failed to set " << (source == cricket::CS_LOCAL ? "local" : "remote")
		<< " " << SdpTypeToString(type) << " sdp: " << error.message();
	return oss.Release();
}

void ReportSimulcastApiVersion(const char* name,
	const cricket::SessionDescription& session) {
	bool has_legacy = false;
	bool has_spec_compliant = false;
	for (const cricket::ContentInfo& content : session.contents()) {
		if (!content.media_description()) {
			continue;
		}
		has_spec_compliant |= content.media_description()->HasSimulcast();
		for (const cricket::StreamParams& sp : content.media_description()->streams()) {
			has_legacy |= sp.has_ssrc_group(cricket::kSimSsrcGroupSemantics);
		}
	}
}


void AddPlanBRtpSenderOptions(
	const std::vector < rtc::scoped_refptr <
	RTCRtpSenderProxyWithInternal<RTCRtpSenderInternal >> >& senders,
	cricket::MediaDescriptionOptions* audio_media_description_options,
	cricket::MediaDescriptionOptions* video_media_description_options,
	int num_sim_layers) {
	for (const auto& sender : senders) {
		if (sender->media_type() == cricket::MEDIA_TYPE_AUDIO) {
			if (audio_media_description_options) {
				audio_media_description_options->AddAudioSender(
					sender->id(), sender->internal()->stream_ids());
			}
		}
		else {
			RTC_DCHECK(sender->media_type() == cricket::MEDIA_TYPE_VIDEO);
			if (video_media_description_options) {
				video_media_description_options->AddVideoSender(
					sender->id(), sender->internal()->stream_ids(), {},
					cricket::SimulcastLayerList(), num_sim_layers);
			}
		}
	}
}


// The SDP parser used to populate these values by default for the 'content
// name' if an a=mid line was absent.
static absl::string_view GetDefaultMidForPlanB(cricket::MediaType media_type) {
	switch (media_type) {
	case cricket::MEDIA_TYPE_AUDIO:
		return cricket::CN_AUDIO;
	case cricket::MEDIA_TYPE_VIDEO:
		return cricket::CN_VIDEO;
	case cricket::MEDIA_TYPE_DATA:
		return cricket::CN_DATA;
	case cricket::MEDIA_TYPE_UNSUPPORTED:
		return "not supported";
	}
	RTC_NOTREACHED();
	return "";
}

// Returns true if |new_desc| requests an ICE restart (i.e., new ufrag/pwd).
bool CheckForRemoteIceRestart(const SessionDescriptionInterface* old_desc,
	const SessionDescriptionInterface* new_desc,
	const std::string& content_name) {
	if (!old_desc) {
		return false;
	}
	const cricket::SessionDescription* new_sd = new_desc->description();
	const cricket::SessionDescription* old_sd = old_desc->description();
	const cricket::ContentInfo* cinfo = new_sd->GetContentByName(content_name);
	if (!cinfo || cinfo->rejected) {
		return false;
	}
	// If the content isn't rejected, check if ufrag and password has changed.
	const cricket::TransportDescription* new_transport_desc =
		new_sd->GetTransportDescriptionByName(content_name);
	const cricket::TransportDescription* old_transport_desc =
		old_sd->GetTransportDescriptionByName(content_name);
	if (!new_transport_desc || !old_transport_desc) {
		// No transport description exists. This is not an ICE restart.
		return false;
	}
	if (cricket::IceCredentialsChanged(
		old_transport_desc->ice_ufrag, old_transport_desc->ice_pwd,
		new_transport_desc->ice_ufrag, new_transport_desc->ice_pwd)) {
		RTC_LOG(LS_INFO) << "Remote peer requests ICE restart for " << content_name
			<< ".";
		return true;
	}
	return false;
}

static std::vector<RtpEncodingParameters> GetSendEncodingsFromRemoteDescription(
	const cricket::MediaContentDescription& desc) {
	if (!desc.HasSimulcast()) {
		return{};
	}
	std::vector<RtpEncodingParameters> result;
	const cricket::SimulcastDescription& simulcast = desc.simulcast_description();

	// This is a remote description, the parameters we are after should appear
	// as receive streams.
	for (const auto& alternatives : simulcast.receive_layers()) {
		RTC_DCHECK(!alternatives.empty());
		// There is currently no way to specify or choose from alternatives.
		// We will always use the first alternative, which is the most preferred.
		const SimulcastLayer& layer = alternatives[0];
		RtpEncodingParameters parameters;
		parameters.rid = layer.rid;
		parameters.active = !layer.is_paused;
		result.push_back(parameters);
	}

	return result;
}

static bool SimulcastIsRejected(
	const ContentInfo* local_content,
	const MediaContentDescription& answer_media_desc) {
	bool simulcast_offered = local_content &&
		local_content->media_description() &&
		local_content->media_description()->HasSimulcast();
	bool simulcast_answered = answer_media_desc.HasSimulcast();
	bool rids_supported = RtpExtension::FindHeaderExtensionByUri(
		answer_media_desc.rtp_header_extensions(), RtpExtension::kRidUri);
	return simulcast_offered && (!simulcast_answered || !rids_supported);
}

static RTCError DisableSimulcastInSender(
	rtc::scoped_refptr<RTCRtpSenderInternal> sender) {
	RTC_DCHECK(sender);
	RtpParameters parameters = sender->GetParametersInternal();
	if (parameters.encodings.size() <= 1) {
		return RTCError::OK();
	}

	std::vector<std::string> disabled_layers;
	std::transform(
		parameters.encodings.begin() + 1, parameters.encodings.end(),
		std::back_inserter(disabled_layers),
		[](const RtpEncodingParameters& encoding) { return encoding.rid; });
	return sender->DisableEncodingLayers(disabled_layers);
}

static RTCError UpdateSimulcastLayerStatusInSender(
	const std::vector<SimulcastLayer>& layers,
	rtc::scoped_refptr<RTCRtpSenderInternal> sender) {
	RTC_DCHECK(sender);
	RtpParameters parameters = sender->GetParametersInternal();
	std::vector<std::string> disabled_layers;

	// The simulcast envelope cannot be changed, only the status of the streams.
	// So we will iterate over the send encodings rather than the layers.
	for (RtpEncodingParameters& encoding : parameters.encodings) {
		auto iter = std::find_if(layers.begin(), layers.end(),
			[&encoding](const SimulcastLayer& layer) {
			return layer.rid == encoding.rid;
		});
		// A layer that cannot be found may have been removed by the remote party.
		if (iter == layers.end()) {
			disabled_layers.push_back(encoding.rid);
			continue;
		}

		encoding.active = !iter->is_paused;
	}

	RTCError result = sender->SetParametersInternal(parameters);
	if (result.ok()) {
		result = sender->DisableEncodingLayers(disabled_layers);
	}

	return result;
}

static const ContentInfo* GetContentByIndex(
	const SessionDescriptionInterface* sdesc,
	size_t i) {
	if (!sdesc) {
		return nullptr;
	}
	const ContentInfos& contents = sdesc->description()->contents();
	return (i < contents.size() ? &contents[i] : nullptr);
}


static cricket::MediaDescriptionOptions
GetMediaDescriptionOptionsForTransceiver(
		rtc::scoped_refptr<RTCRtpTransceiverProxyWithInternal<RTCRtpTransceiver>>
		transceiver,
		const std::string& mid,
		bool is_create_offer) {
	// NOTE: a stopping transceiver should be treated as a stopped one in
	// createOffer as specified in
	// https://w3c.github.io/webrtc-pc/#dom-rtcpeerconnection-createoffer.
	bool stopped =
		is_create_offer ? transceiver->stopping() : transceiver->stopped();
	cricket::MediaDescriptionOptions media_description_options(
		transceiver->media_type(), mid, transceiver->direction(), stopped);
	media_description_options.codec_preferences =
		transceiver->codec_preferences();
	media_description_options.header_extensions =
		transceiver->HeaderExtensionsToOffer();
	// This behavior is specified in JSEP. The gist is that:
	// 1. The MSID is included if the RtpTransceiver's direction is sendonly or
	//    sendrecv.
	// 2. If the MSID is included, then it must be included in any subsequent
	//    offer/answer exactly the same until the RtpTransceiver is stopped.
	if (stopped || (!RtpTransceiverDirectionHasSend(transceiver->direction()) &&
		!transceiver->internal()->has_ever_been_used_to_send())) {
		return media_description_options;
	}

	cricket::SenderOptions sender_options;
	sender_options.track_id = transceiver->sender()->id();
	sender_options.stream_ids = transceiver->sender()->stream_ids();

	if (!is_create_offer) {
		absl::optional<uint32_t> out_ssrc = transceiver->sender()->GetOutSsrc();
		if (out_ssrc.has_value()) {
			sender_options._out_set_ssrc = out_ssrc;
		}

		absl::optional<uint32_t> out_rtx_ssrc = transceiver->sender()->GetOutRtxSsrc();
		if (out_rtx_ssrc.has_value()) {
			sender_options._out_set_rtx_ssrc = out_rtx_ssrc;
		}
	}

	// The following sets up RIDs and Simulcast.
	// RIDs are included if Simulcast is requested or if any RID was specified.
	RtpParameters send_parameters =
		transceiver->internal()->sender_internal()->GetParametersInternal();
	bool has_rids = std::any_of(send_parameters.encodings.begin(),
		send_parameters.encodings.end(),
		[](const RtpEncodingParameters& encoding) {
		return !encoding.rid.empty();
	});

	std::vector<RidDescription> send_rids;
	SimulcastLayerList send_layers;
	for (const RtpEncodingParameters& encoding : send_parameters.encodings) {
		if (encoding.rid.empty()) {
			continue;
		}
		send_rids.push_back(RidDescription(encoding.rid, RidDirection::kSend));
		send_layers.AddLayer(SimulcastLayer(encoding.rid, !encoding.active));
	}

	if (has_rids) {
		sender_options.rids = send_rids;
	}

	sender_options.simulcast_layers = send_layers;
	// When RIDs are configured, we must set num_sim_layers to 0 to.
	// Otherwise, num_sim_layers must be 1 because either there is no
	// simulcast, or simulcast is acheived by munging the SDP.
	sender_options.num_sim_layers = has_rids ? 0 : 1;
	media_description_options.sender_options.push_back(sender_options);

	return media_description_options;
}

void AddRtpDataChannelOptions(
	const std::map<std::string, rtc::scoped_refptr<RtpDataChannel>>&
	rtp_data_channels,
	cricket::MediaDescriptionOptions* data_media_description_options) {
	if (!data_media_description_options) {
		return;
	}
	// Check for data channels.
	for (const auto& kv : rtp_data_channels) {
		const RtpDataChannel* channel = kv.second;
		if (channel->state() == RtpDataChannel::kConnecting ||
			channel->state() == RtpDataChannel::kOpen) {
			// Legacy RTP data channels are signaled with the track/stream ID set to
			// the data channel's label.
			data_media_description_options->AddRtpDataChannel(channel->label(),
				channel->label());
		}
	}
}


// If the direction is "recvonly" or "inactive", treat the description
// as containing no streams.
// See: https://code.google.com/p/webrtc/issues/detail?id=5054
std::vector<cricket::StreamParams> GetActiveStreams(
	const cricket::MediaContentDescription* desc) {
	return RtpTransceiverDirectionHasSend(desc->direction())
		? desc->streams()
		: std::vector<cricket::StreamParams>();
}

class RTCSdpOfferAnswerHandler::SetSessionDescriptionObserverAdapter
	: public SetLocalDescriptionObserverInterface,
	public SetRemoteDescriptionObserverInterface{
public:
	SetSessionDescriptionObserverAdapter(
		rtc::WeakPtr<RTCSdpOfferAnswerHandler> handler,
		rtc::scoped_refptr<SetSessionDescriptionObserver> inner_observer)
		: handler_(std::move(handler)),
		inner_observer_(std::move(inner_observer)) {}

	// SetLocalDescriptionObserverInterface implementation.
	void OnSetLocalDescriptionComplete(RTCError error) override {
		OnSetDescriptionComplete(std::move(error));
	}
	// SetRemoteDescriptionObserverInterface implementation.
	void OnSetRemoteDescriptionComplete(RTCError error) override {
		OnSetDescriptionComplete(std::move(error));
	}

private:
	void OnSetDescriptionComplete(RTCError error) {
		if (!handler_)
			return;
		if (error.ok()) {
			handler_->_rtc_connection->message_handler()->PostSetSessionDescriptionSuccess(
				inner_observer_);
		}
		else {
			handler_->_rtc_connection->message_handler()->PostSetSessionDescriptionFailure(
				inner_observer_, std::move(error));
		}
	}

	rtc::WeakPtr<RTCSdpOfferAnswerHandler> handler_;
	rtc::scoped_refptr<SetSessionDescriptionObserver> inner_observer_;
};

class RTCSdpOfferAnswerHandler::LocalIceCredentialsToReplace {
public:
	// Sets the ICE credentials that need restarting to the ICE credentials of
	// the current and pending descriptions.
	void SetIceCredentialsFromLocalDescriptions(
		const SessionDescriptionInterface* current_local_description,
		const SessionDescriptionInterface* pending_local_description) {
		ice_credentials_.clear();
		if (current_local_description) {
			AppendIceCredentialsFromSessionDescription(*current_local_description);
		}
		if (pending_local_description) {
			AppendIceCredentialsFromSessionDescription(*pending_local_description);
		}
	}

	void ClearIceCredentials() { ice_credentials_.clear(); }

	// Returns true if we have ICE credentials that need restarting.
	bool HasIceCredentials() const { return !ice_credentials_.empty(); }

	// Returns true if |local_description| shares no ICE credentials with the
	// ICE credentials that need restarting.
	bool SatisfiesIceRestart(
		const SessionDescriptionInterface& local_description) const {
		for (const auto& transport_info :
			local_description.description()->transport_infos()) {
			if (ice_credentials_.find(std::make_pair(
				transport_info.description.ice_ufrag,
				transport_info.description.ice_pwd)) != ice_credentials_.end()) {
				return false;
			}
		}
		return true;
	}

private:
	void AppendIceCredentialsFromSessionDescription(
		const SessionDescriptionInterface& desc) {
		for (const auto& transport_info : desc.description()->transport_infos()) {
			ice_credentials_.insert(
				std::make_pair(transport_info.description.ice_ufrag,
				transport_info.description.ice_pwd));
		}
	}

	std::set<std::pair<std::string, std::string>> ice_credentials_;
};

RTCSdpOfferAnswerHandler::RTCSdpOfferAnswerHandler(RTCConnectionInternal *rtc_connection,
	rtc::RTCThreadManagerInterface* rtc_thread_manager
	)
	: _rtc_connection(rtc_connection),
	weak_ptr_factory_(this),
	local_streams_(StreamCollection::Create()),
	remote_streams_(StreamCollection::Create()),
	operations_chain_(rtc::OperationsChain::Create()),
	rtcp_cname_(GenerateRtcpCname()),
	local_ice_credentials_to_replace_(new LocalIceCredentialsToReplace()) 	, _rtc_thread_manager(rtc_thread_manager)
{

	operations_chain_->SetOnChainEmptyCallback(
		[this_weak_ptr = weak_ptr_factory_.GetWeakPtr()]() {
		if (!this_weak_ptr)
			return;
		this_weak_ptr->OnOperationsChainEmpty();
	});
}

RTCSdpOfferAnswerHandler::~RTCSdpOfferAnswerHandler() {}

std::unique_ptr<RTCSdpOfferAnswerHandler> RTCSdpOfferAnswerHandler::Create(
	RTCConnectionInternal *rtc_connection,
	const PeerConnectionInterfaceDefs::RTCConfiguration& configuration,
	std::unique_ptr<rtc::RTCCertificateGeneratorInterface> cert_generator,
	rtc::RTCThreadManagerInterface* rtc_thread_manager
	) {

	auto handler = absl::WrapUnique(new RTCSdpOfferAnswerHandler(rtc_connection, rtc_thread_manager));
	handler->Initialize(configuration, std::move(cert_generator));

	return handler;
}

void RTCSdpOfferAnswerHandler::Initialize(
	const PeerConnectionInterfaceDefs::RTCConfiguration& configuration,
	std::unique_ptr<rtc::RTCCertificateGeneratorInterface> cert_generator) {
	RTC_DCHECK_RUN_ON(signaling_thread());

	audio_options_.combined_audio_video_bwe =
		configuration.combined_audio_video_bwe;

	audio_options_.audio_jitter_buffer_max_packets =
		configuration.audio_jitter_buffer_max_packets;

	audio_options_.audio_jitter_buffer_fast_accelerate =
		configuration.audio_jitter_buffer_fast_accelerate;

	audio_options_.audio_jitter_buffer_min_delay_ms =
		configuration.audio_jitter_buffer_min_delay_ms;

	audio_options_.audio_jitter_buffer_enable_rtx_handling =
		configuration.audio_jitter_buffer_enable_rtx_handling;


	// Obtain a certificate from RTCConfiguration if any were provided (optional).
	rtc::scoped_refptr<rtc::RTCCertificate> certificate;
	if (!configuration.certificates.empty()) {
		// TODO(hbos,torbjorng): Decide on certificate-selection strategy instead of
		// just picking the first one. The decision should be made based on the DTLS
		// handshake. The DTLS negotiations need to know about all certificates.
		certificate = configuration.certificates[0];
	}

	webrtc_session_desc_factory_ = std::make_unique<WebRtcSessionDescriptionFactory>(
		signaling_thread(), channel_manager(), this, _rtc_connection->session_id(),
		_rtc_connection->dtls_enabled(), std::move(cert_generator),
		certificate, &ssrc_generator_,
		[this](const rtc::scoped_refptr<rtc::RTCCertificate>& certificate) {
		transport_controller()->SetLocalCertificate(certificate);
	});

	if (_rtc_connection->IsDisableEncryption()) {
		webrtc_session_desc_factory_->SetSdesPolicy(cricket::SEC_DISABLED);
	}

	webrtc_session_desc_factory_->set_enable_encrypted_rtp_header_extensions(
		configuration.crypto_options->srtp.enable_encrypted_rtp_header_extensions);
	webrtc_session_desc_factory_->set_is_unified_plan(IsUnifiedPlan());

	webrtc_session_desc_factory_->set_audio_codecs(_rtc_connection->GetSendAudioCodecs(), _rtc_connection->GetRecvAudioCodecs());
	webrtc_session_desc_factory_->set_video_codecs(_rtc_connection->GetSendVideoCodecs(), _rtc_connection->GetRecvVideoCodecs());
}

//////////////////////////////////////////////////////////////////////////
// access to rtc_connection variables
cricket::RTCChannelManager* RTCSdpOfferAnswerHandler::channel_manager() const {
	return _rtc_connection->channel_manager();
}

RTCTransceiverList* RTCSdpOfferAnswerHandler::transceivers() {
	if (!_rtc_connection->rtp_manager()) {
		return nullptr;
	}
	return _rtc_connection->rtp_manager()->transceivers();
}

const RTCTransceiverList* RTCSdpOfferAnswerHandler::transceivers() const {
	if (!_rtc_connection->rtp_manager()) {
		return nullptr;
	}
	return _rtc_connection->rtp_manager()->transceivers();
}

JsepTransportController* RTCSdpOfferAnswerHandler::transport_controller() {
	return _rtc_connection->transport_controller();
}

const JsepTransportController* RTCSdpOfferAnswerHandler::transport_controller() const {
	return _rtc_connection->transport_controller();
}

RTCDataChannelController* RTCSdpOfferAnswerHandler::data_channel_controller() {
	return _rtc_connection->data_channel_controller();
}

const RTCDataChannelController* RTCSdpOfferAnswerHandler::data_channel_controller() const {
	return _rtc_connection->data_channel_controller();
}

cricket::PortAllocator* RTCSdpOfferAnswerHandler::port_allocator() {
	return _rtc_connection->port_allocator();
}

const cricket::PortAllocator* RTCSdpOfferAnswerHandler::port_allocator() const {
	return _rtc_connection->port_allocator();
}

RTCRtpTransmissionManager* RTCSdpOfferAnswerHandler::rtp_manager() {
	return _rtc_connection->rtp_manager();
}

const RTCRtpTransmissionManager* RTCSdpOfferAnswerHandler::rtp_manager() const {
	return _rtc_connection->rtp_manager();
}

//////////////////////////////////////////////////////////////////////////

void RTCSdpOfferAnswerHandler::PrepareForShutdown() {
	RTC_LOG(LS_VERBOSE) << "RTCSdpOfferAnswerHandler::PrepareForShutdown";
	RTC_DCHECK_RUN_ON(signaling_thread());
	weak_ptr_factory_.InvalidateWeakPtrs();
}

void RTCSdpOfferAnswerHandler::Close() {
	ChangeSignalingState(PeerConnectionInterfaceDefs::kClosed);
}

void RTCSdpOfferAnswerHandler::RestartIce() {
	RTC_DCHECK_RUN_ON(signaling_thread());
	local_ice_credentials_to_replace_->SetIceCredentialsFromLocalDescriptions(
		current_local_description(), pending_local_description());
	UpdateNegotiationNeeded();
}

rtc::Thread* RTCSdpOfferAnswerHandler::signaling_thread() const {
	return _rtc_connection->signaling_thread();
}

void RTCSdpOfferAnswerHandler::CreateOffer(
	CreateSessionDescriptionObserver* observer,
	const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions& options) {
	RTC_DCHECK_RUN_ON(signaling_thread());

	// Chain this operation. If asynchronous operations are pending on the chain,
	// this operation will be queued to be invoked, otherwise the contents of the
	// lambda will execute immediately.
	operations_chain_->ChainOperation(
		[this_weak_ptr = weak_ptr_factory_.GetWeakPtr(),
		observer_refptr =
		rtc::scoped_refptr<CreateSessionDescriptionObserver>(observer),
		options](std::function<void()> operations_chain_callback) {
		// Abort early if |this_weak_ptr| is no longer valid.
		if (!this_weak_ptr) {
			observer_refptr->OnFailure(
				RTCError(RTCErrorType::INTERNAL_ERROR,
				"CreateOffer failed because the session was shut down"));
			operations_chain_callback();
			return;
		}
		// The operation completes asynchronously when the wrapper is invoked.
		rtc::scoped_refptr<CreateSessionDescriptionObserverOperationWrapper>
			observer_wrapper(new rtc::RefCountedObject<
			CreateSessionDescriptionObserverOperationWrapper>(
			std::move(observer_refptr),
			std::move(operations_chain_callback)));
		this_weak_ptr->DoCreateOffer(options, observer_wrapper);
	});
}

void RTCSdpOfferAnswerHandler::SetLocalDescription(SetSessionDescriptionObserver* observer,
	SessionDescriptionInterface* desc_ptr) {
	RTC_DCHECK_RUN_ON(signaling_thread());

	operations_chain_->ChainOperation(
		[this_weak_ptr = weak_ptr_factory_.GetWeakPtr(),
		observer_refptr =
		rtc::scoped_refptr<SetSessionDescriptionObserver>(observer),
		desc = std::unique_ptr<SessionDescriptionInterface>(desc_ptr)](
		std::function<void()> operations_chain_callback) mutable {
		// Abort early if |this_weak_ptr| is no longer valid.
		if (!this_weak_ptr) {
			// For consistency with SetSessionDescriptionObserverAdapter whose
			// posted messages doesn't get processed when the PC is destroyed, we
			// do not inform |observer_refptr| that the operation failed.
			operations_chain_callback();
			return;
		}
		// SetSessionDescriptionObserverAdapter takes care of making sure the
		// |observer_refptr| is invoked in a posted message.
		this_weak_ptr->DoSetLocalDescription(
			std::move(desc),
			rtc::scoped_refptr<SetLocalDescriptionObserverInterface>(
			new rtc::RefCountedObject<SetSessionDescriptionObserverAdapter>(
			this_weak_ptr, observer_refptr)));
		// For backwards-compatability reasons, we declare the operation as
		// completed here (rather than in a post), so that the operation chain
		// is not blocked by this operation when the observer is invoked. This
		// allows the observer to trigger subsequent offer/answer operations
		// synchronously if the operation chain is now empty.
		operations_chain_callback();
	});
}

void RTCSdpOfferAnswerHandler::SetLocalDescription(rtc::scoped_refptr<SetSessionDescriptionObserver> observer,
	std::unique_ptr<SessionDescriptionInterface> desc) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	operations_chain_->ChainOperation(
		[this_weak_ptr = weak_ptr_factory_.GetWeakPtr(),
		observer,
		desc = std::move(desc)](
		std::function<void()> operations_chain_callback) mutable {
		// Abort early if |this_weak_ptr| is no longer valid.
		if (!this_weak_ptr) {
			operations_chain_callback();
			return;
		}
		this_weak_ptr->DoSetLocalDescription(std::move(desc),
			rtc::scoped_refptr<SetLocalDescriptionObserverInterface>(
			new rtc::RefCountedObject<SetSessionDescriptionObserverAdapter>(
			this_weak_ptr, observer)));
		// DoSetRemoteDescription() is implemented as a synchronous operation.
		// The |observer| will already have been informed that it completed, and
		// we can mark this operation as complete without any loose ends.
		operations_chain_callback();
	});
}

RTCError RTCSdpOfferAnswerHandler::ApplyLocalDescription(
	std::unique_ptr<SessionDescriptionInterface> desc) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	RTC_DCHECK(desc);

	const SessionDescriptionInterface* old_local_description =
		local_description();
	std::unique_ptr<SessionDescriptionInterface> replaced_local_description;
	SdpType type = desc->GetType();
	if (type == SdpType::kAnswer) {
		replaced_local_description = pending_local_description_
			? std::move(pending_local_description_)
			: std::move(current_local_description_);
		current_local_description_ = std::move(desc);
		pending_local_description_ = nullptr;
		current_remote_description_ = std::move(pending_remote_description_);
	}
	else {
		replaced_local_description = std::move(pending_local_description_);
		pending_local_description_ = std::move(desc);
	}

	RTC_DCHECK(local_description());

// 	ReportSimulcastApiVersion(kSimulcastVersionApplyLocalDescription,
// 		*local_description()->description());

	if (!is_caller_) {
		if (remote_description()) {
			// Remote description was applied first, so this PC is the callee.
			is_caller_ = false;
		}
		else {
			// Local description is applied first, so this PC is the caller.
			is_caller_ = true;
		}
	}

	RTCError error = PushdownTransportDescription(cricket::CS_LOCAL, type);
	if (!error.ok()) {
		return error;
	}


	if (IsUnifiedPlan()) {
		RTCError error = UpdateTransceiversAndDataChannels(
			cricket::CS_LOCAL, *local_description(), old_local_description,
			remote_description());
		if (!error.ok()) {
			return error;
		}

		std::vector<rtc::scoped_refptr<RTCRtpTransceiverInterface>> remove_list;
		std::vector<rtc::scoped_refptr<MediaStreamInterface>> removed_streams;
		for (const auto& transceiver : transceivers()->List()) {
			if (transceiver->stopped()) {
				continue;
			}

			// 2.2.7.1.1.(6-9): Set sender and receiver's transport slots.
			// Note that code paths that don't set MID won't be able to use
			// information about DTLS transports.
			if (transceiver->mid()) {
				auto dtls_transport = transport_controller()->LookupDtlsTransportByMid(
					*transceiver->mid());
				transceiver->internal()->sender_internal()->set_transport(
					dtls_transport);
				transceiver->internal()->receiver_internal()->set_transport(
					dtls_transport);
			}

			const ContentInfo* content =
				FindMediaSectionForTransceiver(transceiver, local_description());
			if (!content) {
				continue;
			}
			const MediaContentDescription* media_desc = content->media_description();
			// 2.2.7.1.6: If description is of type "answer" or "pranswer", then run
			// the following steps:
			if (type == SdpType::kPrAnswer || type == SdpType::kAnswer) {
				// 2.2.7.1.6.1: If direction is "sendonly" or "inactive", and
				// transceiver's [[FiredDirection]] slot is either "sendrecv" or
				// "recvonly", process the removal of a remote track for the media
				// description, given transceiver, removeList, and muteTracks.
				if (!RtpTransceiverDirectionHasRecv(media_desc->direction()) &&
					(transceiver->internal()->fired_direction() &&
					RtpTransceiverDirectionHasRecv(
					*transceiver->internal()->fired_direction()))) {
					ProcessRemovalOfRemoteTrack(transceiver, &remove_list,
						&removed_streams);
				}
				// 2.2.7.1.6.2: Set transceiver's [[CurrentDirection]] and
				// [[FiredDirection]] slots to direction.
				transceiver->internal()->set_current_direction(media_desc->direction());
				transceiver->internal()->set_fired_direction(media_desc->direction());
			}
		}
		auto observer = _rtc_connection->Observer();
		for (const auto& transceiver : remove_list) {
			observer->OnRemoveTrack(transceiver->receiver());
		}
		for (const auto& stream : removed_streams) {
			observer->OnRemoveStream(stream);
		}
	}
	else {
		if (type == SdpType::kOffer) {
			// TODO(bugs.webrtc.org/4676) - Handle CreateChannel failure, as new local
			// description is applied. Restore back to old description.
			RTCError error = CreateChannels(*local_description()->description());
			if (!error.ok()) {
				return error;
			}
		}

		// Remove unused channels if MediaContentDescription is rejected.
		RemoveUnusedChannels(local_description()->description());
	}

	error = UpdateSessionState(type, cricket::CS_LOCAL,
		local_description()->description());

	if (!error.ok()) {
		return error;
	}

	if (remote_description()) {
		// Now that we have a local description, we can push down remote candidates.
		UseCandidatesInSessionDescription(remote_description());
	}


	pending_ice_restarts_.clear();
	if (session_error() != SessionError::kNone) {
		LOG_AND_RETURN_ERROR(RTCErrorType::INTERNAL_ERROR, GetSessionErrorMsg());
	}

	rtc::SSLRole role;
	if (IsSctpLike(_rtc_connection->data_channel_type()) && _rtc_connection->GetSctpSslRole(&role)) {
		data_channel_controller()->AllocateSctpSids(role);
	}


	if (IsUnifiedPlan()) {
		for (const auto& transceiver : transceivers()->List()) {
			if (transceiver->stopped()) {
				continue;
			}
			const ContentInfo* content =
				FindMediaSectionForTransceiver(transceiver, local_description());
			if (!content) {
				continue;
			}
			cricket::ChannelInterface* channel = transceiver->internal()->channel();
			if (content->rejected || !channel || channel->local_streams().empty()) {
				// 0 is a special value meaning "this sender has no associated send
				// stream". Need to call this so the sender won't attempt to configure
				// a no longer existing stream and run into DCHECKs in the lower
				// layers.

				transceiver->internal()->sender_internal()->SetSsrc(0);
			}
			else {
				// Get the StreamParams from the channel which could generate SSRCs.
				const std::vector<StreamParams>& streams = channel->local_streams();
				transceiver->internal()->sender_internal()->set_stream_ids(
					streams[0].stream_ids());
				transceiver->internal()->sender_internal()->SetSsrc(
					streams[0].first_ssrc());
			}
		}
	}
	else {
		// Plan B semantics.
		const cricket::ContentInfo* audio_content =
			GetFirstAudioContent(local_description()->description());
		if (audio_content) {
			if (audio_content->rejected) {
				RemoveSenders(cricket::MEDIA_TYPE_AUDIO);
			}
			else {
				const cricket::AudioContentDescription* audio_desc =
					audio_content->media_description()->as_audio();
				UpdateLocalSenders(audio_desc->streams(), audio_desc->type());
			}
		}

		const cricket::ContentInfo* video_content =
			GetFirstVideoContent(local_description()->description());
		if (video_content) {
			if (video_content->rejected) {
				RemoveSenders(cricket::MEDIA_TYPE_VIDEO);
			}
			else {
				const cricket::VideoContentDescription* video_desc =
					video_content->media_description()->as_video();
				UpdateLocalSenders(video_desc->streams(), video_desc->type());
			}
		}
	}

	const cricket::ContentInfo* data_content =
		GetFirstDataContent(local_description()->description());
	if (data_content) {
		const cricket::RtpDataContentDescription* rtp_data_desc =
			data_content->media_description()->as_rtp_data();
		// rtp_data_desc will be null if this is an SCTP description.
		if (rtp_data_desc) {
			data_channel_controller()->UpdateLocalRtpDataChannels(
				rtp_data_desc->streams());
		}
	}

	if (type == SdpType::kAnswer &&
		local_ice_credentials_to_replace_->SatisfiesIceRestart(
		*current_local_description_)) {
		local_ice_credentials_to_replace_->ClearIceCredentials();
	}

	return RTCError::OK();
}

void RTCSdpOfferAnswerHandler::SetRemoteDescription(SetSessionDescriptionObserver* observer,
	SessionDescriptionInterface* desc_ptr) {
	RTC_DCHECK_RUN_ON(signaling_thread());

	operations_chain_->ChainOperation(
		[this_weak_ptr = weak_ptr_factory_.GetWeakPtr(),
		observer_refptr =
		rtc::scoped_refptr<SetSessionDescriptionObserver>(observer),
		desc = std::unique_ptr<SessionDescriptionInterface>(desc_ptr)](
		std::function<void()> operations_chain_callback) mutable {
		// Abort early if |this_weak_ptr| is no longer valid.
		if (!this_weak_ptr) {
			// For consistency with SetSessionDescriptionObserverAdapter whose
			// posted messages doesn't get processed when the PC is destroyed, we
			// do not inform |observer_refptr| that the operation failed.
			operations_chain_callback();
			return;
		}
		// SetSessionDescriptionObserverAdapter takes care of making sure the
		// |observer_refptr| is invoked in a posted message.
		RTC_LOG(LS_VERBOSE) << "RTCSdpOfferAnswerHandler::SetRemoteDescription ptr DoSetRemoteDescription";
		this_weak_ptr->DoSetRemoteDescription(
			std::move(desc),
			rtc::scoped_refptr<SetRemoteDescriptionObserverInterface>(
			new rtc::RefCountedObject<SetSessionDescriptionObserverAdapter>(
			this_weak_ptr, observer_refptr)));
		// For backwards-compatability reasons, we declare the operation as
		// completed here (rather than in a post), so that the operation chain
		// is not blocked by this operation when the observer is invoked. This
		// allows the observer to trigger subsequent offer/answer operations
		// synchronously if the operation chain is now empty.
		operations_chain_callback();
	});
}

void RTCSdpOfferAnswerHandler::SetRemoteDescription(
	rtc::scoped_refptr<SetSessionDescriptionObserver> observer,
	std::unique_ptr<SessionDescriptionInterface> desc) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	// Chain this operation. If asynchronous operations are pending on the chain,
	// this operation will be queued to be invoked, otherwise the contents of the
	// lambda will execute immediately.
	operations_chain_->ChainOperation(
		[this_weak_ptr = weak_ptr_factory_.GetWeakPtr(),
		observer,
		desc = std::move(desc)](
		std::function<void()> operations_chain_callback) mutable {
		// Abort early if |this_weak_ptr| is no longer valid.
		if (!this_weak_ptr) {
			operations_chain_callback();
			return;
		}
		this_weak_ptr->DoSetRemoteDescription(std::move(desc),
			rtc::scoped_refptr<SetRemoteDescriptionObserverInterface>(
			new rtc::RefCountedObject<SetSessionDescriptionObserverAdapter>(
			this_weak_ptr, observer)));
		// DoSetRemoteDescription() is implemented as a synchronous operation.
		// The |observer| will already have been informed that it completed, and
		// we can mark this operation as complete without any loose ends.
		operations_chain_callback();
	});
}

RTCError RTCSdpOfferAnswerHandler::ApplyRemoteDescription(
	std::unique_ptr<SessionDescriptionInterface> desc) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	RTC_DCHECK(desc);

	const SessionDescriptionInterface* old_remote_description =
		remote_description();
	std::unique_ptr<SessionDescriptionInterface> replaced_remote_description;
	SdpType type = desc->GetType();
	if (type == SdpType::kAnswer) {
		replaced_remote_description = pending_remote_description_
			? std::move(pending_remote_description_)
			: std::move(current_remote_description_);
		current_remote_description_ = std::move(desc);
		pending_remote_description_ = nullptr;
		current_local_description_ = std::move(pending_local_description_);
	}
	else {
		replaced_remote_description = std::move(pending_remote_description_);
		pending_remote_description_ = std::move(desc);
	}

	RTC_DCHECK(remote_description());

	// Report statistics about any use of simulcast.
	ReportSimulcastApiVersion(kSimulcastVersionApplyRemoteDescription,
		*remote_description()->description());

	RTCError error = PushdownTransportDescription(cricket::CS_REMOTE, type);
	if (!error.ok()) {
		return error;
	}

	if (IsUnifiedPlan()) {
		RTCError error = UpdateTransceiversAndDataChannels(
			cricket::CS_REMOTE, *remote_description(), local_description(),
			old_remote_description);
		if (!error.ok()) {
			return error;
		}
	} else {
		if (type == SdpType::kOffer) {
			RTCError error = CreateChannels(*remote_description()->description());
			if (!error.ok()) {
				return error;
			}
		}
		RemoveUnusedChannels(remote_description()->description());
	}

	error = UpdateSessionState(type, cricket::CS_REMOTE,
								remote_description()->description());
	if (!error.ok()) {
		return error;
	}

	if (local_description() &&
		!UseCandidatesInSessionDescription(remote_description())) {
		LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_PARAMETER, kInvalidCandidates);
	}

	if (old_remote_description) {
		for (const cricket::ContentInfo& content :
			old_remote_description->description()->contents()) {
			// Check if this new SessionDescription contains new ICE ufrag and
			// password that indicates the remote peer requests an ICE restart.
			// TODO(deadbeef): When we start storing both the current and pending
			// remote description, this should reset pending_ice_restarts and compare
			// against the current description.
			if (CheckForRemoteIceRestart(old_remote_description, remote_description(),
				content.name)) {
				if (type == SdpType::kOffer) {
					pending_ice_restarts_.insert(content.name);
				}
			}
			else {
				// We retain all received candidates only if ICE is not restarted.
				// When ICE is restarted, all previous candidates belong to an old
				// generation and should not be kept.
				// TODO(deadbeef): This goes against the W3C spec which says the remote
				// description should only contain candidates from the last set remote
				// description plus any candidates added since then. We should remove
				// this once we're sure it won't break anything.
				WebRtcSessionDescriptionFactory::CopyCandidatesFromSessionDescription(
					old_remote_description, content.name, mutable_remote_description());
			}
		}
	}

	if (session_error() != SessionError::kNone) {
		LOG_AND_RETURN_ERROR(RTCErrorType::INTERNAL_ERROR, GetSessionErrorMsg());
	}

	if (remote_description()->GetType() != SdpType::kOffer &&
		remote_description()->number_of_mediasections() > 0u &&
		_rtc_connection->ice_connection_state() ==
		PeerConnectionInterfaceDefs::kIceConnectionNew) {
		_rtc_connection->SetIceConnectionState(PeerConnectionInterfaceDefs::kIceConnectionChecking);
	}

	rtc::SSLRole role;
	if (IsSctpLike(_rtc_connection->data_channel_type()) && _rtc_connection->GetSctpSslRole(&role)) {
		data_channel_controller()->AllocateSctpSids(role);
	}

	if (IsUnifiedPlan()) {
		std::vector<rtc::scoped_refptr<RTCRtpTransceiverInterface>>
			now_receiving_transceivers;
		std::vector<rtc::scoped_refptr<RTCRtpTransceiverInterface>> remove_list;
		std::vector<rtc::scoped_refptr<MediaStreamInterface>> added_streams;
		std::vector<rtc::scoped_refptr<MediaStreamInterface>> removed_streams;

		for (const auto& transceiver : transceivers()->List()) {
			const ContentInfo* content =
				FindMediaSectionForTransceiver(transceiver, remote_description());
			if (!content) {
				continue;
			}
			const MediaContentDescription* media_desc = content->media_description();
			RtpTransceiverDirection local_direction =
				RtpTransceiverDirectionReversed(media_desc->direction());

			if (RtpTransceiverDirectionHasRecv(local_direction)) {
				std::vector<std::string> stream_ids;
				if (!media_desc->streams().empty()) {
					// The remote description has signaled the stream IDs.
					stream_ids = media_desc->streams()[0].stream_ids();
				}
				transceivers()
					->StableState(transceiver)
					->SetRemoteStreamIdsIfUnset(transceiver->receiver()->stream_ids());

				RTC_LOG(LS_INFO) << "Processing the MSIDs for MID=" << content->name
					<< " (" << GetStreamIdsString(stream_ids) << ").";
				SetAssociatedRemoteStreams(transceiver->internal()->receiver_internal(),
					stream_ids, &added_streams,
					&removed_streams);

				if (!transceiver->fired_direction() ||
					!RtpTransceiverDirectionHasRecv(*transceiver->fired_direction())) {
					RTC_LOG(LS_INFO)
						<< "Processing the addition of a remote track for MID="
						<< content->name << ".";
					now_receiving_transceivers.push_back(transceiver);
				}
			}

			if (!RtpTransceiverDirectionHasRecv(local_direction) &&
				(transceiver->fired_direction() &&
				  RtpTransceiverDirectionHasRecv(*transceiver->fired_direction()))) {
				ProcessRemovalOfRemoteTrack(transceiver, &remove_list,
					&removed_streams);
			}

			transceiver->internal()->set_fired_direction(local_direction);

			if (type == SdpType::kPrAnswer || type == SdpType::kAnswer) {
				transceiver->internal()->set_current_direction(local_direction);
				if (transceiver->mid()) {
					auto dtls_transport =
						transport_controller()->LookupDtlsTransportByMid(
						*transceiver->mid());
					transceiver->internal()->sender_internal()->set_transport(
						dtls_transport);

					if (media_desc->type() == cricket::MediaType::MEDIA_TYPE_VIDEO) {
						std::vector<rtc::scoped_refptr<RTCRtpReceiverProxyWithInternal<RTCRtpReceiverInternal>>> receivers =transceiver->internal()->receivers();
						for (rtc::scoped_refptr<RTCRtpReceiverProxyWithInternal<RTCRtpReceiverInternal>> receiver : receivers) {
							receiver->internal()->set_transport(dtls_transport);
						}
					}
					else {
						transceiver->internal()->receiver_internal()->set_transport(
							dtls_transport);
					}
				}
			}
			if (content->rejected && !transceiver->stopped()) {
				RTC_LOG(LS_INFO) << "Stopping transceiver for MID=" << content->name
					<< " since the media section was rejected.";
				transceiver->internal()->StopTransceiverProcedure();
			}
			if (!content->rejected &&
				RtpTransceiverDirectionHasRecv(local_direction)) {
				if (!media_desc->streams().empty() &&
					media_desc->streams()[0].has_ssrcs()) {

					if (media_desc->type() == cricket::MediaType::MEDIA_TYPE_VIDEO) {	
						std::vector<uint32_t> primarySsrcs;
						media_desc->streams()[0].GetPrimarySsrcs(&primarySsrcs);
						uint8_t i = 0;
						std::vector<rtc::scoped_refptr<RTCRtpReceiverProxyWithInternal<RTCRtpReceiverInternal>>> receivers = transceiver->internal()->receivers();
						for (rtc::scoped_refptr<RTCRtpReceiverProxyWithInternal<RTCRtpReceiverInternal>> receiver : receivers) {
							receiver->internal()->SetupMediaChannel(primarySsrcs[i]);
							i++;
						}
					}
					else {
						uint32_t ssrc = media_desc->streams()[0].first_ssrc();
						transceiver->internal()->receiver_internal()->SetupMediaChannel(ssrc);
					}
				}
				else {
					transceiver->internal()
						->receiver_internal()
						->SetupUnsignaledMediaChannel();
				}
			}
		}
		auto observer = _rtc_connection->Observer();
		for (const auto& transceiver : now_receiving_transceivers) {
			if (IsUnifiedPlan()) {
				observer->OnTrack(transceiver);
			}
			else {
				observer->OnAddTrack(transceiver->receiver(),
					transceiver->receiver()->streams());
			}
		}
		for (const auto& stream : added_streams) {
			observer->OnAddStream(stream);
		}
		for (const auto& transceiver : remove_list) {
			observer->OnRemoveTrack(transceiver->receiver());
		}
		for (const auto& stream : removed_streams) {
			observer->OnRemoveStream(stream);
		}
	}

	const cricket::ContentInfo* audio_content =
		GetFirstAudioContent(remote_description()->description());
	const cricket::ContentInfo* video_content =
		GetFirstVideoContent(remote_description()->description());
	const cricket::AudioContentDescription* audio_desc =
		GetFirstAudioContentDescription(remote_description()->description());
	const cricket::VideoContentDescription* video_desc =
		GetFirstVideoContentDescription(remote_description()->description());
	const cricket::RtpDataContentDescription* rtp_data_desc =
		GetFirstRtpDataContentDescription(remote_description()->description());

	if (remote_description()->description()->msid_supported() ||
		(audio_desc && !audio_desc->streams().empty()) ||
		(video_desc && !video_desc->streams().empty())) {
		remote_peer_supports_msid_ = true;
	}

	rtc::scoped_refptr<StreamCollection> new_streams(StreamCollection::Create());

	if (!IsUnifiedPlan()) {
		if (audio_content) {
			if (audio_content->rejected) {
				RemoveSenders(cricket::MEDIA_TYPE_AUDIO);
			}
			else {
				bool default_audio_track_needed =
					!remote_peer_supports_msid_ &&
					RtpTransceiverDirectionHasSend(audio_desc->direction());
				UpdateRemoteSendersList(GetActiveStreams(audio_desc),
					default_audio_track_needed, audio_desc->type(),
					new_streams);
			}
		}

		if (video_content) {
			if (video_content->rejected) {
				RemoveSenders(cricket::MEDIA_TYPE_VIDEO);
			}
			else {
				bool default_video_track_needed =
					!remote_peer_supports_msid_ &&
					RtpTransceiverDirectionHasSend(video_desc->direction());
				UpdateRemoteSendersList(GetActiveStreams(video_desc),
					default_video_track_needed, video_desc->type(),
					new_streams);
			}
		}

		if (rtp_data_desc) {
			data_channel_controller()->UpdateRemoteRtpDataChannels(
				GetActiveStreams(rtp_data_desc));
		}

		auto observer = _rtc_connection->Observer();
		for (size_t i = 0; i < new_streams->count(); ++i) {
			MediaStreamInterface* new_stream = new_streams->at(i);
			observer->OnAddStream(
				rtc::scoped_refptr<MediaStreamInterface>(new_stream));
		}

		UpdateEndedRemoteMediaStreams();
	}

	if (type == SdpType::kAnswer &&
		local_ice_credentials_to_replace_->SatisfiesIceRestart(
			*current_local_description_)) {
		local_ice_credentials_to_replace_->ClearIceCredentials();
	}

	return RTCError::OK();
}

void RTCSdpOfferAnswerHandler::DoSetLocalDescription(
	std::unique_ptr<SessionDescriptionInterface> desc,
	rtc::scoped_refptr<SetLocalDescriptionObserverInterface> observer) {
	RTC_DCHECK_RUN_ON(signaling_thread());

	if (!observer) {
		RTC_LOG(LS_ERROR) << "SetLocalDescription - observer is NULL.";
		return;
	}

	if (!desc) {
		observer->OnSetLocalDescriptionComplete(
			RTCError(RTCErrorType::INTERNAL_ERROR, "SessionDescription is NULL."));
		return;
	}

	if (session_error() != SessionError::kNone) {
		std::string error_message = GetSessionErrorMsg();
		RTC_LOG(LS_ERROR) << "SetLocalDescription: " << error_message;
		observer->OnSetLocalDescriptionComplete(
			RTCError(RTCErrorType::INTERNAL_ERROR, std::move(error_message)));
		return;
	}

	if (desc->GetType() == SdpType::kRollback) {
		if (IsUnifiedPlan()) {
			observer->OnSetLocalDescriptionComplete(Rollback(desc->GetType()));
		}
		else {
			observer->OnSetLocalDescriptionComplete(
				RTCError(RTCErrorType::UNSUPPORTED_OPERATION,
				"Rollback not supported in Plan B"));
		}
		return;
	}

	RTCError error = ValidateSessionDescription(desc.get(), cricket::CS_LOCAL);
	if (!error.ok()) {
		std::string error_message = GetSetDescriptionErrorMessage(
			cricket::CS_LOCAL, desc->GetType(), error);
		RTC_LOG(LS_ERROR) << error_message;
		observer->OnSetLocalDescriptionComplete(
			RTCError(RTCErrorType::INTERNAL_ERROR, std::move(error_message)));
		return;
	}

	const SdpType type = desc->GetType();
	error = ApplyLocalDescription(std::move(desc));
	if (!error.ok()) {
		SetSessionError(SessionError::kContent, error.message());
		std::string error_message =
			GetSetDescriptionErrorMessage(cricket::CS_LOCAL, type, error);
		RTC_LOG(LS_ERROR) << error_message;
		observer->OnSetLocalDescriptionComplete(
			RTCError(RTCErrorType::INTERNAL_ERROR, std::move(error_message)));
		return;
	}
	RTC_DCHECK(local_description());

	if (local_description()->GetType() == SdpType::kAnswer) {
		RemoveStoppedTransceivers();

		_rtc_connection->network_thread()->Invoke<void>(
			RTC_FROM_HERE, [this] { port_allocator()->DiscardCandidatePool(); });
	}

	observer->OnSetLocalDescriptionComplete(RTCError::OK());

	if (IsUnifiedPlan()) {
		bool was_negotiation_needed = is_negotiation_needed_;
		UpdateNegotiationNeeded();
		if (signaling_state() == PeerConnectionInterfaceDefs::kStable &&
			was_negotiation_needed && is_negotiation_needed_) {
			// Legacy version.
			_rtc_connection->Observer()->OnRenegotiationNeeded();
			// Spec-compliant version; the event may get invalidated before firing.
			GenerateNegotiationNeededEvent();
		}
	}

	transport_controller()->MaybeStartGathering();
}

void RTCSdpOfferAnswerHandler::DoCreateOffer(
	const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions& options,
	rtc::scoped_refptr<CreateSessionDescriptionObserver> observer) {
	RTC_DCHECK_RUN_ON(signaling_thread());

	if (!observer) {
		RTC_LOG(LS_ERROR) << "CreateOffer - observer is NULL.";
		return;
	}

	if (_rtc_connection->IsClosed()) {
		std::string error = "CreateOffer called when PeerConnection is closed.";
		RTC_LOG(LS_ERROR) << error;
		_rtc_connection->message_handler()->PostCreateSessionDescriptionFailure(
			observer, RTCError(RTCErrorType::INVALID_STATE, std::move(error)));
		return;
	}

	if (session_error() != SessionError::kNone) {
		std::string error_message = GetSessionErrorMsg();
		RTC_LOG(LS_ERROR) << "CreateOffer: " << error_message;
		_rtc_connection->message_handler()->PostCreateSessionDescriptionFailure(
			observer,
			RTCError(RTCErrorType::INTERNAL_ERROR, std::move(error_message)));
		return;
	}

	if (!ValidateOfferAnswerOptions(options)) {
		std::string error = "CreateOffer called with invalid options.";
		RTC_LOG(LS_ERROR) << error;
		_rtc_connection->message_handler()->PostCreateSessionDescriptionFailure(
			observer, RTCError(RTCErrorType::INVALID_PARAMETER, std::move(error)));
		return;
	}

	if (IsUnifiedPlan()) {
		RTCError error = HandleLegacyOfferOptions(options);
		if (!error.ok()) {
			_rtc_connection->message_handler()->PostCreateSessionDescriptionFailure(
				observer, std::move(error));
			return;
		}
	}

	cricket::MediaSessionOptions session_options;
	GetOptionsForOffer(options, &session_options);
	webrtc_session_desc_factory_->CreateOffer(observer, options, session_options);
}

void RTCSdpOfferAnswerHandler::CreateAnswer(
	CreateSessionDescriptionObserver* observer,
	const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions& options) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	// Chain this operation. If asynchronous operations are pending on the chain,
	// this operation will be queued to be invoked, otherwise the contents of the
	// lambda will execute immediately.
	operations_chain_->ChainOperation(
		[this_weak_ptr = weak_ptr_factory_.GetWeakPtr(),
		observer_refptr =
		rtc::scoped_refptr<CreateSessionDescriptionObserver>(observer),
		options](std::function<void()> operations_chain_callback) {
		// Abort early if |this_weak_ptr| is no longer valid.
		if (!this_weak_ptr) {
			observer_refptr->OnFailure(RTCError(
				RTCErrorType::INTERNAL_ERROR,
				"CreateAnswer failed because the session was shut down"));
			operations_chain_callback();
			return;
		}
		// The operation completes asynchronously when the wrapper is invoked.
		rtc::scoped_refptr<CreateSessionDescriptionObserverOperationWrapper>
			observer_wrapper(new rtc::RefCountedObject<
			CreateSessionDescriptionObserverOperationWrapper>(
			std::move(observer_refptr),
			std::move(operations_chain_callback)));
		this_weak_ptr->DoCreateAnswer(options, observer_wrapper);
	});
}

void RTCSdpOfferAnswerHandler::DoCreateAnswer(
	const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions& options,
	rtc::scoped_refptr<CreateSessionDescriptionObserver> observer) {
	RTC_DCHECK_RUN_ON(signaling_thread());

	if (!observer) {
		RTC_LOG(LS_ERROR) << "CreateAnswer - observer is NULL.";
		return;
	}

	if (session_error() != SessionError::kNone) {
		std::string error_message = GetSessionErrorMsg();
		RTC_LOG(LS_ERROR) << "CreateAnswer: " << error_message;
		_rtc_connection->message_handler()->PostCreateSessionDescriptionFailure(
			observer,
			RTCError(RTCErrorType::INTERNAL_ERROR, std::move(error_message)));
		return;
	}

	if (!(signaling_state_ == PeerConnectionInterfaceDefs::kHaveRemoteOffer ||
		signaling_state_ == PeerConnectionInterfaceDefs::kHaveLocalPrAnswer)) {
		std::string error =
			"PeerConnection cannot create an answer in a state other than "
			"have-remote-offer or have-local-pranswer.";
		RTC_LOG(LS_ERROR) << error;
		_rtc_connection->message_handler()->PostCreateSessionDescriptionFailure(
			observer, RTCError(RTCErrorType::INVALID_STATE, std::move(error)));
		return;
	}

	RTC_DCHECK(remote_description());

	if (IsUnifiedPlan()) {
		if (options.offer_to_receive_audio !=
			PeerConnectionInterfaceDefs::RTCOfferAnswerOptions::kUndefined) {
			RTC_LOG(LS_WARNING) << "CreateAnswer: offer_to_receive_audio is not "
				"supported with Unified Plan semantics. Use the "
				"RtpTransceiver API instead.";
		}
		if (options.offer_to_receive_video !=
			PeerConnectionInterfaceDefs::RTCOfferAnswerOptions::kUndefined) {
			RTC_LOG(LS_WARNING) << "CreateAnswer: offer_to_receive_video is not "
				"supported with Unified Plan semantics. Use the "
				"RtpTransceiver API instead.";
		}
	}

	cricket::MediaSessionOptions session_options;
	GetOptionsForAnswer(options, &session_options);
	webrtc_session_desc_factory_->CreateAnswer(observer, session_options);
}

void RTCSdpOfferAnswerHandler::DoSetRemoteDescription(
	std::unique_ptr<SessionDescriptionInterface> desc,
	rtc::scoped_refptr<SetRemoteDescriptionObserverInterface> observer) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	if (!observer) {
		RTC_LOG(LS_ERROR) << "SetRemoteDescription - observer is NULL.";
		return;
	}

	if (!desc) {
		observer->OnSetRemoteDescriptionComplete(RTCError(
			RTCErrorType::INVALID_PARAMETER, "SessionDescription is NULL."));
		return;
	}

	if (session_error() != SessionError::kNone) {
		std::string error_message = GetSessionErrorMsg();
		RTC_LOG(LS_ERROR) << "SetRemoteDescription: " << error_message;
		observer->OnSetRemoteDescriptionComplete(
			RTCError(RTCErrorType::INTERNAL_ERROR, std::move(error_message)));
		return;
	}

	if (IsUnifiedPlan()) {
		if (_rtc_connection->configuration()->enable_implicit_rollback) {
			if (desc->GetType() == SdpType::kOffer &&
				signaling_state() == PeerConnectionInterfaceDefs::kHaveLocalOffer) {
				Rollback(desc->GetType());
			}
		}
		// Explicit rollback.
		if (desc->GetType() == SdpType::kRollback) {
			observer->OnSetRemoteDescriptionComplete(Rollback(desc->GetType()));
			return;
		}
	}
	else if (desc->GetType() == SdpType::kRollback) {
		observer->OnSetRemoteDescriptionComplete(
			RTCError(RTCErrorType::UNSUPPORTED_OPERATION,
			"Rollback not supported in Plan B"));
		return;
	}

	FillInMissingRemoteMids(desc->description());

	RTCError error = ValidateSessionDescription(desc.get(), cricket::CS_REMOTE);
	if (!error.ok()) {
		std::string error_message = GetSetDescriptionErrorMessage(
			cricket::CS_REMOTE, desc->GetType(), error);
		RTC_LOG(LS_ERROR) << error_message;
		observer->OnSetRemoteDescriptionComplete(
			RTCError(error.type(), std::move(error_message)));
		return;
	}

	const SdpType type = desc->GetType();

	error = ApplyRemoteDescription(std::move(desc));
	if (!error.ok()) {
		// If ApplyRemoteDescription fails, the PeerConnection could be in an
		// inconsistent state, so act conservatively here and set the session error
		// so that future calls to SetLocalDescription/SetRemoteDescription fail.
		SetSessionError(SessionError::kContent, error.message());
		std::string error_message =
			GetSetDescriptionErrorMessage(cricket::CS_REMOTE, type, error);
		RTC_LOG(LS_ERROR) << error_message;
		observer->OnSetRemoteDescriptionComplete(
			RTCError(error.type(), std::move(error_message)));
		return;
	}
	RTC_DCHECK(remote_description());

	if (type == SdpType::kAnswer) {
		RemoveStoppedTransceivers();
		// TODO(deadbeef): We already had to hop to the network thread for
		// MaybeStartGathering...
		_rtc_connection->network_thread()->Invoke<void>(
			RTC_FROM_HERE, rtc::Bind(&cricket::PortAllocator::DiscardCandidatePool,
			port_allocator()));
	}

	observer->OnSetRemoteDescriptionComplete(RTCError::OK());

	if (IsUnifiedPlan()) {
		bool was_negotiation_needed = is_negotiation_needed_;
		UpdateNegotiationNeeded();
		if (signaling_state() == PeerConnectionInterfaceDefs::kStable &&
			was_negotiation_needed && is_negotiation_needed_) {
			// Legacy version.
			_rtc_connection->Observer()->OnRenegotiationNeeded();
			// Spec-compliant version; the event may get invalidated before firing.
			GenerateNegotiationNeededEvent();
		}
	}
}

void RTCSdpOfferAnswerHandler::SetAssociatedRemoteStreams(
	rtc::scoped_refptr<RTCRtpReceiverInternal> receiver,
	const std::vector<std::string>& stream_ids,
	std::vector<rtc::scoped_refptr<MediaStreamInterface>>* added_streams,
	std::vector<rtc::scoped_refptr<MediaStreamInterface>>* removed_streams) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	std::vector<rtc::scoped_refptr<MediaStreamInterface>> media_streams;

	for (const std::string& stream_id : stream_ids) {
		rtc::scoped_refptr<MediaStreamInterface> stream =
			remote_streams_->find(stream_id);
		if (!stream) {
			stream = MediaStreamProxy::Create(
				rtc::Thread::Current(_rtc_thread_manager),
				MediaStream::Create(stream_id));
			remote_streams_->AddStream(stream);
			added_streams->push_back(stream);
		}
		media_streams.push_back(stream);
	}

	if (media_streams.empty() &&
		!(remote_description()->description()->msid_signaling() &
		cricket::kMsidSignalingMediaSection)) {
		if (!missing_msid_default_stream_) {
			missing_msid_default_stream_ = MediaStreamProxy::Create(
				rtc::Thread::Current(_rtc_thread_manager),
				MediaStream::Create(rtc::CreateRandomUuid()));
			added_streams->push_back(missing_msid_default_stream_);
		}
		media_streams.push_back(missing_msid_default_stream_);
	}

	std::vector<rtc::scoped_refptr<MediaStreamInterface>> previous_streams =
		receiver->streams();
	receiver->SetStreams(media_streams);
	RemoveRemoteStreamsIfEmpty(previous_streams, removed_streams);
}

bool RTCSdpOfferAnswerHandler::AddIceCandidate(
	const IceCandidateInterface* ice_candidate) {
	RTC_DCHECK_RUN_ON(signaling_thread());

	if (_rtc_connection->IsClosed()) {
		RTC_LOG(LS_ERROR) << "AddIceCandidate: RTCConnection is closed.";
		return false;
	}

	if (!remote_description()) {
		RTC_LOG(LS_ERROR) << "AddIceCandidate: ICE candidates can't be added "
			"without any remote session description.";
		return false;
	}

	if (!ice_candidate) {
		RTC_LOG(LS_ERROR) << "AddIceCandidate: Candidate is null.";
		return false;
	}

	bool valid = false;
	bool ready = ReadyToUseRemoteCandidate(ice_candidate, nullptr, &valid);
	if (!valid) {
		return false;
	}

	if (!mutable_remote_description()->AddCandidate(ice_candidate)) {
		RTC_LOG(LS_ERROR) << "AddIceCandidate: Candidate cannot be used.";
		return false;
	}

	if (ready) {
		return  UseCandidate(ice_candidate);
	}
	else {
		RTC_LOG(LS_INFO) << "AddIceCandidate: Not ready to use candidate.";
		return true;
	}
}

void RTCSdpOfferAnswerHandler::AddIceCandidate(
	std::unique_ptr<IceCandidateInterface> candidate,
	std::function<void(RTCError)> callback) {
	RTC_DCHECK_RUN_ON(signaling_thread());

	operations_chain_->ChainOperation(
		[this_weak_ptr = weak_ptr_factory_.GetWeakPtr(),
		candidate = std::move(candidate), callback = std::move(callback)](
		std::function<void()> operations_chain_callback) {
		if (!this_weak_ptr) {
			operations_chain_callback();
			callback(RTCError(
				RTCErrorType::INVALID_STATE,
				"AddIceCandidate failed because the session was shut down"));
			return;
		}
		if (!this_weak_ptr->AddIceCandidate(candidate.get())) {
			operations_chain_callback();
			// Fail with an error type and message consistent with Chromium.
			// TODO(hbos): Fail with error types according to spec.
			callback(RTCError(RTCErrorType::UNSUPPORTED_OPERATION,
				"Error processing ICE candidate"));
			return;
		}
		operations_chain_callback();
		callback(RTCError::OK());
	});
}

bool RTCSdpOfferAnswerHandler::RemoveIceCandidates(
	const std::vector<cricket::Candidate>& candidates) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	if (_rtc_connection->IsClosed()) {
		RTC_LOG(LS_ERROR) << "RemoveIceCandidates: PeerConnection is closed.";
		return false;
	}

	if (!remote_description()) {
		RTC_LOG(LS_ERROR) << "RemoveIceCandidates: ICE candidates can't be removed "
			"without any remote session description.";
		return false;
	}

	if (candidates.empty()) {
		RTC_LOG(LS_ERROR) << "RemoveIceCandidates: candidates are empty.";
		return false;
	}

	size_t number_removed =
		mutable_remote_description()->RemoveCandidates(candidates);
	if (number_removed != candidates.size()) {
		RTC_LOG(LS_ERROR)
			<< "RemoveIceCandidates: Failed to remove candidates. Requested "
			<< candidates.size() << " but only " << number_removed
			<< " are removed.";
	}

	// Remove the candidates from the transport controller.
	RTCError error = transport_controller()->RemoveRemoteCandidates(candidates);
	if (!error.ok()) {
		RTC_LOG(LS_ERROR)
			<< "RemoveIceCandidates: Error when removing remote candidates: "
			<< error.message();
	}
	return true;
}

void RTCSdpOfferAnswerHandler::AddLocalIceCandidate(
	const JsepIceCandidate* candidate) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	if (local_description()) {
		mutable_local_description()->AddCandidate(candidate);
	}
}

void RTCSdpOfferAnswerHandler::RemoveLocalIceCandidates(
	const std::vector<cricket::Candidate>& candidates) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	if (local_description()) {
		mutable_local_description()->RemoveCandidates(candidates);
	}
}

const SessionDescriptionInterface*
RTCSdpOfferAnswerHandler::local_description() const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	return pending_local_description_ ? pending_local_description_.get()
		: current_local_description_.get();
}

const SessionDescriptionInterface*
RTCSdpOfferAnswerHandler::remote_description() const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	return pending_remote_description_ ? pending_remote_description_.get()
		: current_remote_description_.get();
}

const SessionDescriptionInterface*
RTCSdpOfferAnswerHandler::current_local_description() const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	return current_local_description_.get();
}

const SessionDescriptionInterface*
RTCSdpOfferAnswerHandler::current_remote_description() const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	return current_remote_description_.get();
}

const SessionDescriptionInterface*
RTCSdpOfferAnswerHandler::pending_local_description() const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	return pending_local_description_.get();
}

const SessionDescriptionInterface*
RTCSdpOfferAnswerHandler::pending_remote_description() const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	return pending_remote_description_.get();
}

PeerConnectionInterfaceDefs::SignalingState
RTCSdpOfferAnswerHandler::signaling_state() const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	return signaling_state_;
}

void RTCSdpOfferAnswerHandler::ChangeSignalingState(
	PeerConnectionInterfaceDefs::SignalingState signaling_state) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	if (signaling_state_ == signaling_state) {
		return;
	}

	signaling_state_ = signaling_state;
	_rtc_connection->Observer()->OnSignalingChange(signaling_state_);
}

RTCError RTCSdpOfferAnswerHandler::UpdateSessionState(SdpType type,
	cricket::ContentSource source,
	const cricket::SessionDescription* description) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	RTC_DCHECK(session_error() == SessionError::kNone);

	if (type == SdpType::kPrAnswer || type == SdpType::kAnswer) {
		EnableSending();
	}

	if (type == SdpType::kOffer) {
		ChangeSignalingState(source == cricket::CS_LOCAL
			? PeerConnectionInterfaceDefs::kHaveLocalOffer
			: PeerConnectionInterfaceDefs::kHaveRemoteOffer);
	}
	else if (type == SdpType::kPrAnswer) {
		ChangeSignalingState(source == cricket::CS_LOCAL
			? PeerConnectionInterfaceDefs::kHaveLocalPrAnswer
			: PeerConnectionInterfaceDefs::kHaveRemotePrAnswer);
	}
	else {
		RTC_DCHECK(type == SdpType::kAnswer);
		ChangeSignalingState(PeerConnectionInterfaceDefs::kStable);
		transceivers()->DiscardStableStates();
		have_pending_rtp_data_channel_ = false;
	}

	RTCError error = PushdownMediaDescription(type, source);
	if (!error.ok()) {
		return error;
	}
	return RTCError::OK();
}

bool RTCSdpOfferAnswerHandler::ShouldFireNegotiationNeededEvent(
	uint32_t event_id) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	if (!IsUnifiedPlan()) {
		return true;
	}

	if (event_id != negotiation_needed_event_id_) {
		return false;
	}

	if (!operations_chain_->IsEmpty()) {
		is_negotiation_needed_ = false;
		update_negotiation_needed_on_empty_chain_ = true;
		return false;
	}

	if (signaling_state_ != PeerConnectionInterfaceDefs::kStable) {
		return false;
	}

	return true;
}

RTCError RTCSdpOfferAnswerHandler::Rollback(SdpType desc_type) {
	auto state = signaling_state();
	if (state != PeerConnectionInterfaceDefs::kHaveLocalOffer &&
		state != PeerConnectionInterfaceDefs::kHaveRemoteOffer) {
		return RTCError(RTCErrorType::INVALID_STATE,
			"Called in wrong signalingState: " +
			GetSignalingStateString(signaling_state()));
	}

	RTC_DCHECK_RUN_ON(signaling_thread());
	RTC_DCHECK(IsUnifiedPlan());

	std::vector<rtc::scoped_refptr<MediaStreamInterface>> all_added_streams;
	std::vector<rtc::scoped_refptr<MediaStreamInterface>> all_removed_streams;
	std::vector<rtc::scoped_refptr<RTCRtpReceiverInterface>> removed_receivers;

	for (auto&& transceivers_stable_state_pair : transceivers()->StableStates()) {
		auto transceiver = transceivers_stable_state_pair.first;
		auto state = transceivers_stable_state_pair.second;

		if (state.remote_stream_ids()) {
			std::vector<rtc::scoped_refptr<MediaStreamInterface>> added_streams;
			std::vector<rtc::scoped_refptr<MediaStreamInterface>> removed_streams;
			SetAssociatedRemoteStreams(transceiver->internal()->receiver_internal(),
				state.remote_stream_ids().value(),
				&added_streams, &removed_streams);
			all_added_streams.insert(all_added_streams.end(), added_streams.begin(),
				added_streams.end());
			all_removed_streams.insert(all_removed_streams.end(),
				removed_streams.begin(),
				removed_streams.end());
			if (!state.has_m_section() && !state.newly_created()) {
				continue;
			}
		}

		RTC_DCHECK(transceiver->internal()->mid().has_value());
		DestroyTransceiverChannel(transceiver);

		if (signaling_state() == PeerConnectionInterfaceDefs::kHaveRemoteOffer &&
			transceiver->receiver()) {
			removed_receivers.push_back(transceiver->receiver());
		}
		if (state.newly_created()) {
			if (transceiver->internal()->reused_for_addtrack()) {
				transceiver->internal()->set_created_by_addtrack(true);
			}
			else {
				transceivers()->Remove(transceiver);
			}
		}
		transceiver->internal()->sender_internal()->set_transport(nullptr);
		transceiver->internal()->receiver_internal()->set_transport(nullptr);
		transceiver->internal()->set_mid(state.mid());
		transceiver->internal()->set_mline_index(state.mline_index());
	}

	transport_controller()->RollbackTransports();
	if (have_pending_rtp_data_channel_) {
		DestroyDataChannelTransport();
		have_pending_rtp_data_channel_ = false;
	}
	transceivers()->DiscardStableStates();
	pending_local_description_.reset();
	pending_remote_description_.reset();
	ChangeSignalingState(PeerConnectionInterfaceDefs::kStable);

	for (const auto& receiver : removed_receivers) {
		_rtc_connection->Observer()->OnRemoveTrack(receiver);
	}
	for (const auto& stream : all_added_streams) {
		_rtc_connection->Observer()->OnAddStream(stream);
	}
	for (const auto& stream : all_removed_streams) {
		_rtc_connection->Observer()->OnRemoveStream(stream);
	}

	if (desc_type == SdpType::kRollback) {
		UpdateNegotiationNeeded();
		if (is_negotiation_needed_) {
			// Legacy version.
			_rtc_connection->Observer()->OnRenegotiationNeeded();
			// Spec-compliant version; the event may get invalidated before firing.
			GenerateNegotiationNeededEvent();
		}
	}
	return RTCError::OK();
}

bool RTCSdpOfferAnswerHandler::IsUnifiedPlan() const {
	return _rtc_connection->IsUnifiedPlan();
}

void RTCSdpOfferAnswerHandler::OnOperationsChainEmpty() {
	RTC_DCHECK_RUN_ON(signaling_thread());
	if (_rtc_connection->IsClosed() || !update_negotiation_needed_on_empty_chain_)
		return;
	update_negotiation_needed_on_empty_chain_ = false;

	if (IsUnifiedPlan()) {
		UpdateNegotiationNeeded();
	}
}

absl::optional<bool> RTCSdpOfferAnswerHandler::is_caller() {
	RTC_DCHECK_RUN_ON(signaling_thread());
	return is_caller_;
}

bool RTCSdpOfferAnswerHandler::HasNewIceCredentials() {
	RTC_DCHECK_RUN_ON(signaling_thread());
	return local_ice_credentials_to_replace_->HasIceCredentials();
}

bool RTCSdpOfferAnswerHandler::IceRestartPending(const std::string& content_name) const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	return pending_ice_restarts_.find(content_name) !=
		pending_ice_restarts_.end();
}

bool RTCSdpOfferAnswerHandler::NeedsIceRestart(const std::string& content_name) const {
	return transport_controller()->NeedsIceRestart(content_name);
}

absl::optional<rtc::SSLRole> RTCSdpOfferAnswerHandler::GetDtlsRole(const std::string& mid) const {
	return transport_controller()->GetDtlsRole(mid);
}

void RTCSdpOfferAnswerHandler::UpdateNegotiationNeeded() {
	RTC_DCHECK_RUN_ON(signaling_thread());
	if (!IsUnifiedPlan()) {
		_rtc_connection->Observer()->OnRenegotiationNeeded();
		GenerateNegotiationNeededEvent();
		return;
	}

	if (_rtc_connection->IsClosed()) {
		return;
	}

	if (signaling_state() != PeerConnectionInterfaceDefs::kStable)
		return;

	bool is_negotiation_needed = CheckIfNegotiationIsNeeded();
	if (!is_negotiation_needed) {
		is_negotiation_needed_ = false;
		++negotiation_needed_event_id_;
		return;
	}

	if (is_negotiation_needed_)
		return;

	is_negotiation_needed_ = true;

	_rtc_connection->Observer()->OnRenegotiationNeeded();
	GenerateNegotiationNeededEvent();
}

bool RTCSdpOfferAnswerHandler::CheckIfNegotiationIsNeeded() {
	RTC_DCHECK_RUN_ON(signaling_thread());

	if (local_ice_credentials_to_replace_->HasIceCredentials()) {
		return true;
	}

	const SessionDescriptionInterface* description = current_local_description();
	if (!description)
		return true;

	if (data_channel_controller()->HasSctpDataChannels()) {
		if (!cricket::GetFirstDataContent(description->description()->contents()))
			return true;
	}

	for (const auto& transceiver : transceivers()->List()) {
		const ContentInfo* current_local_msection =
			FindTransceiverMSection(transceiver.get(), description);

		const ContentInfo* current_remote_msection = FindTransceiverMSection(
			transceiver.get(), current_remote_description());

		if (transceiver->stopped()) {
			RTC_DCHECK(transceiver->stopping());
			if (current_local_msection && !current_local_msection->rejected &&
				((current_remote_msection && !current_remote_msection->rejected) ||
				!current_remote_msection)) {
				return true;
			}
			continue;
		}

		if (transceiver->stopping() && !transceiver->stopped())
			return true;

		if (!current_local_msection)
			return true;

		const MediaContentDescription* current_local_media_description =
			current_local_msection->media_description();

		if (RtpTransceiverDirectionHasSend(transceiver->direction())) {
			if (current_local_media_description->streams().size() == 0)
				return true;

			std::vector<std::string> msection_msids;
			for (const auto& stream : current_local_media_description->streams()) {
				for (const std::string& msid : stream.stream_ids())
					msection_msids.push_back(msid);
			}

			std::vector<std::string> transceiver_msids =
				transceiver->sender()->stream_ids();
			if (msection_msids.size() != transceiver_msids.size())
				return true;

			absl::c_sort(transceiver_msids);
			absl::c_sort(msection_msids);
			if (transceiver_msids != msection_msids)
				return true;
		}

		if (description->GetType() == SdpType::kOffer) {
			if (!current_remote_description())
				return true;

			if (!current_remote_msection)
				return true;

			RtpTransceiverDirection current_local_direction =
				current_local_media_description->direction();
			RtpTransceiverDirection current_remote_direction =
				current_remote_msection->media_description()->direction();
			if (transceiver->direction() != current_local_direction &&
				transceiver->direction() !=
				RtpTransceiverDirectionReversed(current_remote_direction)) {
				return true;
			}
		}

		if (description->GetType() == SdpType::kAnswer) {
			if (!remote_description())
				return true;

			const ContentInfo* offered_remote_msection =
				FindTransceiverMSection(transceiver.get(), remote_description());

			RtpTransceiverDirection offered_direction =
				offered_remote_msection
				? offered_remote_msection->media_description()->direction()
				: RtpTransceiverDirection::kInactive;

			if (current_local_media_description->direction() !=
				(RtpTransceiverDirectionIntersection(
				transceiver->direction(),
				RtpTransceiverDirectionReversed(offered_direction)))) {
				return true;
			}
		}
	}

	return false;
}

void RTCSdpOfferAnswerHandler::GenerateNegotiationNeededEvent() {
	RTC_DCHECK_RUN_ON(signaling_thread());
	++negotiation_needed_event_id_;
	_rtc_connection->Observer()->OnNegotiationNeededEvent(negotiation_needed_event_id_);
}

RTCError RTCSdpOfferAnswerHandler::ValidateSessionDescription(
	const SessionDescriptionInterface* sdesc,
	cricket::ContentSource source) {
	if (session_error() != SessionError::kNone) {
		LOG_AND_RETURN_ERROR(RTCErrorType::INTERNAL_ERROR, GetSessionErrorMsg());
	}

	if (!sdesc || !sdesc->description()) {
		LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_PARAMETER, kInvalidSdp);
	}

	SdpType type = sdesc->GetType();
	if ((source == cricket::CS_LOCAL && !ExpectSetLocalDescription(type)) ||
		(source == cricket::CS_REMOTE && !ExpectSetRemoteDescription(type))) {
		LOG_AND_RETURN_ERROR(
			RTCErrorType::INVALID_STATE,
			"Called in wrong state: " + GetSignalingStateString(signaling_state()));
	}

	RTCError error = ValidateMids(*sdesc->description());
	if (!error.ok()) {
		return error;
	}

	std::string crypto_error;
	if (webrtc_session_desc_factory_->SdesPolicy() == cricket::SEC_REQUIRED ||
		_rtc_connection->dtls_enabled()) {
		RTCError crypto_error =
			VerifyCrypto(sdesc->description(), _rtc_connection->dtls_enabled());
		if (!crypto_error.ok()) {
			return crypto_error;
		}
	}

	if (!VerifyIceUfragPwdPresent(sdesc->description())) {
		LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_PARAMETER,
			kSdpWithoutIceUfragPwd);
	}

	if (!_rtc_connection->ValidateBundleSettings(sdesc->description())) {
		LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_PARAMETER,
			kBundleWithoutRtcpMux);
	}

	if (type == SdpType::kPrAnswer || type == SdpType::kAnswer) {
		const cricket::SessionDescription* offer_desc =
			(source == cricket::CS_LOCAL) ? remote_description()->description()
			: local_description()->description();
		if (!MediaSectionsHaveSameCount(*offer_desc, *sdesc->description()) ||
			!MediaSectionsInSameOrder(*offer_desc, nullptr, *sdesc->description(),
			type)) {
			LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_PARAMETER,
				kMlineMismatchInAnswer);
		}
	}
	else {
		const cricket::SessionDescription* current_desc = nullptr;
		const cricket::SessionDescription* secondary_current_desc = nullptr;
		if (local_description()) {
			current_desc = local_description()->description();
			if (remote_description()) {
				secondary_current_desc = remote_description()->description();
			}
		}
		else if (remote_description()) {
			current_desc = remote_description()->description();
		}
		if (current_desc &&
			!MediaSectionsInSameOrder(*current_desc, secondary_current_desc,
										*sdesc->description(), type)) {
			LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_PARAMETER,
				kMlineMismatchInSubsequentOffer);
		}
	}

	if (IsUnifiedPlan()) {
		for (const ContentInfo& content : sdesc->description()->contents()) {
			const MediaContentDescription& desc = *content.media_description();
			if ((desc.type() == cricket::MEDIA_TYPE_AUDIO ||
				desc.type() == cricket::MEDIA_TYPE_VIDEO) &&
				desc.streams().size() > 1u) {
				LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_PARAMETER,
					"Media section has more than one track specified "
					"with a=ssrc lines which is not supported with "
					"Unified Plan.");
			}
		}
	}

	return RTCError::OK();
}

RTCError RTCSdpOfferAnswerHandler::UpdateTransceiversAndDataChannels(
	cricket::ContentSource source,
	const SessionDescriptionInterface& new_session,
	const SessionDescriptionInterface* old_local_description,
	const SessionDescriptionInterface* old_remote_description) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	RTC_DCHECK(IsUnifiedPlan());

	const cricket::ContentGroup* bundle_group = nullptr;
	if (new_session.GetType() == SdpType::kOffer) {
		auto bundle_group_or_error =
			GetEarlyBundleGroup(*new_session.description());
		if (!bundle_group_or_error.ok()) {
			return bundle_group_or_error.MoveError();
		}
		bundle_group = bundle_group_or_error.MoveValue();
	}

	const ContentInfos& new_contents = new_session.description()->contents();
	for (size_t i = 0; i < new_contents.size(); ++i) {
		const cricket::ContentInfo& new_content = new_contents[i];
		cricket::MediaType media_type = new_content.media_description()->type();

		mid_generator_.AddKnownId(new_content.name);
		if (media_type == cricket::MEDIA_TYPE_AUDIO ||
			media_type == cricket::MEDIA_TYPE_VIDEO) {
			const cricket::ContentInfo* old_local_content = nullptr;
			if (old_local_description &&
				i < old_local_description->description()->contents().size()) {
				old_local_content =
					&old_local_description->description()->contents()[i];
			}
			const cricket::ContentInfo* old_remote_content = nullptr;
			if (old_remote_description &&
				i < old_remote_description->description()->contents().size()) {
				old_remote_content =
					&old_remote_description->description()->contents()[i];
			}

			auto transceiver_or_error =
				AssociateTransceiver(source, new_session.GetType(), i, new_content,
				old_local_content, old_remote_content);
			if (!transceiver_or_error.ok()) {
				if (new_content.rejected) {
					continue;
				}
				return transceiver_or_error.MoveError();
			}
			auto transceiver = transceiver_or_error.MoveValue();
			RTCError error =
				UpdateTransceiverChannel(transceiver, new_content, bundle_group);
			if (!error.ok()) {
				return error;
			}
		}
		else if (media_type == cricket::MEDIA_TYPE_DATA) {
			if (_rtc_connection->GetDataMid() && new_content.name != *(_rtc_connection->GetDataMid())) {
				// Ignore all but the first data section.
				RTC_LOG(LS_INFO) << "Ignoring data media section with MID="
					<< new_content.name;
				continue;
			}
			RTCError error = UpdateDataChannel(source, new_content, bundle_group);
			if (!error.ok()) {
				return error;
			}
		} else if (media_type == cricket::MEDIA_TYPE_UNSUPPORTED) {
			RTC_LOG(LS_INFO) << "Ignoring unsupported media type";
		}
		else {
			LOG_AND_RETURN_ERROR(RTCErrorType::INTERNAL_ERROR,
				"Unknown section type.");
		}
	}
	return RTCError::OK();
}

RTCErrorOr<	rtc::scoped_refptr<RTCRtpTransceiverProxyWithInternal<RTCRtpTransceiver>>>
RTCSdpOfferAnswerHandler::AssociateTransceiver(cricket::ContentSource source,
		SdpType type,
		size_t mline_index,
		const cricket::ContentInfo& content,
		const cricket::ContentInfo* old_local_content,
		const cricket::ContentInfo* old_remote_content) {
	RTC_DCHECK(IsUnifiedPlan());
	const MediaContentDescription* media_desc = content.media_description();
	auto transceiver = transceivers()->FindByMid(content.name);
	if (source == cricket::CS_LOCAL) {
		if (!transceiver) {
			transceiver = transceivers()->FindByMLineIndex(mline_index);
		}
		if (!transceiver) {
			LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_PARAMETER,
				"Transceiver not found based on m-line index");
		}
	}
	else {
		RTC_DCHECK_EQ(source, cricket::CS_REMOTE);

		if (!transceiver && content.rejected) {
			LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_PARAMETER,
				"local m-line discard");
		}

		if (!transceiver &&
			RtpTransceiverDirectionHasRecv(media_desc->direction()) &&
			!media_desc->HasSimulcast()) {
			transceiver = FindAvailableTransceiverToReceive(media_desc->type());
		}

		if (!transceiver) {
			RTC_LOG(LS_INFO) << "Adding "
				<< cricket::MediaTypeToString(media_desc->type())
				<< " transceiver for MID=" << content.name
				<< " at i=" << mline_index
				<< " in response to the remote description.";
			std::string sender_id = rtc::CreateRandomUuid();
			std::vector<RtpEncodingParameters> send_encodings =
				GetSendEncodingsFromRemoteDescription(*media_desc);
			
			auto sender = rtp_manager()->CreateSender(media_desc->type(), sender_id,
				nullptr, {}, send_encodings);
			std::string receiver_id;
			if (!media_desc->streams().empty()) {
				receiver_id = media_desc->streams()[0].id;
			}
			else {
				receiver_id = rtc::CreateRandomUuid();
			}
			auto receiver =
				rtp_manager()->CreateReceiver(media_desc->type(), receiver_id);
			transceiver = rtp_manager()->CreateAndAddTransceiver(sender, receiver);
			transceiver->internal()->set_direction(RtpTransceiverDirection::kRecvOnly);


			if (media_desc->type() == cricket::MEDIA_TYPE_VIDEO && source == cricket::ContentSource::CS_REMOTE) {
				if (!media_desc->streams().empty()) {
					std::vector<uint32_t> ssrcs;
					media_desc->streams()[0].GetPrimarySsrcs(&ssrcs);

					for (uint8_t i = 0; i < (ssrcs.size() - 1); i++) {
						auto receiver = rtp_manager()->CreateReceiver(media_desc->type(), receiver_id);
						transceiver->internal()->AddReceiver(receiver);
					}
				}
			}

			if (type == SdpType::kOffer) {
				transceivers()->StableState(transceiver)->set_newly_created();
			}
		}

		RTC_DCHECK(transceiver);

		if (SimulcastIsRejected(old_local_content, *media_desc)) {
			RTCError error =
				DisableSimulcastInSender(transceiver->internal()->sender_internal());
			if (!error.ok()) {
				RTC_LOG(LS_ERROR) << "Failed to remove rejected simulcast.";
				return std::move(error);
			}
		}
	}

	if (transceiver->media_type() != media_desc->type()) {
		LOG_AND_RETURN_ERROR(
			RTCErrorType::INVALID_PARAMETER,
			"Transceiver type does not match media description type.");
	}

	if (media_desc->HasSimulcast()) {
		std::vector<SimulcastLayer> layers =
			source == cricket::CS_LOCAL
			? media_desc->simulcast_description().send_layers().GetAllLayers()
			: media_desc->simulcast_description()
			.receive_layers()
			.GetAllLayers();
		RTCError error = UpdateSimulcastLayerStatusInSender(
			layers, transceiver->internal()->sender_internal());
		if (!error.ok()) {
			RTC_LOG(LS_ERROR) << "Failed updating status for simulcast layers.";
			return std::move(error);
		}
	}
	if (type == SdpType::kOffer) {
		bool state_changes = transceiver->internal()->mid() != content.name ||
			transceiver->internal()->mline_index() != mline_index;
		if (state_changes) {
			transceivers()
				->StableState(transceiver)
				->SetMSectionIfUnset(transceiver->internal()->mid(),
				transceiver->internal()->mline_index());
		}
	}
	// Associate the found or created RtpTransceiver with the m= section by
	// setting the value of the RtpTransceiver's mid property to the MID of the m=
	// section, and establish a mapping between the transceiver and the index of
	// the m= section.
	transceiver->internal()->set_mid(content.name);
	transceiver->internal()->set_mline_index(mline_index);

	return std::move(transceiver);
}

RTCErrorOr<const cricket::ContentGroup*> RTCSdpOfferAnswerHandler::GetEarlyBundleGroup(
	const cricket::SessionDescription& desc) const {
	const cricket::ContentGroup* bundle_group = nullptr;
	if (_rtc_connection->configuration()->bundle_policy ==
		PeerConnectionInterfaceDefs::kBundlePolicyMaxBundle) {
		bundle_group = desc.GetGroupByName(cricket::GROUP_TYPE_BUNDLE);
		if (!bundle_group) {
			LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_PARAMETER,
				"max-bundle configured but session description "
				"has no BUNDLE group");
		}
	}
	return bundle_group;
}

RTCError RTCSdpOfferAnswerHandler::UpdateTransceiverChannel(
	rtc::scoped_refptr<RTCRtpTransceiverProxyWithInternal<RTCRtpTransceiver>>
	transceiver,
	const cricket::ContentInfo& content,
	const cricket::ContentGroup* bundle_group) {
	RTC_DCHECK(IsUnifiedPlan());
	RTC_DCHECK(transceiver);
	cricket::ChannelInterface* channel = transceiver->internal()->channel();
	if (content.rejected) {
		if (channel) {
			transceiver->internal()->SetChannel(nullptr);
			DestroyChannelInterface(channel);
		}
	}
	else {
		if (!channel) {
			if (transceiver->media_type() == cricket::MEDIA_TYPE_AUDIO) {
				channel = CreateVoiceChannel(content.name);
			}
			else {
				RTC_DCHECK_EQ(cricket::MEDIA_TYPE_VIDEO, transceiver->media_type());
				channel = CreateVideoChannel(content.name);
			}
			if (!channel) {
				LOG_AND_RETURN_ERROR(
					RTCErrorType::INTERNAL_ERROR,
					"Failed to create channel for mid=" + content.name);
			}
			transceiver->internal()->SetChannel(channel);
		}
	}
	return RTCError::OK();
}

RTCError RTCSdpOfferAnswerHandler::UpdateDataChannel(
	cricket::ContentSource source,
	const cricket::ContentInfo& content,
	const cricket::ContentGroup* bundle_group) {
	if (_rtc_connection->data_channel_type() == cricket::DCT_NONE) {
		return RTCError::OK();
	}
	if (content.rejected) {
		RTC_LOG(LS_INFO) << "Rejected data channel, mid=" << content.mid();
		DestroyDataChannelTransport();
	}
	else {
		if (!data_channel_controller()->rtp_data_channel() &&
			!data_channel_controller()->data_channel_transport()) {
			RTC_LOG(LS_INFO) << "Creating data channel, mid=" << content.mid();
			if (!CreateDataChannel(content.name)) {
				LOG_AND_RETURN_ERROR(RTCErrorType::INTERNAL_ERROR,
					"Failed to create data channel.");
			}
		}
		if (source == cricket::CS_REMOTE) {
			const MediaContentDescription* data_desc = content.media_description();
			if (data_desc && cricket::IsRtpProtocol(data_desc->protocol())) {
				data_channel_controller()->UpdateRemoteRtpDataChannels(
					GetActiveStreams(data_desc));
			}
		}
	}
	return RTCError::OK();
}

bool RTCSdpOfferAnswerHandler::ExpectSetLocalDescription(SdpType type) {
	PeerConnectionInterfaceDefs::SignalingState state = signaling_state();
	if (type == SdpType::kOffer) {
		return (state == PeerConnectionInterfaceDefs::kStable) ||
			(state == PeerConnectionInterfaceDefs::kHaveLocalOffer);
	}
	else {
		RTC_DCHECK(type == SdpType::kPrAnswer || type == SdpType::kAnswer);
		return (state == PeerConnectionInterfaceDefs::kHaveRemoteOffer) ||
			(state == PeerConnectionInterfaceDefs::kHaveLocalPrAnswer);
	}
}

bool RTCSdpOfferAnswerHandler::ExpectSetRemoteDescription(SdpType type) {
	PeerConnectionInterfaceDefs::SignalingState state = signaling_state();
	if (type == SdpType::kOffer) {
		return (state == PeerConnectionInterfaceDefs::kStable) ||
			(state == PeerConnectionInterfaceDefs::kHaveRemoteOffer);
	}
	else {
		RTC_DCHECK(type == SdpType::kPrAnswer || type == SdpType::kAnswer);
		return (state == PeerConnectionInterfaceDefs::kHaveLocalOffer) ||
			(state == PeerConnectionInterfaceDefs::kHaveRemotePrAnswer);
	}
}

void RTCSdpOfferAnswerHandler::FillInMissingRemoteMids(
	cricket::SessionDescription* new_remote_description) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	RTC_DCHECK(new_remote_description);
	const cricket::ContentInfos no_infos;
	const cricket::ContentInfos& local_contents =
		(local_description() ? local_description()->description()->contents()
		: no_infos);
	const cricket::ContentInfos& remote_contents =
		(remote_description() ? remote_description()->description()->contents()
		: no_infos);
	for (size_t i = 0; i < new_remote_description->contents().size(); ++i) {
		cricket::ContentInfo& content = new_remote_description->contents()[i];
		if (!content.name.empty()) {
			continue;
		}
		std::string new_mid;
		absl::string_view source_explanation;
		if (IsUnifiedPlan()) {
			if (i < local_contents.size()) {
				new_mid = local_contents[i].name;
				source_explanation = "from the matching local media section";
			}
			else if (i < remote_contents.size()) {
				new_mid = remote_contents[i].name;
				source_explanation = "from the matching previous remote media section";
			}
			else {
				new_mid = mid_generator_.GenerateString();
				source_explanation = "generated just now";
			}
		}
		else {
			new_mid = std::string(
				GetDefaultMidForPlanB(content.media_description()->type()));
			source_explanation = "to match pre-existing behavior";
		}
		RTC_DCHECK(!new_mid.empty());
		content.name = new_mid;
		new_remote_description->transport_infos()[i].content_name = new_mid;
		RTC_LOG(LS_INFO) << "SetRemoteDescription: Remote media section at i=" << i
			<< " is missing an a=mid line. Filling in the value '"
			<< new_mid << "' " << source_explanation << ".";
	}
}

rtc::scoped_refptr<RTCRtpTransceiverProxyWithInternal<RTCRtpTransceiver>>
RTCSdpOfferAnswerHandler::FindAvailableTransceiverToReceive(
cricket::MediaType media_type) const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	RTC_DCHECK(IsUnifiedPlan());
	for (auto transceiver : transceivers()->List()) {
		if (transceiver->media_type() == media_type &&
			/*transceiver->internal()->created_by_addtrack() &&*/ !transceiver->mid() &&		
			!transceiver->stopped()) {
			return transceiver;
		}
	}
	return nullptr;
}

const cricket::ContentInfo* RTCSdpOfferAnswerHandler::FindMediaSectionForTransceiver(
	rtc::scoped_refptr<RTCRtpTransceiverProxyWithInternal<RTCRtpTransceiver>>
	transceiver,
	const SessionDescriptionInterface* sdesc) const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	RTC_DCHECK(transceiver);
	RTC_DCHECK(sdesc);
	if (IsUnifiedPlan()) {
		if (!transceiver->internal()->mid()) {
			return nullptr;
		}
		return sdesc->description()->GetContentByName(
			*transceiver->internal()->mid());
	}
	else {
		return cricket::GetFirstMediaContent(sdesc->description()->contents(),
			transceiver->media_type());
	}
}

void RTCSdpOfferAnswerHandler::GetOptionsForOffer(
	const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions& 
	offer_answer_options,
	cricket::MediaSessionOptions* session_options) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	ExtractSharedMediaSessionOptions(offer_answer_options, session_options);

	if (IsUnifiedPlan()) {
		GetOptionsForUnifiedPlanOffer(offer_answer_options, session_options);
	}
	else {
		GetOptionsForPlanBOffer(offer_answer_options, session_options);
	}

	if (data_channel_controller()->HasRtpDataChannels() ||
		_rtc_connection->data_channel_type() != cricket::DCT_RTP) {
		session_options->data_channel_type = _rtc_connection->data_channel_type();
	}

	bool ice_restart = offer_answer_options.ice_restart || HasNewIceCredentials();
	for (auto& options : session_options->media_description_options) {
		options.transport_options.ice_restart = ice_restart;
		options.transport_options.enable_ice_renomination =
			_rtc_connection->configuration()->enable_ice_renomination;
	}

	session_options->rtcp_cname = rtcp_cname_;
	session_options->crypto_options = _rtc_connection->GetCryptoOptions();
	session_options->pooled_ice_credentials =
		_rtc_connection->network_thread()->Invoke<std::vector<cricket::IceParameters>>(
		RTC_FROM_HERE,
		[this] { return port_allocator()->GetPooledIceCredentials(); });
	session_options->offer_extmap_allow_mixed =
		_rtc_connection->configuration()->offer_extmap_allow_mixed;

	session_options->use_obsolete_sctp_sdp = offer_answer_options.use_obsolete_sctp_sdp;
}

void RTCSdpOfferAnswerHandler::GetOptionsForPlanBOffer(
	const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions& offer_answer_options,
	cricket::MediaSessionOptions* session_options) {
	bool send_audio =
		!rtp_manager()->GetAudioTransceiver()->internal()->senders().empty();;
	bool send_video =
		!rtp_manager()->GetVideoTransceiver()->internal()->senders().empty();;

	// By default, generate sendrecv/recvonly m= sections.
	bool recv_audio = true;
	bool recv_video = true;

	// By default, only offer a new m= section if we have media to send with it.
	bool offer_new_audio_description = send_audio;
	bool offer_new_video_description = send_video;
	bool offer_new_data_description = data_channel_controller()->HasDataChannels();

	// The "offer_to_receive_X" options allow those defaults to be overridden.
	if (offer_answer_options.offer_to_receive_audio !=
		PeerConnectionInterfaceDefs::RTCOfferAnswerOptions::kUndefined) {
		recv_audio = (offer_answer_options.offer_to_receive_audio > 0);
		offer_new_audio_description =
			offer_new_audio_description ||
			(offer_answer_options.offer_to_receive_audio > 0);
	}
	if (offer_answer_options.offer_to_receive_video !=
		PeerConnectionInterfaceDefs::RTCOfferAnswerOptions::kUndefined) {
		recv_video = (offer_answer_options.offer_to_receive_video > 0);
		offer_new_video_description =
			offer_new_video_description ||
			(offer_answer_options.offer_to_receive_video > 0);
	}

	absl::optional<size_t> audio_index;
	absl::optional<size_t> video_index;
	absl::optional<size_t> data_index;
	// If a current description exists, generate m= sections in the same order,
	// using the first audio/video/data section that appears and rejecting
	// extraneous ones.
	if (local_description()) {
		GenerateMediaDescriptionOptions(
			local_description(),
			RtpTransceiverDirectionFromSendRecv(send_audio, recv_audio),
			RtpTransceiverDirectionFromSendRecv(send_video, recv_video),
			&audio_index, &video_index, &data_index, session_options);
	}

	// Add audio/video/data m= sections to the end if needed.
	if (!audio_index && offer_new_audio_description) {
		cricket::MediaDescriptionOptions options(
			cricket::MEDIA_TYPE_AUDIO, cricket::CN_AUDIO,
			RtpTransceiverDirectionFromSendRecv(send_audio, recv_audio), false);
		options.header_extensions =
			_rtc_connection->GetAudioRtpHeaderExtensions();

		session_options->media_description_options.push_back(options);
		audio_index = session_options->media_description_options.size() - 1;
	}
	if (!video_index && offer_new_video_description) {
		cricket::MediaDescriptionOptions options(
			cricket::MEDIA_TYPE_VIDEO, cricket::CN_VIDEO,
			RtpTransceiverDirectionFromSendRecv(send_video, recv_video), false);
		options.header_extensions =
			_rtc_connection->GetVideoRtpHeaderExtensions();

		session_options->media_description_options.push_back(options);
		video_index = session_options->media_description_options.size() - 1;
	}

	if (!data_index && offer_new_data_description) {
		session_options->media_description_options.push_back(
			GetMediaDescriptionOptionsForActiveData(cricket::CN_DATA));
		data_index = session_options->media_description_options.size() - 1;
	}

	cricket::MediaDescriptionOptions* audio_media_description_options =
		!audio_index ? nullptr
		: &session_options->media_description_options[*audio_index];
	cricket::MediaDescriptionOptions* video_media_description_options =
		!video_index ? nullptr
		: &session_options->media_description_options[*video_index];


	AddPlanBRtpSenderOptions(rtp_manager()->GetSendersInternal(),
		audio_media_description_options,
		video_media_description_options,
		offer_answer_options.num_simulcast_layers);
}

void RTCSdpOfferAnswerHandler::GetOptionsForUnifiedPlanOffer(
	const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions& offer_answer_options,
	cricket::MediaSessionOptions* session_options) {

	RTC_DCHECK_EQ(session_options->media_description_options.size(), 0);
	const ContentInfos no_infos;
	const ContentInfos& local_contents =
		(local_description() ? local_description()->description()->contents()
		: no_infos);
	const ContentInfos& remote_contents =
		(remote_description() ? remote_description()->description()->contents()
		: no_infos);

	std::queue<size_t> recycleable_mline_indices;

	for (size_t i = 0;
		i < std::max(local_contents.size(), remote_contents.size()); ++i) {
		const ContentInfo* local_content =
			(i < local_contents.size() ? &local_contents[i] : nullptr);
		const ContentInfo* current_local_content =
			GetContentByIndex(current_local_description(), i);
		const ContentInfo* remote_content =
			(i < remote_contents.size() ? &remote_contents[i] : nullptr);
		const ContentInfo* current_remote_content =
			GetContentByIndex(current_remote_description(), i);
		bool had_been_rejected =
			(current_local_content && current_local_content->rejected) ||
			(current_remote_content && current_remote_content->rejected);
		const std::string& mid =
			(local_content ? local_content->name : remote_content->name);
		cricket::MediaType media_type =
			(local_content ? local_content->media_description()->type()
			: remote_content->media_description()->type());
		if (media_type == cricket::MEDIA_TYPE_AUDIO ||
			media_type == cricket::MEDIA_TYPE_VIDEO) {
			auto transceiver = transceivers()->FindByMid(mid);
			if (!transceiver) {
				// No associated transceiver. The media section has been stopped.
				recycleable_mline_indices.push(i);
				session_options->media_description_options.push_back(
					cricket::MediaDescriptionOptions(media_type, mid,
					RtpTransceiverDirection::kInactive,
					/*stopped=*/true));
			}
			else {
				if (had_been_rejected && transceiver->stopping()) {
					session_options->media_description_options.push_back(
						cricket::MediaDescriptionOptions(
						transceiver->media_type(), mid,
						RtpTransceiverDirection::kInactive,
						/*stopped=*/true));
					recycleable_mline_indices.push(i);
				}
				else {
					session_options->media_description_options.push_back(
						GetMediaDescriptionOptionsForTransceiver(
						transceiver, mid,
						/*is_create_offer=*/true));
					transceiver->internal()->set_mline_index(i);
				}
			}
		}
		else if (media_type == cricket::MEDIA_TYPE_UNSUPPORTED) {
			RTC_DCHECK(local_content->rejected);
			session_options->media_description_options.push_back(
				cricket::MediaDescriptionOptions(media_type, mid,
				RtpTransceiverDirection::kInactive,
				/*stopped=*/true));
		}
		else {
			RTC_CHECK_EQ(cricket::MEDIA_TYPE_DATA, media_type);
			if (had_been_rejected) {
				session_options->media_description_options.push_back(
					GetMediaDescriptionOptionsForRejectedData(mid));
			}
			else {
				RTC_CHECK(_rtc_connection->GetDataMid());
				if (mid == *(_rtc_connection->GetDataMid())) {
					session_options->media_description_options.push_back(
						GetMediaDescriptionOptionsForActiveData(mid));
				}
				else {
					session_options->media_description_options.push_back(
						GetMediaDescriptionOptionsForRejectedData(mid));
				}
			}
		}
	}

	// Next, look for transceivers that are newly added (that is, are not stopped
	// and not associated). Reuse media sections marked as recyclable first,
	// otherwise append to the end of the offer. New media sections should be
	// added in the order they were added to the PeerConnection.
	for (const auto& transceiver : transceivers()->List()) {
		if (transceiver->mid() || transceiver->stopping()) {
			continue;
		}
		size_t mline_index;
		if (!recycleable_mline_indices.empty()) {
			mline_index = recycleable_mline_indices.front();
			recycleable_mline_indices.pop();
			session_options->media_description_options[mline_index] =
				GetMediaDescriptionOptionsForTransceiver(
				transceiver, mid_generator_.GenerateString(),
				/*is_create_offer=*/true);
		}
		else {
			mline_index = session_options->media_description_options.size();
			session_options->media_description_options.push_back(
				GetMediaDescriptionOptionsForTransceiver(
				transceiver, mid_generator_.GenerateString(),
				/*is_create_offer=*/true));
		}
		// See comment above for why CreateOffer changes the transceiver's state.
		transceiver->internal()->set_mline_index(mline_index);
	}
	// Lastly, add a m-section if we have local data channels and an m section
	// does not already exist.
	if (!_rtc_connection->GetDataMid() && data_channel_controller()->HasDataChannels()) {
		session_options->media_description_options.push_back(
			GetMediaDescriptionOptionsForActiveData(
			mid_generator_.GenerateString()));
	}
}

void RTCSdpOfferAnswerHandler::GetOptionsForAnswer(
	const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions&
	offer_answer_options,
	cricket::MediaSessionOptions* session_options) {

	RTC_DCHECK_RUN_ON(signaling_thread());
	ExtractSharedMediaSessionOptions(offer_answer_options, session_options);

	if (IsUnifiedPlan()) {
		GetOptionsForUnifiedPlanAnswer(offer_answer_options, session_options);
	}
	else {
		GetOptionsForPlanBAnswer(offer_answer_options, session_options);
	}

	if (data_channel_controller()->HasRtpDataChannels() ||
		_rtc_connection->data_channel_type() != cricket::DCT_RTP) {
		session_options->data_channel_type = _rtc_connection->data_channel_type();
	}

	// Apply ICE renomination flag.
	for (auto& options : session_options->media_description_options) {
		options.transport_options.enable_ice_renomination =
			_rtc_connection->configuration()->enable_ice_renomination;
	}

	session_options->rtcp_cname = rtcp_cname_;
	session_options->crypto_options = _rtc_connection->GetCryptoOptions();
	session_options->pooled_ice_credentials =
		_rtc_connection->network_thread()->Invoke<std::vector<cricket::IceParameters>>(
		RTC_FROM_HERE,
		rtc::Bind(&cricket::PortAllocator::GetPooledIceCredentials,
		port_allocator()));
}

void RTCSdpOfferAnswerHandler::GetOptionsForPlanBAnswer(
	const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions&
	offer_answer_options,
	cricket::MediaSessionOptions* session_options) {
	// Figure out transceiver directional preferences.
	bool send_audio =
		!rtp_manager()->GetAudioTransceiver()->internal()->senders().empty();
	bool send_video =
		!rtp_manager()->GetVideoTransceiver()->internal()->senders().empty();

	// By default, generate sendrecv/recvonly m= sections. The direction is also
	// restricted by the direction in the offer.
	bool recv_audio = true;
	bool recv_video = true;

	// The "offer_to_receive_X" options allow those defaults to be overridden.
	if (offer_answer_options.offer_to_receive_audio !=
		PeerConnectionInterfaceDefs::RTCOfferAnswerOptions::kUndefined) {
		recv_audio = (offer_answer_options.offer_to_receive_audio > 0);
	}
	if (offer_answer_options.offer_to_receive_video !=
		PeerConnectionInterfaceDefs::RTCOfferAnswerOptions::kUndefined) {
		recv_video = (offer_answer_options.offer_to_receive_video > 0);
	}

	absl::optional<size_t> audio_index;
	absl::optional<size_t> video_index;
	absl::optional<size_t> data_index;

	// Generate m= sections that match those in the offer.
	// Note that mediasession.cc will handle intersection our preferred
	// direction with the offered direction.
	GenerateMediaDescriptionOptions(
		remote_description(),
		RtpTransceiverDirectionFromSendRecv(send_audio, recv_audio),
		RtpTransceiverDirectionFromSendRecv(send_video, recv_video), &audio_index,
		&video_index, &data_index, session_options);

	cricket::MediaDescriptionOptions* audio_media_description_options =
		!audio_index ? nullptr
		: &session_options->media_description_options[*audio_index];
	cricket::MediaDescriptionOptions* video_media_description_options =
		!video_index ? nullptr
		: &session_options->media_description_options[*video_index];

	AddPlanBRtpSenderOptions(rtp_manager()->GetSendersInternal(),
		audio_media_description_options,
		video_media_description_options,
		offer_answer_options.num_simulcast_layers);
}

void RTCSdpOfferAnswerHandler::GetOptionsForUnifiedPlanAnswer(
		const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions& offer_answer_options,
		cricket::MediaSessionOptions* session_options) {
	// Rules for generating an answer are dictated by JSEP sections 5.3.1 (Initial
	// Answers) and 5.3.2 (Subsequent Answers).
	RTC_DCHECK(remote_description());
	RTC_DCHECK(remote_description()->GetType() == SdpType::kOffer);
	for (const ContentInfo& content :
		remote_description()->description()->contents()) {
		cricket::MediaType media_type = content.media_description()->type();
		if (media_type == cricket::MEDIA_TYPE_AUDIO ||
			media_type == cricket::MEDIA_TYPE_VIDEO) {
			auto transceiver = transceivers()->FindByMid(content.name);
			if (transceiver) {
				session_options->media_description_options.push_back(
					GetMediaDescriptionOptionsForTransceiver(
					transceiver, content.name,
					/*is_create_offer=*/false));
			}
			else {
				RTC_DCHECK(content.rejected);
				session_options->media_description_options.push_back(
					cricket::MediaDescriptionOptions(media_type, content.name,
					RtpTransceiverDirection::kInactive,
					/*stopped=*/true));
			}
		}
		else if (media_type == cricket::MEDIA_TYPE_UNSUPPORTED) {
			RTC_DCHECK(content.rejected);
			session_options->media_description_options.push_back(
				cricket::MediaDescriptionOptions(media_type, content.name,
				RtpTransceiverDirection::kInactive,
				/*stopped=*/true));
		}
		else {
			RTC_CHECK_EQ(cricket::MEDIA_TYPE_DATA, media_type);
			if (_rtc_connection->data_channel_type() == cricket::DCT_NONE || content.rejected ||
				content.name != *(_rtc_connection->GetDataMid())) {
				session_options->media_description_options.push_back(
					GetMediaDescriptionOptionsForRejectedData(content.name));
			}
			else {
				session_options->media_description_options.push_back(
					GetMediaDescriptionOptionsForActiveData(content.name));
			}
		}
	}
}

const char* RTCSdpOfferAnswerHandler::SessionErrorToString(
	SessionError error) const {
	switch (error) {
	case SessionError::kNone:
		return "ERROR_NONE";
	case SessionError::kContent:
		return "ERROR_CONTENT";
	case SessionError::kTransport:
		return "ERROR_TRANSPORT";
	}
	RTC_NOTREACHED();
	return "";
}

std::string RTCSdpOfferAnswerHandler::GetSessionErrorMsg() {
	RTC_DCHECK_RUN_ON(signaling_thread());
	rtc::StringBuilder desc;
	desc << kSessionError << SessionErrorToString(session_error()) << ". ";
	desc << kSessionErrorDesc << session_error_desc() << ".";
	return desc.Release();
}

void RTCSdpOfferAnswerHandler::SetSessionError(SessionError error,
	const std::string& error_desc) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	if (error != session_error_) {
		session_error_ = error;
		session_error_desc_ = error_desc;
	}
}

RTCError RTCSdpOfferAnswerHandler::HandleLegacyOfferOptions(
	const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions& options) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	RTC_DCHECK(IsUnifiedPlan());

	if (options.offer_to_receive_audio == 0) {
		RemoveRecvDirectionFromReceivingTransceiversOfType(
			cricket::MEDIA_TYPE_AUDIO);
	}
	else if (options.offer_to_receive_audio == 1) {
		AddUpToOneReceivingTransceiverOfType(cricket::MEDIA_TYPE_AUDIO);
	}
	else if (options.offer_to_receive_audio > 1) {
		LOG_AND_RETURN_ERROR(RTCErrorType::UNSUPPORTED_PARAMETER,
			"offer_to_receive_audio > 1 is not supported.");
	}

	if (options.offer_to_receive_video == 0) {
		RemoveRecvDirectionFromReceivingTransceiversOfType(
			cricket::MEDIA_TYPE_VIDEO);
	}
	else if (options.offer_to_receive_video == 1) {
		AddUpToOneReceivingTransceiverOfType(cricket::MEDIA_TYPE_VIDEO);
	}
	else if (options.offer_to_receive_video > 1) {
		LOG_AND_RETURN_ERROR(RTCErrorType::UNSUPPORTED_PARAMETER,
			"offer_to_receive_video > 1 is not supported.");
	}

	return RTCError::OK();
}

void RTCSdpOfferAnswerHandler::RemoveRecvDirectionFromReceivingTransceiversOfType(
	cricket::MediaType media_type) {
	for (const auto& transceiver : GetReceivingTransceiversOfType(media_type)) {
		RtpTransceiverDirection new_direction =
			RtpTransceiverDirectionWithRecvSet(transceiver->direction(), false);
		if (new_direction != transceiver->direction()) {
			RTC_LOG(LS_INFO) << "Changing " << cricket::MediaTypeToString(media_type)
				<< " transceiver (MID="
				<< transceiver->mid().value_or("<not set>") << ") from "
				<< RtpTransceiverDirectionToString(
				transceiver->direction())
				<< " to "
				<< RtpTransceiverDirectionToString(new_direction)
				<< " since CreateOffer specified offer_to_receive=0";
			transceiver->internal()->set_direction(new_direction);
		}
	}
}

void RTCSdpOfferAnswerHandler::AddUpToOneReceivingTransceiverOfType(
	cricket::MediaType media_type) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	if (GetReceivingTransceiversOfType(media_type).empty()) {
		RTC_LOG(LS_INFO)
			<< "Adding one recvonly " << cricket::MediaTypeToString(media_type)
			<< " transceiver since CreateOffer specified offer_to_receive=1";
		RTCRtpTransceiverInit init;
		init.direction = RtpTransceiverDirection::kRecvOnly;
		_rtc_connection->AddTransceiver(media_type, nullptr, init,
			/*update_negotiation_needed=*/false);
	}
}

std::vector<rtc::scoped_refptr<RTCRtpTransceiverProxyWithInternal<RTCRtpTransceiver>>>
RTCSdpOfferAnswerHandler::GetReceivingTransceiversOfType(
	cricket::MediaType media_type) {
	std::vector<
		rtc::scoped_refptr<RTCRtpTransceiverProxyWithInternal<RTCRtpTransceiver>>>
		receiving_transceivers;
	for (const auto& transceiver : transceivers()->List()) {
		if (!transceiver->stopped() && transceiver->media_type() == media_type &&
			RtpTransceiverDirectionHasRecv(transceiver->direction())) {
			receiving_transceivers.push_back(transceiver);
		}
	}
	return receiving_transceivers;
}

void RTCSdpOfferAnswerHandler::ProcessRemovalOfRemoteTrack(
	rtc::scoped_refptr<RTCRtpTransceiverProxyWithInternal<RTCRtpTransceiver>>
	transceiver,
	std::vector<rtc::scoped_refptr<RTCRtpTransceiverInterface>>* remove_list,
	std::vector<rtc::scoped_refptr<MediaStreamInterface>>* removed_streams) {
	RTC_DCHECK(transceiver->mid());
	RTC_LOG(LS_INFO) << "Processing the removal of a track for MID="
		<< *transceiver->mid();
	std::vector<rtc::scoped_refptr<MediaStreamInterface>> previous_streams =
		transceiver->internal()->receiver_internal()->streams();
	// This will remove the remote track from the streams.
	transceiver->internal()->receiver_internal()->set_stream_ids({});
	remove_list->push_back(transceiver);
	RemoveRemoteStreamsIfEmpty(previous_streams, removed_streams);
}

void RTCSdpOfferAnswerHandler::RemoveRemoteStreamsIfEmpty(
	const std::vector<rtc::scoped_refptr<MediaStreamInterface>>& remote_streams,
	std::vector<rtc::scoped_refptr<MediaStreamInterface>>* removed_streams) {
	RTC_DCHECK_RUN_ON(signaling_thread());

	for (const auto& remote_stream : remote_streams) {
		if (remote_stream->GetAudioTracks().empty() &&
			remote_stream->GetVideoTracks().empty()) {
			remote_streams_->RemoveStream(remote_stream);
			removed_streams->push_back(remote_stream);
		}
	}
}

void RTCSdpOfferAnswerHandler::RemoveSenders(cricket::MediaType media_type) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	UpdateLocalSenders(std::vector<cricket::StreamParams>(), media_type);
	UpdateRemoteSendersList(std::vector<cricket::StreamParams>(), false,
		media_type, nullptr);
}

void RTCSdpOfferAnswerHandler::UpdateLocalSenders(const std::vector<cricket::StreamParams>& streams,
	cricket::MediaType media_type) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	std::vector<RTCRtpSenderInfo>* current_senders =
		rtp_manager()->GetLocalSenderInfos(media_type);

	for (auto sender_it = current_senders->begin();
		sender_it != current_senders->end();
		/* incremented manually */) {
		const RTCRtpSenderInfo& info = *sender_it;
		const cricket::StreamParams* params =
			cricket::GetStreamBySsrc(streams, info.first_ssrc);
		if (!params || params->id != info.sender_id ||
			params->first_stream_id() != info.stream_id) {
			rtp_manager()->OnLocalSenderRemoved(info, media_type);
			sender_it = current_senders->erase(sender_it);
		}
		else {
			++sender_it;
		}
	}

	for (const cricket::StreamParams& params : streams) {
		const std::string& stream_id = params.first_stream_id();
		const std::string& sender_id = params.id;
		uint32_t ssrc = params.first_ssrc();
		const RTCRtpSenderInfo* sender_info =
			rtp_manager()->FindSenderInfo(*current_senders, stream_id, sender_id);
		if (!sender_info) {
			current_senders->push_back(RTCRtpSenderInfo(stream_id, sender_id, ssrc));
			rtp_manager()->OnLocalSenderAdded(current_senders->back(), media_type);
		}
	}
}

void RTCSdpOfferAnswerHandler::UpdateRemoteSendersList(
	const std::vector<cricket::StreamParams>& streams,
	bool default_sender_needed,
	cricket::MediaType media_type,
	StreamCollection* new_streams) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	RTC_DCHECK(!IsUnifiedPlan());

	std::vector<RTCRtpSenderInfo>* current_senders =
		rtp_manager()->GetRemoteSenderInfos(media_type);

	if (current_senders == nullptr) {
		RTC_CHECK(0);
	}

	for (auto sender_it = current_senders->begin();
		sender_it != current_senders->end();
		/* incremented manually */) {
		const RTCRtpSenderInfo& info = *sender_it;
		const cricket::StreamParams* params =
			cricket::GetStreamBySsrc(streams, info.first_ssrc);
		std::string params_stream_id;
		if (params) {
			params_stream_id =
				(!params->first_stream_id().empty() ? params->first_stream_id()
				: kDefaultStreamId);
		}
		bool sender_exists = params && params->id == info.sender_id &&
			params_stream_id == info.stream_id;
		if ((info.stream_id == kDefaultStreamId && default_sender_needed) ||
			sender_exists) {
			++sender_it;
		}
		else {
			rtp_manager()->OnRemoteSenderRemoved(
				info, remote_streams_->find(info.stream_id), media_type);
			sender_it = current_senders->erase(sender_it);
		}
	}

	for (const cricket::StreamParams& params : streams) {
		if (!params.has_ssrcs()) {
			default_sender_needed = true;
			break;
		}

		const std::string& stream_id =
			(!params.first_stream_id().empty() ? params.first_stream_id()
			: kDefaultStreamId);
		const std::string& sender_id = params.id;
		uint32_t ssrc = params.first_ssrc();

		rtc::scoped_refptr<MediaStreamInterface> stream =
			remote_streams_->find(stream_id);
		if (!stream) {
			stream = MediaStreamProxy::Create(
				rtc::Thread::Current(_rtc_thread_manager),
				MediaStream::Create(stream_id));
			remote_streams_->AddStream(stream);
			new_streams->AddStream(stream);
		}

		const RTCRtpSenderInfo* sender_info =
			rtp_manager()->FindSenderInfo(*current_senders, stream_id, sender_id);
		if (!sender_info) {
			if (media_type == cricket::MEDIA_TYPE_VIDEO && params.has_ssrc_group(cricket::kSimSsrcGroupSemantics)) {
				RTCRtpSenderInfo sinfo(stream_id, sender_id, ssrc);
				const cricket::SsrcGroup* ssrc_sim_group = params.get_ssrc_group(cricket::kSimSsrcGroupSemantics);
				for (uint32_t ssrc : ssrc_sim_group->ssrcs) {
					sinfo.ssrcs.push_back(ssrc);
				}
				current_senders->push_back(sinfo);
			}
			else {
				current_senders->push_back(RTCRtpSenderInfo(stream_id, sender_id, ssrc));
			}
			rtp_manager()->OnRemoteSenderAdded(current_senders->back(), stream,
				media_type);
		}
	}

	if (default_sender_needed) {
		rtc::scoped_refptr<MediaStreamInterface> default_stream =
			remote_streams_->find(kDefaultStreamId);
		if (!default_stream) {
			// Create the new default MediaStream.
			default_stream = MediaStreamProxy::Create(
				rtc::Thread::Current(_rtc_thread_manager),
				MediaStream::Create(kDefaultStreamId));
			remote_streams_->AddStream(default_stream);
			new_streams->AddStream(default_stream);
		}
		std::string default_sender_id = (media_type == cricket::MEDIA_TYPE_AUDIO)
			? kDefaultAudioSenderId
			: kDefaultVideoSenderId;
		const RTCRtpSenderInfo* default_sender_info = rtp_manager()->FindSenderInfo(
			*current_senders, kDefaultStreamId, default_sender_id);
		if (!default_sender_info) {
			current_senders->push_back(
				RTCRtpSenderInfo(kDefaultStreamId, default_sender_id, /*ssrc=*/0));
			rtp_manager()->OnRemoteSenderAdded(current_senders->back(),
				default_stream, media_type);
		}
	}
}

void RTCSdpOfferAnswerHandler::EnableSending() {
	RTC_DCHECK_RUN_ON(signaling_thread());
	for (const auto& transceiver : transceivers()->List()) {
		cricket::ChannelInterface* channel = transceiver->internal()->channel();
		if (channel && !channel->enabled()) {
			channel->Enable(true);
		}
	}

	if (data_channel_controller()->rtp_data_channel() &&
		!data_channel_controller()->rtp_data_channel()->enabled()) {
		data_channel_controller()->rtp_data_channel()->Enable(true);
	}
}

RTCError RTCSdpOfferAnswerHandler::PushdownMediaDescription(SdpType type,
	cricket::ContentSource source) {
	const SessionDescriptionInterface* sdesc =
		(source == cricket::CS_LOCAL ? local_description()
		: remote_description());
	RTC_DCHECK_RUN_ON(signaling_thread());
	RTC_DCHECK(sdesc);

	if (!UpdatePayloadTypeDemuxingState(source)) {
		LOG_AND_RETURN_ERROR(RTCErrorType::INTERNAL_ERROR,
			"Failed to update payload type demuxing state.");
	}

	// Push down the new SDP media section for each audio/video transceiver.
	for (const auto& transceiver : transceivers()->List()) {
		const cricket::ContentInfo* content_info =
			FindMediaSectionForTransceiver(transceiver, sdesc);
		cricket::ChannelInterface* channel = transceiver->internal()->channel();
		if (!channel || !content_info || content_info->rejected) {
			continue;
		}
		const cricket::MediaContentDescription* content_desc =
			content_info->media_description();
		if (!content_desc) {
			continue;
		}
		std::string error;
		bool success = (source == cricket::CS_LOCAL)
			? channel->SetLocalContent(content_desc, type, &error)
			: channel->SetRemoteContent(content_desc, type, &error);
		if (!success) {
			LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_PARAMETER, error);
		}
	}

	if (data_channel_controller()->rtp_data_channel()) {
		const cricket::ContentInfo* data_content =
			cricket::GetFirstDataContent(sdesc->description());
		if (data_content && !data_content->rejected) {
			const cricket::MediaContentDescription* data_desc =
				data_content->media_description();
			if (data_desc) {
				std::string error;
				bool success = (source == cricket::CS_LOCAL)
					? data_channel_controller()
					->rtp_data_channel()
					->SetLocalContent(data_desc, type, &error)
					: data_channel_controller()
					->rtp_data_channel()
					->SetRemoteContent(data_desc, type, &error);
				if (!success) {
					LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_PARAMETER, error);
				}
			}
		}
	}

	if (_rtc_connection->sctp_mid() && local_description() && remote_description()) {
		rtc::scoped_refptr<SctpTransport> sctp_transport =
			transport_controller()->GetSctpTransport(*(_rtc_connection->sctp_mid()));
		auto local_sctp_description = cricket::GetFirstSctpDataContentDescription(
			local_description()->description());
		auto remote_sctp_description = cricket::GetFirstSctpDataContentDescription(
			remote_description()->description());
		if (sctp_transport && local_sctp_description && remote_sctp_description) {
			int max_message_size;
			if (remote_sctp_description->max_message_size() == 0) {
				max_message_size = local_sctp_description->max_message_size();
			}
			else {
				max_message_size =
					std::min(local_sctp_description->max_message_size(),
					remote_sctp_description->max_message_size());
			}
			sctp_transport->Start(local_sctp_description->port(),
				remote_sctp_description->port(), max_message_size);
		}
	}

	return RTCError::OK();
}

RTCError RTCSdpOfferAnswerHandler::PushdownTransportDescription(
	cricket::ContentSource source,
	SdpType type) {
	RTC_DCHECK_RUN_ON(signaling_thread());

	if (source == cricket::CS_LOCAL) {
		const SessionDescriptionInterface* sdesc = local_description();
		RTC_DCHECK(sdesc);
		return transport_controller()->SetLocalDescription(type,
			sdesc->description());
	}
	else {
		const SessionDescriptionInterface* sdesc = remote_description();
		RTC_DCHECK(sdesc);
		return transport_controller()->SetRemoteDescription(type,
			sdesc->description());
	}
}

void RTCSdpOfferAnswerHandler::RemoveStoppedTransceivers() {
	RTC_DCHECK_RUN_ON(signaling_thread());
	if (!IsUnifiedPlan())
		return;

	auto transceiver_list = transceivers()->List();
	for (auto transceiver : transceiver_list) {
		if (!transceiver->stopped()) {
			continue;
		}
		const ContentInfo* local_content =
			FindMediaSectionForTransceiver(transceiver, local_description());
		const ContentInfo* remote_content =
			FindMediaSectionForTransceiver(transceiver, remote_description());
		if ((local_content && local_content->rejected) ||
			(remote_content && remote_content->rejected)) {
			RTC_LOG(LS_INFO) << "Dissociating transceiver"
				<< " since the media section is being recycled.";
			transceiver->internal()->set_mid(absl::nullopt);
			transceiver->internal()->set_mline_index(absl::nullopt);
			transceivers()->Remove(transceiver);
			continue;
		}
		if (!local_content && !remote_content) {
			RTC_LOG(LS_INFO)
				<< "Dropping stopped transceiver that was never associated";
			transceivers()->Remove(transceiver);
			continue;
		}
	}
}

void RTCSdpOfferAnswerHandler::RemoveUnusedChannels(const cricket::SessionDescription* desc) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	// Destroy video channel first since it may have a pointer to the
	// voice channel.
	const cricket::ContentInfo* video_info = cricket::GetFirstVideoContent(desc);
	if (!video_info || video_info->rejected) {
		DestroyTransceiverChannel(rtp_manager()->GetVideoTransceiver());
	}

	const cricket::ContentInfo* audio_info = cricket::GetFirstAudioContent(desc);
	if (!audio_info || audio_info->rejected) {
		DestroyTransceiverChannel(rtp_manager()->GetAudioTransceiver());
	}

	const cricket::ContentInfo* data_info = cricket::GetFirstDataContent(desc);
	if (!data_info || data_info->rejected) {
		DestroyDataChannelTransport();
	}
}

void RTCSdpOfferAnswerHandler::UpdateEndedRemoteMediaStreams() {
	RTC_DCHECK_RUN_ON(signaling_thread());
	std::vector<rtc::scoped_refptr<MediaStreamInterface>> streams_to_remove;
	for (size_t i = 0; i < remote_streams_->count(); ++i) {
		MediaStreamInterface* stream = remote_streams_->at(i);
		if (stream->GetAudioTracks().empty() && stream->GetVideoTracks().empty()) {
			streams_to_remove.push_back(stream);
		}
	}

	for (auto& stream : streams_to_remove) {
		remote_streams_->RemoveStream(stream);
		_rtc_connection->Observer()->OnRemoveStream(std::move(stream));
	}
}

bool RTCSdpOfferAnswerHandler::UseCandidatesInSessionDescription(
	const SessionDescriptionInterface* remote_desc) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	if (!remote_desc) {
		return true;
	}
	bool ret = true;

	for (size_t m = 0; m < remote_desc->number_of_mediasections(); ++m) {
		const IceCandidateCollection* candidates = remote_desc->candidates(m);
		for (size_t n = 0; n < candidates->count(); ++n) {
			const IceCandidateInterface* candidate = candidates->at(n);
			bool valid = false;
			if (!ReadyToUseRemoteCandidate(candidate, remote_desc, &valid)) {
				if (valid) {
					RTC_LOG(LS_INFO)
						<< "UseCandidatesInSessionDescription: Not ready to use "
						"candidate.";
				}
				continue;
			}
			ret = UseCandidate(candidate);
			if (!ret) {
				break;
			}
		}
	}
	return ret;
}

bool RTCSdpOfferAnswerHandler::UseCandidate(const IceCandidateInterface* candidate) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	RTCErrorOr<const cricket::ContentInfo*> result =
		FindContentInfo(remote_description(), candidate);
	if (!result.ok()) {
		RTC_LOG(LS_ERROR) << "UseCandidate: Invalid candidate. "
			<< result.error().message();
		return false;
	}
	std::vector<cricket::Candidate> candidates;
	candidates.push_back(candidate->candidate());
	RTCError error = transport_controller()->AddRemoteCandidates(
		result.value()->name, candidates);
	if (error.ok()) {
		if (_rtc_connection->ice_connection_state() ==
			PeerConnectionInterfaceDefs::kIceConnectionNew ||
			_rtc_connection->ice_connection_state() ==
			PeerConnectionInterfaceDefs::kIceConnectionDisconnected) {
			_rtc_connection->SetIceConnectionState(
				PeerConnectionInterfaceDefs::kIceConnectionChecking);
		}
	}
	else {
		RTC_LOG(LS_WARNING) << error.message();
	}
	return true;
}

bool RTCSdpOfferAnswerHandler::ReadyToUseRemoteCandidate(const IceCandidateInterface* candidate,
	const SessionDescriptionInterface* remote_desc,
	bool* valid) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	*valid = true;

	const SessionDescriptionInterface* current_remote_desc =
		remote_desc ? remote_desc : remote_description();

	if (!current_remote_desc) {
		return false;
	}

	RTCErrorOr<const cricket::ContentInfo*> result =
		FindContentInfo(current_remote_desc, candidate);
	if (!result.ok()) {
		RTC_LOG(LS_ERROR) << "ReadyToUseRemoteCandidate: Invalid candidate. "
			<< result.error().message();

		*valid = false;
		return false;
	}

	std::string transport_name = GetTransportName(result.value()->name);
	return !transport_name.empty();
}

RTCErrorOr<const cricket::ContentInfo*> RTCSdpOfferAnswerHandler::FindContentInfo(
	const SessionDescriptionInterface* description,
	const IceCandidateInterface* candidate) {
	if (candidate->sdp_mline_index() >= 0) {
		size_t mediacontent_index =
			static_cast<size_t>(candidate->sdp_mline_index());
		size_t content_size = description->description()->contents().size();
		if (mediacontent_index < content_size) {
			return &description->description()->contents()[mediacontent_index];
		}
		else {
			return RTCError(RTCErrorType::INVALID_RANGE,
				"Media line index (" +
				rtc::ToString(candidate->sdp_mline_index()) +
				") out of range (number of mlines: " +
				rtc::ToString(content_size) + ").");
		}
	}
	else if (!candidate->sdp_mid().empty()) {
		auto& contents = description->description()->contents();
		auto it = absl::c_find_if(
			contents, [candidate](const cricket::ContentInfo& content_info) {
			return content_info.mid() == candidate->sdp_mid();
		});
		if (it == contents.end()) {
			return RTCError(
				RTCErrorType::INVALID_PARAMETER,
				"Mid " + candidate->sdp_mid() +
				" specified but no media section with that mid found.");
		}
		else {
			return &*it;
		}
	}

	return RTCError(RTCErrorType::INVALID_PARAMETER,
		"Neither sdp_mline_index nor sdp_mid specified.");
}

RTCError RTCSdpOfferAnswerHandler::CreateChannels(const cricket::SessionDescription& desc) {
	RTC_DCHECK_RUN_ON(signaling_thread());

	const cricket::ContentInfo* voice = cricket::GetFirstAudioContent(&desc);
	if (voice && !voice->rejected &&
		!rtp_manager()->GetAudioTransceiver()->internal()->channel()) {
		cricket::RTCVoiceChannel* voice_channel = CreateVoiceChannel(voice->name);
		if (!voice_channel) {
			LOG_AND_RETURN_ERROR(RTCErrorType::INTERNAL_ERROR,
				"Failed to create voice channel.");
		}
		rtp_manager()->GetAudioTransceiver()->internal()->SetChannel(voice_channel);
	}

	const cricket::ContentInfo* video = cricket::GetFirstVideoContent(&desc);
	if (video && !video->rejected &&
		!rtp_manager()->GetVideoTransceiver()->internal()->channel()) {
		cricket::RTCVideoChannel* video_channel = CreateVideoChannel(video->name);
		if (!video_channel) {
			LOG_AND_RETURN_ERROR(RTCErrorType::INTERNAL_ERROR,
				"Failed to create video channel.");
		}
		rtp_manager()->GetVideoTransceiver()->internal()->SetChannel(video_channel);
	}

	const cricket::ContentInfo* data = cricket::GetFirstDataContent(&desc);
	if (_rtc_connection->data_channel_type() != cricket::DCT_NONE && data &&
		!data->rejected && !data_channel_controller()->rtp_data_channel() &&
		!data_channel_controller()->data_channel_transport()) {
		if (!CreateDataChannel(data->name)) {
			LOG_AND_RETURN_ERROR(RTCErrorType::INTERNAL_ERROR,
				"Failed to create data channel.");
		}
	}

	return RTCError::OK();
}

cricket::RTCVoiceChannel* RTCSdpOfferAnswerHandler::CreateVoiceChannel(
	const std::string& mid) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	RtpTransportInternal* rtp_transport = _rtc_connection->GetRtpTransport(mid);

	cricket::RTCVoiceChannel* voice_channel;
	{
		RTC_DCHECK_RUN_ON(_rtc_connection->signaling_thread());
		voice_channel = channel_manager()->CreateVoiceChannel(
			_rtc_connection->call_ptr(),
			_rtc_connection->configuration()->media_config,
			rtp_transport,
			signaling_thread(), mid,
			_rtc_connection->SrtpRequired(),
			_rtc_connection->GetCryptoOptions(),
			&ssrc_generator_,
			audio_options());
	}
	if (!voice_channel) {
		return nullptr;
	}

	_rtc_connection->BaseChannelConnectSentPacketSignal(voice_channel);

	voice_channel->SetRtpTransport(rtp_transport);

	return voice_channel;
}

cricket::RTCVideoChannel* RTCSdpOfferAnswerHandler::CreateVideoChannel(const std::string& mid) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	RtpTransportInternal* rtp_transport = _rtc_connection->GetRtpTransport(mid);

	cricket::RTCVideoChannel* video_channel;
	{
		RTC_DCHECK_RUN_ON(_rtc_connection->signaling_thread());
		video_channel = channel_manager()->CreateVideoChannel(
			_rtc_connection->call_ptr(),
			_rtc_connection->configuration()->media_config,
			rtp_transport,
			signaling_thread(), mid,
			_rtc_connection->SrtpRequired(),
			_rtc_connection->GetCryptoOptions(),
			&ssrc_generator_);
	}
	if (!video_channel) {
		return nullptr;
	}

	_rtc_connection->BaseChannelConnectSentPacketSignal(video_channel);

	video_channel->SetRtpTransport(rtp_transport);

	return video_channel;
}

bool RTCSdpOfferAnswerHandler::CreateDataChannel(const std::string& mid) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	switch (_rtc_connection->data_channel_type()) {
	case cricket::DCT_SCTP:
		if (_rtc_connection->network_thread()->Invoke<bool>(RTC_FROM_HERE, [this, &mid] {
			RTC_DCHECK_RUN_ON(_rtc_connection->network_thread());
			return _rtc_connection->SetupDataChannelTransport_n(mid);
		})) {
			_rtc_connection->SetSctpDataMid(mid);
		}
		else {
			return false;
		}
		return true;
	case cricket::DCT_RTP:
	default:
		RtpTransportInternal* rtp_transport = _rtc_connection->GetRtpTransport(mid);
		{
			RTC_DCHECK_RUN_ON(_rtc_connection->signaling_thread());
			data_channel_controller()->set_rtp_data_channel(
				channel_manager()->CreateRtpDataChannel(
				_rtc_connection->configuration()->media_config, rtp_transport,
				signaling_thread(), mid, _rtc_connection->SrtpRequired(),
				_rtc_connection->GetCryptoOptions(), &ssrc_generator_));
		}
		if (!data_channel_controller()->rtp_data_channel()) {
			return false;
		}

		_rtc_connection->BaseChannelConnectSentPacketSignal(data_channel_controller()->rtp_data_channel());

		data_channel_controller()->rtp_data_channel()->SetRtpTransport(
			rtp_transport);
		SetHavePendingRtpDataChannel();
		return true;
	}
	return false;
}

void RTCSdpOfferAnswerHandler::DestroyTransceiverChannel(
	rtc::scoped_refptr<RTCRtpTransceiverProxyWithInternal<RTCRtpTransceiver>>
	transceiver) {
	RTC_LOG(LS_VERBOSE) << "RTCSdpOfferAnswerHandler::DestroyTransceiverChannel media type " << transceiver->media_type();
	RTC_DCHECK(transceiver);

	cricket::ChannelInterface* channel = transceiver->internal()->channel();
	if (channel) {
		transceiver->internal()->SetChannel(nullptr);
		DestroyChannelInterface(channel);
	}
}

void RTCSdpOfferAnswerHandler::DestroyDataChannelTransport() {
	RTC_DCHECK_RUN_ON(signaling_thread());
	if (data_channel_controller()->rtp_data_channel()) {
		data_channel_controller()->OnTransportChannelClosed();
		DestroyChannelInterface(data_channel_controller()->rtp_data_channel());
		data_channel_controller()->set_rtp_data_channel(nullptr);
	}

	if (_rtc_connection->sctp_mid()) {
		RTC_DCHECK_RUN_ON(_rtc_connection->signaling_thread());
		data_channel_controller()->OnTransportChannelClosed();
		_rtc_connection->network_thread()->Invoke<void>(RTC_FROM_HERE, [this] {
			RTC_DCHECK_RUN_ON(_rtc_connection->network_thread());
			_rtc_connection->TeardownDataChannelTransport_n();
		});
		_rtc_connection->ResetSctpDataMid();
	}
}

void RTCSdpOfferAnswerHandler::DestroyChannelInterface(cricket::ChannelInterface* channel) {
	RTC_DCHECK(channel);
	switch (channel->media_type()) {
	case cricket::MEDIA_TYPE_AUDIO:
		channel_manager()->DestroyVoiceChannel(
			static_cast<cricket::RTCVoiceChannel*>(channel));
		break;
	case cricket::MEDIA_TYPE_VIDEO:
		channel_manager()->DestroyVideoChannel(
			static_cast<cricket::RTCVideoChannel*>(channel));
		break;
	case cricket::MEDIA_TYPE_DATA:
		channel_manager()->DestroyRtpDataChannel(
			static_cast<cricket::RTCRtpDataChannel*>(channel));
		break;
	default:
		RTC_NOTREACHED() << "Unknown media type: " << channel->media_type();
		break;
	}
}

void RTCSdpOfferAnswerHandler::DestroyAllChannels() {
	RTC_DCHECK_RUN_ON(signaling_thread());
	if (!transceivers()) {
		return;
	}

	for (const auto& transceiver : transceivers()->List()) {
		if (transceiver->media_type() == cricket::MEDIA_TYPE_VIDEO) {
			DestroyTransceiverChannel(transceiver);
		}
	}
	for (const auto& transceiver : transceivers()->List()) {
		if (transceiver->media_type() == cricket::MEDIA_TYPE_AUDIO) {
			DestroyTransceiverChannel(transceiver);
		}
	}

	DestroyDataChannelTransport();
}

void RTCSdpOfferAnswerHandler::GenerateMediaDescriptionOptions(
	const SessionDescriptionInterface* session_desc,
	RtpTransceiverDirection audio_direction,
	RtpTransceiverDirection video_direction,
	absl::optional<size_t>* audio_index,
	absl::optional<size_t>* video_index,
	absl::optional<size_t>* data_index,
	cricket::MediaSessionOptions* session_options) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	for (const cricket::ContentInfo& content :
		session_desc->description()->contents()) {
		if (IsAudioContent(&content)) {
			// If we already have an audio m= section, reject this extra one.
			if (*audio_index) {
				session_options->media_description_options.push_back(
					cricket::MediaDescriptionOptions(
					cricket::MEDIA_TYPE_AUDIO, content.name,
					RtpTransceiverDirection::kInactive, /*stopped=*/true));
			}
			else {
				bool stopped = (audio_direction == RtpTransceiverDirection::kInactive);
				session_options->media_description_options.push_back(
					cricket::MediaDescriptionOptions(cricket::MEDIA_TYPE_AUDIO,
					content.name, audio_direction,
					stopped));
				*audio_index = session_options->media_description_options.size() - 1;
			}
			session_options->media_description_options.back().header_extensions =
				_rtc_connection->GetAudioRtpHeaderExtensions();
		}
		else if (IsVideoContent(&content)) {
			// If we already have an video m= section, reject this extra one.
			if (*video_index) {
				session_options->media_description_options.push_back(
					cricket::MediaDescriptionOptions(
					cricket::MEDIA_TYPE_VIDEO, content.name,
					RtpTransceiverDirection::kInactive, /*stopped=*/true));
			}
			else {
				bool stopped = (video_direction == RtpTransceiverDirection::kInactive);
				session_options->media_description_options.push_back(
					cricket::MediaDescriptionOptions(cricket::MEDIA_TYPE_VIDEO,
					content.name, video_direction,
					stopped));
				*video_index = session_options->media_description_options.size() - 1;
			}
			session_options->media_description_options.back().header_extensions =
				_rtc_connection->GetVideoRtpHeaderExtensions();
		}
		else if (IsUnsupportedContent(&content)) {
			session_options->media_description_options.push_back(
				cricket::MediaDescriptionOptions(cricket::MEDIA_TYPE_UNSUPPORTED,
				content.name,
				RtpTransceiverDirection::kInactive,
				/*stopped=*/true));
		}
		else {
			RTC_DCHECK(IsDataContent(&content));
			// If we already have an data m= section, reject this extra one.
			if (*data_index) {
				session_options->media_description_options.push_back(
					GetMediaDescriptionOptionsForRejectedData(content.name));
			}
			else {
				session_options->media_description_options.push_back(
					GetMediaDescriptionOptionsForActiveData(content.name));
				*data_index = session_options->media_description_options.size() - 1;
			}
		}
	}
}

cricket::MediaDescriptionOptions RTCSdpOfferAnswerHandler::GetMediaDescriptionOptionsForActiveData(
	const std::string& mid) const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	// Direction for data sections is meaningless, but legacy endpoints might
	// expect sendrecv.
	cricket::MediaDescriptionOptions options(cricket::MEDIA_TYPE_DATA, mid,
		RtpTransceiverDirection::kSendRecv,
		/*stopped=*/false);
	AddRtpDataChannelOptions(*(data_channel_controller()->rtp_data_channels()),
		&options);
	return options;
}

cricket::MediaDescriptionOptions RTCSdpOfferAnswerHandler::GetMediaDescriptionOptionsForRejectedData(
	const std::string& mid) const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	cricket::MediaDescriptionOptions options(cricket::MEDIA_TYPE_DATA, mid,
		RtpTransceiverDirection::kInactive,
		/*stopped=*/true);
	AddRtpDataChannelOptions(*(data_channel_controller()->rtp_data_channels()),
		&options);
	return options;
}


const std::string RTCSdpOfferAnswerHandler::GetTransportName(const std::string& content_name) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	cricket::ChannelInterface* channel = _rtc_connection->GetChannel(content_name);
	if (channel) {
		return channel->transport_name();
	}

	if (data_channel_controller()->data_channel_transport()) {
		RTC_DCHECK(_rtc_connection->sctp_mid());
		if (content_name == *(_rtc_connection->sctp_mid())) {
			return *(_rtc_connection->sctp_transport_name());
		}
	}
	// Return an empty string if failed to retrieve the transport name.
	return "";
}

bool RTCSdpOfferAnswerHandler::UpdatePayloadTypeDemuxingState(cricket::ContentSource source) {
	RTC_DCHECK_RUN_ON(signaling_thread());

	const SessionDescriptionInterface* sdesc =
		(source == cricket::CS_LOCAL ? local_description()
		: remote_description());

	const cricket::ContentGroup* bundle_group =
		sdesc->description()->GetGroupByName(cricket::GROUP_TYPE_BUNDLE);
	std::set<int> audio_payload_types;
	std::set<int> video_payload_types;
	bool pt_demuxing_enabled_audio = true;
	bool pt_demuxing_enabled_video = true;
	for (auto& content_info : sdesc->description()->contents()) {
		// If this m= section isn't bundled, it's safe to demux by payload type
		// since other m= sections using the same payload type will also be using
		// different transports.
		if (!bundle_group || !bundle_group->HasContentName(content_info.name)) {
			continue;
		}
		if (content_info.rejected ||
			(source == cricket::ContentSource::CS_LOCAL &&
			!RtpTransceiverDirectionHasRecv(
			content_info.media_description()->direction())) ||
			(source == cricket::ContentSource::CS_REMOTE &&
			!RtpTransceiverDirectionHasSend(
			content_info.media_description()->direction()))) {
			// Ignore transceivers that are not receiving.
			continue;
		}
		switch (content_info.media_description()->type()) {
		case cricket::MediaType::MEDIA_TYPE_AUDIO: {
			const cricket::AudioContentDescription* audio_desc =
				content_info.media_description()->as_audio();
			for (const cricket::AudioCodec& audio : audio_desc->codecs()) {
				if (audio_payload_types.count(audio.id)) {
					// Two m= sections are using the same payload type, thus demuxing
					// by payload type is not possible.
					pt_demuxing_enabled_audio = false;
				}
				audio_payload_types.insert(audio.id);
			}
			break;
		}
		case cricket::MediaType::MEDIA_TYPE_VIDEO: {
			const cricket::VideoContentDescription* video_desc =
				content_info.media_description()->as_video();
			for (const cricket::VideoCodec& video : video_desc->codecs()) {
				if (video_payload_types.count(video.id)) {
					// Two m= sections are using the same payload type, thus demuxing
					// by payload type is not possible.
					pt_demuxing_enabled_video = false;
				}
				video_payload_types.insert(video.id);
			}
			break;
		}
		default:
			// Ignore data channels.
			continue;
		}
	}

	// Gather all updates ahead of time so that all channels can be updated in a
	// single Invoke; necessary due to thread guards.
	std::vector<std::pair<RtpTransceiverDirection, cricket::ChannelInterface*>>
		channels_to_update;
	for (const auto& transceiver : transceivers()->List()) {
		cricket::ChannelInterface* channel = transceiver->internal()->channel();
		const cricket::ContentInfo* content =
			FindMediaSectionForTransceiver(transceiver, sdesc);
		if (!channel || !content) {
			continue;
		}
		RtpTransceiverDirection local_direction =
			content->media_description()->direction();
		if (source == cricket::CS_REMOTE) {
			local_direction = RtpTransceiverDirectionReversed(local_direction);
		}
		channels_to_update.emplace_back(local_direction,
			transceiver->internal()->channel());
	}

	if (channels_to_update.empty()) {
		return true;
	}
	
	return _rtc_connection->worker_thread()->Invoke<bool>(
		RTC_FROM_HERE, [&channels_to_update, bundle_group,
		pt_demuxing_enabled_audio, pt_demuxing_enabled_video]() {
		for (const auto& it : channels_to_update) {
			RtpTransceiverDirection local_direction = it.first;
			cricket::ChannelInterface* channel = it.second;
			cricket::MediaType media_type = channel->media_type();
			bool in_bundle_group = (bundle_group && bundle_group->HasContentName(
				channel->content_name()));
			if (media_type == cricket::MediaType::MEDIA_TYPE_AUDIO) {
				if (!channel->SetPayloadTypeDemuxingEnabled(
					(!in_bundle_group || pt_demuxing_enabled_audio) &&
					RtpTransceiverDirectionHasRecv(local_direction))) {
					return false;
				}
			}
			else if (media_type == cricket::MediaType::MEDIA_TYPE_VIDEO) {
				if (!channel->SetPayloadTypeDemuxingEnabled(
					(!in_bundle_group || pt_demuxing_enabled_video) &&
					RtpTransceiverDirectionHasRecv(local_direction))) {
					return false;
				}
			}
		}
		return true;
	});
}

}