//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /pc/rtp_transmission_manager.cc
//
//////////////////////////////////////////////////////////////////////////


#include <algorithm>

#include "absl/types/optional.h"

#include "api/rtp_transceiver_direction.h"
#include "api/scoped_refptr.h"
#include "api/make_ref_counted.h"

#include "rtc_base/checks.h"
#include "rtc_base/helpers.h"
#include "rtc_base/logging.h"

#include "RTCAudioRtpReceiver.h"
#include "RTCVideoRtpReceiver.h"
#include "RTCChannel.h"
#include "RTCRtpTransmissionManager.h"


namespace webrtc {

static const char kDefaultAudioSenderId[] = "defaulta0";
static const char kDefaultVideoSenderId[] = "defaultv0";

RTCRtpTransmissionManager::RTCRtpTransmissionManager(
	bool is_unified_plan,
	rtc::Thread* signaling_thread,
	rtc::Thread* worker_thread,
	cricket::RTCChannelManager* channel_manager,
	RTCConnectionObserver* observer,
	std::function<void()> on_negotiation_needed,
	rtc::RTCThreadManagerInterface* rtc_thread_manager
	)
	: is_unified_plan_(is_unified_plan),
	  signaling_thread_(signaling_thread),
	  worker_thread_(worker_thread),
	  channel_manager_(channel_manager),
	  observer_(observer),
	  on_negotiation_needed_(on_negotiation_needed),
	  weak_ptr_factory_(this),
	  _rtc_thread_manager(rtc_thread_manager)
{}

void RTCRtpTransmissionManager::Close() {
	closed_ = true;
	observer_ = nullptr;
}

// Implementation of SetStreamsObserver
void RTCRtpTransmissionManager::OnSetStreams() {
	RTC_DCHECK_RUN_ON(signaling_thread());
	if (IsUnifiedPlan())
		OnNegotiationNeeded();
}

// Function to call back to the PeerConnection when negotiation is needed
void RTCRtpTransmissionManager::OnNegotiationNeeded() {
	on_negotiation_needed_();
}

// Function that returns the currently valid observer
RTCConnectionObserver* RTCRtpTransmissionManager::Observer() const {
	RTC_DCHECK(!closed_);
	RTC_DCHECK(observer_);
	return observer_;
}

cricket::VoiceMediaChannel* RTCRtpTransmissionManager::voice_media_channel()
	const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	RTC_DCHECK(!IsUnifiedPlan());
	auto* voice_channel = static_cast<cricket::RTCVoiceChannel*>(
		GetAudioTransceiver()->internal()->channel());
	if (voice_channel) {
		return voice_channel->media_channel();
	}
	else {
		return nullptr;
	}
}

cricket::VideoMediaChannel* RTCRtpTransmissionManager::video_media_channel()
	const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	RTC_DCHECK(!IsUnifiedPlan());
	auto* video_channel = static_cast<cricket::RTCVideoChannel*>(
		GetVideoTransceiver()->internal()->channel());
	if (video_channel) {
		return video_channel->media_channel();
	}
	else {
		return nullptr;
	}
}

RTCErrorOr<rtc::scoped_refptr<RTCRtpSenderInterface>>
RTCRtpTransmissionManager::AddTrack(
	rtc::scoped_refptr<MediaStreamTrackInterface> track,
	const std::vector<std::string>& stream_ids) {
	RTC_DCHECK_RUN_ON(signaling_thread());

	return (IsUnifiedPlan() ? AddTrackUnifiedPlan(track, stream_ids)
		: AddTrackPlanB(track, stream_ids));
}

RTCErrorOr<rtc::scoped_refptr<RTCRtpSenderInterface>>
RTCRtpTransmissionManager::AddTrackPlanB(
	rtc::scoped_refptr<MediaStreamTrackInterface> track,
	const std::vector<std::string>& stream_ids) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	RTC_LOG(LS_VERBOSE) << "AddTrackPlanB";
	if (stream_ids.size() > 1u) {
		LOG_AND_RETURN_ERROR(RTCErrorType::UNSUPPORTED_OPERATION,
			"AddTrack with more than one stream is not "
			"supported with Plan B semantics.");
	}
	std::vector<std::string> adjusted_stream_ids = stream_ids;
	if (adjusted_stream_ids.empty()) {
		adjusted_stream_ids.push_back(rtc::CreateRandomUuid());
	}
	cricket::MediaType media_type =
		(track->kind() == MediaStreamTrackInterface::kAudioKind
		? cricket::MEDIA_TYPE_AUDIO
		: cricket::MEDIA_TYPE_VIDEO);
	auto new_sender =
		CreateSender(media_type, track->id(), track, adjusted_stream_ids, {});
	if (track->kind() == MediaStreamTrackInterface::kAudioKind) {
		new_sender->internal()->SetMediaChannel(voice_media_channel());
		GetAudioTransceiver()->internal()->AddSender(new_sender);
		const RTCRtpSenderInfo* sender_info =
			FindSenderInfo(local_audio_sender_infos_,
			new_sender->internal()->stream_ids()[0], track->id());
		if (sender_info) {
			new_sender->internal()->SetSsrc(sender_info->first_ssrc);
		}
	}
	else {
		RTC_DCHECK_EQ(MediaStreamTrackInterface::kVideoKind, track->kind());
		new_sender->internal()->SetMediaChannel(video_media_channel());
		GetVideoTransceiver()->internal()->AddSender(new_sender);
		const RTCRtpSenderInfo* sender_info =
			FindSenderInfo(local_video_sender_infos_,
			new_sender->internal()->stream_ids()[0], track->id());
		if (sender_info) {
			new_sender->internal()->SetSsrc(sender_info->first_ssrc);
		}
	}
	return rtc::scoped_refptr<RTCRtpSenderInterface>(new_sender);
}

RTCErrorOr<rtc::scoped_refptr<RTCRtpSenderInterface>>
RTCRtpTransmissionManager::AddTrackUnifiedPlan(
	rtc::scoped_refptr<MediaStreamTrackInterface> track,
	const std::vector<std::string>& stream_ids) {
	auto transceiver = FindFirstTransceiverForAddedTrack(track);
	if (transceiver) {
		RTC_LOG(LS_INFO) << "Reusing an existing "
			<< cricket::MediaTypeToString(transceiver->media_type())
			<< " transceiver for AddTrack.";
		if (transceiver->stopping()) {
			LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_PARAMETER,
				"The existing transceiver is stopping.");
		}

		if (transceiver->direction() == RtpTransceiverDirection::kRecvOnly) {
			transceiver->internal()->set_direction(
				RtpTransceiverDirection::kSendRecv);
		}
		else if (transceiver->direction() == RtpTransceiverDirection::kInactive) {
			transceiver->internal()->set_direction(
				RtpTransceiverDirection::kSendOnly);
		}
		transceiver->sender()->SetTrack(track);
		transceiver->internal()->sender_internal()->set_stream_ids(stream_ids);
		transceiver->internal()->set_reused_for_addtrack(true);
	}
	else {
		cricket::MediaType media_type =
			(track->kind() == MediaStreamTrackInterface::kAudioKind
			? cricket::MEDIA_TYPE_AUDIO
			: cricket::MEDIA_TYPE_VIDEO);
		RTC_LOG(LS_INFO) << "Adding " << cricket::MediaTypeToString(media_type)
			<< " transceiver in response to a call to AddTrack.";
		std::string sender_id = track->id();
		// Avoid creating a sender with an existing ID by generating a random ID.
		// This can happen if this is the second time AddTrack has created a sender
		// for this track.
		if (FindSenderById(sender_id)) {
			sender_id = rtc::CreateRandomUuid();
		}
		auto sender = CreateSender(media_type, sender_id, track, stream_ids, {});
		auto receiver = CreateReceiver(media_type, rtc::CreateRandomUuid());
		transceiver = CreateAndAddTransceiver(sender, receiver);
		transceiver->internal()->set_created_by_addtrack(true);
		transceiver->internal()->set_direction(RtpTransceiverDirection::kSendRecv);
	}
	return transceiver->sender();
}

rtc::scoped_refptr<RTCRtpSenderProxyWithInternal<RTCRtpSenderInternal>>
RTCRtpTransmissionManager::CreateSender(
	cricket::MediaType media_type,
	const std::string& id,
	rtc::scoped_refptr<MediaStreamTrackInterface> track,
	const std::vector<std::string>& stream_ids,
	const std::vector<RtpEncodingParameters>& send_encodings) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	rtc::scoped_refptr<RTCRtpSenderProxyWithInternal<RTCRtpSenderInternal>> sender;
	if (media_type == cricket::MEDIA_TYPE_AUDIO) {
		RTC_DCHECK(!track ||
			(track->kind() == MediaStreamTrackInterface::kAudioKind));
		sender = RTCRtpSenderProxyWithInternal<RTCRtpSenderInternal>::Create(
			signaling_thread(),
			RTCAudioRtpSender::Create(worker_thread(), id, this));
	}
	else {
		RTC_DCHECK_EQ(media_type, cricket::MEDIA_TYPE_VIDEO);
		RTC_DCHECK(!track ||
			(track->kind() == MediaStreamTrackInterface::kVideoKind));
		sender = RTCRtpSenderProxyWithInternal<RTCRtpSenderInternal>::Create(
			signaling_thread(), RTCVideoRtpSender::Create(worker_thread(), id, this));
	}
	bool set_track_succeeded = sender->SetTrack(track);
	RTC_DCHECK(set_track_succeeded);
	sender->internal()->set_stream_ids(stream_ids);
	sender->internal()->set_init_send_encodings(send_encodings);

	return sender;
}

rtc::scoped_refptr<RTCRtpReceiverProxyWithInternal<RTCRtpReceiverInternal>>
RTCRtpTransmissionManager::CreateReceiver(cricket::MediaType media_type,
										  const std::string& receiver_id) {
	RTC_DCHECK_RUN_ON(signaling_thread());

	rtc::scoped_refptr<RTCRtpReceiverProxyWithInternal<RTCRtpReceiverInternal>>
		receiver;
	if (media_type == cricket::MEDIA_TYPE_AUDIO) {
		receiver = RTCRtpReceiverProxyWithInternal<RTCRtpReceiverInternal>::Create(
			signaling_thread(), worker_thread(),
			rtc::make_ref_counted<RTCAudioRtpReceiver>(worker_thread(), receiver_id,
			std::vector<std::string>({}),
			_rtc_thread_manager
			));
	}
	else {
		RTC_DCHECK_EQ(media_type, cricket::MEDIA_TYPE_VIDEO);
		receiver = RTCRtpReceiverProxyWithInternal<RTCRtpReceiverInternal>::Create(
			signaling_thread(), worker_thread(),
			rtc::make_ref_counted<RTCVideoRtpReceiver>(worker_thread(), receiver_id,
			std::vector<std::string>({}),
			_rtc_thread_manager
			));
	}
	return receiver;
}

rtc::scoped_refptr<RTCRtpTransceiverProxyWithInternal<RTCRtpTransceiver>>
RTCRtpTransmissionManager::CreateAndAddTransceiver(
	rtc::scoped_refptr<RTCRtpSenderProxyWithInternal<RTCRtpSenderInternal>> sender,
	rtc::scoped_refptr<RTCRtpReceiverProxyWithInternal<RTCRtpReceiverInternal>>
	receiver) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	// Ensure that the new sender does not have an ID that is already in use by
	// another sender.
	// Allow receiver IDs to conflict since those come from remote SDP (which
	// could be invalid, but should not cause a crash).
	RTC_DCHECK(!FindSenderById(sender->id()));

	auto transceiver = RTCRtpTransceiverProxyWithInternal<RTCRtpTransceiver>::Create(
		signaling_thread(),
		rtc::make_ref_counted<RTCRtpTransceiver>(
		sender, receiver, channel_manager(),
		sender->media_type() == cricket::MEDIA_TYPE_AUDIO
		? channel_manager()->GetSupportedAudioRtpHeaderExtensions()
		: channel_manager()->GetSupportedVideoRtpHeaderExtensions(),
		[this_weak_ptr = weak_ptr_factory_.GetWeakPtr()]() {
		if (this_weak_ptr) {
			this_weak_ptr->OnNegotiationNeeded();
		}
	}, _rtc_thread_manager	));

	transceivers()->Add(transceiver);
	return transceiver;
}

rtc::scoped_refptr<RTCRtpTransceiverProxyWithInternal<RTCRtpTransceiver>>
RTCRtpTransmissionManager::FindFirstTransceiverForAddedTrack(
	rtc::scoped_refptr<MediaStreamTrackInterface> track) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	RTC_DCHECK(track);
	for (auto transceiver : transceivers()->List()) {
		if (!transceiver->sender()->track() &&
			cricket::MediaTypeToString(transceiver->media_type()) ==
			track->kind() &&
			!transceiver->internal()->has_ever_been_used_to_send() &&
			!transceiver->stopped()) {
			return transceiver;
		}
	}
	return nullptr;
}

std::vector<rtc::scoped_refptr<RTCRtpSenderProxyWithInternal<RTCRtpSenderInternal>>>
RTCRtpTransmissionManager::GetSendersInternal() const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	std::vector<rtc::scoped_refptr<RTCRtpSenderProxyWithInternal<RTCRtpSenderInternal>>>
		all_senders;
	for (const auto& transceiver : transceivers_.List()) {
		if (IsUnifiedPlan() && transceiver->internal()->stopped())
			continue;

		auto senders = transceiver->internal()->senders();
		all_senders.insert(all_senders.end(), senders.begin(), senders.end());
	}
	return all_senders;
}

std::vector<rtc::scoped_refptr<RTCRtpReceiverProxyWithInternal<RTCRtpReceiverInternal>>>
RTCRtpTransmissionManager::GetReceiversInternal() const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	std::vector<
		rtc::scoped_refptr<RTCRtpReceiverProxyWithInternal<RTCRtpReceiverInternal>>>
		all_receivers;
	for (const auto& transceiver : transceivers_.List()) {
		if (IsUnifiedPlan() && transceiver->internal()->stopped())
			continue;

		auto receivers = transceiver->internal()->receivers();
		all_receivers.insert(all_receivers.end(), receivers.begin(),
			receivers.end());
	}
	return all_receivers;
}

rtc::scoped_refptr<RTCRtpTransceiverProxyWithInternal<RTCRtpTransceiver>>
RTCRtpTransmissionManager::GetAudioTransceiver() const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	// This method only works with Plan B SDP, where there is a single
	// audio/video transceiver.
	RTC_DCHECK(!IsUnifiedPlan());
	for (auto transceiver : transceivers_.List()) {
		if (transceiver->media_type() == cricket::MEDIA_TYPE_AUDIO) {
			return transceiver;
		}
	}
	RTC_NOTREACHED();
	return nullptr;
}

rtc::scoped_refptr<RTCRtpTransceiverProxyWithInternal<RTCRtpTransceiver>>
RTCRtpTransmissionManager::GetVideoTransceiver() const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	// This method only works with Plan B SDP, where there is a single
	// audio/video transceiver.
	RTC_DCHECK(!IsUnifiedPlan());
	for (auto transceiver : transceivers_.List()) {
		if (transceiver->media_type() == cricket::MEDIA_TYPE_VIDEO) {
			return transceiver;
		}
	}
	RTC_NOTREACHED();
	return nullptr;
}

void RTCRtpTransmissionManager::AddAudioTrack(AudioTrackInterface* track,
											  MediaStreamInterface* stream) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	RTC_DCHECK(track);
	RTC_DCHECK(stream);
	auto sender = FindSenderForTrack(track);
	if (sender) {
		// We already have a sender for this track, so just change the stream_id
		// so that it's correct in the next call to CreateOffer.
		sender->internal()->set_stream_ids({ stream->id() });
		return;
	}

	// Normal case; we've never seen this track before.
	auto new_sender = CreateSender(cricket::MEDIA_TYPE_AUDIO, track->id(), track,
	{ stream->id() }, {});
	new_sender->internal()->SetMediaChannel(voice_media_channel());
	GetAudioTransceiver()->internal()->AddSender(new_sender);
	// If the sender has already been configured in SDP, we call SetSsrc,
	// which will connect the sender to the underlying transport. This can
	// occur if a local session description that contains the ID of the sender
	// is set before AddStream is called. It can also occur if the local
	// session description is not changed and RemoveStream is called, and
	// later AddStream is called again with the same stream.
	const RTCRtpSenderInfo* sender_info =
		FindSenderInfo(local_audio_sender_infos_, stream->id(), track->id());
	if (sender_info) {
		new_sender->internal()->SetSsrc(sender_info->first_ssrc);
	}
}

// TODO(deadbeef): Don't destroy RtpSenders here; they should be kept around
// indefinitely, when we have unified plan SDP.
void RTCRtpTransmissionManager::RemoveAudioTrack(AudioTrackInterface* track,
												 MediaStreamInterface* stream) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	RTC_DCHECK(!IsUnifiedPlan());
	auto sender = FindSenderForTrack(track);
	if (!sender) {
		RTC_LOG(LS_WARNING) << "RtpSender for track with id " << track->id()
			<< " doesn't exist.";
		return;
	}
	GetAudioTransceiver()->internal()->RemoveSender(sender);
}

void RTCRtpTransmissionManager::AddVideoTrack(VideoTrackInterface* track,
											  MediaStreamInterface* stream) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	RTC_DCHECK(track);
	RTC_DCHECK(stream);
	auto sender = FindSenderForTrack(track);
	if (sender) {
		// We already have a sender for this track, so just change the stream_id
		// so that it's correct in the next call to CreateOffer.
		sender->internal()->set_stream_ids({ stream->id() });
		return;
	}

	// Normal case; we've never seen this track before.
	auto new_sender = CreateSender(cricket::MEDIA_TYPE_VIDEO, track->id(), track,
	{ stream->id() }, {});
	new_sender->internal()->SetMediaChannel(video_media_channel());
	GetVideoTransceiver()->internal()->AddSender(new_sender);
	const RTCRtpSenderInfo* sender_info =
		FindSenderInfo(local_video_sender_infos_, stream->id(), track->id());
	if (sender_info) {
		new_sender->internal()->SetSsrc(sender_info->first_ssrc);
	}
}

void RTCRtpTransmissionManager::RemoveVideoTrack(VideoTrackInterface* track,
	MediaStreamInterface* stream) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	RTC_DCHECK(!IsUnifiedPlan());
	auto sender = FindSenderForTrack(track);
	if (!sender) {
		RTC_LOG(LS_WARNING) << "RtpSender for track with id " << track->id()
			<< " doesn't exist.";
		return;
	}
	GetVideoTransceiver()->internal()->RemoveSender(sender);
}

void RTCRtpTransmissionManager::CreateAudioReceiver(
								MediaStreamInterface* stream,
								const RTCRtpSenderInfo& remote_sender_info) {
	RTC_DCHECK(!closed_);
	std::vector<rtc::scoped_refptr<MediaStreamInterface>> streams;
	streams.push_back(rtc::scoped_refptr<MediaStreamInterface>(stream));
	// TODO(https://crbug.com/webrtc/9480): When we remove remote_streams(), use
	// the constructor taking stream IDs instead.
	auto audio_receiver = rtc::make_ref_counted<RTCAudioRtpReceiver>(
		worker_thread(), remote_sender_info.sender_id, streams, _rtc_thread_manager);
	audio_receiver->SetMediaChannel(voice_media_channel());
	if (remote_sender_info.sender_id == kDefaultAudioSenderId) {
		audio_receiver->SetupUnsignaledMediaChannel();
	}
	else {
		audio_receiver->SetupMediaChannel(remote_sender_info.first_ssrc);
	}
	auto receiver = RTCRtpReceiverProxyWithInternal<RTCRtpReceiverInternal>::Create(
		signaling_thread(), worker_thread(), std::move(audio_receiver));
	GetAudioTransceiver()->internal()->AddReceiver(receiver);
	Observer()->OnAddTrack(receiver, streams);
}

void RTCRtpTransmissionManager::CreateVideoReceiver(
	MediaStreamInterface* stream,
	const RTCRtpSenderInfo& remote_sender_info) {
	RTC_DCHECK(!closed_);
	std::vector<rtc::scoped_refptr<MediaStreamInterface>> streams;
	streams.push_back(rtc::scoped_refptr<MediaStreamInterface>(stream));
	// TODO(https://crbug.com/webrtc/9480): When we remove remote_streams(), use
	// the constructor taking stream IDs instead.

	if (remote_sender_info.ssrcs.empty()) {
		auto video_receiver = rtc::make_ref_counted<RTCVideoRtpReceiver>(
			worker_thread(), remote_sender_info.sender_id, streams, _rtc_thread_manager);
		video_receiver->SetMediaChannel(video_media_channel());
		if (remote_sender_info.sender_id == kDefaultVideoSenderId) {
			video_receiver->SetupUnsignaledMediaChannel();
		}
		else {
			video_receiver->SetupMediaChannel(remote_sender_info.first_ssrc);
		}

		auto receiver = RTCRtpReceiverProxyWithInternal<RTCRtpReceiverInternal>::Create(
			signaling_thread(), worker_thread(), std::move(video_receiver));
		GetVideoTransceiver()->internal()->AddReceiver(receiver);
		Observer()->OnAddTrack(receiver, streams);
	}
	else {
		std::vector<rtc::scoped_refptr<RTCRtpReceiverInterface>> receiverVec;
		for (uint32_t ssrc : remote_sender_info.ssrcs) {
			auto video_receiver = rtc::make_ref_counted<RTCVideoRtpReceiver>(
				worker_thread(), remote_sender_info.sender_id, streams, _rtc_thread_manager);
			video_receiver->SetMediaChannel(video_media_channel());
			video_receiver->SetupMediaChannel(ssrc);

			auto receiver = RTCRtpReceiverProxyWithInternal<RTCRtpReceiverInternal>::Create(
				signaling_thread(), worker_thread(), std::move(video_receiver));
			GetVideoTransceiver()->internal()->AddReceiver(receiver);
			receiverVec.push_back(receiver);
		}

		Observer()->OnAddSimulcastTrack(receiverVec, streams);
	}
}

// TODO(deadbeef): Keep RtpReceivers around even if track goes away in remote
// description.
rtc::scoped_refptr<RTCRtpReceiverInterface>
RTCRtpTransmissionManager::RemoveAndStopReceiver(
	const RTCRtpSenderInfo& remote_sender_info) {
	auto receiver = FindReceiverById(remote_sender_info.sender_id);
	if (!receiver) {
		RTC_LOG(LS_WARNING) << "RtpReceiver for track with id "
			<< remote_sender_info.sender_id << " doesn't exist.";
		return nullptr;
	}
	if (receiver->media_type() == cricket::MEDIA_TYPE_AUDIO) {
		GetAudioTransceiver()->internal()->RemoveReceiver(receiver);
	}
	else {
		GetVideoTransceiver()->internal()->RemoveReceiver(receiver);
	}
	return receiver;
}

std::vector<rtc::scoped_refptr<RTCRtpReceiverInterface>>
RTCRtpTransmissionManager::RemoveAndStopReceivers(
	const RTCRtpSenderInfo& remote_sender_info) {
	
	std::vector<rtc::scoped_refptr<RTCRtpReceiverInterface>> receivers;
	for (uint32_t ssrc : remote_sender_info.ssrcs) {
		auto receiver = FindReceiverBySsrc(ssrc);
		if (receiver != nullptr) {
			GetVideoTransceiver()->internal()->RemoveReceiver(receiver);
		}
		receivers.push_back(receiver);
	}
	return receivers;
}

void RTCRtpTransmissionManager::OnRemoteSenderAdded(
	const RTCRtpSenderInfo& sender_info,			
	MediaStreamInterface* stream,
	cricket::MediaType media_type) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	RTC_LOG(LS_INFO) << "Creating " << cricket::MediaTypeToString(media_type)
		<< " receiver for track_id=" << sender_info.sender_id
		<< " and stream_id=" << sender_info.stream_id;

	if (media_type == cricket::MEDIA_TYPE_AUDIO) {
		CreateAudioReceiver(stream, sender_info);
	}
	else if (media_type == cricket::MEDIA_TYPE_VIDEO) {
		CreateVideoReceiver(stream, sender_info);
	}
	else {
		RTC_NOTREACHED() << "Invalid media type";
	}
}

void RTCRtpTransmissionManager::OnRemoteSenderRemoved(
	const RTCRtpSenderInfo& sender_info,
	MediaStreamInterface* stream,
	cricket::MediaType media_type) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	RTC_LOG(LS_INFO) << "Removing " << cricket::MediaTypeToString(media_type)
		<< " receiver for track_id=" << sender_info.sender_id
		<< " and stream_id=" << sender_info.stream_id;

	if (media_type == cricket::MEDIA_TYPE_AUDIO) {
		// When the MediaEngine audio channel is destroyed, the RemoteAudioSource
		// will be notified which will end the AudioRtpReceiver::track().
		rtc::scoped_refptr<RTCRtpReceiverInterface> receiver;
		receiver = RemoveAndStopReceiver(sender_info);
		rtc::scoped_refptr<AudioTrackInterface> audio_track =
			stream->FindAudioTrack(sender_info.sender_id);
		if (audio_track) {
			stream->RemoveTrack(audio_track);
		}

		if (receiver) {
			RTC_DCHECK(!closed_);
			Observer()->OnRemoveTrack(receiver);
		}
	}
	else if (media_type == cricket::MEDIA_TYPE_VIDEO) {
		// Stopping or destroying a VideoRtpReceiver will end the
		// VideoRtpReceiver::track().
		if (sender_info.ssrcs.empty()) {	
			rtc::scoped_refptr<RTCRtpReceiverInterface> receiver;
			receiver = RemoveAndStopReceiver(sender_info);
			rtc::scoped_refptr<VideoTrackInterface> video_track =
				stream->FindVideoTrack(sender_info.sender_id);
			if (video_track) {
				// There's no guarantee the track is still available, e.g. the track may
				// have been removed from the stream by an application.
				stream->RemoveTrack(video_track);
			}

			if (receiver) {
				RTC_DCHECK(!closed_);
				Observer()->OnRemoveTrack(receiver);
			}
		}
		else {
			std::vector < rtc::scoped_refptr<RTCRtpReceiverInterface>> receivers;
			receivers = RemoveAndStopReceivers(sender_info);
			rtc::scoped_refptr<VideoTrackInterface> video_track =
				stream->FindVideoTrack(sender_info.sender_id);
			if (video_track) {
				// There's no guarantee the track is still available, e.g. the track may
				// have been removed from the stream by an application.
				stream->RemoveTrack(video_track);
			}

			if (!receivers.empty()) {
			}
		}
	}
	else {
		RTC_NOTREACHED() << "Invalid media type";
	}
}

void RTCRtpTransmissionManager::OnLocalSenderAdded(
	const RTCRtpSenderInfo& sender_info,
	cricket::MediaType media_type) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	RTC_DCHECK(!IsUnifiedPlan());
	auto sender = FindSenderById(sender_info.sender_id);
	if (!sender) {
		RTC_LOG(LS_WARNING) << "An unknown RtpSender with id "
			<< sender_info.sender_id
			<< " has been configured in the local description.";
		return;
	}

	if (sender->media_type() != media_type) {
		RTC_LOG(LS_WARNING) << "An RtpSender has been configured in the local"
			" description with an unexpected media type.";
		return;
	}

	sender->internal()->set_stream_ids({ sender_info.stream_id });
	sender->internal()->SetSsrc(sender_info.first_ssrc);
}

void RTCRtpTransmissionManager::OnLocalSenderRemoved(
	const RTCRtpSenderInfo& sender_info,
	cricket::MediaType media_type) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	auto sender = FindSenderById(sender_info.sender_id);
	if (!sender) {
		// This is the normal case. I.e., RemoveStream has been called and the
		// SessionDescriptions has been renegotiated.
		return;
	}

	// A sender has been removed from the SessionDescription but it's still
	// associated with the PeerConnection. This only occurs if the SDP doesn't
	// match with the calls to CreateSender, AddStream and RemoveStream.
	if (sender->media_type() != media_type) {
		RTC_LOG(LS_WARNING) << "An RtpSender has been configured in the local"
			" description with an unexpected media type.";
		return;
	}

	sender->internal()->SetSsrc(0);
}

std::vector<RTCRtpSenderInfo>* RTCRtpTransmissionManager::GetRemoteSenderInfos(
	cricket::MediaType media_type) {
	RTC_DCHECK(media_type == cricket::MEDIA_TYPE_AUDIO ||
		media_type == cricket::MEDIA_TYPE_VIDEO);
	return (media_type == cricket::MEDIA_TYPE_AUDIO)
		? &remote_audio_sender_infos_
		: &remote_video_sender_infos_;
}

std::vector<RTCRtpSenderInfo>* RTCRtpTransmissionManager::GetLocalSenderInfos(
	cricket::MediaType media_type) {
	RTC_DCHECK(media_type == cricket::MEDIA_TYPE_AUDIO ||
		media_type == cricket::MEDIA_TYPE_VIDEO);
	return (media_type == cricket::MEDIA_TYPE_AUDIO) ? &local_audio_sender_infos_
		: &local_video_sender_infos_;
}

const RTCRtpSenderInfo* RTCRtpTransmissionManager::FindSenderInfo(
	const std::vector<RTCRtpSenderInfo>& infos,
	const std::string& stream_id,
	const std::string sender_id) const {

	for (const RTCRtpSenderInfo& sender_info : infos) {
		if (sender_info.stream_id == stream_id &&
			sender_info.sender_id == sender_id) {
			return &sender_info;
		}
	}
	return nullptr;
}

rtc::scoped_refptr<RTCRtpSenderProxyWithInternal<RTCRtpSenderInternal>>
RTCRtpTransmissionManager::FindSenderForTrack(
	MediaStreamTrackInterface* track) const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	for (const auto& transceiver : transceivers_.List()) {
		for (auto sender : transceiver->internal()->senders()) {
			if (sender->track() == track) {
				return sender;
			}
		}
	}
	return nullptr;
}

rtc::scoped_refptr<RTCRtpSenderProxyWithInternal<RTCRtpSenderInternal>>
RTCRtpTransmissionManager::FindSenderById(const std::string& sender_id) const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	for (const auto& transceiver : transceivers_.List()) {
		for (auto sender : transceiver->internal()->senders()) {
			if (sender->id() == sender_id) {
				return sender;
			}
		}
	}
	return nullptr;
}

rtc::scoped_refptr<RTCRtpReceiverProxyWithInternal<RTCRtpReceiverInternal>>
RTCRtpTransmissionManager::FindReceiverById(const std::string& receiver_id) const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	for (const auto& transceiver : transceivers_.List()) {
		for (auto receiver : transceiver->internal()->receivers()) {
			if (receiver->id() == receiver_id) {
				return receiver;
			}
		}
	}
	return nullptr;
}

rtc::scoped_refptr<RTCRtpReceiverProxyWithInternal<RTCRtpReceiverInternal>>
RTCRtpTransmissionManager::FindReceiverBySsrc(uint32_t ssrc) const {
	RTC_DCHECK_RUN_ON(signaling_thread());
	for (const auto& transceiver : transceivers_.List()) {
		for (auto receiver : transceiver->internal()->receivers()) {
			if (receiver->ssrc() == ssrc) {
				return receiver;
			}
		}
	}
	return nullptr;
}

}  // namespace webrtc
