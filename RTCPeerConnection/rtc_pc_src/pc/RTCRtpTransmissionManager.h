//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /pc/rtp_transmission_manager.h
//
//////////////////////////////////////////////////////////////////////////


#ifndef __RTC_PC_RTP_TRANSMISSION_MANAGER_H__
#define __RTC_PC_RTP_TRANSMISSION_MANAGER_H__

#include <stdint.h>
#include <functional>
#include <string>
#include <vector>

#include "../rtc_pc_src/_deprecate_defines.h"

#include "api/media_types.h"
#include "api/rtc_error.h"
#include "api/rtp_parameters.h"
#include "api/scoped_refptr.h"
#include "rtc_base/synchronization/sequence_checker.h"
#include "rtc_base/third_party/sigslot/sigslot.h"
#include "rtc_base/thread.h"
#include "rtc_base/thread_annotations.h"

#include "../../src_update/api/_media_stream_interface.h"
#include "../../src_update/media/base/_media_channel.h"
#include "../api/RTCConnectionInterface.h"
#include "../api/RTCRtpReceiverInterface.h"
#include "../api/RTCRtpSenderInterface.h"
#include "RTCChannelManager.h"
#include "RTCRtpReceiver.h"
#include "RTCRtpSender.h"
#include "RTCRtpTransceiver.h"
#include "RTCTransceiverList.h"

#include "../rtc_pc_src/RTCThreadManagerInterface.h"

namespace webrtc {

// This class contains information about
// an RTPSender, used for things like looking it up by SSRC.
struct RTCRtpSenderInfo {
	RTCRtpSenderInfo() : first_ssrc(0) {}
	RTCRtpSenderInfo(const std::string& stream_id,
		const std::string sender_id,
		uint32_t ssrc)
		: stream_id(stream_id), sender_id(sender_id), first_ssrc(ssrc) {}
	bool operator==(const RTCRtpSenderInfo& other) {
		return this->stream_id == other.stream_id &&
			this->sender_id == other.sender_id &&
			this->first_ssrc == other.first_ssrc;
	}
	std::string stream_id;
	std::string sender_id;
	// An RtpSender can have many SSRCs. The first one is used as a sort of ID
	// for communicating with the lower layers.
	uint32_t first_ssrc;

	std::vector<uint32_t> ssrcs;	
};

// The RtpTransmissionManager class is responsible for managing the lifetime
// and relationships between objects of type RtpSender, RtpReceiver and
// RtpTransceiver.
class RTCRtpTransmissionManager : public RTCRtpSenderBase::SetStreamsObserver {
public:
	RTCRtpTransmissionManager(bool is_unified_plan,
		rtc::Thread* signaling_thread,
		rtc::Thread* worker_thread,
		cricket::RTCChannelManager* channel_manager,
		RTCConnectionObserver* observer,
		std::function<void()> on_negotiation_needed,
		rtc::RTCThreadManagerInterface* rtc_thread_manager
		);

	// No move or copy permitted.
	RTCRtpTransmissionManager(const RTCRtpTransmissionManager&) = delete;
	RTCRtpTransmissionManager& operator=(const RTCRtpTransmissionManager&) = delete;

	// Stop activity. In particular, don't call observer_ any more.
	void Close();

	// RtpSenderBase::SetStreamsObserver override.
	void OnSetStreams() override;

	// Add a new track, creating transceiver if required.
	RTCErrorOr<rtc::scoped_refptr<RTCRtpSenderInterface>> AddTrack(
		rtc::scoped_refptr<MediaStreamTrackInterface> track,
		const std::vector<std::string>& stream_ids);

	// Create a new RTP sender. Does not associate with a transceiver.
	rtc::scoped_refptr<RTCRtpSenderProxyWithInternal<RTCRtpSenderInternal>>
		CreateSender(cricket::MediaType media_type,
		const std::string& id,
		rtc::scoped_refptr<MediaStreamTrackInterface> track,
		const std::vector<std::string>& stream_ids,
		const std::vector<RtpEncodingParameters>& send_encodings);

	// Create a new RTP receiver. Does not associate with a transceiver.
	rtc::scoped_refptr<RTCRtpReceiverProxyWithInternal<RTCRtpReceiverInternal>>
		CreateReceiver(cricket::MediaType media_type, const std::string& receiver_id);

	// Create a new RtpTransceiver of the given type and add it to the list of
	// registered transceivers.		// only unified plan
	rtc::scoped_refptr<RTCRtpTransceiverProxyWithInternal<RTCRtpTransceiver>>
	CreateAndAddTransceiver(
		rtc::scoped_refptr<RTCRtpSenderProxyWithInternal<RTCRtpSenderInternal>> sender,
		rtc::scoped_refptr<RTCRtpReceiverProxyWithInternal<RTCRtpReceiverInternal>>
		receiver);


	// Returns the first RtpTransceiver suitable for a newly added track, if such
	// transceiver is available.
	rtc::scoped_refptr<RTCRtpTransceiverProxyWithInternal<RTCRtpTransceiver>>
	FindFirstTransceiverForAddedTrack(
		rtc::scoped_refptr<MediaStreamTrackInterface> track);

	// Returns the list of senders currently associated with some
	// registered transceiver
	std::vector<rtc::scoped_refptr<RTCRtpSenderProxyWithInternal<RTCRtpSenderInternal>>>
	GetSendersInternal() const;

	// Returns the list of receivers currently associated with a transceiver
	std::vector<rtc::scoped_refptr<RTCRtpReceiverProxyWithInternal<RTCRtpReceiverInternal>>>
	GetReceiversInternal() const;

	// Plan B: Get the transceiver containing all audio senders and receivers
	rtc::scoped_refptr<RTCRtpTransceiverProxyWithInternal<RTCRtpTransceiver>>
		GetAudioTransceiver() const;
	// Plan B: Get the transceiver containing all video senders and receivers
	rtc::scoped_refptr<RTCRtpTransceiverProxyWithInternal<RTCRtpTransceiver>>
		GetVideoTransceiver() const;

	// Add an audio track, reusing or creating the sender.
	void AddAudioTrack(AudioTrackInterface* track, MediaStreamInterface* stream);
	// Plan B: Remove an audio track, removing the sender.
	void RemoveAudioTrack(AudioTrackInterface* track,
		MediaStreamInterface* stream);
	// Add a video track, reusing or creating the sender.
	void AddVideoTrack(VideoTrackInterface* track, MediaStreamInterface* stream);
	// Plan B: Remove a video track, removing the sender.
	void RemoveVideoTrack(VideoTrackInterface* track,
		MediaStreamInterface* stream);

	// Triggered when a remote sender has been seen for the first time in a remote
	// session description. It creates a remote MediaStreamTrackInterface
	// implementation and triggers CreateAudioReceiver or CreateVideoReceiver.
	void OnRemoteSenderAdded(const RTCRtpSenderInfo& sender_info,
		MediaStreamInterface* stream,
		cricket::MediaType media_type);

	// Triggered when a remote sender has been removed from a remote session
	// description. It removes the remote sender with id |sender_id| from a remote
	// MediaStream and triggers DestroyAudioReceiver or DestroyVideoReceiver.
	void OnRemoteSenderRemoved(const RTCRtpSenderInfo& sender_info,
		MediaStreamInterface* stream,
		cricket::MediaType media_type);

	// Triggered when a local sender has been seen for the first time in a local
	// session description.
	// This method triggers CreateAudioSender or CreateVideoSender if the rtp
	// streams in the local SessionDescription can be mapped to a MediaStreamTrack
	// in a MediaStream in |local_streams_|
	void OnLocalSenderAdded(const RTCRtpSenderInfo& sender_info,
		cricket::MediaType media_type);

	// Triggered when a local sender has been removed from a local session
	// description.
	// This method triggers DestroyAudioSender or DestroyVideoSender if a stream
	// has been removed from the local SessionDescription and the stream can be
	// mapped to a MediaStreamTrack in a MediaStream in |local_streams_|.
	void OnLocalSenderRemoved(const RTCRtpSenderInfo& sender_info,
		cricket::MediaType media_type);

	std::vector<RTCRtpSenderInfo>* GetRemoteSenderInfos(
		cricket::MediaType media_type);
	std::vector<RTCRtpSenderInfo>* GetLocalSenderInfos(
		cricket::MediaType media_type);
	const RTCRtpSenderInfo* FindSenderInfo(const std::vector<RTCRtpSenderInfo>& infos,
		const std::string& stream_id,
		const std::string sender_id) const;

	// Return the RtpSender with the given track attached.
	rtc::scoped_refptr<RTCRtpSenderProxyWithInternal<RTCRtpSenderInternal>>
		FindSenderForTrack(MediaStreamTrackInterface* track) const;

	// Return the RtpSender with the given id, or null if none exists.
	rtc::scoped_refptr<RTCRtpSenderProxyWithInternal<RTCRtpSenderInternal>>
		FindSenderById(const std::string& sender_id) const;

	// Return the RtpReceiver with the given id, or null if none exists.
	rtc::scoped_refptr<RTCRtpReceiverProxyWithInternal<RTCRtpReceiverInternal>>
		FindReceiverById(const std::string& receiver_id) const;

	rtc::scoped_refptr<RTCRtpReceiverProxyWithInternal<RTCRtpReceiverInternal>>
		FindReceiverBySsrc(uint32_t ssrc) const;	

	RTCTransceiverList* transceivers() { return &transceivers_; }
	const RTCTransceiverList* transceivers() const { return &transceivers_; }

	// Plan B helpers for getting the voice/video media channels for the single
	// audio/video transceiver, if it exists.
	cricket::VoiceMediaChannel* voice_media_channel() const;
	cricket::VideoMediaChannel* video_media_channel() const;

private:
	rtc::Thread* signaling_thread() const { return signaling_thread_; }
	rtc::Thread* worker_thread() const { return worker_thread_; }
	cricket::RTCChannelManager* channel_manager() const { return channel_manager_; }
	bool IsUnifiedPlan() const { return is_unified_plan_; }

	// AddTrack implementation when Unified Plan is specified.
	RTCErrorOr<rtc::scoped_refptr<RTCRtpSenderInterface>> AddTrackUnifiedPlan(
		rtc::scoped_refptr<MediaStreamTrackInterface> track,
		const std::vector<std::string>& stream_ids);
	
	// AddTrack implementation when Plan B is specified.
	RTCErrorOr<rtc::scoped_refptr<RTCRtpSenderInterface>> AddTrackPlanB(
		rtc::scoped_refptr<MediaStreamTrackInterface> track,
		const std::vector<std::string>& stream_ids);

	// Create an RtpReceiver that sources an audio track.
	void CreateAudioReceiver(MediaStreamInterface* stream,
		const RTCRtpSenderInfo& remote_sender_info)
		RTC_RUN_ON(signaling_thread());

	// Create an RtpReceiver that sources a video track.
	void CreateVideoReceiver(MediaStreamInterface* stream,
		const RTCRtpSenderInfo& remote_sender_info)
		RTC_RUN_ON(signaling_thread());

	rtc::scoped_refptr<RTCRtpReceiverInterface> RemoveAndStopReceiver(
		const RTCRtpSenderInfo& remote_sender_info) RTC_RUN_ON(signaling_thread());

	std::vector<rtc::scoped_refptr<RTCRtpReceiverInterface>> RemoveAndStopReceivers(
		const RTCRtpSenderInfo& remote_sender_info) RTC_RUN_ON(signaling_thread());	

	RTCConnectionObserver* Observer() const;
	void OnNegotiationNeeded();

	RTCTransceiverList transceivers_;

	// These lists store sender info seen in local/remote descriptions.
	std::vector<RTCRtpSenderInfo> remote_audio_sender_infos_
		RTC_GUARDED_BY(signaling_thread());
	std::vector<RTCRtpSenderInfo> remote_video_sender_infos_
		RTC_GUARDED_BY(signaling_thread());
	std::vector<RTCRtpSenderInfo> local_audio_sender_infos_
		RTC_GUARDED_BY(signaling_thread());
	std::vector<RTCRtpSenderInfo> local_video_sender_infos_
		RTC_GUARDED_BY(signaling_thread());

	bool closed_ = false;
	bool const is_unified_plan_;
	rtc::Thread* signaling_thread_;
	rtc::Thread* worker_thread_;
	cricket::RTCChannelManager* channel_manager_;
	RTCConnectionObserver* observer_;
	std::function<void()> on_negotiation_needed_;
	rtc::WeakPtrFactory<RTCRtpTransmissionManager> weak_ptr_factory_
		RTC_GUARDED_BY(signaling_thread());

	rtc::RTCThreadManagerInterface* _rtc_thread_manager;
};

}  // namespace webrtc

#endif	// __RTC_PC_RTP_TRANSMISSION_MANAGER_H__