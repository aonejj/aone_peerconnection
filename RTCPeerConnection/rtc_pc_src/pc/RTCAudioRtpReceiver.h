//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /pc/audio_rtp_receiver.h
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_PC_AUDIO_RTP_RECEIVER_H__
#define __RTC_PC_AUDIO_RTP_RECEIVER_H__

#include <stdint.h>

#include <string>
#include <vector>

#include "../rtc_pc_src/_deprecate_defines.h"

#include "absl/types/optional.h"

#include "api/media_types.h"
#include "api/rtp_parameters.h"
#include "api/scoped_refptr.h"
#include "pc/audio_track.h"
#include "pc/remote_audio_source.h"
#include "rtc_base/ref_counted_object.h"
#include "rtc_base/thread.h"
#include "rtc_base/task_utils/pending_task_safety_flag.h"

#include "../../src_update/api/_media_stream_interface.h"
#include "../../src_update/api/_media_stream_track_proxy.h"
#include "../../src_update/media/base/_media_channel.h"
#include "RTCRtpReceiver.h"

#include "../rtc_pc_src/RTCThreadManagerInterface.h"

namespace webrtc {

class RTCAudioRtpReceiver : public ObserverInterface,
							public AudioSourceInterface::AudioObserver,
							public RTCRtpReceiverInternal {
public:
	RTCAudioRtpReceiver(rtc::Thread* worker_thread,
						std::string receiver_id,
						std::vector<std::string> stream_ids,
						rtc::RTCThreadManagerInterface* rtc_thread_manager
						);
	// TODO(https://crbug.com/webrtc/9480): Remove this when streams() is removed.
	RTCAudioRtpReceiver(
		rtc::Thread* worker_thread,
		const std::string& receiver_id,
		const std::vector<rtc::scoped_refptr<MediaStreamInterface>>& streams,
		rtc::RTCThreadManagerInterface* rtc_thread_manager
		);
	virtual ~RTCAudioRtpReceiver();

	// ObserverInterface implementation
	void OnChanged() override;

	// AudioSourceInterface::AudioObserver implementation
	void OnSetVolume(double volume) override;

	rtc::scoped_refptr<AudioTrackInterface> audio_track() const {
		return track_.get();
	}

	// RtpReceiverInterface implementation
	rtc::scoped_refptr<MediaStreamTrackInterface> track() const override {
		return track_.get();
	}
	rtc::scoped_refptr<DtlsTransportInterface> dtls_transport() const override {
		return dtls_transport_;
	}
	std::vector<std::string> stream_ids() const override;
	std::vector<rtc::scoped_refptr<MediaStreamInterface>> streams()
		const override {
		return streams_;
	}

	cricket::MediaType media_type() const override {
		return cricket::MEDIA_TYPE_AUDIO;
	}

	std::string id() const override { return id_; }

	RtpParameters GetParameters() const override;

	// RtpReceiverInternal implementation.
	void Stop() override;
	void StopAndEndTrack() override;
	void SetupMediaChannel(uint32_t ssrc) override;
	void SetupUnsignaledMediaChannel() override;
	uint32_t ssrc() const override { return ssrc_.value_or(0); }
	void NotifyFirstPacketReceived() override;
	void set_stream_ids(std::vector<std::string> stream_ids) override;
	void set_transport(
		rtc::scoped_refptr<DtlsTransportInterface> dtls_transport) override {
		dtls_transport_ = dtls_transport;
	}
	void SetStreams(const std::vector<rtc::scoped_refptr<MediaStreamInterface>>&
		streams) override;
	void SetObserver(RTCRtpReceiverObserverInterface* observer) override;

	void SetJitterBufferMinimumDelay(
				absl::optional<double> delay_seconds) override;

	void SetMediaChannel(cricket::MediaChannel* media_channel) override;

	std::vector<RtpSource> GetSources() const override;
	int AttachmentId() const override { return attachment_id_; }

private:
	void RestartMediaChannel(absl::optional<uint32_t> ssrc);
	void Reconfigure();
	bool SetOutputVolume(double volume);

	rtc::Thread* const worker_thread_;
	const std::string id_;
	const rtc::scoped_refptr<RemoteAudioSource> source_;
	const rtc::scoped_refptr<AudioTrackProxyWithInternal<AudioTrack>> track_;
	cricket::VoiceMediaChannel* media_channel_ = nullptr;
	absl::optional<uint32_t> ssrc_;
	std::vector<rtc::scoped_refptr<MediaStreamInterface>> streams_;
	bool cached_track_enabled_;
	double cached_volume_ = 1;
	bool stopped_ = true;
	RTCRtpReceiverObserverInterface* observer_ = nullptr;
	bool received_first_packet_ = false;
	int attachment_id_ = 0;
	rtc::scoped_refptr<DtlsTransportInterface> dtls_transport_;
	// Allows to thread safely change playout delay. Handles caching cases if
	// |SetJitterBufferMinimumDelay| is called before start.
//	rtc::scoped_refptr<JitterBufferDelayInterface> delay_;
//	JitterBufferDelay delay_ RTC_GUARDED_BY(worker_thread_);;
//	const rtc::scoped_refptr<PendingTaskSafetyFlag> worker_thread_safety_;

	rtc::RTCThreadManagerInterface* _rtc_thread_manager;
};

}  // namespace webrtc


#endif	// __RTC_PC_AUDIO_RTP_RECEIVER_H__