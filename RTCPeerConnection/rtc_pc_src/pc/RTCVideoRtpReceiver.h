//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /pc/video_rtp_receiver.h
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_PC_VIDEO_RTP_RECEIVER_H__
#define __RTC_PC_VIDEO_RTP_RECEIVER_H__

#include <stdint.h>

#include <string>
#include <vector>

#include "../rtc_pc_src/_deprecate_defines.h"

#include "absl/types/optional.h"

#include "api/media_types.h"
#include "api/rtp_parameters.h"
#include "api/scoped_refptr.h"
#include "api/make_ref_counted.h"
#include "rtc_base/ref_counted_object.h"
#include "rtc_base/thread.h"

#include "../../src_update/api/_media_stream_interface.h"
#include "../../src_update/api/_media_stream_track_proxy.h"
#include "../../src_update/media/base/_media_channel.h"
#include "../api/RTCRtpReceiverInterface.h"
#include "RTCRtpReceiver.h"
#include "RTCVideoRtpTrackSource.h"
#include "RTCVideoTrack.h"

#include "../rtc_pc_src/RTCThreadManagerInterface.h"

namespace webrtc {

class RTCVideoRtpReceiver : public RTCRtpReceiverInternal,
							public RTCVideoRtpTrackSource::Callback {
public:
	// An SSRC of 0 will create a receiver that will match the first SSRC it
	// sees. Must be called on signaling thread.
	RTCVideoRtpReceiver(rtc::Thread* worker_thread,
						std::string receiver_id,
						std::vector<std::string> streams_ids,
						rtc::RTCThreadManagerInterface* rtc_thread_manager
						);
	// TODO(hbos): Remove this when streams() is removed.
	// https://crbug.com/webrtc/9480
	RTCVideoRtpReceiver(
					rtc::Thread* worker_thread,
					const std::string& receiver_id,
					const std::vector<rtc::scoped_refptr<MediaStreamInterface>>& streams,
					rtc::RTCThreadManagerInterface* rtc_thread_manager
					);

	virtual ~RTCVideoRtpReceiver();

	rtc::scoped_refptr<VideoTrackInterface> video_track() const {
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
		return cricket::MEDIA_TYPE_VIDEO;
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

	int AttachmentId() const override { return attachment_id_; }

	std::vector<RtpSource> GetSources() const override;

private:
	void RestartMediaChannel(absl::optional<uint32_t> ssrc);

	void OnGenerateKeyFrame() override;

	rtc::Thread* const worker_thread_;

	const std::string id_;
	cricket::VideoMediaChannel* media_channel_ = nullptr;
	absl::optional<uint32_t> ssrc_;
	// |source_| is held here to be able to change the state of the source when
	// the VideoRtpReceiver is stopped.
	rtc::scoped_refptr<RTCVideoRtpTrackSource> source_;
	rtc::scoped_refptr<VideoTrackProxyWithInternal<RTCVideoTrack>> track_;
	std::vector<rtc::scoped_refptr<MediaStreamInterface>> streams_;
	bool stopped_ = true;
	RTCRtpReceiverObserverInterface* observer_ = nullptr;
	bool received_first_packet_ = false;
	int attachment_id_ = 0;
	rtc::scoped_refptr<DtlsTransportInterface> dtls_transport_;

	// Records if we should generate a keyframe when |media_channel_| gets set up
	// or switched.
	bool saved_generate_keyframe_ RTC_GUARDED_BY(worker_thread_) = false;

	rtc::RTCThreadManagerInterface* _rtc_thread_manager;
};

}  // namespace webrtc


#endif	// __RTC_PC_VIDEO_RTP_RECEIVER_H__