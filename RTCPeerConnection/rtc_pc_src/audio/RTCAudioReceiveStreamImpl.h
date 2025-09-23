//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /audio/audio_receive_stream.h
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_AUDIO_RECEIVE_STREAM_IMPL_H__
#define __RTC_AUDIO_RECEIVE_STREAM_IMPL_H__

#include <memory>
#include <vector>

#include "api/rtp_headers.h"
#include "modules/rtp_rtcp/source/source_tracker.h"
#include "modules/utility/include/process_thread.h"
#include "rtc_base/thread_checker.h"
#include "system_wrappers/include/clock.h"

#include "../call/RTCAudioReceiveStream.h"

namespace webrtc {

class RtpStreamReceiverControllerInterface;
class RtpStreamReceiverInterface;
class RTCAudioRtpPacketListenSinkInterface;
class RTCRtpTransportControllerSendInterface;

namespace voe {
	class RTCChannelRtpPacketReceiverInterface;
}  // namespace voe


class RTCAudioReceiveStreamImpl final : public RTCAudioReceiveStream {
public:
	RTCAudioReceiveStreamImpl(Clock* clock,
		RTCRtpTransportControllerSendInterface* rtp_transport,
		RtpStreamReceiverControllerInterface* receiver_controller,
		const RTCAudioReceiveStream::Config& config,
		RTCAudioRtpPacketListenSinkInterface *audio_rtp_packet_listen_sink);

	RTCAudioReceiveStreamImpl(Clock* clock,
		RTCRtpTransportControllerSendInterface* rtp_transport,
		RtpStreamReceiverControllerInterface* receiver_controller,
		const RTCAudioReceiveStream::Config& config,
		RTCAudioRtpPacketListenSinkInterface *audio_rtp_packet_listen_sink,
		std::unique_ptr<voe::RTCChannelRtpPacketReceiverInterface> channel_rtp_packet_receive);

	RTCAudioReceiveStreamImpl() = delete;
	RTCAudioReceiveStreamImpl(const RTCAudioReceiveStreamImpl&) = delete;
	RTCAudioReceiveStreamImpl& operator=(const RTCAudioReceiveStreamImpl&) = delete;

	~RTCAudioReceiveStreamImpl() override;

	void Reconfigure(const RTCAudioReceiveStream::Config& config) override;
	void Start() override;
	void Stop() override;
	bool IsRunning() const override;

	std::vector<webrtc::RtpSource> GetSources() const override;

	void DeliverRtcp(const uint8_t* packet, size_t length);
	const RTCAudioReceiveStream::Config& config() const;

private:
	static void ConfigureStream(RTCAudioReceiveStreamImpl* stream,
		const Config& new_config,
		bool first_time);

	rtc::ThreadChecker worker_thread_checker_;
	RTCAudioReceiveStream::Config config_;

	const std::unique_ptr<voe::RTCChannelRtpPacketReceiverInterface> channel_rtp_packet_receive_;
	SourceTracker source_tracker_;

	bool playing_ RTC_GUARDED_BY(worker_thread_checker_) = false;

	std::unique_ptr<RtpStreamReceiverInterface> rtp_stream_receiver_;
	RTCAudioRtpPacketListenSinkInterface *audio_rtp_packet_listen_sink_;
};

}


#endif	// __RTC_AUDIO_RECEIVE_STREAM_IMPL_H__