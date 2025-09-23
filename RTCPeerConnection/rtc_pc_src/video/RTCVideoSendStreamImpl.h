//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_VIDEO_SEND_STREAM_IMPL_H__
#define __RTC_VIDEO_SEND_STREAM_IMPL_H__

#include <map>
#include <memory>
#include <vector>

#include "rtc_base/event.h"
#include "rtc_base/task_queue.h"
#include "rtc_base/thread_checker.h"
#include "modules/utility/include/process_thread.h"
#include "modules/rtp_rtcp/source/rtp_packet_received.h"

#include "../modules/feedback/RTCFeedbackPacketRouter.h"
#include "../api/video/RTCVideoPacketRouteSourceInterface.h"
#include "../call/RTCVideoSendStream.h"
#include "RTCVideoSendStreamRouter.h"

#include "../rtc_pc_src/_deprecate_defines.h"

namespace webrtc {
class RTCRtpTransportControllerSendInterface;

class RTCVideoSendStreamImpl : public RTCVideoSendStream {
public:
	RTCVideoSendStreamImpl(Clock* clock,
		RTCVideoSendStream::Config config,
		RTCRtpTransportControllerSendInterface* transport,
		RtcpRttStats* call_stats);

	~RTCVideoSendStreamImpl() override;

public:
	void DeliverRtcp(const uint8_t* packet, size_t length);
	void Start() override;
	void Stop() override;

	void SetSource(rtc::RTCVideoPacketRouteSourceInterface<std::unique_ptr<webrtc::RtpPacketReceived>>* source) override;
	Stats GetStats() override;

	void StopPermanently();

private:
	SequenceChecker thread_checker_;
	TaskQueueBase* const worker_queue_;
	ScopedTaskSafety safety_;

	rtc::Event thread_sync_event_;
	const RTCVideoSendStream::Config config_;

	std::unique_ptr<RTCVideoSendStreamRouter> router_;
	std::unique_ptr<RTCVideoPacketRouteFeeder> feeder_;
};

}	// namespace webrtc

#endif // __RTC_VIDEO_SEND_STREAM_IMPL_H__