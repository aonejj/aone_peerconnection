//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_AUDIO_AUDIO_SEND_STREAM_IMPL_H__
#define __RTC_AUDIO_AUDIO_SEND_STREAM_IMPL_H__

#include <memory>
#include<vector>

#include "rtc_base/race_checker.h"
#include "rtc_base/task_queue.h"
#include "rtc_base/thread_checker.h"
#include "modules/utility/include/process_thread.h"
#include "modules/rtp_rtcp/source/rtp_packet_received.h"

#include "../modules/feedback/RTCFeedbackPacketRouter.h"
#include "../api/audio/RTCAudioPacketRouteSourceInterface.h"
#include "../call/RTCAudioSendStream.h"
#include "RTCAudioSendStreamRouter.h"

#include "../rtc_pc_src/_deprecate_defines.h"

namespace webrtc {

class RTCRtpTransportControllerSendInterface;

class RTCAudioSendStreamImpl : public RTCAudioSendStream {
public:
	RTCAudioSendStreamImpl(Clock* clock,
		RTCAudioSendStream::Config config,
		RTCRtpTransportControllerSendInterface* rtp_transport,
		RtcpRttStats* rtt_stats);

	RTCAudioSendStreamImpl() = delete;
	RTCAudioSendStreamImpl(const RTCAudioSendStreamImpl&) = delete;
	RTCAudioSendStreamImpl& operator=(const RTCAudioSendStreamImpl&) = delete;

	~RTCAudioSendStreamImpl() override;

public:
	void DeliverRtcp(const uint8_t* packet, size_t length) override;
	void Start() override;
	void Stop() override;

	const RTCAudioSendStream::Config& GetConfig() const override;
	void SetSource(rtc::RTCAudioPacketRouteSourceInterface<std::unique_ptr<webrtc::RtpPacketReceived>>* source) override;
	Stats GetStats() const override;

	void StopPermanently();

private:
	SequenceChecker thread_checker_;
	TaskQueueBase* const worker_queue_;
	ScopedTaskSafety safety_;

	rtc::Event thread_sync_event_;
	const RTCAudioSendStream::Config config_;

	std::unique_ptr<RTCAudioSendStreamRouter> router_;
	std::unique_ptr<RTCAudioPacketRouteFeeder> feeder_;
};

}	// namespace webrtc


#endif	// __RTC_AUDIO_AUDIO_SEND_STREAM_IMPL_H__