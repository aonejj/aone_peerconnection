//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_AUDIO_AUDIO_SEND_STREAM_ROUTER_H__
#define __RTC_AUDIO_AUDIO_SEND_STREAM_ROUTER_H__

#include "rtc_base/event.h"
#include "rtc_base/task_queue.h"
#include "rtc_base/weak_ptr.h"
#include "rtc_base/task_utils/repeating_task.h"
#include "rtc_base/random.h"
#include "modules/include/module.h"
#include "modules/utility/include/process_thread.h"
#include "modules/rtp_rtcp/include/rtp_rtcp_defines.h"
#include "modules/rtp_rtcp/include/rtp_header_extension_map.h"
#include "modules/rtp_rtcp/include/rtp_packet_sender.h"
#include "modules/rtp_rtcp/source/rtp_packet_to_send.h"
#include "modules/rtp_rtcp/source/rtcp_sender.h"
#include "modules/rtp_rtcp/source/rtcp_receiver.h"
#include "modules/rtp_rtcp/source/rtcp_packet/tmmb_item.h"
#include "modules/congestion_controller/rtp/transport_feedback_demuxer.h"
#include "system_wrappers/include/clock.h"

#include "../call/RTCAudioSendStream.h"
#include "../call/RTCRtpTransportControllerSendInterface.h"
#include "RTCAudioPacketRoutingSink.h"
#include "../modules/rtp_rtcp/source/RTCRtpRtcpSendImpl.h"
#include "../modules/feedback/RTCFeedbackPacketRouter.h"

#include "../rtc_pc_src/_deprecate_defines.h"

namespace webrtc {

class RTCAudioPacketRouteFeeder;

class RTCAudioSendStreamRouter : public RTCAudioPacketRoutingSink,
								 public RtcpBandwidthObserver,
								 public TransportFeedbackObserver,
								 public StreamFeedbackObserver {

public:
	RTCAudioSendStreamRouter(
		Clock* clock,
		const RTCAudioSendStream::Config* config,
		RTCAudioPacketRouteFeeder *feeder,
		RTCRtpTransportControllerSendInterface* transport,
		RtcpRttStats* call_stats);

	~RTCAudioSendStreamRouter() override;

public:
	int32_t SendRTCP(RTCPPacketType rtcpPacketType);

	void DeliverRtcp(const uint8_t* packet, size_t length);
	void Start();
	void Stop();

	int64_t GetRTT() const;

public:
	//////////////////////////////////////////////////////////////////////////
	// RtcpBandwidthObserver methods
	void OnReceivedEstimatedBitrate(uint32_t bitrate) override;
	void OnReceivedRtcpReceiverReport(const ReportBlockList& report_blocks,
		int64_t rtt, int64_t now_ms) override;

	//////////////////////////////////////////////////////////////////////////
	// TransportFeedbackObserver
	void OnAddPacket(const RtpPacketSendInfo& packet_info) override;
	void OnTransportFeedback(const rtcp::TransportFeedback& feedback) override;

	//////////////////////////////////////////////////////////////////////////
	// StreamFeedbackObserver
	void OnPacketFeedbackVector(std::vector<StreamPacketInfo> packet_feedback_vector) override;


private:
	void OnRoutingPacket(std::unique_ptr<RtpPacketReceived> packet) override;

private:
	Clock* const clock_;
	const RTCAudioSendStream::Config* const config_;
	RtcpBandwidthObserver* bandwidth_observer_;
	TransportFeedbackObserver* feedback_observer_;
	RTCAudioPacketRouteFeeder *feeder_;

	rtc::WeakPtrFactory<RTCAudioSendStreamRouter> weak_ptr_factory_;
	rtc::WeakPtr<RTCAudioSendStreamRouter> weak_ptr_;

	RtpHeaderExtensionMap header_extension_map_;

	RTCRtpRtcpInterface::Configuration configuration_;
	std::unique_ptr<RTCRtpRtcpSendImpl> rtp_rtcp_sender_;

	mutable Mutex mutex_rtt_;
	int64_t rtt_ms_ RTC_GUARDED_BY(mutex_rtt_);

	mutable Mutex send_mutex_;
	bool sending_media_ RTC_GUARDED_BY(send_mutex_);

	RTCRtpTransportControllerSendInterface* const rtp_transport_;
};

}	// namespace webrtc

#endif // __RTC_AUDIO_AUDIO_SEND_STREAM_ROUTER_H__