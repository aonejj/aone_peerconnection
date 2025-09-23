//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_VIDEO_SEND_STREAM_ROUTER_H__
#define __RTC_VIDEO_SEND_STREAM_ROUTER_H__

#include <map>

#include "rtc_base/task_queue.h"
#include "rtc_base/weak_ptr.h"
#include "rtc_base/task_utils/repeating_task.h"
#include "rtc_base/random.h"
#include "modules/include/module.h"
#include "modules/utility/include/process_thread.h"
#include "modules/rtp_rtcp/include/rtp_rtcp_defines.h"
#include "modules/rtp_rtcp/include/rtp_header_extension_map.h"
#include "modules/rtp_rtcp/source/rtp_packet_to_send.h"
#include "modules/rtp_rtcp/source/rtcp_sender.h"
#include "modules/rtp_rtcp/source/rtcp_receiver.h"
#include "modules/rtp_rtcp/source/rtcp_packet/tmmb_item.h"
#include "modules/rtp_rtcp/source/rtp_packet_history.h"
#include "modules/congestion_controller/rtp/transport_feedback_demuxer.h"
#include "system_wrappers/include/clock.h"

#include "../call/RTCVideoSendStream.h"
#include "../call/RTCRtpTransportControllerSendInterface.h"
#include "RTCVideoPacketRoutingSink.h"
#include "../modules/rtp_rtcp/source/RTCRtpRtcpSendImpl.h"
#include "../modules/feedback/RTCFeedbackPacketRouter.h"

#include "../rtc_pc_src/_deprecate_defines.h"

namespace webrtc {

class RTCVideoPacketRouteFeeder;

class RTCVideoSendStreamRouter : public RTCVideoPacketRoutingSink,
								 public RtcpBandwidthObserver,
							     public RtcpIntraFrameObserver,
								 public RtcpLossNotificationObserver,
								 public NetworkStateEstimateObserver,
								 public TransportFeedbackObserver,
								 public StreamFeedbackObserver {

public:
	RTCVideoSendStreamRouter(
		Clock* clock,
		const RTCVideoSendStream::Config* config,
		RTCVideoPacketRouteFeeder *feeder,
		RTCRtpTransportControllerSendInterface* rtp_transport,
		RtcpRttStats* call_stats);

	~RTCVideoSendStreamRouter() override;

public:
	void DeliverRtcp(const uint8_t* packet, size_t length);
	void Start();
	void Stop();

public:
	//////////////////////////////////////////////////////////////////////////
	// RtcpBandwidthObserver methods
	void OnReceivedEstimatedBitrate(uint32_t bitrate) override;
	void OnReceivedRtcpReceiverReport(const ReportBlockList& report_blocks,
		int64_t rtt, int64_t now_ms) override;

	//////////////////////////////////////////////////////////////////////////
	// RtcpIntraFrameObserver
	void OnReceivedIntraFrameRequest(uint32_t ssrc) override;

	//////////////////////////////////////////////////////////////////////////
	// RtcpLossNotificationObserver
	void OnReceivedLossNotification(uint32_t ssrc,
		uint16_t seq_num_of_last_decodable,
		uint16_t seq_num_of_last_received,
		bool decodability_flag) override;

	//////////////////////////////////////////////////////////////////////////
	// NetworkStateEstimateObserver
	void OnRemoteNetworkEstimate(NetworkStateEstimate estimate) override;

	//////////////////////////////////////////////////////////////////////////
	// TransportFeedbackObserver
	void OnAddPacket(const RtpPacketSendInfo& packet_info) override;
	void OnTransportFeedback(const rtcp::TransportFeedback& feedback) override;
	
	//////////////////////////////////////////////////////////////////////////
	// StreamFeedbackObserver
	void OnPacketFeedbackVector(std::vector<StreamPacketInfo> packet_feedback_vector) override;

private:
	//////////////////////////////////////////////////////////////////////////
	// RTCVideoPacketRoutingSink
	void OnRoutingPacket(std::unique_ptr<RtpPacketReceived> packet) override;

	void PeriodicUpdate();

private:
	Clock* const clock_;
	const RTCVideoSendStream::Config* const config_;
	RTCVideoPacketRouteFeeder *feeder_;

	rtc::WeakPtrFactory<RTCVideoSendStreamRouter> weak_ptr_factory_;
	rtc::WeakPtr<RTCVideoSendStreamRouter> weak_ptr_;

	RtpHeaderExtensionMap header_extension_map_;

	int64_t	keyframe_request_ms_;

	RTCRtpRtcpInterface::Configuration configuration_;

	std::unique_ptr<RTCRtpRtcpSendImpl> rtp_rtcp_sender_;

	RTCRtpTransportControllerSendInterface* const rtp_transport_;
	RtcpBandwidthObserver* const bandwidth_observer_;
	TransportFeedbackObserver* const feedback_observer_;
	NetworkStateEstimateObserver* const network_state_estimate_observer_;

	mutable Mutex send_mutex_;
	bool sending_media_ RTC_GUARDED_BY(send_mutex_);

	int64_t		debug_timestamp = 0;
	int64_t		debug_rtx_timestamp = 0;
};

}	// namespace webrtc

#endif //__RTC_VIDEO_SEND_STREAM_ROUTER_H__
