//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /video/video_receive_stream2.h
//
//////////////////////////////////////////////////////////////////////////


#ifndef __RTC_VIDEO_RECEIVE_STREAM_2_IMPL_H__
#define __RTC_VIDEO_RECEIVE_STREAM_2_IMPL_H__

#include <memory>
#include <vector>

#include "api/task_queue/task_queue_factory.h"
#include "api/units/timestamp.h"
#include "api/transport/rtp/rtp_source.h"
#include "call/rtp_packet_sink_interface.h"
#include "call/syncable.h"

#include "call/rtp_stream_receiver_controller_interface.h"
#include "modules/rtp_rtcp/include/flexfec_receiver.h"
#include "modules/rtp_rtcp/source/source_tracker.h"
#include "modules/rtp_rtcp/source/rtp_packet_received.h"
#include "modules/include/module_common_types.h"
#include "modules/utility/include/process_thread.h"
#include "rtc_base/synchronization/sequence_checker.h"
#include "rtc_base/system/no_unique_address.h"
#include "rtc_base/task_queue.h"
#include "rtc_base/task_utils/pending_task_safety_flag.h"
#include "system_wrappers/include/clock.h"
#include "video/transport_adapter.h"
#include "video/call_stats2.h"
#include "api/video/video_codec_type.h"

#include "../call/RTCVideoReceiveStream.h"
#include "../api/video/RTCVideoRtpPacketListenSinkInterface.h"
#include "RTCRtpVideoPacketStreamReceiver.h"
#include "../modules/feedback/RTCFeedbackPacketRouter.h"
#include "../call/RTCRtxReceiveStream.h"

#include "../rtc_pc_src/_deprecate_defines.h"

namespace webrtc {

class RTCVideoRtpPacketListenSinkInterface;
class RTCRtpTransportControllerSendInterface;

class RTCVideoReceiveStream2Impl : public RTCVideoReceiveStream,
								   public NackSender,
								   public CallStatsObserver,
								   public RTCVideoRtpPacketListenSinkInterface {
public:
	RTCVideoReceiveStream2Impl(
		TaskQueueBase* current_queue,
		RTCRtpTransportControllerSendInterface* rtp_transport,
		RtpStreamReceiverControllerInterface* receiver_controller,
		RTCVideoReceiveStream::Config config,
		CallStats* call_stats,
		Clock* clock,
		RTCVideoRtpPacketListenSinkInterface *video_rtp_packet_listen_sink);
	~RTCVideoReceiveStream2Impl() override;

	const RTCVideoReceiveStream::Config& config() const { return config_; }

	void SignalNetworkState(NetworkState state);
	bool DeliverRtcp(const uint8_t* packet, size_t length);

	// Implements webrtc::VideoReceiveStream.
	void Start() override;
	void Stop() override;

	RTCVideoReceiveStream::Stats GetStats() const override;

	void AddSecondarySink(RtpPacketSinkInterface* sink) override;
	void RemoveSecondarySink(const RtpPacketSinkInterface* sink) override;


	void SendNack(const std::vector<uint16_t>& sequence_numbers,
		bool buffering_allowed) override;

	// Implements CallStatsObserver::OnRttUpdate
	void OnRttUpdate(int64_t avg_rtt_ms, int64_t max_rtt_ms) override;

	std::vector<webrtc::RtpSource> GetSources() const override;

	void GenerateKeyFrame() override;
	void GenerateKeyFramePli() override;

	// implements RTCVideoRtpPacketListenSinkInterface
	void OnStart() override;
	void OnStop() override;

	// implements RtpPacketSinkInterface
	void OnRtpPacket(const RtpPacketReceived& packet) override;
	void OnRtxRtpPacket(const RtpPacketReceived& packet) override;

	uint32_t RemoteSsrc();

private:
	void RequestKeyFrame(int64_t timestamp_ms, bool is_fir = true);
		RTC_RUN_ON(worker_sequence_checker_);

	VideoCodecType _get_rtp_codec_type(const RtpPacketReceived& packet);


private:
	RTC_NO_UNIQUE_ADDRESS SequenceChecker worker_sequence_checker_;

	internal::TransportAdapter transport_adapter_;
	const RTCVideoReceiveStream::Config config_;
	TaskQueueBase* const worker_thread_;

	Clock* const clock_;

	CallStats* const call_stats_;

	SourceTracker source_tracker_;

	const std::unique_ptr<ReceiveStatistics> rtp_receive_statistics_;

	RTCRtpVideoPacketStreamReceiver rtp_video_stream_receiver_;

	std::unique_ptr<RtpStreamReceiverInterface> media_receiver_;
	std::unique_ptr<RTCRtxReceiveStream> rtx_receive_stream_;

	std::unique_ptr<RtpStreamReceiverInterface> rtx_receiver_;
	
	RTCVideoRtpPacketListenSinkInterface *video_rtp_packet_listen_sink_;

	bool keyframe_generation_requested_ RTC_GUARDED_BY(worker_sequence_checker_) =
		false;
};

}


#endif		// __RTC_VIDEO_RECEIVE_STREAM_2_IMPL_H__