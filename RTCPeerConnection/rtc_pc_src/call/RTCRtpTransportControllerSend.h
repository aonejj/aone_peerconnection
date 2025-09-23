//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /call/rtc_transport_controller_send.h
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_CALL_RTP_TRANSPORT_CONTROLLER_SEND_H__
#define __RTC_CALL_RTP_TRANSPORT_CONTROLLER_SEND_H__

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "api/network_state_predictor.h"
#include "api/transport/network_control.h"
#include "api/units/data_rate.h"
#include "call/rtp_bitrate_configurator.h"
#include "modules/congestion_controller/rtp/control_handler.h"
#include "modules/congestion_controller/rtp/transport_feedback_adapter.h"
#include "modules/congestion_controller/rtp/transport_feedback_demuxer.h"
#include "modules/utility/include/process_thread.h"
#include "rtc_base/constructor_magic.h"
#include "rtc_base/network_route.h"
#include "rtc_base/race_checker.h"
#include "rtc_base/task_queue.h"
#include "rtc_base/task_utils/repeating_task.h"
#include "rtc_base/task_utils/pending_task_safety_flag.h"
#include "rtc_base/experiments/field_trial_parser.h"

#include "RTCRtpTransportControllerSendInterface.h"

#include "../rtc_pc_src/modules/pacing/RTCTaskQueuePacedSender.h"
#include "../rtc_pc_src/modules/pacing/RTCPacketRouter.h"

#include "../rtc_pc_src/_deprecate_defines.h"

namespace webrtc {
class Clock;

// TODO(nisse): When we get the underlying transports here, we should
// have one object implementing RtpTransportControllerSendInterface
// per transport, sharing the same congestion controller.
class RTCRtpTransportControllerSend final
	: public RTCRtpTransportControllerSendInterface,
	  public RtcpBandwidthObserver,
	  public TransportFeedbackObserver,
	  public NetworkStateEstimateObserver {
public:
	RTCRtpTransportControllerSend(
		Clock* clock,
		const BitrateConstraints& bitrate_config,
		TaskQueueFactory* task_queue_factory);
	~RTCRtpTransportControllerSend() override;

	// Implements RtpTransportControllerSendInterface
	RTCPacketRouter* packet_router() override;

	NetworkStateEstimateObserver* network_state_estimate_observer() override;
	TransportFeedbackObserver* transport_feedback_observer() override;

	RtpPacketSender* packet_sender() override;

	StreamFeedbackProvider* GetStreamFeedbackProvider() override;
	void RegisterTargetTransferRateObserver(
		TargetTransferRateObserver* observer) override;
	void OnNetworkRouteChanged(const std::string& transport_name,
		const rtc::NetworkRoute& network_route) override;
	void OnNetworkAvailability(bool network_available) override;
	RtcpBandwidthObserver* GetBandwidthObserver() override;

	void OnSentPacket(const rtc::SentPacket& sent_packet) override;
	void OnReceivedPacket(const ReceivedPacket& packet_msg) override;

	void SetSdpBitrateParameters(const BitrateConstraints& constraints) override;
	void SetClientBitratePreferences(const BitrateSettings& preferences) override;

	void OnTransportOverheadChanged(
		size_t transport_overhead_bytes_per_packet) override;

	void EnsureStarted() override;

	// Implements RtcpBandwidthObserver interface
	void OnReceivedEstimatedBitrate(uint32_t bitrate) override;
	void OnReceivedRtcpReceiverReport(const ReportBlockList& report_blocks,
		int64_t rtt,
		int64_t now_ms) override;

	// Implements TransportFeedbackObserver interface
	void OnAddPacket(const RtpPacketSendInfo& packet_info) override;
	void OnTransportFeedback(const rtcp::TransportFeedback& feedback) override;

	// Implements NetworkStateEstimateObserver interface
	void OnRemoteNetworkEstimate(NetworkStateEstimate estimate) override;

private:
	void MaybeCreateControllers() RTC_RUN_ON(sequence_checker_);
	void UpdateInitialConstraints(TargetRateConstraints new_contraints)
		RTC_RUN_ON(sequence_checker_);

	void StartProcessPeriodicTasks() RTC_RUN_ON(sequence_checker_);
	void UpdateControllerWithTimeInterval() RTC_RUN_ON(sequence_checker_);
	void UpdateStreamsConfig() RTC_RUN_ON(sequence_checker_);

	void OnReceivedRtcpReceiverReportBlocks(const ReportBlockList& report_blocks,
		int64_t now_ms) RTC_RUN_ON(sequence_checker_);
	void PostUpdates(NetworkControlUpdate update) RTC_RUN_ON(sequence_checker_);

	void UpdateControlState() RTC_RUN_ON(sequence_checker_);
	void UpdateCongestedState() RTC_RUN_ON(sequence_checker_);

	absl::optional<bool> GetCongestedStateUpdate() const
		RTC_RUN_ON(sequence_checker_);
	void ProcessSentPacket(const rtc::SentPacket& sent_packet)
		RTC_RUN_ON(sequence_checker_);
	void ProcessSentPacketUpdates(NetworkControlUpdate updates)
		RTC_RUN_ON(sequence_checker_);

	absl::optional<BitrateConstraints> ApplyOrLiftRelayCap(bool is_relayed);
	bool IsRelevantRouteChange(const rtc::NetworkRoute& old_route,
		const rtc::NetworkRoute& new_route) const;
	void UpdateBitrateConstraints(const BitrateConstraints& updated);

private:
	Clock* const clock_;

	RtpBitrateConfigurator bitrate_configurator_;

	std::map<std::string, rtc::NetworkRoute> network_routes_ RTC_GUARDED_BY(sequence_checker_);
	TargetTransferRateObserver* observer_ RTC_GUARDED_BY(sequence_checker_);

	TransportFeedbackDemuxer feedback_demuxer_;		

	TransportFeedbackAdapter transport_feedback_adapter_
		RTC_GUARDED_BY(sequence_checker_);
	const std::unique_ptr<NetworkControllerFactoryInterface>
		controller_factory_fallback_ RTC_PT_GUARDED_BY(sequence_checker_);

	std::unique_ptr<CongestionControlHandler> control_handler_
		RTC_GUARDED_BY(sequence_checker_) RTC_PT_GUARDED_BY(sequence_checker_);

	std::unique_ptr<NetworkControllerInterface> controller_
		RTC_GUARDED_BY(sequence_checker_) RTC_PT_GUARDED_BY(sequence_checker_);

	TimeDelta process_interval_ RTC_GUARDED_BY(sequence_checker_);

	std::map<uint32_t, RTCPReportBlock> last_report_blocks_
		RTC_GUARDED_BY(sequence_checker_);

	Timestamp last_report_block_time_ RTC_GUARDED_BY(sequence_checker_);
	NetworkControllerConfig initial_config_ RTC_GUARDED_BY(sequence_checker_);
	StreamsConfig streams_config_ RTC_GUARDED_BY(sequence_checker_);

	const bool reset_feedback_on_route_change_;
	const bool send_side_bwe_with_overhead_;
	FieldTrialParameter<DataRate> relay_bandwidth_cap_;

	bool network_available_ RTC_GUARDED_BY(sequence_checker_);
	RepeatingTaskHandle pacer_queue_update_task_ RTC_GUARDED_BY(sequence_checker_);
	RepeatingTaskHandle controller_task_ RTC_GUARDED_BY(sequence_checker_);
	DataSize congestion_window_size_ RTC_GUARDED_BY(sequence_checker_);
	bool is_congested_ RTC_GUARDED_BY(sequence_checker_);

	// TODO(perkj): |task_queue_| is supposed to replace |process_thread_|.
	// |task_queue_| is defined last to ensure all pending tasks are cancelled
	// and deleted before any other members.
	SequenceChecker sequence_checker_;
	TaskQueueBase* task_queue_;
	ScopedTaskSafety safety_;

	size_t transport_overhead_bytes_per_packet_;

	RTCPacketRouter packet_router_;
	bool pacer_started_ RTC_GUARDED_BY(sequence_checker_);

	RTCTaskQueuePacedSender pacer_;

	RTC_DISALLOW_COPY_AND_ASSIGN(RTCRtpTransportControllerSend);
};

}  // namespace webrtc

#endif // __RTC_CALL_RTP_TRANSPORT_CONTROLLER_SEND_H__