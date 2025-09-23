//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /module/pacing/task_queue_paced_sender.h
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_MODULE_PACING_TASK_QUEUE_PACED_SENDER__
#define __RTC_MODULE_PACING_TASK_QUEUE_PACED_SENDER__

#include <unordered_map>

#include <pthread.h>

#include "absl/types/optional.h"

#include "api/units/data_rate.h"
#include "api/units/time_delta.h"
#include "api/units/timestamp.h"
#include "modules/rtp_rtcp/include/rtp_packet_sender.h"
#include "modules/pacing/rtp_packet_pacer.h"
#include "modules/pacing/bitrate_prober.h"
#include "rtc_base/numerics/exp_filter.h"
#include "rtc_base/task_utils/pending_task_safety_flag.h"

#include "../rtp_rtcp/source/RTCRtpRtcpInterface.h"
#include "RTCSFUPacingListener.h"
#include "RTCPacingController.h"
#include "RTCSFUPacingController.h"

#include "../rtc_pc_src/_deprecate_defines.h"

namespace webrtc {

class Clock;

class RTCTaskQueuePacedSender : public RtpPacketPacer, public RtpPacketSender
#ifdef __USING_RTC_SFU_PACING__
	, public RTCSFUPacingListener
#endif
{
public:
	static const int kNoPacketHoldback;

	RTCTaskQueuePacedSender(
		Clock* clock,
		RTCPacingController::RTCPacketSender* packet_sender,
		TimeDelta max_hold_back_window,
		int max_hold_back_window_in_packets,
		absl::optional<TimeDelta> burst_interval = absl::nullopt);

	~RTCTaskQueuePacedSender() override;

	void EnsureStarted();

	// RtpPacketSender method
	void EnqueuePackets(
		std::vector<std::unique_ptr<RtpPacketToSend>> packets) override;
	void EnqueuePacket(std::unique_ptr<RtpPacketToSend> packet) override;
	void RemovePacketsForSsrc(uint32_t ssrc) override;

	// RtpPacketPacer
	void CreateProbeClusters(
		std::vector<ProbeClusterConfig> probe_cluster_configs) override;
	void Pause() override;
	void Resume() override;
	void SetCongested(bool congested) override;
	void SetPacingRates(DataRate pacing_rate, DataRate padding_rate) override;
	void SetAccountForAudioPackets(bool account_for_audio) override;
	void SetIncludeOverhead() override;
	void SetTransportOverhead(DataSize overhead_per_packet) override;
	TimeDelta OldestPacketWaitTime() const override;
	DataSize QueueSizeData() const override;
	absl::optional<Timestamp> FirstSentPacketTime() const override;
	TimeDelta ExpectedQueueTime() const override;
	void SetQueueTimeLimit(TimeDelta limit) override;

#ifdef __USING_RTC_SFU_PACING__
	// RTCSFUPacingListener method
	void AddPacingRtpModule(RTCRtpRtcpInterface* rtp_module) override;
	void RemovePacingRtpModule(RTCRtpRtcpInterface* rtp_module) override;
#endif

protected:
	struct Stats {
		Stats()
			: oldest_packet_enqueue_time(Timestamp::MinusInfinity()),
			queue_size(DataSize::Zero()),
			expected_queue_time(TimeDelta::Zero()) {}
		Timestamp oldest_packet_enqueue_time;
		DataSize queue_size;
		TimeDelta expected_queue_time;
		absl::optional<Timestamp> first_sent_packet_time;
	};
	void OnStatsUpdated(const Stats& stats);

#ifdef __USING_RTC_SFU_PACING__
private:
	void _enqueue_packet_with_sfu_pacing(std::unique_ptr<RtpPacketToSend> packet);
	void _pause_with_sfu_pacing();
	void _resume_with_sfu_pacing();
	void _set_congested_with_sfu_pacing(bool congested);
	void _set_pacing_rate_with_sfu_pacing(DataRate pacing_rate, DataRate padding_rate);
	void _set_include_overhead_with_sfu_pacing();
	void _set_transport_overhead_with_sfu_pacing(DataSize overhead_per_packet);
	void _set_queue_time_limit_with_sfu_pacing(TimeDelta limit);
	void _maybe_process_packets_with_sfu_pacing(Timestamp scheduled_process_time);
	DataRate _pacing_rate_with_sfu_pacing();
	void _add_pacing_rtp_module(RTCRtpRtcpInterface* rtp_module, bool isAudio, uint32_t ssrc, absl::optional<uint32_t> rtx_ssrc);
	void _remove_pacing_rtp_module(RTCRtpRtcpInterface* rtp_module, bool isAudio, uint32_t ssrc, absl::optional<uint32_t> rtx_ssrc);
#endif 

private:
	void MaybeScheduleProcessPackets() RTC_RUN_ON(task_queue_);
	void MaybeProcessPackets(Timestamp scheduled_process_time);
	void UpdateStats() RTC_RUN_ON(task_queue_);
	Stats GetStats() const;

	// for enqueue thread
	static void* enqueue_thread_proc(void *p);

private:
	Clock* const clock_;
	const TimeDelta max_hold_back_window_;
	const int max_hold_back_window_in_packets_;

#ifdef __USING_RTC_SFU_PACING__
	std::unordered_map<RTCRtpRtcpInterface*, std::unique_ptr<RTCSFUPacingController>> pacers_ RTC_GUARDED_BY(task_queue_);
	std::unordered_map<uint32_t, RTCSFUPacingController*> ssrc_to_pacers_ RTC_GUARDED_BY(task_queue_);

	BitrateProber prober_;
	RTCPacingController::RTCPacketSender* packet_sender_;
	DataRate pacing_rate_ = DataRate::Zero();
	DataRate padding_rate_ = DataRate::Zero();
#else
	RTCPacingController pacing_controller_ RTC_GUARDED_BY(task_queue_);
#endif

	Timestamp next_process_time_ RTC_GUARDED_BY(task_queue_);
	bool is_started_ RTC_GUARDED_BY(task_queue_);
	bool is_shutdown_ RTC_GUARDED_BY(task_queue_);

	rtc::ExpFilter packet_size_ RTC_GUARDED_BY(task_queue_);
	bool include_overhead_ RTC_GUARDED_BY(task_queue_);

	Stats current_stats_ RTC_GUARDED_BY(task_queue_);
	bool processing_packets_ RTC_GUARDED_BY(task_queue_) = false;

	ScopedTaskSafety safety_;
	TaskQueueBase* task_queue_;

	absl::optional<TimeDelta> burst_interval_;


	// for enqueue thread
	std::vector<std::unique_ptr<RtpPacketToSend>> enqueue_packets_;
	bool is_run_thread_ = false;
	bool is_quiting_thread_ = false;
	pthread_t handle_;
	pthread_mutex_t mutex_ = PTHREAD_MUTEX_INITIALIZER;
	pthread_cond_t cond_ = PTHREAD_COND_INITIALIZER;
};

}


#endif // __RTC_MODULE_PACING_TASK_QUEUE_PACED_SENDER__