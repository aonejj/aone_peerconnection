//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /module/rtp_rtcp/rtp_sender_egress.h
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_MODULE_RTP_RTCP_SOURCE_RTP_SENDER_EGRESS__
#define __RTC_MODULE_RTP_RTCP_SOURCE_RTP_SENDER_EGRESS__

#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "absl/types/optional.h"
#include "api/call/transport.h"
#include "api/task_queue/task_queue_base.h"
#include "api/units/data_rate.h"
#include "api/units/time_delta.h"
#include "api/units/timestamp.h"
#include "modules/rtp_rtcp/include/rtp_rtcp_defines.h"
#include "modules/rtp_rtcp/source/packet_sequencer.h"
#include "modules/rtp_rtcp/source/rtp_packet_history.h"
#include "modules/rtp_rtcp/source/rtp_packet_to_send.h"
#include "modules/rtp_rtcp/include/rtp_packet_sender.h"
#include "modules/include/module_fec_types.h"
#include "rtc_base/bitrate_tracker.h"
#include "rtc_base/synchronization/mutex.h"
#include "rtc_base/task_utils/pending_task_safety_flag.h"
#include "rtc_base/synchronization/sequence_checker.h"
#include "rtc_base/task_utils/repeating_task.h"
#include "rtc_base/thread_annotations.h"

#include "../rtc_pc_src/api/call/RTCTransportSequenceNumberGenerator.h"

#include "RTCRtpRtcpInterface.h"


namespace webrtc {

class RTCRtpSenderEgress {
public:
	class RTCNonPacedPacketSender : public RtpPacketSender {
	public:
		RTCNonPacedPacketSender(TaskQueueBase& worker_queue,
								RTCRtpSenderEgress* sender,
								PacketSequencer* sequencer);
		virtual ~RTCNonPacedPacketSender();

		// RtpPacketSender method
		void EnqueuePackets(
			std::vector<std::unique_ptr<RtpPacketToSend>> packets) override;
		void EnqueuePacket(std::unique_ptr<RtpPacketToSend> packet) override;
		// Since we don't pace packets, there's no pending packets to remove.
		void RemovePacketsForSsrc(uint32_t ssrc) override {}

		void SetTransportSeqGenerator(RTCTransportSequenceNumberGenerator* twcc_seq_generator);

	private:
		void PrepareForSend(RtpPacketToSend* packet);
		TaskQueueBase& worker_queue_;
		RTCRtpSenderEgress* const sender_;
		PacketSequencer* sequencer_;
		ScopedTaskSafety task_safety_;
		RTCTransportSequenceNumberGenerator* twcc_seq_generator_;
	};

	RTCRtpSenderEgress(const RTCRtpRtcpInterface::Configuration& config,
		RtpPacketHistory* packet_history);
	~RTCRtpSenderEgress();

	void SendPacket(std::unique_ptr<RtpPacketToSend> packet,
		const PacedPacketInfo& pacing_info);
	void OnBatchComplete();
	uint32_t Ssrc() const { return ssrc_; }
	absl::optional<uint32_t> RtxSsrc() const { return rtx_ssrc_; }
	absl::optional<uint32_t> FlexFecSsrc() const { return flexfec_ssrc_; }

	RtpSendRates GetSendRates(Timestamp now) const;
	void GetDataCounters(StreamDataCounters* rtp_stats,
		StreamDataCounters* rtx_stats) const;

	bool MediaHasBeenSent() const;

	void SetFecProtectionParameters(const FecProtectionParams& delta_params,
		const FecProtectionParams& key_params);
//	std::vector<std::unique_ptr<RtpPacketToSend>> FetchFecPackets();

	void OnAbortedRetransmissions(
		rtc::ArrayView<const uint16_t> sequence_numbers);

private:
	struct Packet {
		std::unique_ptr<RtpPacketToSend> rtp_packet;
		PacedPacketInfo info;
		Timestamp now;
	};

	void CompleteSendPacket(const Packet& compound_packet, bool last_in_batch);
	bool HasCorrectSsrc(const RtpPacketToSend& packet) const;
	void AddPacketToTransportFeedback(uint16_t packet_id,
		const RtpPacketToSend& packet,
		const PacedPacketInfo& pacing_info);
	void UpdateDelayStatistics(Timestamp capture_time,
		Timestamp now,
		uint32_t ssrc);
	void RecomputeMaxSendDelay();
	void UpdateOnSendPacket(absl::optional<uint16_t> packet_id,
		Timestamp capture_time,
		uint32_t ssrc);

	bool SendPacketToNetwork(const RtpPacketToSend& packet,
		const PacketOptions& options,
		const PacedPacketInfo& pacing_info);
	void UpdateRtpStats(Timestamp now,
		uint32_t packet_ssrc,
		RtpPacketMediaType packet_type,
		RtpPacketCounter counter,
		size_t packet_size);

	void PeriodicUpdate();

private:
	const bool enable_send_packet_batching_;
	TaskQueueBase* const worker_queue_;
	const uint32_t ssrc_;
	const absl::optional<uint32_t> rtx_ssrc_;
	const absl::optional<uint32_t> flexfec_ssrc_;
	Clock* const clock_;
	RtpPacketHistory* const packet_history_ RTC_GUARDED_BY(worker_queue_);
	Transport* const transport_;
	const bool is_audio_;

	absl::optional<uint16_t> last_sent_seq_ RTC_GUARDED_BY(worker_queue_);
	absl::optional<uint16_t> last_sent_rtx_seq_ RTC_GUARDED_BY(worker_queue_);

	TransportFeedbackObserver* const transport_feedback_observer_;
	SendSideDelayObserver* const send_side_delay_observer_;
	SendPacketObserver* const send_packet_observer_;
	StreamDataCountersCallback* const rtp_stats_callback_;
	BitrateStatisticsObserver* const bitrate_callback_;

	bool media_has_been_sent_ RTC_GUARDED_BY(worker_queue_);

	std::map<Timestamp, TimeDelta> send_delays_ RTC_GUARDED_BY(worker_queue_);
	std::map<Timestamp, TimeDelta>::const_iterator max_delay_it_
		RTC_GUARDED_BY(worker_queue_);

	TimeDelta sum_delays_ RTC_GUARDED_BY(worker_queue_);
	StreamDataCounters rtp_stats_ RTC_GUARDED_BY(worker_queue_);
	StreamDataCounters rtx_rtp_stats_ RTC_GUARDED_BY(worker_queue_);

	std::vector<BitrateTracker> send_rates_ RTC_GUARDED_BY(worker_queue_);
	absl::optional<std::pair<FecProtectionParams, FecProtectionParams>>
		pending_fec_params_ RTC_GUARDED_BY(worker_queue_);

	RepeatingTaskHandle update_task_ RTC_GUARDED_BY(worker_queue_);
	std::vector<Packet> packets_to_send_ RTC_GUARDED_BY(worker_queue_);
	ScopedTaskSafety task_safety_;

private:
	RTC_DISALLOW_COPY_AND_ASSIGN(RTCRtpSenderEgress);
};

}

#endif	// __RTC_MODULE_RTP_RTCP_SOURCE_RTP_SENDER_EGRESS__