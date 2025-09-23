//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /module/pacing/pacing_controller.h
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_MODULE_PACING_RTC_PACING_CONTROLLER_H__
#define __RTC_MODULE_PACING_RTC_PACING_CONTROLLER_H__

#include <stddef.h>
#include <stdint.h>

#include <array>
#include <atomic>
#include <memory>
#include <vector>

#include "absl/types/optional.h"

#include "api/transport/network_types.h"
#include "modules/pacing/bitrate_prober.h"
#include "modules/pacing/interval_budget.h"
#include "modules/pacing/prioritized_packet_queue.h"
#include "modules/rtp_rtcp/source/rtp_packet_to_send.h"
#include "rtc_base/thread_annotations.h"

namespace webrtc {

class RTCPacingController {
public:
	class RTCPacketSender {
	public:
		virtual ~RTCPacketSender() = default;
		virtual void SendPacket(std::unique_ptr<RtpPacketToSend> packet,
			const PacedPacketInfo& cluster_info) = 0;
		// Should be called after each call to SendPacket().
		virtual std::vector<std::unique_ptr<RtpPacketToSend>> FetchFec() = 0;
		virtual std::vector<std::unique_ptr<RtpPacketToSend>> GeneratePadding(
			DataSize size) = 0;
		// TODO(bugs.webrtc.org/1439830): Make pure virtual once subclasses adapt.
		virtual void OnBatchComplete() {}

		// TODO(bugs.webrtc.org/11340): Make pure virtual once downstream projects
		// have been updated.
		virtual void OnAbortedRetransmissions(
			uint32_t ssrc,
			rtc::ArrayView<const uint16_t> sequence_numbers) {}
		virtual absl::optional<uint32_t> GetRtxSsrcForMedia(uint32_t ssrc) const {
			return absl::nullopt;
		}
	};

	static const TimeDelta kMaxExpectedQueueLength;
	static const TimeDelta kPausedProcessInterval;
	static const TimeDelta kMinSleepTime;
	static const TimeDelta kTargetPaddingDuration;
	static const TimeDelta kMaxPaddingReplayDuration;
	static const TimeDelta kMaxEarlyProbeProcessing;

	RTCPacingController(Clock *clock,
						RTCPacketSender* packet_sender);
	~RTCPacingController();

	void EnqueuePacket(std::unique_ptr<RtpPacketToSend> packet);
	void CreateProbeClusters(
		rtc::ArrayView<const ProbeClusterConfig> probe_cluster_configs);

	void Pause();   // Temporarily pause all sending.
	void Resume();  // Resume sending packets.
	bool IsPaused() const;

	void SetCongested(bool congested);

	void SetPacingRates(DataRate pacing_rate, DataRate padding_rate);
	DataRate pacing_rate() const { return adjusted_media_rate_; }

	void SetAccountForAudioPackets(bool account_for_audio);
	void SetIncludeOverhead();

	void SetTransportOverhead(DataSize overhead_per_packet);
	void SetSendBurstInterval(TimeDelta burst_interval);

	Timestamp OldestPacketEnqueueTime() const;
	size_t QueueSizePackets() const;

	const std::array<int, kNumMediaTypes>& SizeInPacketsPerRtpPacketMediaType() const;
	DataSize QueueSizeData() const;
	DataSize CurrentBufferLevel() const;

	absl::optional<Timestamp> FirstSentPacketTime() const;
	TimeDelta ExpectedQueueTime() const;
	void SetQueueTimeLimit(TimeDelta limit);

	void SetProbingEnabled(bool enabled);
	Timestamp NextSendTime() const;

	void ProcessPackets();
	bool IsProbing() const;

	void SetCircuitBreakerThreshold(int num_iterations);
	void RemovePacketsForSsrc(uint32_t ssrc);

	//////////////////////////////////////////////////////////////////////////
	// add kimi... for test
	bool IsEmptyOrPause();
	void ElapsedDebtUpdate(Timestamp now);	// test fail 
	//////////////////////////////////////////////////////////////////////////

private:
	TimeDelta UpdateTimeAndGetElapsed(Timestamp now);
	bool ShouldSendKeepalive(Timestamp now) const;

	void UpdateBudgetWithElapsedTime(TimeDelta delta);
	void UpdateBudgetWithSentData(DataSize size);
	void UpdatePaddingBudgetWithSentData(DataSize size);

	DataSize PaddingToAdd(DataSize recommended_probe_size,
		DataSize data_sent) const;

	std::unique_ptr<RtpPacketToSend> GetPendingPacket(
		const PacedPacketInfo& pacing_info,
		Timestamp target_send_time,
		Timestamp now);
	void OnPacketSent(RtpPacketMediaType packet_type,
		DataSize packet_size,
		Timestamp send_time);
	void MaybeUpdateMediaRateDueToLongQueue(Timestamp now);

	Timestamp CurrentTime() const;
	Timestamp NextUnpacedSendTime() const;

private:
	Clock* const clock_;
	RTCPacketSender* const packet_sender_;

	const bool drain_large_queues_;
	const bool send_padding_if_silent_;
	const bool pace_audio_;
	const bool ignore_transport_overhead_;
	const bool fast_retransmissions_;

	DataSize transport_overhead_per_packet_;
	TimeDelta send_burst_interval_;

	mutable Timestamp last_timestamp_;
	bool paused_;

	DataSize media_debt_;
	DataSize padding_debt_;

	DataRate pacing_rate_;
	DataRate adjusted_media_rate_;
	DataRate padding_rate_;

	BitrateProber prober_;
	bool probing_send_failure_;

	Timestamp last_process_time_;
	Timestamp last_send_time_;
	absl::optional<Timestamp> first_sent_packet_time_;
	bool seen_first_packet_;

	PrioritizedPacketQueue packet_queue_;

	bool congested_;

	TimeDelta queue_time_limit_;
	bool account_for_audio_;
	bool include_overhead_;

	int circuit_breaker_threshold_;

private:
	RTC_DISALLOW_COPY_AND_ASSIGN(RTCPacingController);
};

}

#endif //__RTC_MODULE_PACING_RTC_PACING_CONTROLLER_H__