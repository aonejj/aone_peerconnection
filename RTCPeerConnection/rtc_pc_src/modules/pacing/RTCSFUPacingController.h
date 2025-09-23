//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_SFU_PACING_CONTROLLER_H__
#define __RTC_SFU_PACING_CONTROLLER_H__

#include <stddef.h>
#include <stdint.h>

#include <array>
#include <atomic>
#include <memory>
#include <vector>

#include "absl/types/optional.h"

#include "api/transport/network_types.h"
#include "modules/pacing/prioritized_packet_queue.h"
#include "modules/rtp_rtcp/source/rtp_packet_to_send.h"
#include "rtc_base/thread_annotations.h"

#include "RTCPacingController.h"	// TODO... disable... move to RTCPacingController::RTCPacketSender

namespace webrtc {

class BitrateProber;

class RTCSFUPacingController {
public:
	static const TimeDelta kMaxExpectedQueueLength;
	static const TimeDelta kPausedProcessInterval;
	static const TimeDelta kMinSleepTime;
	static const TimeDelta kTargetPaddingDuration;
	static const TimeDelta kMaxPaddingReplayDuration;
	static const TimeDelta kMaxEarlyProbeProcessing;

	RTCSFUPacingController(Clock *clock,
						   RTCPacingController::RTCPacketSender *packet_sender);
	~RTCSFUPacingController();

	void EnqueuePacket(std::unique_ptr<RtpPacketToSend> packet,
					   BitrateProber* prober);

	void Pause();
	void Resume();
	bool IsPaused() const;

	void SetCongested(bool congested);
	void SetPacingRates(DataRate pacing_rate, DataRate padding_rate);
	DataRate pacing_rate() const { return adjusted_media_rate_; }

	void SetIncludeOverhead();

	void SetTransportOverhead(DataSize overhead_per_packet);
	void SetSendBurstInterval(TimeDelta burst_interval);

	Timestamp OldestPacketEnqueueTime() const;
	size_t QueueSizePackets() const;

	DataSize QueueSizeData() const;
	DataSize CurrentBufferLevel() const;

	absl::optional<Timestamp> FirstSentPacketTime() const;
	TimeDelta ExpectedQueueTime() const;
	void SetQueueTimeLimit(TimeDelta limit);

	Timestamp NextSendTime(BitrateProber* prober) const;

	void ProcessPackets(BitrateProber* prober);

	void SetCircuitBreakerThreshold(int num_iterations);
	void RemovePacketsForSsrc(uint32_t ssrc);

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
	RTCPacingController::RTCPacketSender* const packet_sender_;

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
	RTC_DISALLOW_COPY_AND_ASSIGN(RTCSFUPacingController);
};

}	// namespace webrtc

#endif // __RTC_SFU_PACING_CONTROLLER_H__