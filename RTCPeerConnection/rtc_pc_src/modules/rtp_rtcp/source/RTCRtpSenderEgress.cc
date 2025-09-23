//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /module/rtp_rtcp/rtp_sender_egress.cc
//
//////////////////////////////////////////////////////////////////////////

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

#include "rtc_base/logging.h"

#include "RTCRtpSenderEgress.h"

#include "../rtc_pc_src/_deprecate_defines.h"

namespace webrtc {
namespace {
constexpr uint32_t kTimestampTicksPerMs = 90;
constexpr TimeDelta kSendSideDelayWindow = TimeDelta::Seconds(1);
constexpr TimeDelta kBitrateStatisticsWindow = TimeDelta::Seconds(1);
constexpr TimeDelta kUpdateInterval = kBitrateStatisticsWindow;
}  // namespace

RTCRtpSenderEgress::RTCNonPacedPacketSender::RTCNonPacedPacketSender(
	TaskQueueBase& worker_queue,
	RTCRtpSenderEgress* sender,
	PacketSequencer* sequencer)
	: worker_queue_(worker_queue),
	  sender_(sender),
	  sequencer_(sequencer),
	  twcc_seq_generator_(nullptr){
	RTC_DCHECK(sequencer);
}

RTCRtpSenderEgress::RTCNonPacedPacketSender::~RTCNonPacedPacketSender() {
	RTC_DCHECK_RUN_ON(&worker_queue_);
}

void RTCRtpSenderEgress::RTCNonPacedPacketSender::EnqueuePackets(
	std::vector<std::unique_ptr<RtpPacketToSend>> packets) {
	if (!worker_queue_.IsCurrent()) {
		worker_queue_.PostTask(SafeTask(
			task_safety_.flag(), [this, packets = std::move(packets)]() mutable {
			EnqueuePackets(std::move(packets));
		}));
		return;
	}
	RTC_DCHECK_RUN_ON(&worker_queue_);
	for (auto& packet : packets) {
		PrepareForSend(packet.get());
		sender_->SendPacket(std::move(packet), PacedPacketInfo());
	}

	// current disable...
// 	auto fec_packets = sender_->FetchFecPackets();
// 	if (!fec_packets.empty()) {
// 		EnqueuePackets(std::move(fec_packets));
// 	}
}

void RTCRtpSenderEgress::RTCNonPacedPacketSender::EnqueuePacket(std::unique_ptr<RtpPacketToSend> packet) {
	if (!worker_queue_.IsCurrent()) {
		worker_queue_.PostTask(SafeTask(
			task_safety_.flag(), [this, packet = std::move(packet)]() mutable {
			EnqueuePacket(std::move(packet));
		}));
		return;
	}
	RTC_DCHECK_RUN_ON(&worker_queue_);
	PrepareForSend(packet.get());
	sender_->SendPacket(std::move(packet), PacedPacketInfo());
}

void RTCRtpSenderEgress::RTCNonPacedPacketSender::SetTransportSeqGenerator(RTCTransportSequenceNumberGenerator* twcc_seq_generator) {
	twcc_seq_generator_ = twcc_seq_generator;
}

void RTCRtpSenderEgress::RTCNonPacedPacketSender::PrepareForSend(
	RtpPacketToSend* packet) {

	if (twcc_seq_generator_ != nullptr) {
		uint64_t transprot_sequence_number = twcc_seq_generator_->GetTransportSequenceNumber();
		packet->SetExtension<TransportSequenceNumber>(transprot_sequence_number & 0xFFFF);
	}

	packet->ReserveExtension<TransmissionOffset>();
	packet->ReserveExtension<AbsoluteSendTime>();
}

RTCRtpSenderEgress::RTCRtpSenderEgress(const RTCRtpRtcpInterface::Configuration& config,
	RtpPacketHistory* packet_history) 
	: enable_send_packet_batching_(false),
	  worker_queue_(TaskQueueBase::Current()),
	  ssrc_(config.local_media_ssrc),
	  rtx_ssrc_(config.rtx_send_ssrc), 
	  flexfec_ssrc_(absl::nullopt),
	  clock_(config.clock), 
	  packet_history_(packet_history), 
	  transport_(config.outgoing_transport), 
	  is_audio_(config.audio), 
	  transport_feedback_observer_(config.transport_feedback_callback),
	  send_side_delay_observer_(config.send_side_delay_observer),
	  send_packet_observer_(config.send_packet_observer),
	  rtp_stats_callback_(config.rtp_stats_callback),
	  bitrate_callback_(config.send_bitrate_observer), 
	  media_has_been_sent_(false), 
	  max_delay_it_(send_delays_.end()),
	  sum_delays_(TimeDelta::Zero()),
	  send_rates_(kNumMediaTypes, BitrateTracker(kBitrateStatisticsWindow)) {
	RTC_DCHECK(worker_queue_);
	if (bitrate_callback_) {
		update_task_ = RepeatingTaskHandle::DelayedStart(worker_queue_,
			kUpdateInterval, [this]() {
			PeriodicUpdate();
			return kUpdateInterval;
		});
	}
}

RTCRtpSenderEgress::~RTCRtpSenderEgress() {
	RTC_DCHECK_RUN_ON(worker_queue_);
	update_task_.Stop();
}

void RTCRtpSenderEgress::SendPacket(std::unique_ptr<RtpPacketToSend> packet,
	const PacedPacketInfo& pacing_info) {
	RTC_DCHECK_RUN_ON(worker_queue_);
	RTC_DCHECK(packet);

	if (packet->Ssrc() == ssrc_ &&
		packet->packet_type() != RtpPacketMediaType::kRetransmission) {
		last_sent_seq_ = packet->SequenceNumber();
	}
	else if (packet->Ssrc() == rtx_ssrc_) {
		last_sent_rtx_seq_ = packet->SequenceNumber();
	}

	RTC_DCHECK(packet->packet_type().has_value());
	RTC_DCHECK(HasCorrectSsrc(*packet));

	const Timestamp now = clock_->CurrentTime();

	if (packet->HasExtension<TransmissionOffset>() &&
		packet->capture_time_ms() > 0ll) {
		int64_t diff_ms = now.ms() - packet->capture_time_ms();
		packet->SetExtension<TransmissionOffset>(kTimestampTicksPerMs * diff_ms);
	}
	if (packet->HasExtension<AbsoluteSendTime>()) {
		packet->SetExtension<AbsoluteSendTime>(AbsoluteSendTime::MsTo24Bits(now.ms()));
	}

	auto compound_packet = Packet{ std::move(packet), pacing_info, now };
	if (enable_send_packet_batching_ && !is_audio_) {
		packets_to_send_.push_back(std::move(compound_packet));
	}
	else {
		CompleteSendPacket(compound_packet, false);
	}
}

void RTCRtpSenderEgress::OnBatchComplete() {
	RTC_DCHECK_RUN_ON(worker_queue_);
	for (auto& packet : packets_to_send_) {
		CompleteSendPacket(packet, &packet == &packets_to_send_.back());
	}

	packets_to_send_.clear();
}

void RTCRtpSenderEgress::CompleteSendPacket(const Packet& compound_packet,
	bool last_in_batch) {
	RTC_DCHECK_RUN_ON(worker_queue_);

	auto&[packet, pacing_info, now] = compound_packet;
	RTC_CHECK(packet);

	const bool is_media = packet->packet_type() == RtpPacketMediaType::kAudio ||
		packet->packet_type() == RtpPacketMediaType::kVideo;

	PacketOptions options;
	options.is_retransmit = !is_media;
	absl::optional<uint16_t> packet_id =
		packet->GetExtension<TransportSequenceNumber>();
	if (packet_id.has_value()) {
		options.packet_id = *packet_id;
		options.included_in_feedback = true;
		options.included_in_allocation = true;
		AddPacketToTransportFeedback(*packet_id, *packet, pacing_info);
	}

	options.additional_data = packet->additional_data();

	const uint32_t packet_ssrc = packet->Ssrc();
	if (packet->packet_type() != RtpPacketMediaType::kPadding &&
		packet->packet_type() != RtpPacketMediaType::kRetransmission) {
		UpdateDelayStatistics(Timestamp::Millis(packet->capture_time_ms()), now, packet_ssrc);
		UpdateOnSendPacket(packet_id, Timestamp::Millis(packet->capture_time_ms()), packet_ssrc);
	}
	options.batchable = enable_send_packet_batching_ && !is_audio_;
	options.last_packet_in_batch = last_in_batch;

	const bool send_success = SendPacketToNetwork(*packet, options, pacing_info);

	if (is_media && packet->allow_retransmission()) {
		packet_history_->PutRtpPacket(std::make_unique<RtpPacketToSend>(*packet),
			now.ms());
	}
	else if (packet->retransmitted_sequence_number()) {
		packet_history_->MarkPacketAsSent(*packet->retransmitted_sequence_number());
	}

	if (send_success) {
		media_has_been_sent_ = true;

		RTC_DCHECK(packet->packet_type().has_value());
		RtpPacketMediaType packet_type = *packet->packet_type();
		RtpPacketCounter counter(*packet);
		size_t size = packet->size();
		UpdateRtpStats(now, packet_ssrc, packet_type, std::move(counter), size);
	}
}

RtpSendRates RTCRtpSenderEgress::GetSendRates(Timestamp now) const {
	RTC_DCHECK_RUN_ON(worker_queue_);
	RtpSendRates current_rates;
	for (size_t i = 0; i < kNumMediaTypes; ++i) {
		RtpPacketMediaType type = static_cast<RtpPacketMediaType>(i);
		current_rates[type] = send_rates_[i].Rate(now).value_or(DataRate::Zero());
	}
	return current_rates;
}

void RTCRtpSenderEgress::GetDataCounters(StreamDataCounters* rtp_stats,
	StreamDataCounters* rtx_stats) const {
	RTC_DCHECK_RUN_ON(worker_queue_);
	*rtp_stats = rtp_stats_;
	*rtx_stats = rtx_rtp_stats_;
}

bool RTCRtpSenderEgress::MediaHasBeenSent() const {
	RTC_DCHECK_RUN_ON(worker_queue_);

	return media_has_been_sent_;
}

void RTCRtpSenderEgress::SetFecProtectionParameters(
	const FecProtectionParams& delta_params,
	const FecProtectionParams& key_params) {
	RTC_DCHECK_RUN_ON(worker_queue_);
	pending_fec_params_.emplace(delta_params, key_params);
}

void RTCRtpSenderEgress::OnAbortedRetransmissions(
	rtc::ArrayView<const uint16_t> sequence_numbers) {
	RTC_DCHECK_RUN_ON(worker_queue_);

	for (uint16_t seq_no : sequence_numbers) {
		packet_history_->MarkPacketAsSent(seq_no);
	}
}

bool RTCRtpSenderEgress::HasCorrectSsrc(const RtpPacketToSend& packet) const {
	switch (*packet.packet_type()) {
	case RtpPacketMediaType::kAudio:
	case RtpPacketMediaType::kVideo:
		return packet.Ssrc() == ssrc_;
	case RtpPacketMediaType::kRetransmission:
	case RtpPacketMediaType::kPadding:
		// Both padding and retransmission must be on either the media or the
		// RTX stream.
		return packet.Ssrc() == rtx_ssrc_ || packet.Ssrc() == ssrc_;
	case RtpPacketMediaType::kForwardErrorCorrection:
		// FlexFEC is on separate SSRC, ULPFEC uses media SSRC.
		return packet.Ssrc() == ssrc_ || packet.Ssrc() == flexfec_ssrc_;
	}
	return false;
}

void RTCRtpSenderEgress::AddPacketToTransportFeedback(uint16_t packet_id,
													  const RtpPacketToSend& packet,
													  const PacedPacketInfo& pacing_info) {
	if (transport_feedback_observer_) {
		RtpPacketSendInfo packet_info;
		packet_info.transport_sequence_number = packet_id;
		packet_info.rtp_timestamp = packet.Timestamp();
		packet_info.length = packet.size();
		packet_info.pacing_info = pacing_info;
		packet_info.packet_type = packet.packet_type();

		switch (*packet_info.packet_type) {
		case RtpPacketMediaType::kAudio:
		case RtpPacketMediaType::kVideo:
			packet_info.media_ssrc = ssrc_;
			packet_info.rtp_sequence_number = packet.SequenceNumber();
			break;
		case RtpPacketMediaType::kRetransmission:
			// For retransmissions, we're want to remove the original media packet
			// if the retransmit arrives - so populate that in the packet info.
			packet_info.media_ssrc = ssrc_;
			packet_info.rtp_sequence_number =
				*packet.retransmitted_sequence_number();
			break;
		case RtpPacketMediaType::kPadding:
		case RtpPacketMediaType::kForwardErrorCorrection:
			// We're not interested in feedback about these packets being received
			// or lost.
			break;
		}

		transport_feedback_observer_->OnAddPacket(packet_info);
	}
}

void RTCRtpSenderEgress::UpdateDelayStatistics(Timestamp capture_time,
											   Timestamp now,
											   uint32_t ssrc) {
	RTC_DCHECK_RUN_ON(worker_queue_);

	if (!send_side_delay_observer_ || capture_time.IsInfinite())
		return;

	TimeDelta avg_delay = TimeDelta::Zero();
	TimeDelta max_delay = TimeDelta::Zero();
	{
		// Compute the max and average of the recent capture-to-send delays.
		// The time complexity of the current approach depends on the distribution
		// of the delay values. This could be done more efficiently.

		// Remove elements older than kSendSideDelayWindowMs.
		auto lower_bound = send_delays_.lower_bound(now - kSendSideDelayWindow);
		for (auto it = send_delays_.begin(); it != lower_bound; ++it) {
			if (max_delay_it_ == it) {
				max_delay_it_ = send_delays_.end();
			}
			sum_delays_ -= it->second;
		}
		send_delays_.erase(send_delays_.begin(), lower_bound);
		if (max_delay_it_ == send_delays_.end()) {
			// Removed the previous max. Need to recompute.
			RecomputeMaxSendDelay();
		}

		// Add the new element.
		TimeDelta new_send_delay = now - capture_time;
		auto[it, inserted] = send_delays_.emplace(now, new_send_delay);
		if (!inserted) {
			// TODO(terelius): If we have multiple delay measurements during the same
			// millisecond then we keep the most recent one. It is not clear that this
			// is the right decision, but it preserves an earlier behavior.
			TimeDelta previous_send_delay = it->second;
			sum_delays_ -= previous_send_delay;
			it->second = new_send_delay;
			if (max_delay_it_ == it && new_send_delay < previous_send_delay) {
				RecomputeMaxSendDelay();
			}
		}
		if (max_delay_it_ == send_delays_.end() ||
			it->second >= max_delay_it_->second) {
			max_delay_it_ = it;
		}
		sum_delays_ += new_send_delay;

		size_t num_delays = send_delays_.size();
		RTC_DCHECK(max_delay_it_ != send_delays_.end());
		max_delay = max_delay_it_->second;
		avg_delay = sum_delays_ / num_delays;
	}
	send_side_delay_observer_->SendSideDelayUpdated(avg_delay.ms(),
		max_delay.ms(), ssrc);
}

void RTCRtpSenderEgress::RecomputeMaxSendDelay() {
	RTC_DCHECK_RUN_ON(worker_queue_);
	max_delay_it_ = send_delays_.begin();
	for (auto it = send_delays_.begin(); it != send_delays_.end(); ++it) {
		if (it->second >= max_delay_it_->second) {
			max_delay_it_ = it;
		}
	}
}

void RTCRtpSenderEgress::UpdateOnSendPacket(absl::optional<uint16_t> packet_id,
											Timestamp capture_time,
											uint32_t ssrc) {
	if (!send_packet_observer_ || capture_time.IsInfinite() || !packet_id.has_value()) {
		return;
	}

	send_packet_observer_->OnSendPacket(packet_id.value(), capture_time.ms(), ssrc);
}

bool RTCRtpSenderEgress::SendPacketToNetwork(const RtpPacketToSend& packet,
											 const PacketOptions& options,
											 const PacedPacketInfo& pacing_info) {
	RTC_DCHECK_RUN_ON(worker_queue_);
	int bytes_sent = -1;
	if (transport_) {
		bytes_sent = transport_->SendRtp(packet.data(), packet.size(), options)
			? static_cast<int>(packet.size())
			: -1;
	}

	if (bytes_sent <= 0) {
		RTC_LOG(LS_WARNING) << "Transport failed to send packet.";
		return false;
	}
	return true;
}

void RTCRtpSenderEgress::UpdateRtpStats(Timestamp now,
										uint32_t packet_ssrc,
										RtpPacketMediaType packet_type,
										RtpPacketCounter counter,
										size_t packet_size) {
	RTC_DCHECK_RUN_ON(worker_queue_);

	// TODO(bugs.webrtc.org/11581): send_rates_ should be touched only on the
	// worker thread.
	RtpSendRates send_rates;

	StreamDataCounters* counters =
		packet_ssrc == rtx_ssrc_ ? &rtx_rtp_stats_ : &rtp_stats_;

	counters->MaybeSetFirstPacketTime(now);

	if (packet_type == RtpPacketMediaType::kForwardErrorCorrection) {
		counters->fec.Add(counter);
	}
	else if (packet_type == RtpPacketMediaType::kRetransmission) {
		counters->retransmitted.Add(counter);
	}
	counters->transmitted.Add(counter);

	send_rates_[static_cast<size_t>(packet_type)].Update(packet_size, now);
	if (bitrate_callback_) {
		send_rates = GetSendRates(now);
	}

	if (rtp_stats_callback_) {
		rtp_stats_callback_->DataCountersUpdated(*counters, packet_ssrc);
	}

	// The bitrate_callback_ and rtp_stats_callback_ pointers in practice point
	// to the same object, so these callbacks could be consolidated into one.
	if (bitrate_callback_) {
		bitrate_callback_->Notify(
			send_rates.Sum().bps(),
			send_rates[RtpPacketMediaType::kRetransmission].bps(), ssrc_);
	}
}

void RTCRtpSenderEgress::PeriodicUpdate() {
	RTC_DCHECK_RUN_ON(worker_queue_);
	RTC_DCHECK(bitrate_callback_);
	RtpSendRates send_rates = GetSendRates(clock_->CurrentTime());
	bitrate_callback_->Notify(
		send_rates.Sum().bps(),
		send_rates[RtpPacketMediaType::kRetransmission].bps(), ssrc_);
}

}
