//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#include "../rtc_pc_src/_deprecate_defines.h"

#include "rtc_base/logging.h"
#include "modules/rtp_rtcp/source/byte_io.h"
#include "modules/rtp_rtcp/source/rtp_packet_to_send.h"
#include "modules/rtp_rtcp/source/rtp_utility.h"
#include "modules/rtp_rtcp/source/time_util.h"

#include "RTCRtpRtcpSendImpl.h"

namespace {
	// RED header is first byte of payload, if present.
	constexpr size_t kRedForFecHeaderLength = 1;

	// Timestamps use a 90kHz clock.
	constexpr uint32_t kTimestampTicksPerMs = 90;
}  // namespace

namespace webrtc {

static const int kMinSendSidePacketHistorySize = 1000; // 600;
constexpr uint16_t kMaxInitRtpSeqNumber = 32767;  // 2^15 -1.
constexpr TimeDelta kRttUpdateInterval = TimeDelta::Millis(1000);
const int64_t kRtpRtcpRttProcessTimeMs = 1000;
static const int kMinSendSideRtxPacketHistorySize = 100;
constexpr int kBitrateStatisticsWindowMs = 1000;

#define INIT_RTCP_SEND_TIME_STAMP_OFFSET			0
#define INIT_SIMULCAST_PROXY_TIMESTAMP			270000	


RTCRtpRtcpSendImpl::RTCRtpSenderContext::RTCRtpSenderContext(TaskQueueBase& worker_queue,
	const RTCRtpRtcpInterface::Configuration& config)
	: packet_history(config.clock, true), 
	  sequencer(config.local_media_ssrc,
	            config.rtx_send_ssrc,
	            !config.audio,
	            config.clock),
	  packet_sender(config, &packet_history),
	  non_paced_sender(worker_queue, &packet_sender, &sequencer), 
	  packet_generator(
	  config,
	  &packet_history,
	  config.paced_sender ? config.paced_sender : &non_paced_sender) { }

std::unique_ptr<RTCRtpRtcpSendImpl> RTCRtpRtcpSendImpl::Create(
	Clock* clock, 
	RTCRtpRtcpInterface::Configuration &rtprtcp_config,
	RtcpRttStats* rtt_stats) {

	return std::make_unique<RTCRtpRtcpSendImpl>(
		clock,
		rtprtcp_config,
		rtt_stats);
}

RTCRtpRtcpSendImpl::RTCRtpRtcpSendImpl(
	Clock* clock, RTCRtpRtcpInterface::Configuration &rtprtcp_config,
	RtcpRttStats* rtt_stats)
	: worker_queue_(TaskQueueBase::Current()),
	clock_(clock),
	rtcp_sender_(RTCRtcpSender::AddRtcpSendEvaluationCallback(
	RTCRtcpSender::Configuration::FromRtpRtcpConfiguration(rtprtcp_config),
			[this](TimeDelta duration) {
				ScheduleRtcpSendEvaluation(duration);
			})),
	rtcp_receiver_(rtprtcp_config, this),
	rtt_stats_(rtt_stats),
	ssrc_(rtprtcp_config.local_media_ssrc),
	rtx_ssrc_(rtprtcp_config.rtx_send_ssrc),
	is_video_(!rtprtcp_config.audio),
	rtt_ms_(0) {

	RTC_DCHECK(worker_queue_);
	rtcp_thread_checker_.Detach();

	if (!rtprtcp_config.receiver_only) {
		rtcp_sender_.SetTimestampOffset(INIT_SIMULCAST_PROXY_TIMESTAMP);
	}

	rtp_sender_ = std::make_unique<RTCRtpSenderContext>(*worker_queue_, rtprtcp_config);
	rtp_sender_->packet_history.SetStorePacketsStatus(
		RtpPacketHistory::StorageMode::kStoreAndCull,
		kMinSendSidePacketHistorySize);

	if (rtt_stats_) {
		rtt_update_task_ = RepeatingTaskHandle::DelayedStart(
			worker_queue_, kRttUpdateInterval, [this]() {
			PeriodicUpdate();
			return kRttUpdateInterval;
		});
	}
}

RTCRtpRtcpSendImpl::~RTCRtpRtcpSendImpl() {
	RTC_DCHECK_RUN_ON(worker_queue_);
	rtt_update_task_.Stop();
}

void RTCRtpRtcpSendImpl::SetTmmbn(
	std::vector<rtcp::TmmbItem> bounding_set) {
	rtcp_sender_.SetTmmbn(std::move(bounding_set));
}

void RTCRtpRtcpSendImpl::OnRequestSendReport() {
	SendRTCP(kRtcpSr);
}

void RTCRtpRtcpSendImpl::OnReceivedNack(
	const std::vector<uint16_t>& nack_sequence_numbers) {

	if (!rtp_sender_)
		return;
	
	int64_t rtt = rtt_ms();

	if (rtt == 0) {
		if (absl::optional<TimeDelta> average_rtt = rtcp_receiver_.AverageRtt()) {
			rtt = average_rtt->ms();
		}
	}

	rtp_sender_->packet_generator.OnReceivedNack(nack_sequence_numbers, rtt);
}

void RTCRtpRtcpSendImpl::OnReceivedRtcpReportBlocks(
	const ReportBlockList& report_blocks) {
	// nothing...
}

void RTCRtpRtcpSendImpl::SendCombinedRtcpPacket(
	std::vector<std::unique_ptr<rtcp::RtcpPacket>> rtcp_packets) {
	rtcp_sender_.SendCombinedRtcpPacket(std::move(rtcp_packets));
}

void RTCRtpRtcpSendImpl::SetRemb(int64_t bitrate_bps, std::vector<uint32_t> ssrcs) {
	rtcp_sender_.SetRemb(bitrate_bps, std::move(ssrcs));
}

void RTCRtpRtcpSendImpl::UnsetRemb() {
	rtcp_sender_.UnsetRemb();
}

void RTCRtpRtcpSendImpl::SetSendingMediaStatus(bool sending) {
	rtp_sender_->packet_generator.SetSendingMediaStatus(sending);
}

void RTCRtpRtcpSendImpl::SetVideoSendStreamConfig(
		const RTCVideoSendStream::Config* const config) {
	rtcp_sender_.SetRTCPStatus(RtcpMode::kCompound);
	rtcp_sender_.SetCNAME(config->rtp.c_name.c_str());
	rtcp_sender_.SetMaxRtpPacketSize(config->rtp.max_packet_size);
	rtcp_sender_.SetRtpClockRate(config->rtp.payload_type, kVideoPayloadTypeFrequency);

	rtp_sender_->packet_generator.SetRtxPayloadType(config->rtp.rtx.payload_type, config->rtp.payload_type);
	rtp_sender_->packet_generator.SetRTPHeaderExtensionMap(config->rtp.extensions);
	rtp_sender_->non_paced_sender.SetTransportSeqGenerator(config->transport_sequence_number_generator);
}

void RTCRtpRtcpSendImpl::SetAudioSendStreamConfig(
		const RTCAudioSendStream::Config* const config) {
	rtcp_sender_.SetRTCPStatus(RtcpMode::kCompound);
	rtcp_sender_.SetCNAME(config->rtp.c_name.c_str());
	rtcp_sender_.SetRtpClockRate(config->send_codec_spec->payload_type, config->send_codec_spec->format.clockrate_hz);

	rtp_sender_->packet_generator.SetRTPHeaderExtensionMap(config->rtp.extensions);
	rtp_sender_->non_paced_sender.SetTransportSeqGenerator(config->transport_sequence_number_generator);
}

void RTCRtpRtcpSendImpl::MaybeSendRtcp() {
	RTC_DCHECK_RUN_ON(worker_queue_);
	if (rtcp_sender_.TimeToSendRTCPReport())
		rtcp_sender_.SendRTCP(GetFeedbackState(), kRtcpReport);
}

void RTCRtpRtcpSendImpl::IncomingRtcpPacket(const uint8_t* packet, size_t length) {
	RTC_DCHECK_RUN_ON(&rtcp_thread_checker_);
	rtcp_receiver_.IncomingPacket(packet, length);
}

void RTCRtpRtcpSendImpl::SetRemoteSSRC(uint32_t ssrc) {
	rtcp_sender_.SetRemoteSSRC(ssrc);
	rtcp_receiver_.SetRemoteSSRC(ssrc);
}

void RTCRtpRtcpSendImpl::SetLocalSsrc(uint32_t local_ssrc) {
	RTC_DCHECK_RUN_ON(&rtcp_thread_checker_);
	rtcp_receiver_.set_local_media_ssrc(local_ssrc);
	rtcp_sender_.SetSsrc(local_ssrc);
}

int32_t RTCRtpRtcpSendImpl::SetSendingStatus(bool enabled) {
	if (rtcp_sender_.Sending() != enabled) {
		// Sends RTCP BYE when going from true to false
		rtcp_sender_.SetSendingStatus(GetFeedbackState(), enabled);
	}
	return 0;
}

void RTCRtpRtcpSendImpl::SetLastRtpTime(
	uint32_t rtp_timestamp, int64_t capture_time_ms, int8_t payload_type /* = -1 */) {
	rtcp_sender_.SetLastRtpTime(rtp_timestamp, Timestamp::Millis(capture_time_ms), payload_type);
}

absl::optional<TimeDelta> RTCRtpRtcpSendImpl::PeriodicUpdate() {
	RTC_DCHECK_RUN_ON(worker_queue_);
	Timestamp check_since = clock_->CurrentTime() - kRttUpdateInterval;
	absl::optional<TimeDelta> rtt =
		rtcp_receiver_.OnPeriodicRttUpdate(check_since, rtcp_sender_.Sending());

	if (rtt) {
		rtt_stats_->OnRttUpdate(rtt->ms());
		set_rtt_ms(rtt->ms());
	}

	if (rtcp_sender_.TMMBR() && rtcp_receiver_.UpdateTmmbrTimers()) {
		rtcp_receiver_.NotifyTmmbrUpdated();
	}

	return rtt;
}

void RTCRtpRtcpSendImpl::CheckTMMBR() {
	if (rtcp_sender_.TMMBR() && rtcp_receiver_.UpdateTmmbrTimers())
		rtcp_receiver_.NotifyTmmbrUpdated();
}

void RTCRtpRtcpSendImpl::set_rtt_ms(int64_t rtt_ms) {
	{
		MutexLock lock(&mutex_rtt_);
		rtt_ms_ = rtt_ms;
	}

	if (rtp_sender_) {
		rtp_sender_->packet_history.SetRtt(rtt_ms);
	}
}

int64_t RTCRtpRtcpSendImpl::rtt_ms() const {
	MutexLock lock(&mutex_rtt_);
	return rtt_ms_;
}

int32_t RTCRtpRtcpSendImpl::SendRTCP(RTCPPacketType packet_type) {
	return rtcp_sender_.SendRTCP(GetFeedbackState(), packet_type);
}

int32_t RTCRtpRtcpSendImpl::RemoteRTCPStat(
	std::vector<RTCPReportBlock>* receive_blocks) const {
//	return rtcp_receiver_.StatisticsReceived(receive_blocks);	
	// TODO ... rtcp_receiver_.GetLatestReportBlockData
	return 0;
}

void RTCRtpRtcpSendImpl::SetRTCPStatus(RtcpMode method) {
	rtcp_sender_.SetRTCPStatus(method);
}

absl::optional<TimeDelta> RTCRtpRtcpSendImpl::LastRtt() const {
	absl::optional<TimeDelta> rtt = rtcp_receiver_.LastRtt();
	if (!rtt.has_value()) {
		MutexLock lock(&mutex_rtt_);
		if (rtt_ms_ > 0) {
			rtt = TimeDelta::Millis(rtt_ms_);
		}
	}
	return rtt;
}

bool RTCRtpRtcpSendImpl::IsAudioConfigured() const {
	return !is_video_;
}

RtcpMode RTCRtpRtcpSendImpl::RTCP() const {
	return rtcp_sender_.Status();
}

std::vector<ReportBlockData> RTCRtpRtcpSendImpl::GetLatestReportBlockData() const {
	return rtcp_receiver_.GetLatestReportBlockData();
}

absl::optional<RTCRtpRtcpInterface::RTCSenderReportStats> RTCRtpRtcpSendImpl::GetSenderReportStats() const {
	return rtcp_receiver_.GetSenderReportStats();
}

absl::optional<RTCRtpRtcpInterface::RTCNonSenderRttStats> RTCRtpRtcpSendImpl::GetNonSenderRttStats() const {
	RTCRtcpReceiver::NonSenderRttStats non_sender_rtt_stats =
		rtcp_receiver_.GetNonSenderRTT();
	return{ {
			non_sender_rtt_stats.round_trip_time(),
			non_sender_rtt_stats.total_round_trip_time(),
			non_sender_rtt_stats.round_trip_time_measurements(),
	} };
}

bool RTCRtpRtcpSendImpl::SupportsPadding() const {
	// TODO...
	return true;
}

bool RTCRtpRtcpSendImpl::TrySendPacket(std::unique_ptr<RtpPacketToSend> packet, 
									   const PacedPacketInfo& pacing_info) {
	RTC_DCHECK(rtp_sender_);
	if (!rtp_sender_->packet_generator.SendingMedia()) {
		RTC_LOG(LS_INFO) << "RTCRtpRtcpSendImpl::TrySendPacket !SendingMedia";
		return false;
	}

	if (packet->packet_type() == RtpPacketMediaType::kPadding &&
		packet->Ssrc() == rtp_sender_->packet_generator.SSRC() &&
		!rtp_sender_->sequencer.CanSendPaddingOnMediaSsrc()) {
		// New media packet preempted this generated padding packet, discard it.
		return false;
	}

	rtp_sender_->sequencer.Sequence(*packet);
	rtp_sender_->packet_sender.SendPacket(std::move(packet), pacing_info);

	return true;
}

void RTCRtpRtcpSendImpl::EnqueuePacket(std::unique_ptr<RtpPacketToSend> packet) {
	rtp_sender_->packet_generator.EnqueuePacket(std::move(packet));
}

void RTCRtpRtcpSendImpl::OnBatchComplete() {

}

std::vector<std::unique_ptr<RtpPacketToSend>> RTCRtpRtcpSendImpl::GeneratePadding(
	size_t target_size_bytes) {
	return rtp_sender_->packet_generator.GeneratePadding(
		target_size_bytes, rtp_sender_->packet_sender.MediaHasBeenSent(),
		rtp_sender_->sequencer.CanSendPaddingOnMediaSsrc());
}

void RTCRtpRtcpSendImpl::OnAbortedRetransmissions(
	rtc::ArrayView<const uint16_t> sequence_numbers) {
	RTC_DCHECK(rtp_sender_);
	rtp_sender_->packet_sender.OnAbortedRetransmissions(sequence_numbers);
}

void RTCRtpRtcpSendImpl::OnPacketFeedbackVector(
	std::vector<StreamFeedbackObserver::StreamPacketInfo>& packet_feedback_vector) {
	if (!is_video_)
		return;

	std::map<uint32_t, std::vector<uint16_t>> acked_packets_per_ssrc;
	for (const StreamFeedbackObserver::StreamPacketInfo& packet : packet_feedback_vector) {
		if (packet.received && packet.ssrc) {
			acked_packets_per_ssrc[*packet.ssrc].push_back(packet.rtp_sequence_number);
		}
	}

	std::map<uint32_t, std::vector<uint16_t>> early_loss_detected_per_ssrc;

	for (const StreamFeedbackObserver::StreamPacketInfo& packet : packet_feedback_vector) {
		if (!packet.received && packet.ssrc && !packet.is_retransmission) {
			early_loss_detected_per_ssrc[*packet.ssrc].push_back(packet.rtp_sequence_number);
		}
		else {
			if (packet.ssrc) {
				early_loss_detected_per_ssrc.erase(*packet.ssrc);
			}
		}
	}

	for (const auto& kv : early_loss_detected_per_ssrc) {
		const uint32_t ssrc = kv.first;
		for (uint16_t sequence_number : kv.second) {
			rtp_sender_->packet_generator.ReSendPacket(sequence_number);
		}
	}

	for (const auto& kv : acked_packets_per_ssrc) {
		const uint32_t ssrc = kv.first;
		rtc::ArrayView<const uint16_t> rtp_sequence_numbers(kv.second);
		rtp_sender_->packet_history.CullAcknowledgedPackets(rtp_sequence_numbers);
	}
}

void RTCRtpRtcpSendImpl::SetRtxPayloadType(
	int payload_type, int associated_payload_type) {
	RTC_DCHECK_LE(payload_type, 127);
	RTC_DCHECK_LE(associated_payload_type, 127);

	if (payload_type < 0) {
		RTC_LOG(LS_ERROR) << "Invalid RTX payload type: " << payload_type << ".";
		return;
	}

	rtp_sender_->packet_generator.SetRtxPayloadType(payload_type, associated_payload_type);
}

RtpSendRates RTCRtpRtcpSendImpl::GetSendRates() const {
	return rtp_sender_->packet_sender.GetSendRates(clock_->CurrentTime());
}

RTCRtcpSender::FeedbackState RTCRtpRtcpSendImpl::GetFeedbackState() {
	RTCRtcpSender::FeedbackState state;
	StreamDataCounters rtp_stats;
	StreamDataCounters rtx_stats;

	rtp_sender_->packet_sender.GetDataCounters(&rtp_stats, &rtx_stats);

	state.packets_sent =
		rtp_stats.transmitted.packets + rtx_stats.transmitted.packets;
	state.media_bytes_sent = rtp_stats.transmitted.payload_bytes +
		rtx_stats.transmitted.payload_bytes;
	state.send_bitrate =  rtp_sender_->packet_sender.GetSendRates(clock_->CurrentTime()).Sum().bps<uint32_t>();

	state.receiver = &rtcp_receiver_;

	if (absl::optional<RTCRtpRtcpInterface::RTCSenderReportStats> last_sr =
		rtcp_receiver_.GetSenderReportStats();
		last_sr.has_value()) {
		state.remote_sr = CompactNtp(last_sr->last_remote_timestamp);
		state.last_rr = last_sr->last_arrival_timestamp;
	}

	state.last_xr_rtis = rtcp_receiver_.ConsumeReceivedXrReferenceTimeInfo();

	return state;
}

static void _dump_kimi_rtp_header_extension(RTPHeaderExtension extension) {
	RTC_LOG(LS_INFO) << "[[[[ _dump_rtp_header_extension";
	RTC_LOG(LS_INFO) << "extension hasTransmissionTimeOffset " << extension.hasTransmissionTimeOffset;
	RTC_LOG(LS_INFO) << "extension hasAbsoluteSendTime " << extension.hasAbsoluteSendTime;
	RTC_LOG(LS_INFO) << "extension hasTransportSequenceNumber " << extension.hasTransportSequenceNumber;
	RTC_LOG(LS_INFO) << "extension hasAudioLevel " << extension.hasAudioLevel;
	RTC_LOG(LS_INFO) << "extension hasVideoRotation " << extension.hasVideoRotation;
	RTC_LOG(LS_INFO) << "extension has_video_timing " << extension.has_video_timing;
	RTC_LOG(LS_INFO) << "extension playout_delay min_ms " << extension.playout_delay.min_ms << " max_ms " << extension.playout_delay.max_ms;
	RTC_LOG(LS_INFO) << "extension stream_id " << extension.stream_id.c_str();
	RTC_LOG(LS_INFO) << "extension repaired_stream_id " << extension.repaired_stream_id.c_str();
	RTC_LOG(LS_INFO) << "extension mid " << extension.mid.c_str();
	RTC_LOG(LS_INFO) << "_dump_rtp_header_extension]]]]";
}

std::unique_ptr<RtpPacketToSend> RTCRtpRtcpSendImpl::RtpPacketReceivedToSend(
	const RtpPacketReceived *received_packet, 
	RtpHeaderExtensionMap& header_extension_map,
	bool isVideo /* = true */) {

	std::unique_ptr<RtpPacketToSend> send_packet =
		std::make_unique<RtpPacketToSend>(&header_extension_map, IP_PACKET_SIZE - 28);

	send_packet->SetPayloadType(received_packet->PayloadType());
	send_packet->SetSequenceNumber(received_packet->SequenceNumber());	
	send_packet->SetSsrc(received_packet->Ssrc());
	send_packet->SetMarker(received_packet->Marker());
	send_packet->SetTimestamp(received_packet->Timestamp());
	const std::vector<uint32_t> csrcs = received_packet->Csrcs();
	send_packet->SetCsrcs(csrcs);

	bool is_set_mid = false;
	if (!received_packet->get_mid().empty()) {
		const std::string mid = received_packet->get_mid();
		bool is_set_ext = send_packet->SetExtension<RtpMid>(mid);
		is_set_mid = true;
	}

	for (int extension_num = kRtpExtensionNone + 1;
		extension_num < kRtpExtensionNumberOfExtensions; ++extension_num) {
		auto extension = static_cast<RTPExtensionType>(extension_num);

		if ((extension == kRtpExtensionMid && is_set_mid) ||
			extension == kRtpExtensionRtpStreamId) {
			continue;
		}

		if (!received_packet->HasExtension(extension)) {
			if (header_extension_map.IsRegistered(extension)) {
				if (extension == kRtpExtensionTransmissionTimeOffset) {
					send_packet->ReserveExtension<TransmissionOffset>();
				}
				else if (extension == kRtpExtensionAbsoluteSendTime) {
					send_packet->ReserveExtension<AbsoluteSendTime>();
				}
				else if (extension == kRtpExtensionTransportSequenceNumber) {
					send_packet->ReserveExtension<TransportSequenceNumber>();
				}
			}
			continue;
		}

		rtc::ArrayView<const uint8_t> source = received_packet->FindExtension(extension);

		rtc::ArrayView<uint8_t> destination =
			send_packet->AllocateExtension(extension, source.size());

		if (destination.empty() || source.size() != destination.size()) {
			continue;
		}

		std::memcpy(destination.begin(), source.begin(), destination.size());
	}

	uint8_t* send_payload = send_packet->AllocatePayload(received_packet->payload_size());
	auto received_payload = received_packet->payload();
	memcpy(send_payload, received_payload.data(), received_payload.size());

	if (isVideo) {
		send_packet->set_packet_type(RtpPacketMediaType::kVideo);
		send_packet->set_is_key_frame(received_packet->get_key_frame());
	}
	else {
		send_packet->set_packet_type(RtpPacketMediaType::kAudio);
	}

	return std::move(send_packet);
}

void RTCRtpRtcpSendImpl::MaybeSendRtcpAtOrAfterTimestamp(
	Timestamp execution_time) {
	RTC_DCHECK_RUN_ON(worker_queue_);
	Timestamp now = clock_->CurrentTime();
	if (now >= execution_time) {
		MaybeSendRtcp();
		return;
	}

	TimeDelta delta = execution_time - now;
	// TaskQueue may run task 1ms earlier, so don't print warning if in this case.
	if (delta > TimeDelta::Millis(1)) {
		RTC_DLOG(LS_WARNING) << "BUGBUG: Task queue scheduled delayed call "
			<< delta.ms() << " too early.";
	}

	ScheduleMaybeSendRtcpAtOrAfterTimestamp(execution_time, delta);
}

void RTCRtpRtcpSendImpl::ScheduleRtcpSendEvaluation(TimeDelta duration) {
	// We end up here under various sequences including the worker queue, and
	// the RTCPSender lock is held.
	// We're assuming that the fact that RTCPSender executes under other sequences
	// than the worker queue on which it's created on implies that external
	// synchronization is present and removes this activity before destruction.
	if (duration.IsZero()) {
		worker_queue_->PostTask(SafeTask(task_safety_.flag(), [this] {
			RTC_DCHECK_RUN_ON(worker_queue_);
			MaybeSendRtcp();
		}));
	}
	else {
		Timestamp execution_time = clock_->CurrentTime() + duration;
		ScheduleMaybeSendRtcpAtOrAfterTimestamp(execution_time, duration);
	}
}

void RTCRtpRtcpSendImpl::ScheduleMaybeSendRtcpAtOrAfterTimestamp(
	Timestamp execution_time,
	TimeDelta duration) {
	// We end up here under various sequences including the worker queue, and
	// the RTCPSender lock is held.
	// See note in ScheduleRtcpSendEvaluation about why `worker_queue_` can be
	// accessed.
	worker_queue_->PostDelayedTask(
		SafeTask(task_safety_.flag(),
		[this, execution_time] {
		RTC_DCHECK_RUN_ON(worker_queue_);
		MaybeSendRtcpAtOrAfterTimestamp(execution_time);
	}),
		duration.RoundUpTo(TimeDelta::Millis(1)));
}

}
