//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#include <string.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <utility>

#include "modules/rtp_rtcp/source/rtcp_packet/dlrr.h"
#include "modules/rtp_rtcp/source/rtp_rtcp_config.h"
#include "modules/rtp_rtcp/source/time_util.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"

#include "RTCRtpRtcpRecvImpl.h"

namespace webrtc {

const int64_t kDefaultExpectedRetransmissionTimeMs = 125;
constexpr TimeDelta kRttUpdateInterval = TimeDelta::Millis(1000);

// RTCRtcpSender::Configuration AddRtcpSendEvaluationCallback(
// 	RTCRtcpSender::Configuration config,
// 	std::function<void(TimeDelta)> send_evaluation_callback) {
// 	config.schedule_next_rtcp_send_evaluation_function =
// 		std::move(send_evaluation_callback);
// 	return config;
// }

RTCRtpRtcpRecvImpl::RTCRtpRtcpRecvImpl(
	const RTCRtpRtcpInterface::Configuration& configuration) 
	: worker_queue_(TaskQueueBase::Current()),
	  rtcp_sender_(RTCRtcpSender::AddRtcpSendEvaluationCallback(
		RTCRtcpSender::Configuration::FromRtpRtcpConfiguration(configuration),
		[this](TimeDelta duration) {
			ScheduleRtcpSendEvaluation(duration);
		})), 
	  rtcp_receiver_(configuration, this),
	  clock_(configuration.clock),
	  rtt_stats_(configuration.rtt_stats),
	  rtt_ms_(0),
	  rtx_(kRtxOff),
	  is_video_(!configuration.audio) {
	RTC_DCHECK(worker_queue_);

	rtcp_thread_checker_.Detach();

	if (rtt_stats_) {
		rtt_update_task_ = RepeatingTaskHandle::DelayedStart(
			worker_queue_, kRttUpdateInterval, [this]() {
			PeriodicUpdate();
			return kRttUpdateInterval;
		});
	}
}

RTCRtpRtcpRecvImpl::~RTCRtpRtcpRecvImpl() {
	RTC_DCHECK_RUN_ON(worker_queue_);
	rtt_update_task_.Stop();
}

// static
std::unique_ptr<RTCRtpRtcpRecvImpl> RTCRtpRtcpRecvImpl::Create(
	const Configuration& configuration) {
	RTC_DCHECK(configuration.clock);
	RTC_DCHECK(TaskQueueBase::Current());
	return std::make_unique<RTCRtpRtcpRecvImpl>(configuration);
}

void RTCRtpRtcpRecvImpl::IncomingRtcpPacket(const uint8_t* incoming_packet,
	size_t incoming_packet_length) {
	RTC_DCHECK_RUN_ON(&rtcp_thread_checker_);
	rtcp_receiver_.IncomingPacket(incoming_packet, incoming_packet_length);
}

void RTCRtpRtcpRecvImpl::SetRemoteSSRC(uint32_t ssrc) {
	rtcp_sender_.SetRemoteSSRC(ssrc);
	rtcp_receiver_.SetRemoteSSRC(ssrc);
}

void RTCRtpRtcpRecvImpl::SetLocalSsrc(uint32_t local_ssrc) {
	RTC_DCHECK_RUN_ON(&rtcp_thread_checker_);
	rtcp_receiver_.set_local_media_ssrc(local_ssrc);
	rtcp_sender_.SetSsrc(local_ssrc);
}

int32_t RTCRtpRtcpRecvImpl::SetSendingStatus(bool sending) {
	if (rtcp_sender_.Sending() != sending) {
		// Sends RTCP BYE when going from true to false
		rtcp_sender_.SetSendingStatus(GetFeedbackState(), sending);
	}
	return 0;
}

int32_t RTCRtpRtcpRecvImpl::SendRTCP(RTCPPacketType rtcp_packet_type) {
	return rtcp_sender_.SendRTCP(GetFeedbackState(), rtcp_packet_type);
}

int32_t RTCRtpRtcpRecvImpl::RemoteRTCPStat(
	std::vector<RTCPReportBlock>* receive_blocks) const {
	return 0;
}

void RTCRtpRtcpRecvImpl::SetRTCPStatus(RtcpMode method) {
	rtcp_sender_.SetRTCPStatus(method);
}

absl::optional<TimeDelta> RTCRtpRtcpRecvImpl::LastRtt() const {
	absl::optional<TimeDelta> rtt = rtcp_receiver_.LastRtt();
	if (!rtt.has_value()) {
		MutexLock lock(&mutex_rtt_);
		if (rtt_ms_ > 0) {
			rtt = TimeDelta::Millis(rtt_ms_);
		}
	}
	return rtt;
}

RtcpMode RTCRtpRtcpRecvImpl::RTCP() const {
	return rtcp_sender_.Status();
}

std::vector<ReportBlockData> RTCRtpRtcpRecvImpl::GetLatestReportBlockData() const {
	return rtcp_receiver_.GetLatestReportBlockData();
}

absl::optional<RTCRtpRtcpInterface::RTCSenderReportStats> RTCRtpRtcpRecvImpl::GetSenderReportStats() const {
	return rtcp_receiver_.GetSenderReportStats();
}

absl::optional<RTCRtpRtcpInterface::RTCNonSenderRttStats> RTCRtpRtcpRecvImpl::GetNonSenderRttStats() const {
	RTCRtcpReceiver::NonSenderRttStats non_sender_rtt_stats =
		rtcp_receiver_.GetNonSenderRTT();
	return{ {
			non_sender_rtt_stats.round_trip_time(),
			non_sender_rtt_stats.total_round_trip_time(),
			non_sender_rtt_stats.round_trip_time_measurements(),
	} };
}

bool RTCRtpRtcpRecvImpl::TrySendPacket(std::unique_ptr<RtpPacketToSend> packet, const PacedPacketInfo& pacing_info) {
	return false;
}

std::vector<std::unique_ptr<RtpPacketToSend>> RTCRtpRtcpRecvImpl::GeneratePadding(
	size_t target_size_bytes) {
	return std::vector<std::unique_ptr<RtpPacketToSend>>();
}


//////////////////////////////////////////////////////////////////////////
// RtcpFeedbackSenderInterface method
void RTCRtpRtcpRecvImpl::SendCombinedRtcpPacket(
	std::vector<std::unique_ptr<rtcp::RtcpPacket>> rtcp_packets) {

	rtcp_sender_.SendCombinedRtcpPacket(std::move(rtcp_packets));
}

void RTCRtpRtcpRecvImpl::SetRemb(int64_t bitrate_bps, 
							std::vector<uint32_t> ssrcs) {
	rtcp_sender_.SetRemb(bitrate_bps, std::move(ssrcs));
}

void RTCRtpRtcpRecvImpl::UnsetRemb() {
	rtcp_sender_.UnsetRemb();
}

//////////////////////////////////////////////////////////////////////////
// RTCPReceiver::ModuleRtpRtcp methods
void RTCRtpRtcpRecvImpl::SetTmmbn(std::vector<rtcp::TmmbItem> bounding_set) {
	rtcp_sender_.SetTmmbn(std::move(bounding_set));
}

void RTCRtpRtcpRecvImpl::OnRequestSendReport() {
	// unused
}

void RTCRtpRtcpRecvImpl::OnReceivedNack(const std::vector<uint16_t>& nack_sequence_numbers) {
	// unused
}

void RTCRtpRtcpRecvImpl::OnReceivedRtcpReportBlocks(const ReportBlockList& report_blocks) {
	// unused
}

int32_t RTCRtpRtcpRecvImpl::SendLossNotification(uint16_t last_decoded_seq_num,
	uint16_t last_received_seq_num,
	bool decodability_flag,
	bool buffering_allowed) {
	// unused
	return rtcp_sender_.SendLossNotification(
		GetFeedbackState(), last_decoded_seq_num, last_received_seq_num,
		decodability_flag, buffering_allowed);
}

void RTCRtpRtcpRecvImpl::SendNack(const std::vector<uint16_t>& sequence_numbers) {
	rtcp_sender_.SendRTCP(GetFeedbackState(), kRtcpNack, sequence_numbers.size(),
		sequence_numbers.data());
}

uint32_t RTCRtpRtcpRecvImpl::local_media_ssrc() const {
	RTC_DCHECK_RUN_ON(&rtcp_thread_checker_);
	RTC_DCHECK_EQ(rtcp_receiver_.local_media_ssrc(), rtcp_sender_.SSRC());
	return rtcp_receiver_.local_media_ssrc();
}

void RTCRtpRtcpRecvImpl::PeriodicUpdate() {
	RTC_DCHECK_RUN_ON(worker_queue_);
	
	Timestamp check_since = clock_->CurrentTime() - kRttUpdateInterval;
	absl::optional<TimeDelta> rtt =
		rtcp_receiver_.OnPeriodicRttUpdate(check_since, rtcp_sender_.Sending());
	if (rtt) {
		if (rtt_stats_) {
			rtt_stats_->OnRttUpdate(rtt->ms());
		}
		set_rtt_ms(rtt->ms());
	}
}

void RTCRtpRtcpRecvImpl::MaybeSendRtcp() {
	RTC_DCHECK_RUN_ON(worker_queue_);
	if (rtcp_sender_.TimeToSendRTCPReport())
		rtcp_sender_.SendRTCP(GetFeedbackState(), kRtcpReport);
}

void RTCRtpRtcpRecvImpl::ScheduleRtcpSendEvaluation(TimeDelta duration) {
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

void RTCRtpRtcpRecvImpl::MaybeSendRtcpAtOrAfterTimestamp(Timestamp execution_time) {
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

void RTCRtpRtcpRecvImpl::ScheduleMaybeSendRtcpAtOrAfterTimestamp(Timestamp execution_time,
		TimeDelta duration) {
	worker_queue_->PostDelayedTask(
		SafeTask(task_safety_.flag(),
		[this, execution_time] {
		RTC_DCHECK_RUN_ON(worker_queue_);
		MaybeSendRtcpAtOrAfterTimestamp(execution_time);
	}),
		duration.RoundUpTo(TimeDelta::Millis(1)));
}

RTCRtcpSender::FeedbackState RTCRtpRtcpRecvImpl::GetFeedbackState() {
	RTCRtcpSender::FeedbackState state;
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

void RTCRtpRtcpRecvImpl::set_rtt_ms(int64_t rtt_ms) {
	RTC_DCHECK_RUN_ON(worker_queue_);
	MutexLock lock(&mutex_rtt_);
	rtt_ms_ = rtt_ms;
}

int64_t RTCRtpRtcpRecvImpl::rtt_ms() const {
	MutexLock lock(&mutex_rtt_);
	return rtt_ms_;
}

}
