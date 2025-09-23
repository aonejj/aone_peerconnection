//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /module/pacing/task_queue_paced_sender.cc
//
//////////////////////////////////////////////////////////////////////////

#include "absl/cleanup/cleanup.h"
#include "api/transport/network_types.h"
#include "rtc_base/checks.h"
#include "rtc_base/time_utils.h"

#include "RTCTaskQueuePacedSender.h"

namespace webrtc {

const int RTCTaskQueuePacedSender::kNoPacketHoldback = -1;

RTCTaskQueuePacedSender::RTCTaskQueuePacedSender(
	Clock* clock,
	RTCPacingController::RTCPacketSender* packet_sender,
	TimeDelta max_hold_back_window,
	int max_hold_back_window_in_packets,
	absl::optional<TimeDelta> burst_interval /*= absl::nullopt*/) 
	: clock_(clock),
	  max_hold_back_window_(max_hold_back_window),
	  max_hold_back_window_in_packets_(max_hold_back_window_in_packets), 
#ifdef __USING_RTC_SFU_PACING__
	  packet_sender_(packet_sender),
#else
	  pacing_controller_(clock, packet_sender),
#endif
	  next_process_time_(Timestamp::MinusInfinity()),
	  is_started_(false), 
	  is_shutdown_(false), 
	  packet_size_(/*alpha=*/0.95),		
	  include_overhead_(false), 
	  task_queue_(TaskQueueBase::Current()) {
	RTC_DCHECK_GE(max_hold_back_window_, RTCPacingController::kMinSleepTime);

	burst_interval_ = burst_interval;

#ifndef __USING_RTC_SFU_PACING__
	if (burst_interval_.has_value()) {
		pacing_controller_.SetSendBurstInterval(burst_interval_.value());
	}
#endif

	pthread_create(&handle_, nullptr, RTCTaskQueuePacedSender::enqueue_thread_proc, (void*)this);

}

RTCTaskQueuePacedSender::~RTCTaskQueuePacedSender() {
	RTC_DCHECK_RUN_ON(task_queue_);
	is_shutdown_ = true;

	if (is_run_thread_) {
		pthread_mutex_lock(&mutex_);
		is_quiting_thread_ = true;
		pthread_cond_signal(&cond_);
		pthread_mutex_unlock(&mutex_);
		pthread_join(handle_, nullptr);
	}

}

void RTCTaskQueuePacedSender::EnsureStarted() {
	RTC_DCHECK_RUN_ON(task_queue_);
	is_started_ = true;
	MaybeProcessPackets(Timestamp::MinusInfinity());
}

void RTCTaskQueuePacedSender::EnqueuePackets(
	std::vector<std::unique_ptr<RtpPacketToSend>> packets) {
	task_queue_->PostTask(
		SafeTask(safety_.flag(), [this, packets = std::move(packets)]() mutable {
		RTC_DCHECK_RUN_ON(task_queue_);
		for (auto& packet : packets) {
			size_t packet_size = packet->payload_size() + packet->padding_size();
			if (include_overhead_) {
				packet_size += packet->headers_size();
			}
			packet_size_.Apply(1, packet_size);
			RTC_DCHECK_GE(packet->capture_time_ms(), 0ll);
#ifdef __USING_RTC_SFU_PACING__
			_enqueue_packet_with_sfu_pacing(std::move(packet));
#else
			pacing_controller_.EnqueuePacket(std::move(packet));
#endif
		}
		MaybeProcessPackets(Timestamp::MinusInfinity());
	}));
}

void RTCTaskQueuePacedSender::EnqueuePacket(std::unique_ptr<RtpPacketToSend> packet) {
	int64_t now = rtc::TimeMillis();

	if (is_run_thread_) {
		pthread_mutex_lock(&mutex_);
		enqueue_packets_.push_back(std::move(packet));
		pthread_cond_signal(&cond_);
		pthread_mutex_unlock(&mutex_);
	}
}

void RTCTaskQueuePacedSender::RemovePacketsForSsrc(uint32_t ssrc) {
#ifndef __USING_RTC_SFU_PACING__
	task_queue_->PostTask(SafeTask(safety_.flag(), [this, ssrc] {
		RTC_DCHECK_RUN_ON(task_queue_);
		pacing_controller_.RemovePacketsForSsrc(ssrc);

		MaybeProcessPackets(Timestamp::MinusInfinity());
	}));
#endif
}

void RTCTaskQueuePacedSender::CreateProbeClusters(
	std::vector<ProbeClusterConfig> probe_cluster_configs) {
	RTC_DCHECK_RUN_ON(task_queue_);
#ifdef __USING_RTC_SFU_PACING__
	for (const ProbeClusterConfig probe_cluster_config : probe_cluster_configs) {
		prober_.CreateProbeCluster(probe_cluster_config);
	}
#else
	pacing_controller_.CreateProbeClusters(probe_cluster_configs);
#endif
	MaybeScheduleProcessPackets();
}

void RTCTaskQueuePacedSender::Pause() {
	RTC_DCHECK_RUN_ON(task_queue_);
#ifdef __USING_RTC_SFU_PACING__
	_pause_with_sfu_pacing();
#else
	pacing_controller_.Pause();
#endif
}

void RTCTaskQueuePacedSender::Resume() {
	RTC_DCHECK_RUN_ON(task_queue_);
#ifdef __USING_RTC_SFU_PACING__
	_resume_with_sfu_pacing();
#else
	pacing_controller_.Resume();
#endif
	MaybeProcessPackets(Timestamp::MinusInfinity());
}

void RTCTaskQueuePacedSender::SetCongested(bool congested) {
	RTC_DCHECK_RUN_ON(task_queue_);
#ifdef __USING_RTC_SFU_PACING__
	_set_congested_with_sfu_pacing(congested);
#else
	pacing_controller_.SetCongested(congested);
#endif
	MaybeScheduleProcessPackets();
}

void RTCTaskQueuePacedSender::SetPacingRates(DataRate pacing_rate,
	DataRate padding_rate) {
	RTC_DCHECK_RUN_ON(task_queue_);
#ifdef __USING_RTC_SFU_PACING__
	pacing_rate_ = pacing_rate;
	padding_rate_ = padding_rate;
	_set_pacing_rate_with_sfu_pacing(pacing_rate, padding_rate);
#else
	pacing_controller_.SetPacingRates(pacing_rate, padding_rate);
#endif
	MaybeScheduleProcessPackets();
}

void RTCTaskQueuePacedSender::SetAccountForAudioPackets(bool account_for_audio) {
#ifndef __USING_RTC_SFU_PACING__
	RTC_DCHECK_RUN_ON(task_queue_);
	pacing_controller_.SetAccountForAudioPackets(account_for_audio);
	MaybeProcessPackets(Timestamp::MinusInfinity());
#endif
}

void RTCTaskQueuePacedSender::SetIncludeOverhead() {
	RTC_DCHECK_RUN_ON(task_queue_);
	include_overhead_ = true;
#ifdef __USING_RTC_SFU_PACING__
	_set_include_overhead_with_sfu_pacing();
#else
	pacing_controller_.SetIncludeOverhead();
#endif
	MaybeProcessPackets(Timestamp::MinusInfinity());
}

void RTCTaskQueuePacedSender::SetTransportOverhead(DataSize overhead_per_packet) {
	RTC_DCHECK_RUN_ON(task_queue_);
#ifdef __USING_RTC_SFU_PACING__
	_set_transport_overhead_with_sfu_pacing(overhead_per_packet);
#else
	pacing_controller_.SetTransportOverhead(overhead_per_packet);
#endif
	MaybeProcessPackets(Timestamp::MinusInfinity());
}

TimeDelta RTCTaskQueuePacedSender::OldestPacketWaitTime() const {
	Timestamp oldest_packet = GetStats().oldest_packet_enqueue_time;
	if (oldest_packet.IsInfinite()) {
		return TimeDelta::Zero();
	}

	// (webrtc:9716): The clock is not always monotonic.
	Timestamp current = clock_->CurrentTime();
	if (current < oldest_packet) {
		return TimeDelta::Zero();
	}

	return current - oldest_packet;
}

DataSize RTCTaskQueuePacedSender::QueueSizeData() const {
	return GetStats().queue_size;
}

absl::optional<Timestamp> RTCTaskQueuePacedSender::FirstSentPacketTime() const {
	return GetStats().first_sent_packet_time;
}

TimeDelta RTCTaskQueuePacedSender::ExpectedQueueTime() const {
	return GetStats().expected_queue_time;
}

void RTCTaskQueuePacedSender::SetQueueTimeLimit(TimeDelta limit) {
	RTC_DCHECK_RUN_ON(task_queue_);
#ifdef __USING_RTC_SFU_PACING__
	_set_queue_time_limit_with_sfu_pacing(limit);
#else
	pacing_controller_.SetQueueTimeLimit(limit);
#endif
	MaybeProcessPackets(Timestamp::MinusInfinity());
}

#ifdef __USING_RTC_SFU_PACING__
void RTCTaskQueuePacedSender::AddPacingRtpModule(RTCRtpRtcpInterface* rtp_module) {
	RTC_DCHECK(rtp_module != nullptr);
	bool isAudio = rtp_module->IsAudioConfigured();
	uint32_t ssrc = rtp_module->SSRC();
	absl::optional<uint32_t> rtx_ssrc = rtp_module->RtxSsrc();
	task_queue_->PostTask(SafeTask(safety_.flag(), [this, rtp_module, isAudio, ssrc, rtx_ssrc] {
		RTC_DCHECK_RUN_ON(task_queue_);
		_add_pacing_rtp_module(rtp_module, isAudio, ssrc, rtx_ssrc);
	}));
}

void RTCTaskQueuePacedSender::RemovePacingRtpModule(RTCRtpRtcpInterface* rtp_module) {
	RTC_DCHECK(rtp_module != nullptr);
	bool isAudio = rtp_module->IsAudioConfigured();
	uint32_t ssrc = rtp_module->SSRC();
	absl::optional<uint32_t> rtx_ssrc = rtp_module->RtxSsrc();

	task_queue_->PostTask(SafeTask(safety_.flag(), [this, rtp_module, isAudio, ssrc, rtx_ssrc] {
		RTC_DCHECK_RUN_ON(task_queue_);
		_remove_pacing_rtp_module(rtp_module, isAudio, ssrc, rtx_ssrc);
	}));
}
#endif

void RTCTaskQueuePacedSender::MaybeScheduleProcessPackets() {
	if (!processing_packets_)
		MaybeProcessPackets(Timestamp::MinusInfinity());
}

void RTCTaskQueuePacedSender::MaybeProcessPackets(
	Timestamp scheduled_process_time) {
	RTC_DCHECK_RUN_ON(task_queue_);

	if (is_shutdown_ || !is_started_) {
		return;
	}

	RTC_DCHECK(!processing_packets_);
	processing_packets_ = true;
	absl::Cleanup cleanup = [this] {
		RTC_DCHECK_RUN_ON(task_queue_);
		processing_packets_ = false;
	};

#ifdef __USING_RTC_SFU_PACING__
	_maybe_process_packets_with_sfu_pacing(scheduled_process_time);
#else
	Timestamp next_send_time = pacing_controller_.NextSendTime();
	RTC_DCHECK(next_send_time.IsFinite());
	const Timestamp now = clock_->CurrentTime();

	TimeDelta early_execute_margin =
		pacing_controller_.IsProbing()
		? RTCPacingController::kMaxEarlyProbeProcessing
		: TimeDelta::Zero();

	while (next_send_time <= now + early_execute_margin) {
		pacing_controller_.ProcessPackets();
		next_send_time = pacing_controller_.NextSendTime();
		RTC_DCHECK(next_send_time.IsFinite());
		early_execute_margin = pacing_controller_.IsProbing()
			? RTCPacingController::kMaxEarlyProbeProcessing
			: TimeDelta::Zero();
	}

	UpdateStats();

	if (scheduled_process_time.IsFinite()) {
		if (scheduled_process_time != next_process_time_) {
			return;
		}
		next_process_time_ = Timestamp::MinusInfinity();
	}

	// probing
	TimeDelta hold_back_window = TimeDelta::Zero();
	if (!pacing_controller_.IsProbing()) {
		hold_back_window = max_hold_back_window_;
		DataRate pacing_rate = pacing_controller_.pacing_rate();
		if (max_hold_back_window_in_packets_ != kNoPacketHoldback &&
			!pacing_rate.IsZero() &&
			packet_size_.filtered() != rtc::ExpFilter::kValueUndefined) {
			TimeDelta avg_packet_send_time =
				DataSize::Bytes(packet_size_.filtered()) / pacing_rate;
			hold_back_window =
				std::min(hold_back_window,
				avg_packet_send_time * max_hold_back_window_in_packets_);
		}
	}

	// Calculate next process time.
	TimeDelta time_to_next_process =
		std::max(hold_back_window, next_send_time - now - early_execute_margin);

	next_send_time = now + time_to_next_process;

	if (next_process_time_.IsMinusInfinity() ||
		next_process_time_ > next_send_time) {
		task_queue_->PostDelayedHighPrecisionTask(
			SafeTask(
			safety_.flag(),
			[this, next_send_time]() { MaybeProcessPackets(next_send_time); }),
			time_to_next_process.RoundUpTo(TimeDelta::Millis(1)));
		next_process_time_ = next_send_time;
	}
#endif
}

void RTCTaskQueuePacedSender::OnStatsUpdated(const Stats& stats) {
	RTC_DCHECK_RUN_ON(task_queue_);
	current_stats_ = stats;
}

void RTCTaskQueuePacedSender::UpdateStats() {
#ifndef __USING_RTC_SFU_PACING__
	Stats new_stats;
	new_stats.expected_queue_time = pacing_controller_.ExpectedQueueTime();
	new_stats.first_sent_packet_time = pacing_controller_.FirstSentPacketTime();
	new_stats.oldest_packet_enqueue_time =
		pacing_controller_.OldestPacketEnqueueTime();
	new_stats.queue_size = pacing_controller_.QueueSizeData();
	OnStatsUpdated(new_stats);
#endif
}

RTCTaskQueuePacedSender::Stats RTCTaskQueuePacedSender::GetStats() const {
	RTC_DCHECK_RUN_ON(task_queue_);
	return current_stats_;
}

#ifdef __USING_RTC_SFU_PACING__
void RTCTaskQueuePacedSender::_enqueue_packet_with_sfu_pacing(std::unique_ptr<RtpPacketToSend> packet) {
	auto it = ssrc_to_pacers_.find(packet->Ssrc());
	if(it == ssrc_to_pacers_.end()) {
		return;
	}
	it->second->EnqueuePacket(std::move(packet), &prober_);
}

void RTCTaskQueuePacedSender::_pause_with_sfu_pacing() {
	std::unordered_map<RTCRtpRtcpInterface*, std::unique_ptr<RTCSFUPacingController>>::iterator it;
	it = pacers_.begin();
	while (it != pacers_.end()) {
		it->second->Pause();
		it++;
	}
}

void RTCTaskQueuePacedSender::_resume_with_sfu_pacing() {
	std::unordered_map<RTCRtpRtcpInterface*, std::unique_ptr<RTCSFUPacingController>>::iterator it;
	it = pacers_.begin();
	while (it != pacers_.end()) {
		it->second->Resume();
		it++;
	}
}

void RTCTaskQueuePacedSender::_set_congested_with_sfu_pacing(bool congested) {
	std::unordered_map<RTCRtpRtcpInterface*, std::unique_ptr<RTCSFUPacingController>>::iterator it;
	it = pacers_.begin();
	while (it != pacers_.end()) {
		it->second->SetCongested(congested);
		it++;
	}
}

void RTCTaskQueuePacedSender::_set_pacing_rate_with_sfu_pacing(DataRate pacing_rate, DataRate padding_rate) {
	if(pacers_.empty()) {
		return;
	}
	uint16_t size = pacers_.size();
	int64_t tot_bps = pacing_rate.bps();
	int64_t bps = tot_bps/size;

	std::unordered_map<RTCRtpRtcpInterface*, std::unique_ptr<RTCSFUPacingController>>::iterator it;
	it = pacers_.begin();
	while (it != pacers_.end()) {
		it->second->SetPacingRates(DataRate::BitsPerSec(bps), padding_rate);
		it++;
	}
}

void RTCTaskQueuePacedSender::_set_include_overhead_with_sfu_pacing() {
	std::unordered_map<RTCRtpRtcpInterface*, std::unique_ptr<RTCSFUPacingController>>::iterator it;
	it = pacers_.begin();
	while (it != pacers_.end()) {
		it->second->SetIncludeOverhead();
		it++;
	}
}

void RTCTaskQueuePacedSender::_set_transport_overhead_with_sfu_pacing(DataSize overhead_per_packet) {
	std::unordered_map<RTCRtpRtcpInterface*, std::unique_ptr<RTCSFUPacingController>>::iterator it;
	it = pacers_.begin();
	while (it != pacers_.end()) {
		it->second->SetTransportOverhead(overhead_per_packet);
		it++;
	}
}

void RTCTaskQueuePacedSender::_set_queue_time_limit_with_sfu_pacing(TimeDelta limit) {
	std::unordered_map<RTCRtpRtcpInterface*, std::unique_ptr<RTCSFUPacingController>>::iterator it;
	it = pacers_.begin();
	while (it != pacers_.end()) {
		it->second->SetQueueTimeLimit(limit);
		it++;
	}
}

void RTCTaskQueuePacedSender::_maybe_process_packets_with_sfu_pacing(Timestamp scheduled_process_time) {
	if(pacers_.empty()) {
		return;
	}

	Timestamp min_next_send_time = Timestamp::PlusInfinity();
	const Timestamp now = clock_->CurrentTime();

	TimeDelta early_execute_margin = prober_.is_probing()
		? RTCPacingController::kMaxEarlyProbeProcessing
		: TimeDelta::Zero();

	std::unordered_map<RTCRtpRtcpInterface*, std::unique_ptr<RTCSFUPacingController>>::iterator it;
	it = pacers_.begin();
	while (it != pacers_.end()) {
		Timestamp next_send_time = it->second->NextSendTime(&prober_);
		RTC_DCHECK(next_send_time.IsFinite());

		while (next_send_time <= now + early_execute_margin) {
			it->second->ProcessPackets(&prober_);
			next_send_time = it->second->NextSendTime(&prober_);
			RTC_DCHECK(next_send_time.IsFinite());
			early_execute_margin = prober_.is_probing()
				? RTCPacingController::kMaxEarlyProbeProcessing
				: TimeDelta::Zero();
		}
		min_next_send_time = std::min(min_next_send_time, next_send_time);
		it++;
	}

	if (scheduled_process_time.IsFinite()) {
		if (scheduled_process_time != next_process_time_) {
			return;
		}
		next_process_time_ = Timestamp::MinusInfinity();
	}

	TimeDelta hold_back_window = TimeDelta::Zero();
	if (!prober_.is_probing()) {
		hold_back_window = max_hold_back_window_;
		DataRate pacing_rate = _pacing_rate_with_sfu_pacing();
		if (max_hold_back_window_in_packets_ != kNoPacketHoldback &&
			!pacing_rate.IsZero() &&
			packet_size_.filtered() != rtc::ExpFilter::kValueUndefined) {
			TimeDelta avg_packet_send_time =
				DataSize::Bytes(packet_size_.filtered()) / pacing_rate;
			hold_back_window =
				std::min(hold_back_window,
				avg_packet_send_time * max_hold_back_window_in_packets_);
		}
	}

	// Calculate next process time.
	TimeDelta time_to_next_process =
		std::max(hold_back_window, min_next_send_time - now - early_execute_margin);

	min_next_send_time = now + time_to_next_process;

	if (next_process_time_.IsMinusInfinity() ||
		next_process_time_ > min_next_send_time) {
		task_queue_->PostDelayedHighPrecisionTask(
			SafeTask(
			safety_.flag(),
			[this, min_next_send_time]() { MaybeProcessPackets(min_next_send_time); }),
			time_to_next_process.RoundUpTo(TimeDelta::Millis(1)));
		next_process_time_ = min_next_send_time;
	}
}

DataRate RTCTaskQueuePacedSender::_pacing_rate_with_sfu_pacing() {
	DataRate pacing_rate = DataRate::Zero();
	std::unordered_map<RTCRtpRtcpInterface*, std::unique_ptr<RTCSFUPacingController>>::iterator it;
	it = pacers_.begin();
	while (it != pacers_.end()) {
		pacing_rate += it->second->pacing_rate();
		it++;
	}
	return pacing_rate;
}

void RTCTaskQueuePacedSender::_add_pacing_rtp_module(RTCRtpRtcpInterface* rtp_module,
	bool isAudio, uint32_t ssrc, absl::optional<uint32_t> rtx_ssrc) {
	if(isAudio) {
		RTC_LOG(LS_INFO)<<"_add_pacing_rtp_module skip audio todo...";
		return;
	}
	
	RTC_LOG(LS_INFO)<<"_add_pacing_rtp_module ssrc "<< rtp_module->SSRC();

	RTC_DCHECK(pacers_.find(rtp_module) == pacers_.end());

	std::unique_ptr<RTCSFUPacingController> pacer = std::make_unique<RTCSFUPacingController>(clock_, packet_sender_);
	if (burst_interval_.has_value()) {
		pacer->SetSendBurstInterval(burst_interval_.value());
	}

	RTCSFUPacingController* pacer_ptr = pacer.get();
	pacers_[rtp_module] = std::move(pacer);

	ssrc_to_pacers_[rtp_module->SSRC()] = pacer_ptr;

	if (rtx_ssrc.has_value()) {
		ssrc_to_pacers_[rtx_ssrc.value()] = pacer_ptr;
	}

	_set_pacing_rate_with_sfu_pacing(pacing_rate_, padding_rate_);
}

void RTCTaskQueuePacedSender::_remove_pacing_rtp_module(RTCRtpRtcpInterface* rtp_module,
	bool isAudio, uint32_t ssrc, absl::optional<uint32_t> rtx_ssrc) {
	if(isAudio) {
		RTC_LOG(LS_INFO)<<"_add_pacing_rtp_module skip audio todo...";
		return;
	}

	RTC_DCHECK(rtp_module != nullptr);
	RTC_DCHECK(pacers_.find(rtp_module) != pacers_.end());

	auto it = pacers_.find(rtp_module);
	pacers_.erase(it);
	
	auto ssrc_it = ssrc_to_pacers_.find(ssrc);
	ssrc_to_pacers_.erase(ssrc_it);

	if(rtx_ssrc.has_value()) {
		ssrc_it = ssrc_to_pacers_.find(rtx_ssrc.value());
		ssrc_to_pacers_.erase(ssrc_it);
	}
}
#endif

void* RTCTaskQueuePacedSender::enqueue_thread_proc(void *p) {
	RTCTaskQueuePacedSender* paced_sender = (RTCTaskQueuePacedSender*)p;
	paced_sender->is_run_thread_ = true;
	while (1) {
		std::vector<std::unique_ptr<RtpPacketToSend>> packets;
		pthread_mutex_lock(&(paced_sender->mutex_));
		pthread_cond_wait(&(paced_sender->cond_), &(paced_sender->mutex_));
		if (paced_sender->is_quiting_thread_) {
			pthread_mutex_unlock(&(paced_sender->mutex_));
			break;
		}

		if (!paced_sender->enqueue_packets_.empty()) {
			RTC_LOG(LS_INFO) << "[" << paced_sender << "] enqueue_thread_proc packets vector size " << paced_sender->enqueue_packets_.size();
			packets = std::move(paced_sender->enqueue_packets_);
			paced_sender->enqueue_packets_.clear();
		}
		pthread_mutex_unlock(&(paced_sender->mutex_));

		if (!packets.empty()) {
			paced_sender->task_queue_->PostTask(
				SafeTask(paced_sender->safety_.flag(), [paced_sender, packets = std::move(packets)]() mutable {
				RTC_DCHECK_RUN_ON(paced_sender->task_queue_);
#if 1
				for (auto& packet : packets) {
					size_t packet_size = packet->payload_size() + packet->padding_size();
					if (paced_sender->include_overhead_) {
						packet_size += packet->headers_size();
					}
					paced_sender->packet_size_.Apply(1, packet_size);
					RTC_DCHECK_GE(packet->capture_time_ms(), 0ll);
					paced_sender->_enqueue_packet_with_sfu_pacing(std::move(packet));
				}
				paced_sender->MaybeProcessPackets(Timestamp::MinusInfinity());
#endif
			}));
		}
	}

	paced_sender->is_run_thread_ = false;
	return nullptr;
}

}