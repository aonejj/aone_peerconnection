//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#include <algorithm>
#include <limits>

#include "absl/types/optional.h"
#include "modules/rtp_rtcp/include/rtp_rtcp_defines.h"
#include "modules/rtp_rtcp/source/rtcp_packet.h"
#include "modules/rtp_rtcp/source/rtcp_packet/transport_feedback.h"

#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "rtc_base/time_utils.h"

#include "RTCFeedbackPacketRouter.h"

namespace webrtc {

constexpr int kRembSendIntervalMs = 200;

RTCFeedbackPacketRouter::RTCFeedbackPacketRouter() 
	:last_remb_time_ms_(rtc::TimeMillis()),
	 last_send_bitrate_bps_(0),
	 bitrate_bps_(0), 
	 max_bitrate_bps_(std::numeric_limits<decltype(max_bitrate_bps_)>::max()), 
	 active_remb_module_(nullptr) {
}

RTCFeedbackPacketRouter::~RTCFeedbackPacketRouter() {
	RTC_DCHECK(send_modules_map_.empty());
	RTC_DCHECK(send_modules_list_.empty());
	RTC_DCHECK(rtcp_feedback_senders_.empty());
	RTC_DCHECK(sender_remb_candidates_.empty());
	RTC_DCHECK(receiver_remb_candidates_.empty());
	RTC_DCHECK(active_remb_module_ == nullptr);
}

void RTCFeedbackPacketRouter::AddSendRtpModule(RTCRtpRtcpInterface* rtp_module, bool remb_candidate) {
	MutexLock lock(&modules_mutex_);

	AddSendRtpModuleToMap(rtp_module, rtp_module->SSRC());

	if (remb_candidate) {
		AddRembModuleCandidate(rtp_module, /* media_sender = */ true);
	}
}

void RTCFeedbackPacketRouter::RemoveSendRtpModule(RTCRtpRtcpInterface* rtp_module) {
	MutexLock lock(&modules_mutex_);
	MaybeRemoveRembModuleCandidate(rtp_module, /* media_sender = */ true);

	RemoveSendRtpModuleFromMap(rtp_module->SSRC());
}

void RTCFeedbackPacketRouter::AddSendRtpModuleToMap(RTCRtpRtcpInterface* rtp_module, uint32_t ssrc) {
	RTC_DCHECK(send_modules_map_.find(ssrc) == send_modules_map_.end());
	if (rtp_module->IsAudioConfigured()) {
		send_modules_list_.push_back(rtp_module);
	}
	else {
		send_modules_list_.push_front(rtp_module);
	}
	send_modules_map_[ssrc] = rtp_module;
}

void RTCFeedbackPacketRouter::RemoveSendRtpModuleFromMap(uint32_t ssrc) {
	auto kv = send_modules_map_.find(ssrc);
	RTC_DCHECK(kv != send_modules_map_.end());
	send_modules_list_.remove(kv->second);
	send_modules_map_.erase(kv);
}

void RTCFeedbackPacketRouter::AddReceiveRtpModule(
	RtcpFeedbackSenderInterface* rtcp_sender, bool remb_candidate) {
	MutexLock lock(&modules_mutex_);
	RTC_DCHECK(std::find(rtcp_feedback_senders_.begin(),
		rtcp_feedback_senders_.end(),
		rtcp_sender) == rtcp_feedback_senders_.end());

	rtcp_feedback_senders_.push_back(rtcp_sender);

	if (remb_candidate) {
		AddRembModuleCandidate(rtcp_sender, false);
	}
}

void RTCFeedbackPacketRouter::RemoveReceiveRtpModule(
	RtcpFeedbackSenderInterface* rtcp_sender) {
	MutexLock lock(&modules_mutex_);
	MaybeRemoveRembModuleCandidate(rtcp_sender, false);
	auto it = std::find(rtcp_feedback_senders_.begin(),
		rtcp_feedback_senders_.end(), rtcp_sender);
	RTC_DCHECK(it != rtcp_feedback_senders_.end());
	rtcp_feedback_senders_.erase(it);
}

void RTCFeedbackPacketRouter::OnReceiveBitrateChanged(
	const std::vector<uint32_t>& ssrcs, uint32_t bitrate_bps) {
	const int64_t kSendThresholdPercent = 97;
	int64_t receive_bitrate_bps = static_cast<int64_t>(bitrate_bps);

	int64_t now_ms = rtc::TimeMillis();

	{
		MutexLock lock(&remb_mutex_);
		if (last_send_bitrate_bps_ > 0) {
			int64_t new_remb_bitrate_bps =
				last_send_bitrate_bps_ - bitrate_bps_ + receive_bitrate_bps;

			if (new_remb_bitrate_bps <
				kSendThresholdPercent * last_send_bitrate_bps_ / 100) {
				last_remb_time_ms_ = now_ms - kRembSendIntervalMs;
			}
		}
		bitrate_bps_ = receive_bitrate_bps;

		if (now_ms - last_remb_time_ms_ < kRembSendIntervalMs) {
			return;
		}

		last_remb_time_ms_ = now_ms;
		last_send_bitrate_bps_ = receive_bitrate_bps;

		receive_bitrate_bps = std::min(receive_bitrate_bps, max_bitrate_bps_);
	}

	SendRemb(receive_bitrate_bps, ssrcs);
}

void RTCFeedbackPacketRouter::SetMaxDesiredReceiveBitrate(int64_t bitrate_bps) {
	RTC_DCHECK_GE(bitrate_bps, 0);
	{
		MutexLock lock(&remb_mutex_);
		max_bitrate_bps_ = bitrate_bps;
		if (rtc::TimeMillis() - last_remb_time_ms_ < kRembSendIntervalMs &&
			last_send_bitrate_bps_ > 0 &&
			last_send_bitrate_bps_ <= max_bitrate_bps_) {
			return;
		}
	}

	SendRemb(bitrate_bps, /*ssrcs=*/{});
}

bool RTCFeedbackPacketRouter::SendRemb(int64_t bitrate_bps, 
									   const std::vector<uint32_t>& ssrcs) {
	MutexLock lock(&modules_mutex_);

	if (!active_remb_module_) {
		return false;
	}

	active_remb_module_->SetRemb(bitrate_bps, ssrcs);

	return true;
}


bool RTCFeedbackPacketRouter::SendCombinedRtcpPacket(
	std::vector<std::unique_ptr<rtcp::RtcpPacket>> packets) {

	MutexLock lock(&modules_mutex_);
#if 0	// kimi origin source  
	for (RTCRtpRtcpInterface* rtp_module : send_modules_list_) {
		if (rtp_module->RTCP() == RtcpMode::kOff) {
			continue;
		}
		rtp_module->SendCombinedRtcpPacket(std::move(packets));
		return true;
	}

	if (rtcp_feedback_senders_.empty()) {
		return false;
	}
	auto* rtcp_sender = rtcp_feedback_senders_[0];
	rtcp_sender->SendCombinedRtcpPacket(std::move(packets));

	return true;
#else	// kimi replace
	if (rtcp_feedback_senders_.empty()) {
		for (RTCRtpRtcpInterface* rtp_module : send_modules_list_) {
			if (rtp_module->RTCP() == RtcpMode::kOff) {
				continue;
			}
			rtp_module->SendCombinedRtcpPacket(std::move(packets));
			return true;
		}
	}

	if (rtcp_feedback_senders_.empty()) {
		return false;
	}

	auto* rtcp_sender = rtcp_feedback_senders_[0];
	rtcp_sender->SendCombinedRtcpPacket(std::move(packets));
	return true;
#endif
}

void RTCFeedbackPacketRouter::AddRembModuleCandidate(
		RtcpFeedbackSenderInterface* candidate_module,
		bool media_sender) {
	RTC_DCHECK(candidate_module);

	std::vector<RtcpFeedbackSenderInterface*>& candidates =
		media_sender ? sender_remb_candidates_ : receiver_remb_candidates_;

	RTC_DCHECK(std::find(candidates.cbegin(), candidates.cend(),
		candidate_module) == candidates.cend());
	candidates.push_back(candidate_module);
	DetermineActiveRembModule();
}

void RTCFeedbackPacketRouter::MaybeRemoveRembModuleCandidate(
	RtcpFeedbackSenderInterface* candidate_module,
	bool media_sender) {
	RTC_DCHECK(candidate_module);

	std::vector<RtcpFeedbackSenderInterface*>& candidates =
		media_sender ? sender_remb_candidates_ : receiver_remb_candidates_;

	auto it = std::find(candidates.begin(), candidates.end(),
						candidate_module);

	if (it == candidates.end()) {
		return;  // Function called due to removal of non-REMB-candidate module.
	}

	if (*it == active_remb_module_) {
		UnsetActiveRembModule();
	}
	candidates.erase(it);
	DetermineActiveRembModule();
}

void RTCFeedbackPacketRouter::UnsetActiveRembModule() {
	RTC_CHECK(active_remb_module_);
	active_remb_module_->UnsetRemb();
	active_remb_module_ = nullptr;
}

void RTCFeedbackPacketRouter::DetermineActiveRembModule() {
	RtcpFeedbackSenderInterface* new_active_remb_module;
	
	if (!sender_remb_candidates_.empty()) {
		new_active_remb_module = sender_remb_candidates_.front();
	}
	else if (!receiver_remb_candidates_.empty()) {
		new_active_remb_module = receiver_remb_candidates_.front();
	}
	else {
		new_active_remb_module = nullptr;
	}

	if (new_active_remb_module != active_remb_module_ && active_remb_module_) {
		UnsetActiveRembModule();
	}

	active_remb_module_ = new_active_remb_module;
}

} // namespace webrtc
