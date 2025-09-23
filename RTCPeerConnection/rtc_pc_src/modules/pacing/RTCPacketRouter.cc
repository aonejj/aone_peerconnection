#include <algorithm>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>

#include "absl/types/optional.h"
#include "modules/rtp_rtcp/include/rtp_rtcp_defines.h"
#include "modules/rtp_rtcp/source/rtcp_packet.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"

#include "../rtp_rtcp/source/RTCRtpRtcpInterface.h"
#include "RTCPacingController.h"
#include "RTCSFUPacingListener.h"
#include "RTCPacketRouter.h"

namespace webrtc {

RTCPacketRouter::RTCPacketRouter() : RTCPacketRouter(0) {}

RTCPacketRouter::RTCPacketRouter(uint16_t start_transport_seq)
	: last_send_module_(nullptr),
	  active_remb_module_(nullptr),
	  transport_seq_(start_transport_seq) {
}

RTCPacketRouter::~RTCPacketRouter() {
	RTC_DCHECK(send_modules_map_.empty());
	RTC_DCHECK(send_modules_list_.empty());
	RTC_DCHECK(rtcp_feedback_senders_.empty());
	RTC_DCHECK(sender_remb_candidates_.empty());
	RTC_DCHECK(receiver_remb_candidates_.empty());
	RTC_DCHECK(active_remb_module_ == nullptr);
}

void RTCPacketRouter::AddSendRtpModule(RTCRtpRtcpInterface* rtp_module, 
									   bool remb_candidate) {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	AddSendRtpModuleToMap(rtp_module, rtp_module->SSRC());

	if (absl::optional<uint32_t> rtx_ssrc = rtp_module->RtxSsrc()) {
		AddSendRtpModuleToMap(rtp_module, *rtx_ssrc);
	}

	//  thinking.... TODO... current disable
// 	if (absl::optional<uint32_t> flexfec_ssrc = rtp_module->FlexfecSsrc()) {
// 		AddSendRtpModuleToMap(rtp_module, *flexfec_ssrc);
// 	}
	if (rtp_module->SupportsRtxPayloadPadding()) {
		last_send_module_ = rtp_module;
	}

	if (remb_candidate) {
		AddRembModuleCandidate(rtp_module, /* media_sender = */ true);
	}

#ifdef __USING_RTC_SFU_PACING__
	if(pacing_listener_ != nullptr) {
		pacing_listener_->AddPacingRtpModule(rtp_module);
	}
#endif
}

void RTCPacketRouter::AddSendRtpModuleToMap(RTCRtpRtcpInterface* rtp_module,
											uint32_t ssrc) {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	RTC_CHECK(send_modules_map_.find(ssrc) == send_modules_map_.end());

	if (rtp_module->IsAudioConfigured()) {
		send_modules_list_.push_back(rtp_module);
	}
	else {
		send_modules_list_.push_front(rtp_module);
	}
	send_modules_map_[ssrc] = rtp_module;
}

void RTCPacketRouter::RemoveSendRtpModuleFromMap(uint32_t ssrc) {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	auto it = send_modules_map_.find(ssrc);
	if (it == send_modules_map_.end()) {
		RTC_LOG(LS_ERROR) << "No send module found for ssrc " << ssrc;
		return;
	}
	send_modules_list_.remove(it->second);
	RTC_CHECK(modules_used_in_current_batch_.empty());
	send_modules_map_.erase(it);
}

void RTCPacketRouter::RemoveSendRtpModule(RTCRtpRtcpInterface* rtp_module) {
	RTC_DCHECK_RUN_ON(&thread_checker_);

#ifdef __USING_RTC_SFU_PACING__
	if(pacing_listener_ != nullptr) {
		pacing_listener_->RemovePacingRtpModule(rtp_module);
	}
#endif

	MaybeRemoveRembModuleCandidate(rtp_module, /* media_sender = */ true);

	RemoveSendRtpModuleFromMap(rtp_module->SSRC());
	if (absl::optional<uint32_t> rtx_ssrc = rtp_module->RtxSsrc()) {
		RemoveSendRtpModuleFromMap(*rtx_ssrc);
	}

	//  thinking.... TODO... current disable
// 	if (absl::optional<uint32_t> flexfec_ssrc = rtp_module->FlexfecSsrc()) {
// 		RemoveSendRtpModuleFromMap(*flexfec_ssrc);
// 	}

	if (last_send_module_ == rtp_module) {
		last_send_module_ = nullptr;
	}
//	rtp_module->OnPacketSendingThreadSwitched();
}

void RTCPacketRouter::AddReceiveRtpModule(RtcpFeedbackSenderInterface* rtcp_sender,
	bool remb_candidate) {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	RTC_DCHECK(std::find(rtcp_feedback_senders_.begin(),
		rtcp_feedback_senders_.end(),
		rtcp_sender) == rtcp_feedback_senders_.end());

	rtcp_feedback_senders_.push_back(rtcp_sender);

	if (remb_candidate) {
		AddRembModuleCandidate(rtcp_sender, /* media_sender = */ false);
	}
}

void RTCPacketRouter::RemoveReceiveRtpModule(RtcpFeedbackSenderInterface* rtcp_sender) {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	MaybeRemoveRembModuleCandidate(rtcp_sender, /* media_sender = */ false);
	auto it = std::find(rtcp_feedback_senders_.begin(),
		rtcp_feedback_senders_.end(), rtcp_sender);
	RTC_DCHECK(it != rtcp_feedback_senders_.end());
	rtcp_feedback_senders_.erase(it);
}

void RTCPacketRouter::SendPacket(std::unique_ptr<RtpPacketToSend> packet,
	const PacedPacketInfo& cluster_info) {
	RTC_DCHECK_RUN_ON(&thread_checker_);

	// With the new pacer code path, transport sequence numbers are only set here,
	// on the pacer thread. Therefore we don't need atomics/synchronization.
	bool assign_transport_sequence_number =
		packet->HasExtension<TransportSequenceNumber>();
	if (assign_transport_sequence_number) {
		packet->SetExtension<TransportSequenceNumber>((transport_seq_ + 1) &
			0xFFFF);
	}

	uint32_t ssrc = packet->Ssrc();
	auto it = send_modules_map_.find(ssrc);
	if (it == send_modules_map_.end()) {
		RTC_LOG(LS_WARNING)
			<< "Failed to send packet, matching RTP module not found "
			"or transport error. SSRC = "
			<< packet->Ssrc() << ", sequence number " << packet->SequenceNumber();
		return;
	}

	RTCRtpRtcpInterface* rtp_module = it->second;
	if (!rtp_module->TrySendPacket(std::move(packet), cluster_info)) {
		RTC_LOG(LS_WARNING) << "Failed to send packet, rejected by RTP module.";
		return;
	}
	modules_used_in_current_batch_.insert(rtp_module);

	// Sending succeeded.

	if (assign_transport_sequence_number) {
		++transport_seq_;
	}

	if (rtp_module->SupportsRtxPayloadPadding()) {
		// This is now the last module to send media, and has the desired
		// properties needed for payload based padding. Cache it for later use.
		last_send_module_ = rtp_module;
	}

// thinking... TODO... current disable
// 	for (auto& packet : rtp_module->FetchFecPackets()) {
// 		pending_fec_packets_.push_back(std::move(packet));
// 	}
}

void RTCPacketRouter::OnBatchComplete() {
	RTC_DCHECK_RUN_ON(&thread_checker_);

	for (auto& module : modules_used_in_current_batch_) {
		module->OnBatchComplete();
	}
	modules_used_in_current_batch_.clear();
}

std::vector<std::unique_ptr<RtpPacketToSend>> RTCPacketRouter::FetchFec() {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	std::vector<std::unique_ptr<RtpPacketToSend>> fec_packets =
		std::move(pending_fec_packets_);
	pending_fec_packets_.clear();
	return fec_packets;
}

std::vector<std::unique_ptr<RtpPacketToSend>> RTCPacketRouter::GeneratePadding(
			DataSize size) {
	RTC_DCHECK_RUN_ON(&thread_checker_);

	std::vector<std::unique_ptr<RtpPacketToSend>> padding_packets;
	if (last_send_module_ != nullptr &&
		last_send_module_->SupportsRtxPayloadPadding()) {
		padding_packets = last_send_module_->GeneratePadding(size.bytes());
	}

	if (padding_packets.empty()) {
		for (RTCRtpRtcpInterface* rtp_module : send_modules_list_) {
			if (rtp_module->SupportsPadding()) {
				padding_packets = rtp_module->GeneratePadding(size.bytes());
				if (!padding_packets.empty()) {
					last_send_module_ = rtp_module;
					break;
				}
			}
		}
	}

	return padding_packets;
}

void RTCPacketRouter::OnAbortedRetransmissions(uint32_t ssrc,
	rtc::ArrayView<const uint16_t> sequence_numbers) {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	auto it = send_modules_map_.find(ssrc);
	if (it != send_modules_map_.end()) {
		it->second->OnAbortedRetransmissions(sequence_numbers);
	}
}

absl::optional<uint32_t> RTCPacketRouter::GetRtxSsrcForMedia(uint32_t ssrc) const {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	auto it = send_modules_map_.find(ssrc);
	if (it != send_modules_map_.end() && it->second->SSRC() == ssrc) {
		// A module is registered with the given SSRC, and that SSRC is the main
		// media SSRC for that RTP module.
		return it->second->RtxSsrc();
	}
	return absl::nullopt;
}

uint16_t RTCPacketRouter::CurrentTransportSequenceNumber() const {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	return transport_seq_ & 0xFFFF;
}

void RTCPacketRouter::SendRemb(int64_t bitrate_bps, std::vector<uint32_t> ssrcs) {
	RTC_DCHECK_RUN_ON(&thread_checker_);

	if (!active_remb_module_) {
		return;
	}

	active_remb_module_->SetRemb(bitrate_bps, std::move(ssrcs));
}

void RTCPacketRouter::SendCombinedRtcpPacket(
	std::vector<std::unique_ptr<rtcp::RtcpPacket>> packets) {
	RTC_DCHECK_RUN_ON(&thread_checker_);

	// Prefer send modules.
	for (RTCRtpRtcpInterface* rtp_module : send_modules_list_) {
		if (rtp_module->RTCP() == RtcpMode::kOff) {
			continue;
		}

		rtp_module->SendCombinedRtcpPacket(std::move(packets));
		return;
	}

	if (rtcp_feedback_senders_.empty()) {
		return;
	}
	auto* rtcp_sender = rtcp_feedback_senders_[0];

	rtcp_sender->SendCombinedRtcpPacket(std::move(packets));
}

void RTCPacketRouter::AddRembModuleCandidate(RtcpFeedbackSenderInterface* candidate_module,
											 bool media_sender) {
	RTC_DCHECK(candidate_module);
	std::vector<RtcpFeedbackSenderInterface*>& candidates =
		media_sender ? sender_remb_candidates_ : receiver_remb_candidates_;
	RTC_DCHECK(std::find(candidates.cbegin(), candidates.cend(),
		candidate_module) == candidates.cend());
	candidates.push_back(candidate_module);
	DetermineActiveRembModule();
}

void RTCPacketRouter::MaybeRemoveRembModuleCandidate(
	RtcpFeedbackSenderInterface* candidate_module,
	bool media_sender) {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	RTC_DCHECK(candidate_module);
	std::vector<RtcpFeedbackSenderInterface*>& candidates =
		media_sender ? sender_remb_candidates_ : receiver_remb_candidates_;
	auto it = std::find(candidates.begin(), candidates.end(), candidate_module);

	if (it == candidates.end()) {
		return;  // Function called due to removal of non-REMB-candidate module.
	}

	if (*it == active_remb_module_) {
		UnsetActiveRembModule();
	}
	candidates.erase(it);
	DetermineActiveRembModule();
}

void RTCPacketRouter::UnsetActiveRembModule() {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	RTC_CHECK(active_remb_module_);
	active_remb_module_->UnsetRemb();
	active_remb_module_ = nullptr;
}

void RTCPacketRouter::DetermineActiveRembModule() {
	RTC_DCHECK_RUN_ON(&thread_checker_);

	RtcpFeedbackSenderInterface* new_active_remb_module;

	if (!sender_remb_candidates_.empty()) {
		new_active_remb_module = sender_remb_candidates_.front();
	} else if (!receiver_remb_candidates_.empty()) {
		new_active_remb_module = receiver_remb_candidates_.front();
	} else {
		new_active_remb_module = nullptr;
	}

	if (new_active_remb_module != active_remb_module_ && active_remb_module_) {
		UnsetActiveRembModule();
	}

	active_remb_module_ = new_active_remb_module;
}

}