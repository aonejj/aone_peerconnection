//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#include <algorithm>
#include <limits>
#include <memory>
//#include <utility>

#include "rtc_base/checks.h"
#include "rtc_base/logging.h"

#include "RTCRtpPacketRedundancyChecker.h"

namespace webrtc {

const int64_t kSequencePendingDurationMs = 1500;

RTCRtpPacketRedundancyChecker::RTCRtpPacketRedundancyChecker(size_t number_to_store)
	: clock_(Clock::GetRealTimeClock()),
	number_to_store_(number_to_store) {

}

RTCRtpPacketRedundancyChecker::~RTCRtpPacketRedundancyChecker() {
}

void RTCRtpPacketRedundancyChecker::PutReceivedRtpPacketSequnce(uint16_t seq) {
	MutexLock lock(&lock_);
	int64_t now_ms = clock_->TimeInMilliseconds();
	CullOldSequence(now_ms);
	put_received_rtp_packet_sequnce(seq, now_ms);
}

bool RTCRtpPacketRedundancyChecker::IsDiscardRtpPacket(uint16_t seq) {
	MutexLock lock(&lock_);
	auto it = IsFindSeqSaveTime(seq);
	if (it != seq_history_list_.end()) {
		// duplicate
		return true;
	}

	int64_t now_ms = clock_->TimeInMilliseconds();
	CullOldSequence(now_ms);

	put_received_rtp_packet_sequnce(seq, now_ms);

	return false;
}

void RTCRtpPacketRedundancyChecker::put_received_rtp_packet_sequnce(uint16_t seq, int64_t saveMs) {
	seq_history_list_.push_back(SeqSaveTime(seq, saveMs));
	seq_indexer_[seq] = std::prev(seq_history_list_.end());
}

void RTCRtpPacketRedundancyChecker::CullOldSequence(int64_t now_ms) {

	while (!seq_history_list_.empty()) {
		if (seq_history_list_.size() >= number_to_store_) {
			PopFront();
			continue;
		}

		const SeqSaveTime& seq_save_time = seq_history_list_.front();
		if (seq_save_time.save_time_ms_ + kSequencePendingDurationMs < now_ms) {
			PopFront();
			continue;
		}

		return;
	}
}

void RTCRtpPacketRedundancyChecker::PopFront() {
	if (!seq_history_list_.empty()) {
		SeqSaveTime& seq_save_time = seq_history_list_.front();
		auto it = seq_indexer_.find(seq_save_time.seq_);
		if (it != seq_indexer_.end()) {
			seq_indexer_.erase(it);
		}
		seq_history_list_.pop_front();
	}
}

std::list<RTCRtpPacketRedundancyChecker::SeqSaveTime>::iterator 
RTCRtpPacketRedundancyChecker::IsFindSeqSaveTime(uint16_t seq) {
	auto it = seq_indexer_.find(seq);
	if (it == seq_indexer_.end()) {
		return seq_history_list_.end();
	}

	return it->second;
}

}