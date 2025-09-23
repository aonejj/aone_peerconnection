//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_MODULES_RTP_RTCP_SOURCE_RTP_PACKET_REDUNDANCY_CHECKER_H__
#define __RTC_MODULES_RTP_RTCP_SOURCE_RTP_PACKET_REDUNDANCY_CHECKER_H__

#include <map>
#include <list>

#include "rtc_base/ref_count.h"
#include "rtc_base/constructor_magic.h"
#include "rtc_base/synchronization/mutex.h"
#include "rtc_base/thread_annotations.h"
#include "system_wrappers/include/clock.h"


namespace webrtc {

class RTCRtpPacketRedundancyChecker : public rtc::RefCountInterface {
public:
	RTCRtpPacketRedundancyChecker(size_t number_to_store);
	RTCRtpPacketRedundancyChecker() = delete;

	~RTCRtpPacketRedundancyChecker();

public:
	void PutReceivedRtpPacketSequnce(uint16_t seq);
	bool IsDiscardRtpPacket(uint16_t seq);

private:
	struct SeqSaveTime {
		SeqSaveTime(uint16_t seq, int64_t nowMs) : seq_(seq), save_time_ms_(nowMs) { }
		uint16_t	seq_;
		int64_t		save_time_ms_;
	};

private:
	void put_received_rtp_packet_sequnce(uint16_t seq, int64_t saveMs);
	void CullOldSequence(int64_t now_ms) RTC_EXCLUSIVE_LOCKS_REQUIRED(lock_);
	void PopFront() RTC_EXCLUSIVE_LOCKS_REQUIRED(lock_);
	std::list<SeqSaveTime>::iterator IsFindSeqSaveTime(uint16_t seq) RTC_EXCLUSIVE_LOCKS_REQUIRED(lock_);

private:
	Clock* const clock_;
	mutable Mutex lock_;
	size_t number_to_store_ RTC_GUARDED_BY(lock_);

	std::list<SeqSaveTime> seq_history_list_ RTC_GUARDED_BY(lock_);
	std::map<uint16_t, std::list<SeqSaveTime>::iterator > seq_indexer_ RTC_GUARDED_BY(lock_);

private:
	RTC_DISALLOW_COPY_AND_ASSIGN(RTCRtpPacketRedundancyChecker);
};

}

#endif // __RTC_MODULES_RTP_RTCP_SOURCE_RTP_PACKET_REDUNDANCY_CHECKER_H__
