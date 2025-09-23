//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_MODULES_VIDEO_CODING_NACK_MODULE2_H__
#define __RTC_MODULES_VIDEO_CODING_NACK_MODULE2_H__

#include <stdint.h>

#include <map>
#include <set>
#include <vector>

#include "api/units/time_delta.h"
#include "modules/include/module_common_types.h"
#include "modules/video_coding/histogram.h"
#include "rtc_base/numerics/sequence_number_unwrapper.h"

#include "rtc_base/synchronization/sequence_checker.h"
#include "rtc_base/task_queue.h"
#include "rtc_base/task_utils/pending_task_safety_flag.h"
#include "rtc_base/task_utils/repeating_task.h"
#include "rtc_base/thread_annotations.h"
#include "system_wrappers/include/clock.h"
#include "rtc_base/synchronization/mutex.h"


namespace webrtc {

// TODO(bugs.webrtc.org/11594): This class no longer implements the Module
// interface and therefore "NackModule" may not be a descriptive name anymore.
// Consider renaming to e.g. NackTracker or NackRequester.
class RTCNackModule2 final {
public:
	static constexpr TimeDelta kUpdateInterval = TimeDelta::Millis(20);

	RTCNackModule2(TaskQueueBase* current_queue,
		Clock* clock,
		NackSender* nack_sender,
		KeyFrameRequestSender* keyframe_request_sender,
		TimeDelta update_interval = kUpdateInterval);
	~RTCNackModule2();

	int OnReceivedPacket(uint16_t seq_num, bool is_keyframe);
	int OnReceivedPacket(uint16_t seq_num, bool is_keyframe, bool is_recovered);

	void ClearUpTo(uint16_t seq_num);
	void UpdateRtt(int64_t rtt_ms);

private:
	// Which fields to consider when deciding which packet to nack in
	// GetNackBatch.
	enum NackFilterOptions { kSeqNumOnly, kTimeOnly, kSeqNumAndTime };

	// This class holds the sequence number of the packet that is in the nack list
	// as well as the meta data about when it should be nacked and how many times
	// we have tried to nack this packet.
	struct NackInfo {
		NackInfo();
		NackInfo(uint16_t seq_num,
			uint16_t send_at_seq_num,
			int64_t created_at_time);

		uint16_t seq_num;
		uint16_t send_at_seq_num;
		int64_t created_at_time;
		int64_t sent_at_time;
		int retries;
	};

	struct BackoffSettings {
		BackoffSettings(TimeDelta min_retry, TimeDelta max_rtt, double base);
		static absl::optional<BackoffSettings> ParseFromFieldTrials();

		// Min time between nacks.
		const TimeDelta min_retry_interval;
		// Upper bound on link-delay considered for exponential backoff.
		const TimeDelta max_rtt;
		// Base for the exponential backoff.
		const double base;
	};

	void AddPacketsToNack_l(uint16_t seq_num_start, uint16_t seq_num_end);


	// Removes packets from the nack list until the next keyframe. Returns true
	// if packets were removed.
	bool RemovePacketsUntilKeyFrame_l();
	std::vector<uint16_t> GetNackBatch(NackFilterOptions options);
	std::vector<uint16_t> GetNackBatch_l(NackFilterOptions options);

	// Update the reordering distribution.
	void UpdateReorderingStatistics_l(uint16_t seq_num);

	// Returns how many packets we have to wait in order to receive the packet
	// with probability |probabilty| or higher.
	int WaitNumberOfPackets_l(float probability) const;

	TaskQueueBase* const worker_thread_;

	// Used to regularly call SendNack if needed.
	RepeatingTaskHandle repeating_task_ RTC_GUARDED_BY(worker_thread_);
	const TimeDelta update_interval_;

	Clock* const clock_;
	NackSender* const nack_sender_;
	KeyFrameRequestSender* const keyframe_request_sender_;

	// TODO(philipel): Some of the variables below are consistently used on a
	// known thread (e.g. see |initialized_|). Those probably do not need
	// synchronized access.
	std::map<uint16_t, NackInfo, DescendingSeqNumComp<uint16_t>> nack_list_;

	std::set<uint16_t, DescendingSeqNumComp<uint16_t>> keyframe_list_;

	std::set<uint16_t, DescendingSeqNumComp<uint16_t>> recovered_list_;

	video_coding::Histogram reordering_histogram_;
	bool initialized_;
	int64_t rtt_ms_;
	uint16_t newest_seq_num_;

	// Adds a delay before send nack on packet received.
	const int64_t send_nack_delay_ms_;

	const absl::optional<BackoffSettings> backoff_settings_;

	// Used to signal destruction to potentially pending tasks.
	ScopedTaskSafety task_safety_;

	mutable Mutex nack_modules_mutex_;
};

}  // namespace webrtc


#endif // __RTC_MODULES_VIDEO_CODING_NACK_MODULE2_H__