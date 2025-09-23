//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_MODULES_REMOTE_ESTIMATOR_REMOTE_ESTIMATOR_PROXY_H__
#define __RTC_MODULES_REMOTE_ESTIMATOR_REMOTE_ESTIMATOR_PROXY_H__

#include <map>
#include <vector>
#include <functional>

#include "api/rtp_headers.h"
#include "api/units/time_delta.h"
#include "api/units/timestamp.h"
#include "modules/remote_bitrate_estimator/include/remote_bitrate_estimator.h"
#include "rtc_base/numerics/sequence_number_unwrapper.h"
#include "rtc_base/synchronization/mutex.h"

#include "../rtc_pc_src/_deprecate_defines.h"

namespace webrtc {

class Clock;

namespace rtcp {
class TransportFeedback;
}

class RTCRemoteEstimatorProxy : public RemoteBitrateEstimator {
public:
	using TransportFeedbackSender = std::function<void(
		std::vector<std::unique_ptr<rtcp::RtcpPacket>> packets)>;

	RTCRemoteEstimatorProxy(Clock* clock,
		TransportFeedbackSender feedback_sender);

	~RTCRemoteEstimatorProxy() override;

public:
	void IncomingPacket(const RtpPacketReceived& packet) override;
	void RemoveStream(uint32_t ssrc) override {}
	bool LatestEstimate(std::vector<unsigned int>* ssrcs,
						unsigned int* bitrate_bps) const override;
	void OnRttUpdate(int64_t avg_rtt_ms, int64_t max_rtt_ms) override {}
	void SetMinBitrate(int min_bitrate_bps) override {}
//	int64_t TimeUntilNextProcess() override;
	TimeDelta Process() override;
	void OnBitrateChanged(int bitrate);
	void SetSendPeriodicFeedback(bool send_periodic_feedback);

private:
	void SendPeriodicFeedbacks() RTC_EXCLUSIVE_LOCKS_REQUIRED(&lock_);
	void SendFeedbackOnRequest(int64_t sequence_number,
							   const FeedbackRequest& feedback_request)
							   RTC_EXCLUSIVE_LOCKS_REQUIRED(&lock_);
	static int64_t BuildFeedbackPacket(
		uint8_t feedback_packet_count,
		uint32_t media_ssrc,
		int64_t base_sequence_number,
		std::map<int64_t, int64_t>::const_iterator
		begin_iterator,  // |begin_iterator| is inclusive.
		std::map<int64_t, int64_t>::const_iterator
		end_iterator,  // |end_iterator| is exclusive.
		rtcp::TransportFeedback* feedback_packet);

private:
	static const int kMaxNumberOfPackets;

	Clock* const clock_;
	const TransportFeedbackSender feedback_sender_;

	TimeDelta back_window_;
	TimeDelta min_interval_;
	TimeDelta max_interval_;
	TimeDelta default_interval_;
	double bandwidth_fraction_;

	int64_t last_process_time_ms_;
	
	Mutex lock_;

	uint32_t media_ssrc_ RTC_GUARDED_BY(&lock_);
	uint8_t feedback_packet_count_ RTC_GUARDED_BY(&lock_);
	SeqNumUnwrapper<uint16_t> unwrapper_ RTC_GUARDED_BY(&lock_);
	absl::optional<int64_t> periodic_window_start_seq_ RTC_GUARDED_BY(&lock_);
	// Map unwrapped seq -> time.
	std::map<int64_t, int64_t> packet_arrival_times_ RTC_GUARDED_BY(&lock_);
	int64_t send_interval_ms_ RTC_GUARDED_BY(&lock_);
	bool send_periodic_feedback_ RTC_GUARDED_BY(&lock_);

	// Unwraps absolute send times.
	uint32_t previous_abs_send_time_ RTC_GUARDED_BY(&lock_);
	Timestamp abs_send_timestamp_ RTC_GUARDED_BY(&lock_);
};


}

#endif // __RTC_MODULES_REMOTE_ESTIMATOR_REMOTE_ESTIMATOR_PROXY_H__