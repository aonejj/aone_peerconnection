//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_MODULES_CONGESTION_CONTROLLER_INCLUDE_RECEIVE_SIDE_CONGESTION_CONTOLLER_H__
#define __RTC_MODULES_CONGESTION_CONTROLLER_INCLUDE_RECEIVE_SIDE_CONGESTION_CONTOLLER_H__

#include <memory>
#include <vector>

#include "../rtc_pc_src/_deprecate_defines.h"

#include "api/media_types.h"
#include "modules/include/module.h"
#include "rtc_base/synchronization/mutex.h"
#include "api/rtp_headers.h"
#include "api/units/time_delta.h"

#include "modules/congestion_controller/remb_throttler.h"

#include "../../remote_bitrate_estimator/RTCRemoteEstimatorProxy.h"



namespace webrtc {

class RemoteBitrateEstimator;
class RemoteBitrateObserver;

class RTCReceiveSideCongestionController : public CallStatsObserver {
public:
	
	RTCReceiveSideCongestionController(Clock* clock, 
		RTCRemoteEstimatorProxy::TransportFeedbackSender feedback_sender,
		RembThrottler::RembSender remb_sender);

	~RTCReceiveSideCongestionController() override {}

public:
	virtual void OnReceivedPacket(const RtpPacketReceived& packet,
								  MediaType media_type);
	void SetSendPeriodicFeedback(bool send_periodic_feedback);

	void OnRttUpdate(int64_t avg_rtt_ms, int64_t max_rtt_ms) override;

	void OnBitrateChanged(int bitrate_bps);

	TimeDelta MaybeProcess();

	bool LatestReceiveSideEstimate(std::vector<unsigned int>* ssrcs,
		unsigned int* bitrate_bps) const;
	void RemoveStream(uint32_t ssrc);

private:
	class RTCWrappingBitrateEstimator : public RemoteBitrateEstimator {
	public:
		RTCWrappingBitrateEstimator(RemoteBitrateObserver* observer, Clock* clock);

		RTCWrappingBitrateEstimator() = delete;
		RTCWrappingBitrateEstimator(const RTCWrappingBitrateEstimator&) = delete;
		RTCWrappingBitrateEstimator& operator=(const RTCWrappingBitrateEstimator&) =
			delete;

		~RTCWrappingBitrateEstimator() override;

		void IncomingPacket(const RtpPacketReceived& packet) override;

		TimeDelta Process() override;

		void OnRttUpdate(int64_t avg_rtt_ms, int64_t max_rtt_ms) override;

		void RemoveStream(unsigned int ssrc) override;

		bool LatestEstimate(std::vector<unsigned int>* ssrcs,
			unsigned int* bitrate_bps) const override;

		void SetMinBitrate(int min_bitrate_bps) override;

	private:
		void PickEstimatorFromHeader(const RTPHeader& header)
			RTC_EXCLUSIVE_LOCKS_REQUIRED(mutex_);
		void PickEstimator() RTC_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

	private:
		RemoteBitrateObserver* observer_;
		Clock* const clock_;
		mutable Mutex mutex_;
		std::unique_ptr<RemoteBitrateEstimator> rbe_;
		bool using_absolute_send_time_;
		uint32_t packets_since_absolute_send_time_;
		int min_bitrate_bps_;
	};

private:
	RembThrottler remb_throttler_;
	RTCWrappingBitrateEstimator remote_bitrate_estimator_;
	RTCRemoteEstimatorProxy remote_estimator_proxy_;
};

}	// namespace webrtc


#endif	// __RTC_MODULES_CONGESTION_CONTROLLER_INCLUDE_RECEIVE_SIDE_CONGESTION_CONTOLLER_H__
