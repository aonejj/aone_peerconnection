//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#include "modules/remote_bitrate_estimator/include/bwe_defines.h"
#include "modules/remote_bitrate_estimator/remote_bitrate_estimator_abs_send_time.h"
#include "modules/remote_bitrate_estimator/remote_bitrate_estimator_single_stream.h"
#include "rtc_base/logging.h"

#include "./include/RTCReceiveSideCongestionController.h"


namespace webrtc {

namespace {
	static const uint32_t kTimeOffsetSwitchThreshold = 30;
}  // namespace

RTCReceiveSideCongestionController::RTCWrappingBitrateEstimator::
	RTCWrappingBitrateEstimator(RemoteBitrateObserver* observer, Clock* clock)
	: observer_(observer),
clock_(clock),
rbe_(new RemoteBitrateEstimatorSingleStream(observer_, clock_)),
using_absolute_send_time_(false),
packets_since_absolute_send_time_(0),
min_bitrate_bps_(congestion_controller::GetMinBitrateBps()) {
	rbe_->SetMinBitrate(min_bitrate_bps_);
}

RTCReceiveSideCongestionController::RTCWrappingBitrateEstimator::
~RTCWrappingBitrateEstimator() = default;

void RTCReceiveSideCongestionController::RTCWrappingBitrateEstimator::IncomingPacket(
	const RtpPacketReceived& packet){

	RTPHeader header;
	packet.GetHeader(&header);

	MutexLock lock(&mutex_);
	PickEstimatorFromHeader(header);
	rbe_->IncomingPacket(packet);
}

TimeDelta RTCReceiveSideCongestionController::RTCWrappingBitrateEstimator::Process() {
	MutexLock lock(&mutex_);
	return rbe_->Process();
}

// int64_t RTCReceiveSideCongestionController::RTCWrappingBitrateEstimator::
// 	TimeUntilNextProcess() {
// 	MutexLock lock(&mutex_);
// 	return rbe_->TimeUntilNextProcess();
// }

void RTCReceiveSideCongestionController::RTCWrappingBitrateEstimator::OnRttUpdate(
	int64_t avg_rtt_ms,
	int64_t max_rtt_ms) {
	MutexLock lock(&mutex_);
	rbe_->OnRttUpdate(avg_rtt_ms, max_rtt_ms);
}

void RTCReceiveSideCongestionController::RTCWrappingBitrateEstimator::RemoveStream(
	unsigned int ssrc) {
	MutexLock lock(&mutex_);
	rbe_->RemoveStream(ssrc);
}

bool RTCReceiveSideCongestionController::RTCWrappingBitrateEstimator::LatestEstimate(
	std::vector<unsigned int>* ssrcs,
	unsigned int* bitrate_bps) const {
	MutexLock lock(&mutex_);
	return rbe_->LatestEstimate(ssrcs, bitrate_bps);
}

void RTCReceiveSideCongestionController::RTCWrappingBitrateEstimator::SetMinBitrate(
	int min_bitrate_bps) {
	MutexLock lock(&mutex_);
	rbe_->SetMinBitrate(min_bitrate_bps);
	min_bitrate_bps_ = min_bitrate_bps;
}

void RTCReceiveSideCongestionController::RTCWrappingBitrateEstimator::
PickEstimatorFromHeader(const RTPHeader& header) {
	if (header.extension.hasAbsoluteSendTime) {
		// If we see AST in header, switch RBE strategy immediately.
		if (!using_absolute_send_time_) {
			RTC_LOG(LS_INFO)
				<< "WrappingBitrateEstimator: Switching to absolute send time RBE.";
			using_absolute_send_time_ = true;
			PickEstimator();
		}
		packets_since_absolute_send_time_ = 0;
	}
	else {
		// When we don't see AST, wait for a few packets before going back to TOF.
		if (using_absolute_send_time_) {
			++packets_since_absolute_send_time_;
			if (packets_since_absolute_send_time_ >= kTimeOffsetSwitchThreshold) {
				RTC_LOG(LS_INFO)
					<< "WrappingBitrateEstimator: Switching to transmission "
					"time offset RBE.";
				using_absolute_send_time_ = false;
				PickEstimator();
			}
		}
	}
}

void RTCReceiveSideCongestionController::RTCWrappingBitrateEstimator::PickEstimator() {
	if (using_absolute_send_time_) {
		rbe_.reset(new RemoteBitrateEstimatorAbsSendTime(observer_, clock_));
	}
	else {
		rbe_.reset(new RemoteBitrateEstimatorSingleStream(observer_, clock_));
	}
	rbe_->SetMinBitrate(min_bitrate_bps_);
}

RTCReceiveSideCongestionController::RTCReceiveSideCongestionController(Clock* clock, 
	RTCRemoteEstimatorProxy::TransportFeedbackSender feedback_sender,
	RembThrottler::RembSender remb_sender) 
	: remb_throttler_(std::move(remb_sender), clock),
	  remote_bitrate_estimator_(&remb_throttler_, clock),
	  remote_estimator_proxy_(clock, std::move(feedback_sender)){

}

void RTCReceiveSideCongestionController::OnReceivedPacket(
	const RtpPacketReceived& packet, 
	MediaType media_type) {

	bool has_transport_sequence_number = packet.HasExtension(kRtpExtensionTransportSequenceNumber);

	if (media_type == MediaType::AUDIO && !has_transport_sequence_number) {
		return;
	}

	if (media_type == MediaType::VIDEO) {
		(const_cast<RtpPacketReceived*>(&packet))->_is_video = true;
	}

	if (has_transport_sequence_number) {
		remote_estimator_proxy_.IncomingPacket(packet);
	} 

    {
		remote_bitrate_estimator_.IncomingPacket(packet);
	}

}

void RTCReceiveSideCongestionController::SetSendPeriodicFeedback(
	bool send_periodic_feedback) {
	remote_estimator_proxy_.SetSendPeriodicFeedback(send_periodic_feedback);
}

void RTCReceiveSideCongestionController::OnRttUpdate(int64_t avg_rtt_ms,
	int64_t max_rtt_ms) {
	remote_bitrate_estimator_.OnRttUpdate(avg_rtt_ms, max_rtt_ms);	
}

void RTCReceiveSideCongestionController::OnBitrateChanged(int bitrate_bps) {
	remote_estimator_proxy_.OnBitrateChanged(bitrate_bps);
}

TimeDelta RTCReceiveSideCongestionController::MaybeProcess() {
	TimeDelta time_until_rbe = remote_bitrate_estimator_.Process();
	TimeDelta time_until_rep = remote_estimator_proxy_.Process();
	TimeDelta time_until = std::min(time_until_rbe, time_until_rep);
	return std::max(time_until, TimeDelta::Zero());
}

bool RTCReceiveSideCongestionController::LatestReceiveSideEstimate(std::vector<unsigned int>* ssrcs,
	unsigned int* bitrate_bps) const {
	return remote_bitrate_estimator_.LatestEstimate(ssrcs, bitrate_bps);
}

void RTCReceiveSideCongestionController::RemoveStream(uint32_t ssrc) {
	return remote_bitrate_estimator_.RemoveStream(ssrc);
}

}	// namespace webrtc