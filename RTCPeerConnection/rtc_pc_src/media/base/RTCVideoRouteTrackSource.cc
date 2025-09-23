//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : video packet routing track source
//
//////////////////////////////////////////////////////////////////////////
#include "rtc_base/logging.h"
#include "rtc_base/checks.h"
#include "rtc_base/time_utils.h"

#include "RTCVideoRouteTrackSource.h"


namespace rtc {

RTCVideoRouteTrackSource::RTCVideoRouteTrackSource() 
	: sink_(nullptr),
	  feedback_source_(nullptr) {
	RTC_LOG(LS_VERBOSE) << "["<< this <<"] RTCVideoRouteTrackSource::RTCVideoRouteTrackSource";
}

RTCVideoRouteTrackSource::~RTCVideoRouteTrackSource() {
	RTC_LOG(LS_VERBOSE) << "["<< this <<"] RTCVideoRouteTrackSource::~RTCVideoRouteTrackSource";
}

bool RTCVideoRouteTrackSource::GetStats(Stats* stats) {
	webrtc::MutexLock lock(&stats_mutex_);

	if (!stats_) {
		return false;
	}

	*stats = *stats_;
	return true;
}

void RTCVideoRouteTrackSource::GenerateKeyFrame() {
	if (feedback_source_ != nullptr) {
		feedback_source_->OnGenerateKeyFrameFeedback();
	}
}

void RTCVideoRouteTrackSource::OnRtpPacket(std::unique_ptr<webrtc::RtpPacketReceived> packet) {
	_routing_packet(std::move(packet));
}

void RTCVideoRouteTrackSource::AddPacketRoutingSink(RTCPacketRoutePtrSinkInterface<std::unique_ptr<webrtc::RtpPacketReceived>> *sink) {
	webrtc::MutexLock lock(&sink_mutex_);
	sink_ = sink;
}

void RTCVideoRouteTrackSource::RemovePacketRoutingSink(RTCPacketRoutePtrSinkInterface<std::unique_ptr<webrtc::RtpPacketReceived>> *sink) {
	webrtc::MutexLock lock(&sink_mutex_);
	sink_ = nullptr;
}

void RTCVideoRouteTrackSource::SetRouteFeedbackSource(RTCVideoPacketRouteSourceFeedbackInterface* source) {
	feedback_source_ = source;
}

void RTCVideoRouteTrackSource::_routing_packet(std::unique_ptr<webrtc::RtpPacketReceived> packet) {
	webrtc::MutexLock lock(&sink_mutex_);
	if (sink_ != nullptr) {
		sink_->OnFeedPacketPtr(std::move(packet));
	}
}

}
