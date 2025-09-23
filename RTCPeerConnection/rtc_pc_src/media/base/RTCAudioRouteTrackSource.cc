//////////////////////////////////////////////////////////////////////////
//
//	author : kimi
//
//////////////////////////////////////////////////////////////////////////
#include "rtc_base/logging.h"
#include "rtc_base/checks.h"
#include "rtc_base/time_utils.h"

#include "RTCAudioRouteTrackSource.h"


namespace rtc {

RTCAudioRouteTrackSource::RTCAudioRouteTrackSource()
	: sink_(nullptr) {

}

RTCAudioRouteTrackSource::~RTCAudioRouteTrackSource() {

}

void RTCAudioRouteTrackSource::OnRtpPacket(std::unique_ptr<webrtc::RtpPacketReceived> packet) {
	_routing_packet(std::move(packet));
}

void RTCAudioRouteTrackSource::AddPacketRoutingSink(RTCPacketRoutePtrSinkInterface<std::unique_ptr<webrtc::RtpPacketReceived>> *sink) {
	webrtc::MutexLock lock(&sink_mutex_);
	sink_ = sink;
}

void RTCAudioRouteTrackSource::RemovePacketRoutingSink(RTCPacketRoutePtrSinkInterface<std::unique_ptr<webrtc::RtpPacketReceived>> *sink) {
	webrtc::MutexLock lock(&sink_mutex_);
	sink_ = nullptr;
}

void RTCAudioRouteTrackSource::_routing_packet(std::unique_ptr<webrtc::RtpPacketReceived> packet) {
	webrtc::MutexLock lock(&sink_mutex_);
	if (sink_ != nullptr) {
		sink_->OnFeedPacketPtr(std::move(packet));
	}
}

}	// namespace rtc
