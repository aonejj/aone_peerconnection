//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////
#include "../rtc_pc_src/_deprecate_defines.h"

#include "rtc_base/location.h"
#include "rtc_base/logging.h"
#include "rtc_base/time_utils.h"
#include "modules/rtp_rtcp/source/rtp_header_extensions.h"
#include "RTCVideoPacketRouteFeeder.h"

namespace webrtc {

RTCVideoPacketRouteFeeder::RTCVideoPacketRouteFeeder(
	Clock* clock)
	: clock_(clock),
	  source_(nullptr),
	  sink_(nullptr), 
	  is_run_(false) {
} 

RTCVideoPacketRouteFeeder::~RTCVideoPacketRouteFeeder() {
}

void RTCVideoPacketRouteFeeder::SetSource(
	rtc::RTCVideoPacketRouteSourceInterface<std::unique_ptr<RtpPacketReceived>>* source) {

	rtc::RTCVideoPacketRouteSourceInterface<std::unique_ptr<RtpPacketReceived>>* old_source = source_;

	MutexLock lock(&_source_lock);

	source_ = source;

	if (old_source != source && old_source) {
		old_source->RemovePacketRoutingSink(this);
	}

	if (!source)
		return;

	source->AddPacketRoutingSink(this);
}

void RTCVideoPacketRouteFeeder::SetSink(RTCVideoPacketRoutingSink *sink) {
	MutexLock lock(&_sink_lock);
	sink_ = sink;
}

void RTCVideoPacketRouteFeeder::Start() {
	MutexLock lock(&_run_lock);
	is_run_ = true;
}

void RTCVideoPacketRouteFeeder::Stop() {
	MutexLock lock(&_run_lock);
	is_run_ = false;
}

void RTCVideoPacketRouteFeeder::GenerateKeyFrame() {
	{
		MutexLock lock(&_run_lock);
		if (!is_run_) {
			return;
		}
	}

	MutexLock lock(&_source_lock);
	if(source_ != nullptr) {
		source_->GenerateKeyFrame();
	}
}

void RTCVideoPacketRouteFeeder::OnFeedPacketPtr(std::unique_ptr<RtpPacketReceived> packet) {
	{
		MutexLock lock(&_run_lock);
		if (!is_run_) {
			return;
		}
	}

	MutexLock lock(&_sink_lock);
	if (sink_ != nullptr) {
		sink_->OnRoutingPacket(std::move(packet));
	}
}

}	// namespace webrtc