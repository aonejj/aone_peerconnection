//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////
#include "rtc_base/location.h"
#include "rtc_base/logging.h"
#include "rtc_base/time_utils.h"

#include "RTCAudioPacketRouteFeeder.h"

namespace webrtc {

RTCAudioPacketRouteFeeder::RTCAudioPacketRouteFeeder(
	Clock *clock,
	const RTCAudioSendStream::Config* config)
	: clock_(clock),
	source_(nullptr),
	sink_(nullptr),
	payload_type_(config->send_codec_spec->payload_type),
	is_run_(false) {

}

RTCAudioPacketRouteFeeder::~RTCAudioPacketRouteFeeder() {

}

void RTCAudioPacketRouteFeeder::SetSource(
	rtc::RTCAudioPacketRouteSourceInterface<std::unique_ptr<RtpPacketReceived>>* source) {
	rtc::RTCAudioPacketRouteSourceInterface<std::unique_ptr<RtpPacketReceived>>* old_source = source_;

	MutexLock lock(&_source_lock);
	source_ = source;

	if (old_source != source && old_source) {
		old_source->RemovePacketRoutingSink(this);
	}

	if (!source)
		return;

	source->AddPacketRoutingSink(this);
}

void RTCAudioPacketRouteFeeder::SetSink(RTCAudioPacketRoutingSink *sink) {
	MutexLock lock(&_sink_lock);
	sink_ = sink;
}

void RTCAudioPacketRouteFeeder::Start() {
	MutexLock lock(&_run_lock);
	is_run_ = true;
}

void RTCAudioPacketRouteFeeder::Stop() {
	MutexLock lock(&_run_lock);
	is_run_ = false;
}

void RTCAudioPacketRouteFeeder::OnFeedPacketPtr(std::unique_ptr<RtpPacketReceived> packet) {
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
