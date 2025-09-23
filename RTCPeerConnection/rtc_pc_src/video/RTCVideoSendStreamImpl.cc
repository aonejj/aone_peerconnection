//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#include "rtc_base/logging.h"
#include "api/task_queue/task_queue_factory.h"

#include "RTCVideoPacketRouteFeeder.h"
#include "RTCVideoSendStreamImpl.h"
#include "../call/RTCRtpTransportControllerSendInterface.h"

namespace webrtc {

RTCVideoSendStreamImpl::RTCVideoSendStreamImpl(
	Clock* clock, 
	RTCVideoSendStream::Config config,
	RTCRtpTransportControllerSendInterface* transport,
	RtcpRttStats* call_stats)
	: worker_queue_(TaskQueueBase::Current()),
	  config_(std::move(config)) {

	feeder_ = std::make_unique<RTCVideoPacketRouteFeeder>(clock);

	router_.reset(new RTCVideoSendStreamRouter(clock,
		&config_,
		feeder_.get(),
		transport,
		call_stats));
}

RTCVideoSendStreamImpl::~RTCVideoSendStreamImpl() {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	RTC_DCHECK(!router_);

}

void RTCVideoSendStreamImpl::DeliverRtcp(const uint8_t* packet, size_t length) {
	// Called on a network thread.
	router_->DeliverRtcp(packet, length);
}

void RTCVideoSendStreamImpl::Start() {
	RTC_DCHECK_RUN_ON(&thread_checker_);

	router_->Start();
}

void RTCVideoSendStreamImpl::Stop() {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	router_->Stop();
}

void RTCVideoSendStreamImpl::SetSource(
	rtc::RTCVideoPacketRouteSourceInterface<std::unique_ptr<webrtc::RtpPacketReceived>>* source) {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	feeder_->SetSource(source);
}

RTCVideoSendStream::Stats RTCVideoSendStreamImpl::GetStats() {
	RTCVideoSendStream::Stats stats;
	return stats;
}

void RTCVideoSendStreamImpl::StopPermanently() {
	if (TaskQueueBase::Current() != worker_queue_) {
		worker_queue_->PostTask(SafeTask(safety_.flag(), [this]() {
			RTC_DCHECK_RUN_ON(&thread_checker_);
			router_->Stop();
			router_.reset();
			thread_sync_event_.Set();
		}));
		thread_sync_event_.Wait(rtc::Event::kForever);
		feeder_->Stop();
		feeder_.reset();
		return;
	}
	RTC_DCHECK_RUN_ON(&thread_checker_);
	router_->Stop();
	router_.reset();

	feeder_->Stop();
	feeder_.reset();
}

}	// namespace webrtc
