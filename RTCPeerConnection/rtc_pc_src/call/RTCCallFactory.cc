//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /call/call_factory.cc
//
//////////////////////////////////////////////////////////////////////////

#include "RTCCallFactory.h"

namespace webrtc {

RTCCallFactory::RTCCallFactory() {

}

RTCCall* RTCCallFactory::CreateRTCCall(TaskQueueFactory* task_queue_factory,
	RTCAudioRtpPacketListenSinkInterface* audio_rtp_packet_listener,
	RTCVideeRtpPacketListenerProxyGetInterface* video_rtp_packet_listener_proxy_get_interface
	, rtc::RTCThreadManagerInterface *rtc_thread_manager
	) {

	return RTCCall::Create(
		task_queue_factory, 
		audio_rtp_packet_listener, 
		video_rtp_packet_listener_proxy_get_interface
		, rtc_thread_manager
		);
}

std::unique_ptr<RTCCallFactoryInterface> CreateCallFactory() {
	return std::unique_ptr<RTCCallFactoryInterface>(new RTCCallFactory());
}

} // namespace webrtc