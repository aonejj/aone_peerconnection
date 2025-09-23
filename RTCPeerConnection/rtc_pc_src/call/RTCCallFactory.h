//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /call/call_factory.h
//
//////////////////////////////////////////////////////////////////////////


#ifndef __RTC_CALL_CALL_FACTORY_H__
#define __RTC_CALL_CALL_FACTORY_H__

#include "api/task_queue/task_queue_factory.h"
#include "rtc_base/system/no_unique_address.h"

#include "../api/call/RTCCallFactoryInterface.h"

#include "../api/audio/RTCAudioRtpPacketListenSinkInterface.h"
#include "RTCCall.h"


namespace webrtc {

class RTCCallFactory : public RTCCallFactoryInterface {
public:
	RTCCallFactory();

private:
	~RTCCallFactory() override {}

	RTCCall* CreateRTCCall(TaskQueueFactory* task_queue_factory, 
						   RTCAudioRtpPacketListenSinkInterface* audio_rtp_packet_listener,
						   RTCVideeRtpPacketListenerProxyGetInterface* video_rtp_packet_listener_proxy_get_interface
						   , rtc::RTCThreadManagerInterface *rtc_thread_manager
						   ) override;

};

}

#endif	// __RTC_CALL_CALL_FACTORY_H__