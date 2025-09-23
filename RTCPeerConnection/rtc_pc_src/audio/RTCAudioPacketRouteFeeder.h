//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_AUDIO_AUDIO_PACKET_ROUTE_FEEDER_H__
#define __RTC_AUDIO_AUDIO_PACKET_ROUTE_FEEDER_H__

#include <memory>
#include <queue>

#include "api/task_queue/task_queue_factory.h"
#include "rtc_base/task_queue.h"
#include "rtc_base/constructor_magic.h"
#include "rtc_base/system/no_unique_address.h"
#include "rtc_base/synchronization/mutex.h"
#include "rtc_base/task_utils/repeating_task.h"
#include "modules/include/module.h"
#include "modules/utility/include/process_thread.h"
#include "modules/rtp_rtcp/source/rtp_packet_received.h"
#include "system_wrappers/include/clock.h"

#include "../call/RTCAudioSendStream.h"
#include "../api/RTCPacketRouteSinkInterface.h"
#include "../api/audio/RTCAudioPacketRouteSourceInterface.h"
#include "RTCAudioPacketRoutingSink.h"

#include "../rtc_pc_src/_deprecate_defines.h"		// deprecate

namespace webrtc {

class RTCAudioPacketRouteFeeder :
	public rtc::RTCPacketRoutePtrSinkInterface<std::unique_ptr<RtpPacketReceived>> 
{
public:
	RTCAudioPacketRouteFeeder(Clock *clock, 
							  const RTCAudioSendStream::Config* config);
	~RTCAudioPacketRouteFeeder() override;

public:
	void SetSource(rtc::RTCAudioPacketRouteSourceInterface<std::unique_ptr<RtpPacketReceived>>* source);
	void SetSink(RTCAudioPacketRoutingSink* sink);

	void Start();
	void Stop();

public:
	void OnFeedPacketPtr(std::unique_ptr<RtpPacketReceived> packet) override;

private:
	Clock* clock_;
	rtc::RTCAudioPacketRouteSourceInterface<std::unique_ptr<RtpPacketReceived>> *source_;
	RTCAudioPacketRoutingSink* sink_;

	uint32_t payload_type_;
	bool is_run_;

	Mutex	_source_lock;
	Mutex	_sink_lock;
	Mutex	_run_lock;

private:
	RTC_DISALLOW_COPY_AND_ASSIGN(RTCAudioPacketRouteFeeder);
};

}	// namespace webrtc

#endif // __RTC_AUDIO_AUDIO_PACKET_ROUTE_FEEDER_H__