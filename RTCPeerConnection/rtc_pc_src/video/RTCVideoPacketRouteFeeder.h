//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_VIDEO_PACKET_ROUTE_FEEDER_H__
#define __RTC_VIDEO_PACKET_ROUTE_FEEDER_H__

#include <memory>
#include <queue>

#include "api/task_queue/task_queue_factory.h"
#include "rtc_base/task_queue.h"
#include "rtc_base/constructor_magic.h"
#include "rtc_base/system/no_unique_address.h"
#include "rtc_base/task_utils/repeating_task.h"
#include "modules/include/module.h"
#include "modules/utility/include/process_thread.h"
#include "modules/rtp_rtcp/source/rtp_packet_received.h"
#include "system_wrappers/include/clock.h"
#include "rtc_base/synchronization/mutex.h"

#include "../api/RTCPacketRouteSinkInterface.h"
#include "../api/video/RTCVideoPacketRouteSourceInterface.h"
#include "RTCVideoPacketRoutingSink.h"
#include "../rtc_pc_src/_deprecate_defines.h"


namespace webrtc {

class RTCVideoPacketRouteFeeder :
	public rtc::RTCPacketRoutePtrSinkInterface<std::unique_ptr<RtpPacketReceived>>
{
public:
	RTCVideoPacketRouteFeeder(Clock* clock);
	~RTCVideoPacketRouteFeeder() override;

public:
	void SetSource(rtc::RTCVideoPacketRouteSourceInterface<std::unique_ptr<RtpPacketReceived>> *source);
	void SetSink(RTCVideoPacketRoutingSink *sink);

	void Start();
	void Stop();

	void GenerateKeyFrame();

public:
	void OnFeedPacketPtr(std::unique_ptr<RtpPacketReceived> packet) override;

private:
	Clock* clock_;
	rtc::RTCVideoPacketRouteSourceInterface<std::unique_ptr<RtpPacketReceived>> *source_;
 	RTCVideoPacketRoutingSink* sink_;
	bool is_run_;

	Mutex	_source_lock;
	Mutex	_sink_lock;
	Mutex	_run_lock;

private:
	RTC_DISALLOW_COPY_AND_ASSIGN(RTCVideoPacketRouteFeeder);
};

}	// namespace webrtc

#endif	// __RTC_VIDEO_PACKET_ROUTE_FEEDER_H__