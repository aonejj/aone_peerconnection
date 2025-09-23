//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_VIDEO_PACKET_ROUTING_SINK_H__
#define __RTC_VIDEO_PACKET_ROUTING_SINK_H__

#include <memory>
#include "rtc_base/system/rtc_export.h"

#include "modules/rtp_rtcp/source/rtp_packet_received.h"

namespace webrtc {

class RTC_EXPORT RTCVideoPacketRoutingSink {
public:
	virtual ~RTCVideoPacketRoutingSink() {}

	virtual void OnRoutingPacket(std::unique_ptr<RtpPacketReceived> packet) = 0;
};

}

#endif	// __RTC_VIDEO_PACKET_ROUTING_SINK_H__