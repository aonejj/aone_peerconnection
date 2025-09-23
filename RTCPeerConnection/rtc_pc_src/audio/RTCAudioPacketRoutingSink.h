//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_AUDIO_AUDIO_PACKET_ROUTING_SINK_H__
#define __RTC_AUDIO_AUDIO_PACKET_ROUTING_SINK_H__

#include <memory>
#include "rtc_base/system/rtc_export.h"

#include "modules/rtp_rtcp/source/rtp_packet_received.h"

namespace webrtc {

class RTC_EXPORT RTCAudioPacketRoutingSink {
public:
	virtual ~RTCAudioPacketRoutingSink() {}

	virtual void OnRoutingPacket(std::unique_ptr<RtpPacketReceived> packet) = 0;
};

}	// namespace webrtc

#endif	// __RTC_AUDIO_AUDIO_PACKET_ROUTING_SINK_H__