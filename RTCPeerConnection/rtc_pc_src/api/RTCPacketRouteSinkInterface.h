//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_API_PACKET_ROUTE_SINK_INTERFACE_H__
#define __RTC_API_PACKET_ROUTE_SINK_INTERFACE_H__

#include "rtc_base/checks.h"
#include "api/call/transport.h"

namespace rtc {

template<typename packetT>
class RTCPacketRouteSinkInterface {
public:
	virtual ~RTCPacketRouteSinkInterface() = default;

	virtual void OnFeedPacket(const packetT& packet) = 0;
	virtual void OnFeedRtxPacket(const packetT& packet) {}
};

template<typename packetT>
class RTCPacketRoutePtrSinkInterface {
public:
	virtual ~RTCPacketRoutePtrSinkInterface() = default;

	virtual void OnFeedPacketPtr(packetT packet) = 0;
};

}	// namespace rtc

#endif // __RTC_API_VIDEO_PACKET_ROUTE_SINK_INTERFACE_H__