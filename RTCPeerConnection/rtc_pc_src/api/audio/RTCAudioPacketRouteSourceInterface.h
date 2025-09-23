//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_API_AUDIO_PACKET_ROUTE_SOURCE_INTERFACE_H__
#define __RTC_API_AUDIO_PACKET_ROUTE_SOURCE_INTERFACE_H__

#include "api/notifier.h"

#include "../_rtc_media_stream_interface.h"
#include "../RTCPacketRouteSinkInterface.h"


namespace rtc {

template<typename packetT>
class RTCAudioPacketRouteSourceInterface : 
	public webrtc::Notifier<webrtc::RTCAudioSourceInterface> {
public:
	~RTCAudioPacketRouteSourceInterface() = default;

public:
	virtual void AddPacketRoutingSink(RTCPacketRoutePtrSinkInterface<packetT> *sink) = 0;
	virtual void RemovePacketRoutingSink(RTCPacketRoutePtrSinkInterface<packetT> *sink) = 0;
};

}


#endif	// __RTC_API_AUDIO_PACKET_ROUTE_SOURCE_INTERFACE_H__