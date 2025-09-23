//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_API_VIDEO_PACKET_ROUTE_SOURCE_INTERFACE_H__
#define __RTC_API_VIDEO_PACKET_ROUTE_SOURCE_INTERFACE_H__

#include "api/notifier.h"

#include "../../../src_update/api/_media_stream_interface.h"
#include "../RTCPacketRouteSinkInterface.h"


namespace rtc {

template <typename packetT>
class RTCVideoPacketRouteSourceInterface : 
	public webrtc::Notifier<webrtc::VideoTrackSourceInterface> {
public:
	~RTCVideoPacketRouteSourceInterface() = default;

public:
	virtual void AddPacketRoutingSink(RTCPacketRoutePtrSinkInterface<packetT> *sink) = 0;
	virtual void RemovePacketRoutingSink(RTCPacketRoutePtrSinkInterface<packetT> *sink) = 0;
};

class RTCVideoPacketRouteSourceFeedbackInterface {
public:
	RTCVideoPacketRouteSourceFeedbackInterface() = default;
	~RTCVideoPacketRouteSourceFeedbackInterface() = default;

public:
	virtual void OnGenerateKeyFrameFeedback() = 0;
	virtual void OnGenerateKeyFrameFeedbackPli(uint32_t ssrc) = 0;
};

}

#endif // __RTC_API_VIDEO_PACKET_ROUTE_SOURCE_INTERFACE_H__