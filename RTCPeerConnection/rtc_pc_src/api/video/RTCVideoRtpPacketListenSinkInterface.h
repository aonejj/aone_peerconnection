//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_API_VIDEO_RTP_PACKET_LISTEN_SINK_INTERFACE_H__
#define __RTC_API_VIDEO_RTP_PACKET_LISTEN_SINK_INTERFACE_H__

#include "call/rtp_packet_sink_interface.h"

namespace webrtc {

class RTCVideoRtpPacketListenSinkInterface : public RtpPacketSinkInterface {
public:
	virtual ~RTCVideoRtpPacketListenSinkInterface() = default;

	virtual void OnStart() = 0;
	virtual void OnStop() = 0;
};

}	// namespace webrtc


#endif // __RTC_API_VIDEO_RTP_PACKET_LISTEN_SINK_INTERFACE_H__