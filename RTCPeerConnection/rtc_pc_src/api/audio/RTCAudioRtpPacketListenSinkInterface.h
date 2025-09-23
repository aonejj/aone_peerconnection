//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_API_AUDIO_AUDIO_RTP_PACKET_LISTEN_SINK_INTERFACE_H__
#define __RTC_API_AUDIO_AUDIO_RTP_PACKET_LISTEN_SINK_INTERFACE_H__

#include "call/rtp_packet_sink_interface.h"

namespace webrtc {

class RTCAudioRtpPacketListenSinkInterface : public RtpPacketSinkInterface {
public:
	virtual ~RTCAudioRtpPacketListenSinkInterface() = default;

	virtual void OnStart() = 0;
	virtual void OnStop() = 0;
};

}	// namespace webrtc


#endif // __RTC_API_AUDIO_AUDIO_RTP_PACKET_LISTEN_INTERFACE_H__