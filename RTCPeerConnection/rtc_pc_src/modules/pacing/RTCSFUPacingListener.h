//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_SFU_PACING_LISTENER_H__
#define __RTC_SFU_PACING_LISTENER_H__

namespace webrtc {

class RTCRtpRtcpInterface;

class RTCSFUPacingListener {
public:
	virtual void AddPacingRtpModule(RTCRtpRtcpInterface* rtp_module) = 0;
	virtual void RemovePacingRtpModule(RTCRtpRtcpInterface* rtp_module) = 0;
};

} // webrtc
#endif // __RTC_SFU_PACING_LISTENER_H__