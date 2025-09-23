//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_API_AUDIO_AUDIO_RTP_PACKET_LISTENER_PROXY_H__
#define __RTC_API_AUDIO_AUDIO_RTP_PACKET_LISTENER_PROXY_H__

#include "absl/types/optional.h"

#include "modules/rtp_rtcp/include/rtp_header_extension_map.h"
#include "rtc_base/synchronization/mutex.h"

#include "RTCAudioRtpPacketListenSinkInterface.h"


namespace webrtc {

class RtpPacketReceived;
//class RTCAudioReceivedPacketSink;
class RTCMediaReceivedPacketSink;

class RTCAudioRtpPacketListenerProxy : public RTCAudioRtpPacketListenSinkInterface {
public:
	RTCAudioRtpPacketListenerProxy();
	~RTCAudioRtpPacketListenerProxy();

public:
	void SetSink(RTCMediaReceivedPacketSink* sink);

	void OnStart() override;
	void OnStop() override;
	void OnRtpPacket(const RtpPacketReceived& packet) override;

	void SetRtpHeaderExtensionMap(absl::optional<RtpHeaderExtensionMap> extMap);

private:
	Mutex sink_lock_;
	RTCMediaReceivedPacketSink *sink_;

	absl::optional<webrtc::RtpHeaderExtensionMap> _rtp_extensions;
};

}


#endif // __RTC_API_AUDIO_AUDIO_RTP_PACKET_LISTENER_PROXY_H__