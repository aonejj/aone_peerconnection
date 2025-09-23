//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_API_VIDEO_VIDEO_RTP_PACKET_LISTENER_PROXY_H__
#define __RTC_API_VIDEO_VIDEO_RTP_PACKET_LISTENER_PROXY_H__

#include <vector>

#include "absl/types/optional.h"

#include "modules/rtp_rtcp/include/rtp_header_extension_map.h"
#include "rtc_base/synchronization/mutex.h"

#include "RTCVideoRtpPacketListenSinkInterface.h"

namespace webrtc {

class RtpPacketReceived;
//class RTCVideoReceivedPacketSink;
class RTCMediaReceivedPacketSink;
struct RtpParameters;

class RTCVideoRtpPacketListenerProxy : public RTCVideoRtpPacketListenSinkInterface {
public :
	RTCVideoRtpPacketListenerProxy();
	~RTCVideoRtpPacketListenerProxy();

public:
	void SetSink(RTCMediaReceivedPacketSink* sink);

	void OnStart() override;
	void OnStop() override;
	void OnRtpPacket(const RtpPacketReceived& packet) override;
	void OnRtxRtpPacket(const RtpPacketReceived& packet) override;

	void SetRtpHeaderExtensionMap(absl::optional<RtpHeaderExtensionMap> extMap);

private:
	Mutex sink_lock_;
	RTCMediaReceivedPacketSink *sink_;

	absl::optional<webrtc::RtpHeaderExtensionMap> _rtp_extensions;
};

}

#endif // __RTC_API_VIDEO_VIDEO_RTP_PACKET_LISTENER_PROXY_H__