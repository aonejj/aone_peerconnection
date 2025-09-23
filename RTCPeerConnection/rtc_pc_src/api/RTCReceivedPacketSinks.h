//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_RECEIVED_PACKET_SINKS_H__
#define __RTC_RECEIVED_PACKET_SINKS_H__

#include "absl/types/optional.h"
#include "modules/rtp_rtcp/include/rtp_header_extension_map.h"


namespace webrtc {

class RtpPacketReceived;

class RTCMediaReceivedPacketSink {
public:
	RTCMediaReceivedPacketSink() = default;
	~RTCMediaReceivedPacketSink() = default;

public:
	virtual void OnRtpPacket(const RtpPacketReceived& packet) = 0;
	virtual void OnRtxRtpPacket(const RtpPacketReceived& packet) {}
	virtual void SetRtpHeaderExtensionMap(absl::optional<RtpHeaderExtensionMap> extMap) = 0;
};

} // namespace webrtc


#endif // __RTC_RECEIVED_PACKET_CALLBACKS_H__