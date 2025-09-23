//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_AUDIO_CHANNEL_RTP_PACKET_RECEIVE_INTERFACE_H__
#define __RTC_AUDIO_CHANNEL_RTP_PACKET_RECEIVE_INTERFACE_H__

#include <map>
#include <memory>
#include <vector>

#include "absl/types/optional.h"

#include "api/audio_codecs/audio_format.h"
#include "modules/utility/include/process_thread.h"
#include "call/rtp_packet_sink_interface.h"
#include "system_wrappers/include/clock.h"

#include "../call/RTCAudioReceiveStream.h"

namespace webrtc {

class RTCAudioRtpPacketListenSinkInterface;
class RTCPacketRouter;

namespace voe {

class RTCChannelRtpPacketReceiverInterface : public RtpPacketSinkInterface {
public:
	virtual ~RTCChannelRtpPacketReceiverInterface() = default;

	virtual void SetSink(RTCAudioRtpPacketListenSinkInterface *sink) = 0;

	virtual void SetReceiveCodecs(
		const std::map<int, SdpAudioFormat>& codecs) = 0;

	virtual absl::optional<std::pair<int, SdpAudioFormat>> GetReceiveCodec()
		const = 0;

	virtual void ReceivedRTCPPacket(const uint8_t* data, size_t length) = 0;

	virtual void Start() = 0;
	virtual void Stop() = 0;

	virtual void RegisterReceiverCongestionControlObjects(
		RTCPacketRouter* packet_router) = 0;
	virtual void ResetReceiverCongestionControlObjects() = 0;
};

std::unique_ptr<RTCChannelRtpPacketReceiverInterface> CreateRTCChannelRtpPacketReceiver(Clock* clock, const RTCAudioReceiveStream::Config &config);

} // namespace voe
}	// namespace webrtc


#endif	// __RTC_AUDIO_CHANNEL_RTP_PACKET_RECEIVE_INTERFACE_H__