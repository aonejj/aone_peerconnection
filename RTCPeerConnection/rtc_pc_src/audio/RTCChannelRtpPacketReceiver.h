//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_AUDIO_CHANNEL_RTP_PACKET_RECEIVE_H__
#define __RTC_AUDIO_CHANNEL_RTP_PACKET_RECEIVE_H__

#include "rtc_base/thread_checker.h"
#include "rtc_base/synchronization/mutex.h"
#include "modules/rtp_rtcp/include/receive_statistics.h"
#include "modules/rtp_rtcp/include/remote_ntp_time_estimator.h"

#include "../api/audio/RTCAudioRtpPacketListenSinkInterface.h"
#include "RTCChannelRtpPacketReceiverInterface.h"
#include "../modules/rtp_rtcp/source/RTCRtpRtcpRecvImpl.h"
#include "../modules/pacing/RTCPacketRouter.h"


namespace webrtc {

namespace voe {

class RTCChannelRtpPacketReceiver : public RTCChannelRtpPacketReceiverInterface {
public:
	RTCChannelRtpPacketReceiver(Clock* clock, const RTCAudioReceiveStream::Config &config);

	~RTCChannelRtpPacketReceiver() override;

public:
	void SetSink(RTCAudioRtpPacketListenSinkInterface *sink) override;

	void SetReceiveCodecs(const std::map<int, SdpAudioFormat>& codecs) override;

	absl::optional<std::pair<int32_t, SdpAudioFormat>> GetReceiveCodec()
		const override;

	void OnRtpPacket(const RtpPacketReceived& packet) override;

	void ReceivedRTCPPacket(const uint8_t* data, size_t length) override;

	void Start() override;
	void Stop() override;

	void RegisterReceiverCongestionControlObjects(
		RTCPacketRouter* packet_router) override;
	void ResetReceiverCongestionControlObjects() override;

private:
	Clock* const clock_;
	const RTCAudioReceiveStream::Config& config_;
	rtc::ThreadChecker worker_thread_checker_;
	Mutex callback_mutex_;

	RemoteNtpTimeEstimator ntp_estimator_;

	uint32_t local_ssrc_;
	uint32_t remote_ssrc_;
	RTCAudioRtpPacketListenSinkInterface *sink_;

	const std::unique_ptr<ReceiveStatistics> rtp_receive_statistics_;

	const std::unique_ptr<RTCRtpRtcpRecvImpl> rtp_rtcp_;

	// Indexed by payload type.
	std::map<uint8_t, int> payload_type_frequencies_;

	int64_t _debug_last_recvMs = -1ll;	
	int64_t _debug_abs_ms;	
	int64_t _debug_arrive_time_ms;
	RTCPacketRouter* packet_router_ = nullptr;
};

}	// namespace voe

}	// namespace webrtc

#endif // __RTC_AUDIO_CHANNEL_RTP_PACKET_RECEIVE_H__