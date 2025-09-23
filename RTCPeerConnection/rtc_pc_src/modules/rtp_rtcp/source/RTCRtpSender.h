//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /module/rtp_rtcp/source/rtp_sender.h
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_MODULE_RTP_RTCP_SOURCE_RTP_SENDER__
#define __RTC_MODULE_RTP_RTCP_SOURCE_RTP_SENDER__

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "modules/rtp_rtcp/include/rtp_header_extension_map.h"
#include "modules/rtp_rtcp/include/rtp_packet_sender.h"
#include "modules/rtp_rtcp/include/rtp_rtcp_defines.h"
#include "modules/rtp_rtcp/source/rtp_packet_history.h"
#include "rtc_base/random.h"
#include "rtc_base/synchronization/mutex.h"

#include "RTCRtpRtcpInterface.h"

namespace webrtc {

class RtpPacketToSend;

constexpr size_t kMaxPaddingLength = 255;

class RTCRTPSender {
public:
	RTCRTPSender(const RTCRtpRtcpInterface::Configuration& config,
				 RtpPacketHistory* packet_history,
				 RtpPacketSender* packet_sender);
	~RTCRTPSender();

	void SetRTPHeaderExtensionMap(const std::vector<RtpExtension>& extensions);

	void SetSendingMediaStatus(bool enabled) RTC_LOCKS_EXCLUDED(send_mutex_);
	bool SendingMedia() const RTC_LOCKS_EXCLUDED(send_mutex_);
	bool IsAudioConfigured() const;

	bool SupportsPadding() const RTC_LOCKS_EXCLUDED(send_mutex_);
	bool SupportsRtxPayloadPadding() const RTC_LOCKS_EXCLUDED(send_mutex_);

	std::vector<std::unique_ptr<RtpPacketToSend>> GeneratePadding(size_t target_size_bytes,
																  bool media_has_been_sent,
																  bool can_send_padding_on_media_ssrc) RTC_LOCKS_EXCLUDED(send_mutex_);

	void OnReceivedNack(const std::vector<uint16_t>& nack_sequence_numbers,
		int64_t avg_rtt);
	int32_t ReSendPacket(uint16_t packet_id) RTC_LOCKS_EXCLUDED(send_mutex_);

	int RtxStatus() const { return rtx_; }
	absl::optional<uint32_t> RtxSsrc() const { return rtx_ssrc_; }
	void SetRtxPayloadType(int payload_type, int associated_payload_type);

	uint32_t SSRC() const { return ssrc_; }

	void EnqueuePackets(std::vector<std::unique_ptr<RtpPacketToSend>> packets);
	void EnqueuePacket(std::unique_ptr<RtpPacketToSend> packet);

private:
	std::unique_ptr<RtpPacketToSend> BuildRtxPacket(
		const RtpPacketToSend& packet) RTC_LOCKS_EXCLUDED(send_mutex_);

private:
	Clock* const clock_;
	Random random_;
	const bool audio_configured_;

	const uint32_t ssrc_;
	const absl::optional<uint32_t> rtx_ssrc_;

	RtpPacketHistory* const packet_history_;
	RtpPacketSender* const paced_sender_;

	mutable Mutex send_mutex_;

	bool sending_media_ RTC_GUARDED_BY(send_mutex_);
	size_t max_packet_size_;

	RtpHeaderExtensionMap rtp_header_extension_map_;
	int rtx_;
	std::map<int8_t, int8_t> rtx_payload_type_map_;
	bool supports_bwe_extension_;


private:
	RTC_DISALLOW_COPY_AND_ASSIGN(RTCRTPSender);
};

}



#endif	// __RTC_MODULE_RTP_RTCP_SOURCE_RTP_SENDER__