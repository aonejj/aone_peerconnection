//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////


#ifndef __RTC_RTP_RTCP_INTERFACE_H__
#define __RTC_RTP_RTCP_INTERFACE_H__

#include <memory>
#include <string>
#include <vector>

#include "absl/types/optional.h"

#include "api/video/video_bitrate_allocator.h"
#include "modules/rtp_rtcp/include/rtp_rtcp_defines.h"
#include "modules/rtp_rtcp/include/receive_statistics.h"
#include "modules/rtp_rtcp/include/report_block_data.h"
#include "modules/rtp_rtcp/include/rtp_packet_sender.h"
#include "rtc_base/constructor_magic.h"
#include "system_wrappers/include/ntp_time.h"

namespace webrtc {

class RateLimiter;
class RemoteBitrateEstimator;
class Transport;

class RTCRtpRtcpInterface : public RtcpFeedbackSenderInterface {
public:
	struct Configuration {
		Configuration() = default;
		Configuration(Configuration&& rhs) = default;

		bool audio = false;
		bool receiver_only = false;

		Clock* clock = nullptr;
		ReceiveStatisticsProvider* receive_statistics = nullptr;

		Transport* outgoing_transport = nullptr;
		RtcpIntraFrameObserver* intra_frame_callback = nullptr;
		RtcpLossNotificationObserver* rtcp_loss_notification_observer = nullptr;
		RtcpBandwidthObserver* bandwidth_callback = nullptr;

		NetworkStateEstimateObserver* network_state_estimate_observer = nullptr;
		TransportFeedbackObserver* transport_feedback_callback = nullptr;
		VideoBitrateAllocationObserver* bitrate_allocation_observer = nullptr;

		RtcpRttStats* rtt_stats = nullptr;
		RtcpPacketTypeCounterObserver* rtcp_packet_type_counter_observer = nullptr;

		RtcpStatisticsCallback* rtcp_statistics_callback = nullptr;
		RtcpCnameCallback* rtcp_cname_callback = nullptr;
		ReportBlockDataObserver* report_block_data_observer = nullptr;

		RemoteBitrateEstimator* remote_bitrate_estimator = nullptr;

		BitrateStatisticsObserver* send_bitrate_observer = nullptr;
		SendSideDelayObserver* send_side_delay_observer = nullptr;

		SendPacketObserver* send_packet_observer = nullptr;
		RateLimiter* retransmission_rate_limiter = nullptr;
		StreamDataCountersCallback* rtp_stats_callback = nullptr;
		RtpPacketSender* paced_sender = nullptr;

		int rtcp_report_interval_ms = 0;


		bool populate_network2_timestamp = false;

		bool extmap_allow_mixed = false;

		uint32_t local_media_ssrc = 0;
		absl::optional<uint32_t> rtx_send_ssrc;

		bool need_rtp_packet_infos = false;

		bool enable_rtx_padding_prioritization = true;

		// Estimate RTT as non-sender as described in
		// https://tools.ietf.org/html/rfc3611#section-4.4 and #section-4.5
		bool non_sender_rtt_measurement = false;

		bool enable_send_packet_batching = false;

	private:
		RTC_DISALLOW_COPY_AND_ASSIGN(Configuration);
	};

	struct RTCSenderReportStats {
		NtpTime last_arrival_timestamp;
		NtpTime last_remote_timestamp;
		uint32_t last_remote_rtp_timestamp = 0;
		uint32_t packets_sent = 0;
		uint64_t bytes_sent = 0;
		uint64_t reports_count = 0;
	};

	struct RTCNonSenderRttStats {
		absl::optional<TimeDelta> round_trip_time;
		TimeDelta total_round_trip_time = TimeDelta::Zero();
		int round_trip_time_measurements = 0;
	};

	virtual void IncomingRtcpPacket(const uint8_t* incoming_packet,
		size_t incoming_packet_length) = 0;

	virtual void SetRemoteSSRC(uint32_t ssrc) = 0;

	virtual void SetLocalSsrc(uint32_t local_ssrc) = 0;

	virtual int32_t SetSendingStatus(bool sending) = 0;

	virtual int32_t SendRTCP(RTCPPacketType rtcp_packet_type) = 0;

	virtual int32_t RemoteRTCPStat(
		std::vector<RTCPReportBlock>* receive_blocks) const = 0;

	virtual void SetRTCPStatus(RtcpMode method) = 0;

	virtual absl::optional<TimeDelta> LastRtt() const = 0;

	virtual uint32_t SSRC() const = 0;

	virtual absl::optional<uint32_t> RtxSsrc() const = 0;

	virtual bool IsAudioConfigured() const = 0;

	virtual RtcpMode RTCP() const = 0;

	virtual std::vector<ReportBlockData> GetLatestReportBlockData() const = 0;
	virtual absl::optional<RTCSenderReportStats> GetSenderReportStats() const = 0;
	virtual absl::optional<RTCNonSenderRttStats> GetNonSenderRttStats() const = 0;

	virtual bool SupportsPadding() const = 0;
	virtual bool SupportsRtxPayloadPadding() const = 0;

	virtual bool TrySendPacket(std::unique_ptr<RtpPacketToSend> packet,
		const PacedPacketInfo& pacing_info) = 0;

	virtual void OnBatchComplete() = 0;

	virtual std::vector<std::unique_ptr<RtpPacketToSend>> GeneratePadding(
		size_t target_size_bytes) = 0;

	virtual void OnAbortedRetransmissions(
		rtc::ArrayView<const uint16_t> sequence_numbers) = 0;
	virtual void SetSendingMediaStatus(bool sending) {}
};

}	// namespace webrtc
#endif // __RTC_RTP_RTCP_INTERFACE_H__