//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_RTP_RTCP_RECV_IMPL_H__
#define __RTC_RTP_RTCP_RECV_IMPL_H__

#include <stddef.h>
#include <stdint.h>

#include <memory>
#include <set>
#include <string>
#include <vector>

#include "absl/strings/string_view.h"
#include "absl/types/optional.h"

#include "api/rtp_headers.h"
#include "api/task_queue/task_queue_base.h"
#include "api/video/video_bitrate_allocation.h"
#include "modules/include/module_fec_types.h"
#include "modules/remote_bitrate_estimator/include/remote_bitrate_estimator.h"
#include "modules/rtp_rtcp/include/rtp_rtcp_defines.h" 
#include "modules/rtp_rtcp/source/rtcp_packet/tmmb_item.h"
#include "modules/rtp_rtcp/source/rtp_packet_history.h"
#include "modules/rtp_rtcp/source/rtp_packet_to_send.h"
#include "modules/rtp_rtcp/source/rtp_sender.h"
#include "modules/rtp_rtcp/source/rtp_sender_egress.h"
#include "rtc_base/synchronization/mutex.h"
#include "rtc_base/system/no_unique_address.h"
#include "rtc_base/task_utils/pending_task_safety_flag.h"
#include "rtc_base/task_utils/repeating_task.h"

#include "RTCRtcpSender.h"
#include "RTCRtcpReceiver.h"
#include "RTCRtpRtcpInterface.h"


namespace webrtc {

class Clock;

class RTCRtpRtcpRecvImpl final : public RTCRtpRtcpInterface,
								 public RTCRtcpReceiver::ModuleRtpRtcp {
public:
	explicit RTCRtpRtcpRecvImpl(
		const RTCRtpRtcpInterface::Configuration& configuration);
	~RTCRtpRtcpRecvImpl() override;

public:
	static std::unique_ptr<RTCRtpRtcpRecvImpl> Create(
		const Configuration& configuration);

	//////////////////////////////////////////////////////////////////////////
	// RTCRtpRtcpInterface method
	// receiver part ... used
	void IncomingRtcpPacket(const uint8_t* incoming_packet,
		size_t incoming_packet_length) override;
	void SetRemoteSSRC(uint32_t ssrc) override;
	void SetLocalSsrc(uint32_t local_ssrc) override;
	int32_t SetSendingStatus(bool sending) override;
	int32_t SendRTCP(RTCPPacketType rtcp_packet_type) override;
	int32_t RemoteRTCPStat(std::vector<RTCPReportBlock>* receive_blocks) const override;
	void SetRTCPStatus(RtcpMode method) override;
	absl::optional<TimeDelta> LastRtt() const override;

	uint32_t SSRC() const override { return rtcp_sender_.SSRC(); }
	absl::optional<uint32_t> RtxSsrc() const override { return absl::nullopt; }
	bool IsAudioConfigured() const override { return !is_video_; }
	RtcpMode RTCP() const override;

	std::vector<ReportBlockData> GetLatestReportBlockData() const override;
	absl::optional<RTCSenderReportStats> GetSenderReportStats() const override;
	absl::optional<RTCNonSenderRttStats> GetNonSenderRttStats() const override;

	bool SupportsPadding() const override { return false; }
	bool SupportsRtxPayloadPadding() const override { return false; }
	bool TrySendPacket(std::unique_ptr<RtpPacketToSend> packet, const PacedPacketInfo& pacing_info) override;

	void OnBatchComplete() override {}
	std::vector<std::unique_ptr<RtpPacketToSend>> GeneratePadding(
		size_t target_size_bytes) override;

	void OnAbortedRetransmissions(
		rtc::ArrayView<const uint16_t> sequence_numbers) override {}

	//////////////////////////////////////////////////////////////////////////
	// RtcpFeedbackSenderInterface method
	void SendCombinedRtcpPacket(
		std::vector<std::unique_ptr<rtcp::RtcpPacket>> rtcp_packets) override;
	void SetRemb(int64_t bitrate_bps, std::vector<uint32_t> ssrcs) override;
	void UnsetRemb() override;

	//////////////////////////////////////////////////////////////////////////
	// RTCPReceiver::ModuleRtpRtcp methods
	void SetTmmbn(std::vector<rtcp::TmmbItem> bounding_set) override;
	void OnRequestSendReport() override;
	void OnReceivedNack(const std::vector<uint16_t>& nack_sequence_numbers) override;
	void OnReceivedRtcpReportBlocks(const ReportBlockList& report_blocks) override;

public:
	int32_t SendLossNotification(uint16_t last_decoded_seq_num,
		uint16_t last_received_seq_num,
		bool decodability_flag,
		bool buffering_allowed);
	void SendFullIntraRequest() { SendRTCP(kRtcpFir); }
	void SendPictureLossIndication() { SendRTCP(kRtcpPli); }
	void SendNack(const std::vector<uint16_t>& sequence_numbers);
	uint32_t local_media_ssrc() const;

private:
	RTCRtcpSender::FeedbackState GetFeedbackState();
	void PeriodicUpdate();
	void MaybeSendRtcp() RTC_RUN_ON(worker_queue_);
	void ScheduleRtcpSendEvaluation(TimeDelta duration);
	void MaybeSendRtcpAtOrAfterTimestamp(Timestamp execution_time)
		RTC_RUN_ON(worker_queue_);
	void ScheduleMaybeSendRtcpAtOrAfterTimestamp(Timestamp execution_time,
		TimeDelta duration);

	void set_rtt_ms(int64_t rtt_ms);
	int64_t rtt_ms() const;

private:
	TaskQueueBase* const worker_queue_;
	RTC_NO_UNIQUE_ADDRESS SequenceChecker rtcp_thread_checker_;

	RTCRtcpSender rtcp_sender_;
	RTCRtcpReceiver rtcp_receiver_;

	Clock* const clock_;

	RtcpRttStats* const rtt_stats_;
	RepeatingTaskHandle rtt_update_task_ RTC_GUARDED_BY(worker_queue_);

	// The processed RTT from RtcpRttStats.
	mutable Mutex mutex_rtt_;
	int64_t rtt_ms_ RTC_GUARDED_BY(mutex_rtt_);
	int  rtx_;
	bool is_video_;

	RTC_NO_UNIQUE_ADDRESS ScopedTaskSafety task_safety_;
};

}

#endif // __RTC_RTP_RTCP_RECV_IMPL_H__