//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_RTP_RTCP_SEND_IMPL_H__
#define __RTC_RTP_RTCP_SEND_IMPL_H__

#include "api/task_queue/task_queue_base.h"
#include "modules/rtp_rtcp/include/rtp_rtcp_defines.h"
#include "modules/rtp_rtcp/source/rtp_packet_history.h"
#include "modules/rtp_rtcp/source/packet_sequencer.h"
#include "rtc_base/ref_count.h"
#include "rtc_base/synchronization/sequence_checker.h"
#include "rtc_base/rate_statistics.h"
#include "rtc_base/task_utils/pending_task_safety_flag.h"
#include "rtc_base/task_utils/repeating_task.h"
#include "system_wrappers/include/clock.h"

#include "../rtc_pc_src/api/call/RTCTransportSequenceNumberGenerator.h"
#include "RTCRtcpSender.h"
#include "RTCRtcpReceiver.h"
#include "RTCRtpRtcpInterface.h"
#include "RTCRtpSenderEgress.h"
#include "RTCRtpSender.h"
#include "../../../call/RTCVideoSendStream.h"
#include "../../../call/RTCAudioSendStream.h"

namespace webrtc {

class RTCRtpRtcpSendImpl : public RTCRtpRtcpInterface,
						   public RTCRtcpReceiver::ModuleRtpRtcp {
public:
	static std::unique_ptr<RTCRtpRtcpSendImpl> Create(
		Clock *clock,
		RTCRtpRtcpInterface::Configuration &rtprtcp_config,
		RtcpRttStats* rtt_stats);

	RTCRtpRtcpSendImpl(
		Clock *clock,
		RTCRtpRtcpInterface::Configuration &rtcp_config,
		RtcpRttStats* rtt_stats);

public:
	~RTCRtpRtcpSendImpl();


public:
	//////////////////////////////////////////////////////////////////////////
	// RTCRtpRtcpInterface // method
	void IncomingRtcpPacket(const uint8_t* packet, size_t length) override;
	void SetRemoteSSRC(uint32_t ssrc) override;
	void SetLocalSsrc(uint32_t local_ssrc) override;
	int32_t SetSendingStatus(bool enabled) override;
	int32_t SendRTCP(RTCPPacketType packet_type) override;
	int32_t RemoteRTCPStat(std::vector<RTCPReportBlock>* receive_blocks) const override;
	void SetRTCPStatus(RtcpMode method) override;

	absl::optional<TimeDelta> LastRtt() const override;

	uint32_t SSRC() const override { return rtcp_sender_.SSRC(); }
	absl::optional<uint32_t> RtxSsrc() const override { return rtx_ssrc_; }

	bool IsAudioConfigured() const override;

	RtcpMode RTCP() const override;

	std::vector<ReportBlockData> GetLatestReportBlockData() const override;
	absl::optional<RTCSenderReportStats> GetSenderReportStats() const override;
	absl::optional<RTCNonSenderRttStats> GetNonSenderRttStats() const override;

	bool SupportsPadding() const override;
	bool SupportsRtxPayloadPadding() const override { return rtx_ & kRtxRedundantPayloads; }

	bool TrySendPacket(std::unique_ptr<RtpPacketToSend> packet, const PacedPacketInfo& pacing_info) override;

	void OnBatchComplete() override;
	std::vector<std::unique_ptr<RtpPacketToSend>> GeneratePadding(
		size_t target_size_bytes) override;
	void OnAbortedRetransmissions(
		rtc::ArrayView<const uint16_t> sequence_numbers) override;

	//////////////////////////////////////////////////////////////////////////
	// RTCPReceiver::ModuleRtpRtcp methods
	void SetTmmbn(std::vector<rtcp::TmmbItem> bounding_set) override;
	void OnRequestSendReport() override;
	void OnReceivedNack(const std::vector<uint16_t>& nack_sequence_numbers) override;
	void OnReceivedRtcpReportBlocks(const ReportBlockList& report_blocks) override;

	//////////////////////////////////////////////////////////////////////////
	// RtcpFeedbackSenderInterface method
	void SendCombinedRtcpPacket(
		std::vector<std::unique_ptr<rtcp::RtcpPacket>> rtcp_packets) override;
	void SetRemb(int64_t bitrate_bps, std::vector<uint32_t> ssrcs) override;
	void UnsetRemb() override;
	void SetSendingMediaStatus(bool sending) override;

public:
	void SetVideoSendStreamConfig(const RTCVideoSendStream::Config* const config);
	void SetAudioSendStreamConfig(const RTCAudioSendStream::Config* const config);

	void SetLastRtpTime(uint32_t rtp_timestamp, int64_t capture_time_ms, int8_t payload_type = -1);
	void EnqueuePacket(std::unique_ptr<RtpPacketToSend> packet);
	RtpSendRates GetSendRates() const;

	void OnPacketFeedbackVector(
		std::vector<StreamFeedbackObserver::StreamPacketInfo>& packet_feedback_vector);

	void set_rtt_ms(int64_t rtt_ms);
	int64_t rtt_ms() const;

	absl::optional<TimeDelta> PeriodicUpdate();

	void CheckTMMBR();

	static std::unique_ptr<RtpPacketToSend> RtpPacketReceivedToSend(
		const RtpPacketReceived *received_packet, RtpHeaderExtensionMap& header_extension_map, bool isVideo = true);

private:
	struct RTCRtpSenderContext {
		explicit RTCRtpSenderContext(TaskQueueBase& worker_queue,
			const RTCRtpRtcpInterface::Configuration& config);

		RtpPacketHistory packet_history;
		SequenceChecker sequencing_checker;

		PacketSequencer sequencer RTC_GUARDED_BY(sequencing_checker);
		RTCRtpSenderEgress packet_sender;
		RTCRtpSenderEgress::RTCNonPacedPacketSender non_paced_sender;
		RTCRTPSender packet_generator;
	};

private:
	void SetRtxPayloadType(int payload_type,
		int associated_payload_type);
	RTCRtcpSender::FeedbackState GetFeedbackState();

	void MaybeSendRtcp() RTC_RUN_ON(worker_queue_);
	void MaybeSendRtcpAtOrAfterTimestamp(Timestamp execution_time)
		RTC_RUN_ON(worker_queue_);

	void ScheduleRtcpSendEvaluation(TimeDelta duration);

	// Schedules a call to MaybeSendRtcpAtOrAfterTimestamp delayed by `duration`.
	void ScheduleMaybeSendRtcpAtOrAfterTimestamp(Timestamp execution_time,
		TimeDelta duration);

private:
	TaskQueueBase* const worker_queue_;
	RTC_NO_UNIQUE_ADDRESS SequenceChecker rtcp_thread_checker_;
	Clock* const clock_;

	std::unique_ptr<RTCRtpSenderContext> rtp_sender_;
	RTCRtcpSender rtcp_sender_;
	RTCRtcpReceiver rtcp_receiver_;
	RtcpRttStats* const rtt_stats_;

	const uint32_t ssrc_;
	absl::optional<uint32_t> rtx_ssrc_;

	bool is_video_;

	RepeatingTaskHandle rtt_update_task_ RTC_GUARDED_BY(worker_queue_);

	mutable Mutex mutex_rtt_;
	int64_t rtt_ms_ RTC_GUARDED_BY(mutex_rtt_);

	int rtx_;

	RTC_NO_UNIQUE_ADDRESS ScopedTaskSafety task_safety_;

};

}


#endif // __RTC_RTP_RTCP_SEND_IMPL_H__