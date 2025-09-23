//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_VIDEO_RTP_VIDEO_PACKET_STREAM_RECEIVER_H__
#define __RTC_VIDEO_RTP_VIDEO_PACKET_STREAM_RECEIVER_H__

#include <map>
#include <memory>
#include <vector>

#include "../rtc_pc_src/_deprecate_defines.h"		// deprecate

#include "absl/types/optional.h"

#include "call/rtp_packet_sink_interface.h"

#include "modules/utility/include/process_thread.h"
#include "modules/rtp_rtcp/include/rtp_rtcp_defines.h"
#include "modules/rtp_rtcp/include/remote_ntp_time_estimator.h"
#include "modules/rtp_rtcp/include/receive_statistics.h"
#include "modules/rtp_rtcp/source/rtp_packet_received.h"
#include "modules/rtp_rtcp/source/video_rtp_depacketizer.h"

#ifdef __ROUTING_PACKET_PATH_USING_NETWORK_THREAD__
#include "../rtc_pc_src/modules/video_coding/RTCNackModule2.h"
#else
#include "modules/video_coding/nack_module2.h"
#endif

#include "rtc_base/thread_checker.h"
#include "rtc_base/synchronization/mutex.h"
#include "system_wrappers/include/clock.h"

#include "../call/RTCVideoReceiveStream.h"
#include "../modules/rtp_rtcp/source/RTCRtpRtcpRecvImpl.h"
#include "../call/RTCRtpTransportControllerSendInterface.h"

#include "../rtc_pc_src/_deprecate_defines.h"

namespace webrtc {

class RTCVideoRtpPacketListenSinkInterface;
class RTCFeedbackPacketRouter;

class RTCRtpVideoPacketStreamReceiver : public LossNotificationSender,
										public KeyFrameRequestSender,
										public RtpPacketSinkInterface {
public:
	RTCRtpVideoPacketStreamReceiver(
		TaskQueueBase* current_queue,
		Clock* clock,
		RTCRtpTransportControllerSendInterface* rtp_transport,
		Transport* transport,
		RtcpRttStats* rtt_stats,
		const RTCVideoReceiveStream::Config* config,
		ReceiveStatistics* rtp_receive_statistics,
		NackSender* nack_sender,
		KeyFrameRequestSender* keyframe_request_sender);

	~RTCRtpVideoPacketStreamReceiver();

public:
	void SetSink(RTCVideoRtpPacketListenSinkInterface *sink);
	void OnRtpPacket(const RtpPacketReceived& packet) override;
	void OnRtxRtpPacket(const RtpPacketReceived& packet) override;

	void StartReceive();
	void StopReceive();

	bool DeliverRtcp(const uint8_t* rtcp_packet, size_t rtcp_packet_length);

	// Implements LossNotificationSender.
	void SendLossNotification(uint16_t last_decoded_seq_num,
		uint16_t last_received_seq_num,
		bool decodability_flag,
		bool buffering_allowed) override;

	void RequestKeyFrame() override;
	void RequestKeyFramePli() override;

	void AddReceiveCodecInfo(uint8_t payload_type,
							 VideoCodecType video_codec_type,
							 const std::map<std::string, std::string>& codec_params);

	void RequestPacketRetransmit(const std::vector<uint16_t>& sequence_numbers);

	void UpdateRtt(int64_t max_rtt_ms);

private:
	class RTCRtcpFeedbackBuffer : public KeyFrameRequestSender,
							   public NackSender,
							   public LossNotificationSender {
	public:
		RTCRtcpFeedbackBuffer(KeyFrameRequestSender* key_frame_request_sender,
			NackSender* nack_sender,
			LossNotificationSender* loss_notification_sender);

		~RTCRtcpFeedbackBuffer() override = default;

		// KeyFrameRequestSender implementation.
		void RequestKeyFrame() override;
		void RequestKeyFramePli() override;

		// NackSender implementation.
		void SendNack(const std::vector<uint16_t>& sequence_numbers,
			bool buffering_allowed) override;

		// LossNotificationSender implementation.
		void SendLossNotification(uint16_t last_decoded_seq_num,
			uint16_t last_received_seq_num,
			bool decodability_flag,
			bool buffering_allowed) override;

		// Send all RTCP feedback messages buffered thus far.
		void SendBufferedRtcpFeedback();

	private:
		// LNTF-related state.
		struct LossNotificationState {
			LossNotificationState(uint16_t last_decoded_seq_num,
				uint16_t last_received_seq_num,
				bool decodability_flag)
				: last_decoded_seq_num(last_decoded_seq_num),
				last_received_seq_num(last_received_seq_num),
				decodability_flag(decodability_flag) {}

			uint16_t last_decoded_seq_num;
			uint16_t last_received_seq_num;
			bool decodability_flag;
		};

		RTC_NO_UNIQUE_ADDRESS SequenceChecker worker_task_checker_;
		KeyFrameRequestSender* const key_frame_request_sender_;
		NackSender* const nack_sender_;
		LossNotificationSender* const loss_notification_sender_;


#ifdef __ROUTING_PACKET_PATH_USING_NETWORK_THREAD__
		mutable Mutex fb_buffer_mutex_;
		bool request_key_frame_;
		bool request_key_frame_pli_;
		std::vector<uint16_t> nack_sequence_numbers_;
		absl::optional<LossNotificationState> lntf_state_;
#else
		// Key-frame-request-related state.
		bool request_key_frame_ RTC_GUARDED_BY(worker_task_checker_);
		bool request_key_frame_pli_ RTC_GUARDED_BY(worker_task_checker_);
		// NACK-related state.
		std::vector<uint16_t> nack_sequence_numbers_
			RTC_GUARDED_BY(worker_task_checker_);

		absl::optional<LossNotificationState> lntf_state_
			RTC_GUARDED_BY(worker_task_checker_);
#endif
	};

	void ReceivePacket(const RtpPacketReceived& packet);

private:
	Clock* const clock_;
	const RTCVideoReceiveStream::Config& config_;

	RemoteNtpTimeEstimator ntp_estimator_;

	RtpHeaderExtensionMap rtp_header_extensions_;

	RTC_NO_UNIQUE_ADDRESS SequenceChecker worker_task_checker_;
	bool receiving_ RTC_GUARDED_BY(worker_task_checker_);

	ReceiveStatistics* const rtp_receive_statistics_;

	const std::unique_ptr<RTCRtpRtcpRecvImpl> rtp_rtcp_;

	RTCRtcpFeedbackBuffer rtcp_feedback_buffer_;

#ifdef __ROUTING_PACKET_PATH_USING_NETWORK_THREAD__
	const std::unique_ptr<RTCNackModule2> nack_module_;
#else
	const std::unique_ptr<NackModule2> nack_module_;
#endif

	KeyFrameRequestSender* const keyframe_request_sender_;
	
	Mutex callback_mutex_;
	RTCVideoRtpPacketListenSinkInterface *sink_;

#ifdef __ROUTING_PACKET_PATH_USING_NETWORK_THREAD__
	mutable Mutex pt_mutex_;
	std::map<uint8_t, std::unique_ptr<VideoRtpDepacketizer>> payload_type_map_;
	std::map<uint8_t, std::map<std::string, std::string>> pt_codec_params_;

#else
	// Maps payload id to the depacketizer.
	std::map<uint8_t, std::unique_ptr<VideoRtpDepacketizer>> payload_type_map_
		RTC_GUARDED_BY(worker_task_checker_);

	std::map<uint8_t, std::map<std::string, std::string>> pt_codec_params_
		RTC_GUARDED_BY(worker_task_checker_);
#endif

	uint32_t _prev_packet_timestamp;
	RTCRtpTransportControllerSendInterface* rtp_transport_;
};

}	// namespace webrtc

#endif // __RTC_VIDEO_RTP_VIDEO_PACKET_STREAM_RECEIVER_H__