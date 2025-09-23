//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

//#include "../rtc_pc_src/_deprecate_defines.h"

#include "rtc_base/location.h"
#include "rtc_base/logging.h"
#include "rtc_base/checks.h"
#include "rtc_base/time_utils.h"
#include "api/units/time_delta.h"
#include "modules/rtp_rtcp/source/rtp_rtcp_config.h"
#include "modules/rtp_rtcp/source/create_video_rtp_depacketizer.h"
#include "modules/video_coding/packet_buffer.h"
#include "modules/video_coding/h264_sprop_parameter_sets.h"

#include "../api/video/RTCVideoRtpPacketListenSinkInterface.h"
#include "../modules/feedback/RTCFeedbackPacketRouter.h"
#include "../modules/pacing/RTCPacketRouter.h"

#include "RTCRtpVideoPacketStreamReceiver.h"


namespace webrtc {

std::unique_ptr<RTCRtpRtcpRecvImpl>  CreateRtpRtcpModule(
	Clock* clock,
	ReceiveStatistics* receive_statistics,
	Transport* outgoing_transport,
	RtcpRttStats* rtt_stats,
	RtcpPacketTypeCounterObserver* rtcp_packet_type_counter_observer,
	RtcpCnameCallback* rtcp_cname_callback,
	bool non_sender_rtt_measurement,
	uint32_t local_ssrc) {
	RTCRtpRtcpInterface::Configuration configuration;
	configuration.clock = clock;
	configuration.audio = false;
	configuration.receiver_only = true;
	configuration.receive_statistics = receive_statistics;
	configuration.outgoing_transport = outgoing_transport;
	configuration.rtt_stats = rtt_stats;
	configuration.rtcp_packet_type_counter_observer =
		rtcp_packet_type_counter_observer;
	configuration.rtcp_cname_callback = rtcp_cname_callback;
	configuration.local_media_ssrc = local_ssrc;
	configuration.non_sender_rtt_measurement = non_sender_rtt_measurement;

	std::unique_ptr<RTCRtpRtcpRecvImpl> rtp_rtcp =
		RTCRtpRtcpRecvImpl::Create(configuration);
	rtp_rtcp->SetRTCPStatus(RtcpMode::kCompound);

	return rtp_rtcp;
}

#ifdef __ROUTING_PACKET_PATH_USING_NETWORK_THREAD__
std::unique_ptr<RTCNackModule2> MaybeConstructNRTCNackModule(
	TaskQueueBase* current_queue,
	const RTCVideoReceiveStream::Config& config,
	Clock* clock,
	NackSender* nack_sender,
	KeyFrameRequestSender* keyframe_request_sender) {
	if (config.rtp.nack.rtp_history_ms == 0)
		return nullptr;

	return std::make_unique<RTCNackModule2>(current_queue, clock, nack_sender,
		keyframe_request_sender, TimeDelta::Millis(15));
}
#else
std::unique_ptr<NackModule2> MaybeConstructNackModule(
	TaskQueueBase* current_queue,
	const RTCVideoReceiveStream::Config& config,
	Clock* clock,
	NackSender* nack_sender,
	KeyFrameRequestSender* keyframe_request_sender) {
	if (config.rtp.nack.rtp_history_ms == 0)
		return nullptr;

	return std::make_unique<NackModule2>(current_queue, clock, nack_sender,
		keyframe_request_sender, TimeDelta::Millis(15));
}
#endif

RTCRtpVideoPacketStreamReceiver::RTCRtcpFeedbackBuffer::RTCRtcpFeedbackBuffer(
	KeyFrameRequestSender* key_frame_request_sender,
	NackSender* nack_sender,
	LossNotificationSender* loss_notification_sender)
	: key_frame_request_sender_(key_frame_request_sender),
	nack_sender_(nack_sender),
	loss_notification_sender_(loss_notification_sender),
	request_key_frame_(false) {
	RTC_DCHECK(key_frame_request_sender_);
	RTC_DCHECK(nack_sender_);
	RTC_DCHECK(loss_notification_sender_);
}

void RTCRtpVideoPacketStreamReceiver::RTCRtcpFeedbackBuffer::RequestKeyFrame() {
#ifdef __ROUTING_PACKET_PATH_USING_NETWORK_THREAD__
	MutexLock lock(&fb_buffer_mutex_);
#else
	RTC_DCHECK_RUN_ON(&worker_task_checker_);
#endif
	request_key_frame_ = true;
}

void RTCRtpVideoPacketStreamReceiver::RTCRtcpFeedbackBuffer::RequestKeyFramePli() {
#ifdef __ROUTING_PACKET_PATH_USING_NETWORK_THREAD__
	MutexLock lock(&fb_buffer_mutex_);
#else
	RTC_DCHECK_RUN_ON(&worker_task_checker_);
#endif
	request_key_frame_pli_ = true;
}

void RTCRtpVideoPacketStreamReceiver::RTCRtcpFeedbackBuffer::SendNack(
	const std::vector<uint16_t>& sequence_numbers,
	bool buffering_allowed) {

#ifdef __ROUTING_PACKET_PATH_USING_NETWORK_THREAD__
	MutexLock lock(&fb_buffer_mutex_);
#else
	RTC_DCHECK_RUN_ON(&worker_task_checker_);
#endif

	RTC_DCHECK(!sequence_numbers.empty());
	nack_sequence_numbers_.insert(nack_sequence_numbers_.end(),
		sequence_numbers.cbegin(),
		sequence_numbers.cend());
	if (!buffering_allowed) {
		// Note that while *buffering* is not allowed, *batching* is, meaning that
		// previously buffered messages may be sent along with the current message.
		SendBufferedRtcpFeedback();
	}
}

void RTCRtpVideoPacketStreamReceiver::RTCRtcpFeedbackBuffer::SendLossNotification(
	uint16_t last_decoded_seq_num,
	uint16_t last_received_seq_num,
	bool decodability_flag,
	bool buffering_allowed) {

	RTC_DCHECK_RUN_ON(&worker_task_checker_);

	RTC_DCHECK(buffering_allowed);
	RTC_DCHECK(!lntf_state_)
		<< "SendLossNotification() called twice in a row with no call to "
		"SendBufferedRtcpFeedback() in between.";
	lntf_state_ = absl::make_optional<LossNotificationState>(
		last_decoded_seq_num, last_received_seq_num, decodability_flag);
}

void RTCRtpVideoPacketStreamReceiver::RTCRtcpFeedbackBuffer::SendBufferedRtcpFeedback() {
#ifndef __ROUTING_PACKET_PATH_USING_NETWORK_THREAD__
	RTC_DCHECK_RUN_ON(&worker_task_checker_);
#endif

	bool request_key_frame = false;
	bool request_key_frame_pli = false;
	std::vector<uint16_t> nack_sequence_numbers;
	absl::optional<LossNotificationState> lntf_state;

	std::swap(request_key_frame, request_key_frame_);
	std::swap(request_key_frame_pli, request_key_frame_pli_);
	std::swap(nack_sequence_numbers, nack_sequence_numbers_);
	std::swap(lntf_state, lntf_state_);

	if (lntf_state) {
		// If either a NACK or a key frame request is sent, we should buffer
		// the LNTF and wait for them (NACK or key frame request) to trigger
		// the compound feedback message.
		// Otherwise, the LNTF should be sent out immediately.
		const bool buffering_allowed =
			request_key_frame || !nack_sequence_numbers.empty();

		loss_notification_sender_->SendLossNotification(
			lntf_state->last_decoded_seq_num, lntf_state->last_received_seq_num,
			lntf_state->decodability_flag, buffering_allowed);
	}

	if (request_key_frame) {
		key_frame_request_sender_->RequestKeyFrame();
	}
	else if (request_key_frame_pli) {
		key_frame_request_sender_->RequestKeyFramePli();
	}
	else if (!nack_sequence_numbers.empty()) {
		nack_sender_->SendNack(nack_sequence_numbers, true);
	}
}

RTCRtpVideoPacketStreamReceiver::RTCRtpVideoPacketStreamReceiver(
	TaskQueueBase* current_queue,
	Clock* clock,
	RTCRtpTransportControllerSendInterface* rtp_transport,
	Transport* transport,
	RtcpRttStats* rtt_stats,
	const RTCVideoReceiveStream::Config* config,
	ReceiveStatistics* rtp_receive_statistics,
	NackSender* nack_sender,
	KeyFrameRequestSender* keyframe_request_sender) 
	: clock_(clock),
	  config_(*config),
	  ntp_estimator_(clock),
	  rtp_header_extensions_(config_.rtp.extensions),
	  receiving_(false),
	  rtp_receive_statistics_(rtp_receive_statistics),
	  rtp_rtcp_(CreateRtpRtcpModule(
			  clock,
			  rtp_receive_statistics_,
			  transport,
			  rtt_stats, // rtt_stats,
			  nullptr, // rtcp_packet_type_counter_observer,
			  nullptr, // rtcp_cname_callback,
			  config_.rtp.rtcp_xr.receiver_reference_time_report,
			  config_.rtp.local_ssrc)),
      keyframe_request_sender_(keyframe_request_sender),
	  rtcp_feedback_buffer_(this, nack_sender, this),
#ifdef __ROUTING_PACKET_PATH_USING_NETWORK_THREAD__
	  nack_module_(MaybeConstructNRTCNackModule(current_queue,
		  config_,
		  clock_,
		  &rtcp_feedback_buffer_,
		  &rtcp_feedback_buffer_)),
#else
	  nack_module_(MaybeConstructNackModule(current_queue,
		  config_,
		  clock_,
		  &rtcp_feedback_buffer_,
		  &rtcp_feedback_buffer_)),
#endif
	  sink_(NULL),
	  rtp_transport_(rtp_transport) {

	rtp_transport_->packet_router()->AddReceiveRtpModule(rtp_rtcp_.get(), true);

	RTC_DCHECK(config_.rtp.rtcp_mode != RtcpMode::kOff);

	RTC_DCHECK(config_.rtp.local_ssrc != 0);
	RTC_DCHECK(config_.rtp.remote_ssrc != config_.rtp.local_ssrc);

	rtp_rtcp_->SetRTCPStatus(config_.rtp.rtcp_mode);
	rtp_rtcp_->SetRemoteSSRC(config_.rtp.remote_ssrc);

	static const int kMaxPacketAgeToNack = 450;
	const int max_reordering_threshold = (config_.rtp.nack.rtp_history_ms > 0)
		? kMaxPacketAgeToNack
		: kDefaultMaxReorderingThreshold;

	rtp_receive_statistics_->SetMaxReorderingThreshold(config_.rtp.remote_ssrc,
													   max_reordering_threshold);
	if (config_.rtp.rtx_ssrc) {
		rtp_receive_statistics_->SetMaxReorderingThreshold(
			config_.rtp.rtx_ssrc, max_reordering_threshold);
	}

	_prev_packet_timestamp = 0;	// for debugging
}

RTCRtpVideoPacketStreamReceiver::~RTCRtpVideoPacketStreamReceiver() {
	rtp_transport_->packet_router()->RemoveReceiveRtpModule(rtp_rtcp_.get());
}

void RTCRtpVideoPacketStreamReceiver::SetSink(RTCVideoRtpPacketListenSinkInterface *sink) {
	RTC_DCHECK_RUN_ON(&worker_task_checker_);
	MutexLock lock(&callback_mutex_);
	sink_ = sink;
}

void RTCRtpVideoPacketStreamReceiver::OnRtpPacket(const RtpPacketReceived& packet) {
#ifndef __ROUTING_PACKET_PATH_USING_NETWORK_THREAD__
	RTC_DCHECK_RUN_ON(&worker_task_checker_);
#endif

	if (!receiving_) {
		return;
	}
	ReceivePacket(packet);

	if (!packet.recovered()) {
		rtp_receive_statistics_->OnRtpPacket(packet);
	}

	{
		MutexLock lock(&callback_mutex_);
		if (sink_) {
			sink_->OnRtpPacket(packet);
		}
	}
}

void RTCRtpVideoPacketStreamReceiver::OnRtxRtpPacket(const RtpPacketReceived& packet) {
	{
		MutexLock lock(&callback_mutex_);
		if (sink_) {
			sink_->OnRtxRtpPacket(packet);
		}
	}

	rtc::ArrayView<const uint8_t> payload = packet.payload();
	const uint16_t rtp_seq_no = (payload[0] << 8) + payload[1];

	if (nack_module_) {
		nack_module_->OnReceivedPacket(
			rtp_seq_no, false, true);
	}

}

void RTCRtpVideoPacketStreamReceiver::StartReceive() {
	RTC_DCHECK_RUN_ON(&worker_task_checker_);
	receiving_ = true;
	MutexLock lock(&callback_mutex_);
	if (sink_) {
		sink_->OnStart();
	}
}

void RTCRtpVideoPacketStreamReceiver::StopReceive() {
	RTC_DCHECK_RUN_ON(&worker_task_checker_);
	receiving_ = false;
	MutexLock lock(&callback_mutex_);
	if (sink_) {
		sink_->OnStop();
	}
}

bool RTCRtpVideoPacketStreamReceiver::DeliverRtcp(
	const uint8_t* rtcp_packet, size_t rtcp_packet_length) {

	RTC_DCHECK_RUN_ON(&worker_task_checker_);

	if (!receiving_) {
		return false;
	}

	rtp_rtcp_->IncomingRtcpPacket(rtcp_packet, rtcp_packet_length);

	absl::optional<TimeDelta> rtt = rtp_rtcp_->LastRtt();
	if (!rtt.has_value()) {
		// Waiting for valid rtt.
//		RTC_LOG(LS_INFO) << "rtcp_trace RTCRtpVideoPacketStreamReceiver::DeliverRtcp rtt has not value";
		return true;
	}

	RTC_LOG(LS_INFO) << "rtcp_trace_report RTCRtpVideoPacketStreamReceiver::DeliverRtcp rtt " << rtt->ms();

	absl::optional<RTCRtpRtcpInterface::RTCSenderReportStats> last_sr = rtp_rtcp_->GetSenderReportStats();
	if (!last_sr.has_value()) {
		// Waiting for RTCP.
		return true;
	}

	RTC_LOG(LS_INFO) << "rtcp_trace_report RTCRtpVideoPacketStreamReceiver::DeliverRtcp last_sr arrival " << last_sr->last_arrival_timestamp.ToMs() << " remote "
		<< last_sr->last_remote_timestamp.ToMs() << " packet_sent " << last_sr->packets_sent << " byte sent " << last_sr->bytes_sent;

	int64_t time_since_received = clock_->CurrentNtpInMilliseconds() -
		last_sr->last_arrival_timestamp.ToMs();


	// Don't use old SRs to estimate time.
	if (time_since_received <= 1) {

		ntp_estimator_.UpdateRtcpTimestamp(*rtt, last_sr->last_remote_timestamp,
			last_sr->last_remote_rtp_timestamp);
	}

	return true;
}

void RTCRtpVideoPacketStreamReceiver::SendLossNotification(uint16_t last_decoded_seq_num,
	uint16_t last_received_seq_num,
	bool decodability_flag,
	bool buffering_allowed) {
	RTC_DCHECK(config_.rtp.lntf.enabled);
	rtp_rtcp_->SendLossNotification(last_decoded_seq_num, last_received_seq_num,
		decodability_flag, buffering_allowed);
}

void RTCRtpVideoPacketStreamReceiver::RequestKeyFrame() {
	RTC_DCHECK_RUN_ON(&worker_task_checker_);
	rtp_rtcp_->SendFullIntraRequest();
}

void RTCRtpVideoPacketStreamReceiver::RequestKeyFramePli() {
	RTC_DCHECK_RUN_ON(&worker_task_checker_);
	rtp_rtcp_->SendPictureLossIndication();
}

void RTCRtpVideoPacketStreamReceiver::AddReceiveCodecInfo(uint8_t payload_type,
	VideoCodecType video_codec_type,
	const std::map<std::string, std::string>& codec_params) {
#ifdef __ROUTING_PACKET_PATH_USING_NETWORK_THREAD__
	{
		MutexLock lock(&pt_mutex_);
		payload_type_map_.emplace(
			payload_type, CreateVideoRtpDepacketizer(video_codec_type));
		pt_codec_params_.emplace(payload_type, codec_params);
	}
#else
	payload_type_map_.emplace(
		payload_type, CreateVideoRtpDepacketizer(video_codec_type));
	pt_codec_params_.emplace(payload_type, codec_params);
#endif
}

void RTCRtpVideoPacketStreamReceiver::RequestPacketRetransmit(const std::vector<uint16_t>& sequence_numbers) {
	RTC_DCHECK_RUN_ON(&worker_task_checker_);
	rtp_rtcp_->SendNack(sequence_numbers);
}

void RTCRtpVideoPacketStreamReceiver::UpdateRtt(int64_t max_rtt_ms) {
	RTC_DCHECK_RUN_ON(&worker_task_checker_);
	if (nack_module_)
		nack_module_->UpdateRtt(max_rtt_ms);
}

void RTCRtpVideoPacketStreamReceiver::ReceivePacket(const RtpPacketReceived& packet) {
#ifndef __ROUTING_PACKET_PATH_USING_NETWORK_THREAD__
	RTC_DCHECK_RUN_ON(&worker_task_checker_);
#endif
	if (packet.payload_size() == 0) {
		if (nack_module_) {
			nack_module_->OnReceivedPacket(packet.SequenceNumber(), false, false);
		}
		return;
	}

#ifdef __ROUTING_PACKET_PATH_USING_NETWORK_THREAD__
	absl::optional<VideoRtpDepacketizer::ParsedRtpPayload> parsed_payload;
	{
		MutexLock lock(&pt_mutex_);
		const auto type_it = payload_type_map_.find(packet.PayloadType());
		if (type_it == payload_type_map_.end()) {
			return;
		}

		parsed_payload =
			type_it->second->Parse(packet.PayloadBuffer());
		if (parsed_payload == absl::nullopt) {
			RTC_LOG(LS_WARNING) << "Failed parsing payload.";
			return;
		}
	}
#else
	const auto type_it = payload_type_map_.find(packet.PayloadType());
	if (type_it == payload_type_map_.end()) {
		return;
	}

	absl::optional<VideoRtpDepacketizer::ParsedRtpPayload> parsed_payload =
		type_it->second->Parse(packet.PayloadBuffer());
	if (parsed_payload == absl::nullopt) {
		RTC_LOG(LS_WARNING) << "Failed parsing payload.";
		return;
	}
#endif

	const bool is_keyframe =
		parsed_payload->video_header.frame_type == VideoFrameType::kVideoFrameKey;

	if (is_keyframe) {
		(const_cast<RtpPacketReceived*>(&packet))->set_key_frame();
		(const_cast<RtpPacketReceived*>(&packet))->set_can_discontinuous();
		(const_cast<RtpPacketReceived*>(&packet))->set_frame_size(parsed_payload->video_header.width, parsed_payload->video_header.height);
	}

	if (parsed_payload->video_header.is_first_packet_in_frame) {
		(const_cast<RtpPacketReceived*>(&packet))->set_first_packet_in_frame();
	}

	if (nack_module_) {
		nack_module_->OnReceivedPacket(
			packet.SequenceNumber(), is_keyframe, packet.recovered());
	}
}

}	// namespace webrtc