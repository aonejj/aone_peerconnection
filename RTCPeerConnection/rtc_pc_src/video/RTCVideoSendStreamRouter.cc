//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#include "../rtc_pc_src/_deprecate_defines.h"		// deprecate

#include <memory>

#include "rtc_base/logging.h"
#include "rtc_base/location.h"
#include "rtc_base/time_utils.h"
#include "api/rtp_headers.h"

#include "modules/rtp_rtcp/source/rtcp_packet/transport_feedback.h"
#include "modules/rtp_rtcp/source/byte_io.h"
#include "modules/rtp_rtcp/include/rtp_packet_sender.h"

#include "../modules/rtp_rtcp/source/RTCRtpRtcpInterface.h"
#include "../modules/pacing/RTCPacketRouter.h"

#include "RTCVideoPacketRouteFeeder.h"
#include "RTCVideoSendStreamRouter.h"


namespace webrtc {

static const int64_t kKeyFrameRequestInterval = 1000;
const int64_t kRtpRtcpMaxIdleTimeProcessMs = 5;
constexpr TimeDelta kRttUpdateInterval = TimeDelta::Millis(1000);

RTCRtpRtcpInterface::Configuration CreateRTCPConfigureation(
	Clock *clock, 
	const RTCVideoSendStream::Config *config, 
	RTCVideoSendStreamRouter *thiz,
	RtpPacketSender* rtp_packet_sender) {

	RTCRtpRtcpInterface::Configuration configuration;

	configuration.clock = clock;
	configuration.audio = false;
	configuration.receiver_only = false;
	configuration.outgoing_transport = config->send_transport;
	configuration.local_media_ssrc = config->rtp.ssrcs[0];
	configuration.rtx_send_ssrc = config->rtp.GetRtxSsrcAssociatedWithMediaSsrc(config->rtp.ssrcs[0]);
	configuration.bandwidth_callback = dynamic_cast<RtcpBandwidthObserver*>(thiz);
	configuration.intra_frame_callback = dynamic_cast<RtcpIntraFrameObserver*>(thiz);
	configuration.rtcp_loss_notification_observer = dynamic_cast<RtcpLossNotificationObserver*>(thiz);
	configuration.network_state_estimate_observer = dynamic_cast<NetworkStateEstimateObserver*>(thiz);
	configuration.transport_feedback_callback = dynamic_cast<TransportFeedbackObserver*>(thiz);
	configuration.paced_sender = rtp_packet_sender;

	return configuration;
}

RTCVideoSendStreamRouter::RTCVideoSendStreamRouter(Clock* clock,
												   const RTCVideoSendStream::Config* config,
												   RTCVideoPacketRouteFeeder *feeder,
												   RTCRtpTransportControllerSendInterface* rtp_transport,
												   RtcpRttStats* rtt_stats)
	: clock_(clock),
	config_(config),
	feeder_(feeder),
	weak_ptr_factory_(this),
	keyframe_request_ms_(0ll),
	configuration_(CreateRTCPConfigureation(clock, config, this, rtp_transport->packet_sender())),
	rtp_rtcp_sender_(RTCRtpRtcpSendImpl::Create(clock, configuration_,rtt_stats)),
	rtp_transport_(rtp_transport),
	bandwidth_observer_(rtp_transport->GetBandwidthObserver()),
	feedback_observer_(rtp_transport->transport_feedback_observer()),
	network_state_estimate_observer_(rtp_transport_->network_state_estimate_observer()),
	sending_media_(false) {

	weak_ptr_ = weak_ptr_factory_.GetWeakPtr();
	header_extension_map_ = RtpHeaderExtensionMap(config_->rtp.extensions);

	rtp_rtcp_sender_->SetVideoSendStreamConfig(config_);
	rtp_rtcp_sender_->SetRTCPStatus(RtcpMode::kCompound);
	rtp_rtcp_sender_->SetSendingMediaStatus(true);
	rtp_transport_->packet_router()->AddSendRtpModule(rtp_rtcp_sender_.get(), true);

	feeder_->SetSink(this);
	rtp_transport_->GetStreamFeedbackProvider()->RegisterStreamFeedbackObserver(config_->rtp.ssrcs, this);

}

RTCVideoSendStreamRouter::~RTCVideoSendStreamRouter() {
	rtp_rtcp_sender_->SetSendingMediaStatus(false);

	rtp_transport_->packet_router()->RemoveSendRtpModule(rtp_rtcp_sender_.get());
	rtp_transport_->packet_sender()->RemovePacketsForSsrc(rtp_rtcp_sender_->SSRC());
	if(rtp_rtcp_sender_->RtxSsrc().has_value()) {
		rtp_transport_->packet_sender()->RemovePacketsForSsrc(*(rtp_rtcp_sender_->RtxSsrc()));
	}

	rtp_transport_->GetStreamFeedbackProvider()->DeRegisterStreamFeedbackObserver(this);
}

void RTCVideoSendStreamRouter::DeliverRtcp(const uint8_t* packet, size_t length) {
	rtp_rtcp_sender_->IncomingRtcpPacket(packet, length);
}

void RTCVideoSendStreamRouter::Start() {
	{
		MutexLock lock(&send_mutex_);
		if (sending_media_ == true) {
			return;
		}
		sending_media_ = true;
	}
	feeder_->Start();
	rtp_rtcp_sender_->SetSendingStatus(true);
}

void RTCVideoSendStreamRouter::Stop() {
	{
		MutexLock lock(&send_mutex_);
		if (sending_media_ == false) {
			return;
		}
		sending_media_ = false;
	}

	rtp_rtcp_sender_->SetSendingStatus(false);
	feeder_->Stop();
}

void RTCVideoSendStreamRouter::OnRoutingPacket(
	std::unique_ptr<RtpPacketReceived> packet) {

	int64_t now_ms = clock_->TimeInMilliseconds();

	std::unique_ptr<RtpPacketToSend> routing_packet = RTCRtpRtcpSendImpl::RtpPacketReceivedToSend(
		packet.get(), header_extension_map_);

	if (packet->recovered()) {
		routing_packet->SetPayloadType(config_->rtp.payload_type);
	}
	else {
		if (packet->get_first_packet_in_frame()) {
			RTPHeader hdr;
			packet->GetHeader(&hdr);
			rtp_rtcp_sender_->SetLastRtpTime(hdr.timestamp, routing_packet->capture_time_ms(), hdr.payloadType);
		}
	}

	routing_packet->set_capture_time_ms(now_ms);
	routing_packet->set_allow_retransmission(true);

	{
		MutexLock lock(&send_mutex_);
		if (!sending_media_) {
			return;
		}

		rtp_rtcp_sender_->EnqueuePacket(std::move(routing_packet));
	}
}

void RTCVideoSendStreamRouter::PeriodicUpdate() {
//	RTC_DCHECK_RUN_ON(&worker_queue_);
	// nothing
}

// bool RTCVideoSendStreamRouter::StorePackets() const {
// 	return rtp_rtcp_sender_->StorePackets();
// }

//////////////////////////////////////////////////////////////////////////
// RtcpBandwidthObserver methods
void RTCVideoSendStreamRouter::OnReceivedEstimatedBitrate(uint32_t bitrate) {
	if (bandwidth_observer_ != nullptr) {
		bandwidth_observer_->OnReceivedEstimatedBitrate(bitrate);
	}
}

void RTCVideoSendStreamRouter::OnReceivedRtcpReceiverReport(const ReportBlockList& report_blocks,
															int64_t rtt, int64_t now_ms) {
	if (bandwidth_observer_ != nullptr) {
		bandwidth_observer_->OnReceivedRtcpReceiverReport(report_blocks, rtt, now_ms);
	}
}

//////////////////////////////////////////////////////////////////////////
// RtcpIntraFrameObserver
void RTCVideoSendStreamRouter::OnReceivedIntraFrameRequest(uint32_t ssrc) {
	if (config_->rtp.ssrcs[0] != ssrc) {
		return;
	}

	if (keyframe_request_ms_ == 0ll) {
		feeder_->GenerateKeyFrame();
		keyframe_request_ms_ = clock_->TimeInMilliseconds();
	}
	else {
		int64_t nowMs = clock_->TimeInMilliseconds();
		if ((nowMs - keyframe_request_ms_) > kKeyFrameRequestInterval) {
			feeder_->GenerateKeyFrame();
			keyframe_request_ms_ = nowMs;
		}
	}
}

//////////////////////////////////////////////////////////////////////////
// RtcpLossNotificationObserver
void RTCVideoSendStreamRouter::OnReceivedLossNotification(uint32_t ssrc,
														  uint16_t seq_num_of_last_decodable,
														  uint16_t seq_num_of_last_received,
														  bool decodability_flag) {
}

//////////////////////////////////////////////////////////////////////////
// NetworkStateEstimateObserver
void RTCVideoSendStreamRouter::OnRemoteNetworkEstimate(NetworkStateEstimate estimate) {
	if (network_state_estimate_observer_ != nullptr) {
		network_state_estimate_observer_->OnRemoteNetworkEstimate(estimate);
	}
}

//////////////////////////////////////////////////////////////////////////
// TransportFeedbackObserver
void RTCVideoSendStreamRouter::OnAddPacket(const RtpPacketSendInfo& packet_info) {
	if (feedback_observer_ != nullptr) {
		feedback_observer_->OnAddPacket(packet_info);
	}
}

void RTCVideoSendStreamRouter::OnTransportFeedback(const rtcp::TransportFeedback& feedback) {
	if (feedback_observer_ != nullptr) {
		feedback_observer_->OnTransportFeedback(feedback);
	}
}


//////////////////////////////////////////////////////////////////////////
// StreamFeedbackObserver
void RTCVideoSendStreamRouter::OnPacketFeedbackVector(
	std::vector<StreamPacketInfo> packet_feedback_vector) {
	rtp_rtcp_sender_->OnPacketFeedbackVector(packet_feedback_vector);
}

}	// namespace webrtc