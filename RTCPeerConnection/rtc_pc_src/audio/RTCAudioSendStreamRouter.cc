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
#include "modules/congestion_controller/rtp/transport_feedback_demuxer.h"

#include "../modules/rtp_rtcp/source/RTCRtpRtcpInterface.h"
#include "../modules/pacing/RTCPacketRouter.h"
#include "RTCAudioPacketRouteFeeder.h"
#include "RTCAudioSendStreamRouter.h"



namespace webrtc {

const int64_t kRtpRtcpMaxIdleTimeProcessMs = 5;
constexpr TimeDelta kRttUpdateInterval = TimeDelta::Millis(1000);


RTCRtpRtcpInterface::Configuration CreateAudioRTCPConfiguration(
	Clock *clock,
	const RTCAudioSendStream::Config *config,
	RTCAudioSendStreamRouter *thiz,
	RtpPacketSender* rtp_packet_sender) {

	RTCRtpRtcpInterface::Configuration configuration;

		configuration.clock = clock;
	configuration.audio = true;
	configuration.receiver_only = false;
	configuration.outgoing_transport = config->send_transport;
	configuration.local_media_ssrc = config->rtp.ssrc;
	configuration.bandwidth_callback = dynamic_cast<RtcpBandwidthObserver*>(thiz);
	configuration.transport_feedback_callback = dynamic_cast<TransportFeedbackObserver*>(thiz);
	configuration.paced_sender = rtp_packet_sender;

	return configuration;
}

RTCAudioSendStreamRouter::RTCAudioSendStreamRouter(Clock* clock,
	const RTCAudioSendStream::Config* config,
	RTCAudioPacketRouteFeeder *feeder,
	RTCRtpTransportControllerSendInterface* transport,
	RtcpRttStats* rtt_stats)
	:clock_(clock),
	config_(config),
	bandwidth_observer_(transport->GetBandwidthObserver()),
	feedback_observer_(transport->transport_feedback_observer()),
	feeder_(feeder), 
	weak_ptr_factory_(this),
	configuration_(CreateAudioRTCPConfiguration(clock, config, this, transport->packet_sender())),
	rtp_rtcp_sender_(RTCRtpRtcpSendImpl::Create(clock, configuration_, rtt_stats)),
	sending_media_(false),
	rtp_transport_(transport) {

	weak_ptr_ = weak_ptr_factory_.GetWeakPtr();
	header_extension_map_ = RtpHeaderExtensionMap(config_->rtp.extensions);

	rtp_rtcp_sender_->SetAudioSendStreamConfig(config_);
	rtp_rtcp_sender_->SetRTCPStatus(RtcpMode::kCompound);
	rtp_rtcp_sender_->SetSendingMediaStatus(true);
	rtp_transport_->packet_router()->AddSendRtpModule(rtp_rtcp_sender_.get(), false);
	feeder_->SetSink(this);
}

RTCAudioSendStreamRouter::~RTCAudioSendStreamRouter() {
	rtp_rtcp_sender_->SetSendingMediaStatus(false);
	rtp_transport_->packet_router()->RemoveSendRtpModule(rtp_rtcp_sender_.get());
	rtp_transport_->packet_sender()->RemovePacketsForSsrc(rtp_rtcp_sender_->SSRC());
}


int32_t RTCAudioSendStreamRouter::SendRTCP(RTCPPacketType packet_type) {
	return rtp_rtcp_sender_->SendRTCP(packet_type);
}

void RTCAudioSendStreamRouter::DeliverRtcp(const uint8_t* packet, size_t length) {
	// Runs on a network thread.
//	RTC_DCHECK(!worker_queue_.IsCurrent());
	rtp_rtcp_sender_->IncomingRtcpPacket(packet, length);
}

void RTCAudioSendStreamRouter::Start() {
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

void RTCAudioSendStreamRouter::Stop() {
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

int64_t RTCAudioSendStreamRouter::GetRTT() const {

#if 1
	std::vector<ReportBlockData> report_blocks =
		rtp_rtcp_sender_->GetLatestReportBlockData();
	if (report_blocks.empty()) {
		return 0;
	}

	// We don't know in advance the remote ssrc used by the other end's receiver
	// reports, so use the first report block for the RTT.
	return report_blocks.front().last_rtt_ms();
#else
	std::vector<RTCPReportBlock> report_blocks;
	rtp_rtcp_sender_->RemoteRTCPStat(&report_blocks);

	if (report_blocks.empty()) {
		return 0;
	}

	// We don't know in advance the remote ssrc used by the other end's receiver
	// reports, so use the SSRC of the first report block for calculating the RTT.

	absl::optional<TimeDelta> rtt = rtp_rtcp_sender_->LastRtt();
	if (!rtt.has_value()) {
		// Waiting for valid rtt.
		return 0;
	}

	return *rtt;
#endif
}

void RTCAudioSendStreamRouter::OnRoutingPacket(std::unique_ptr<RtpPacketReceived> packet) {
	
	int64_t now_ms = clock_->TimeInMilliseconds();
	
	std::unique_ptr<RtpPacketToSend> routing_packet = RTCRtpRtcpSendImpl::RtpPacketReceivedToSend(
		packet.get(), header_extension_map_, false);
	if (packet->get_first_packet_in_frame()) {
		RTPHeader hdr;
		packet->GetHeader(&hdr);
		rtp_rtcp_sender_->SetLastRtpTime(hdr.timestamp, routing_packet->capture_time_ms(), hdr.payloadType);
	}

	{
		MutexLock lock(&send_mutex_);
		if (!sending_media_) {
			return;
		}

		rtp_rtcp_sender_->EnqueuePacket(std::move(routing_packet));
	}
}

//////////////////////////////////////////////////////////////////////////
// RtcpBandwidthObserver methods
void RTCAudioSendStreamRouter::OnReceivedEstimatedBitrate(uint32_t bitrate) {
	if (bandwidth_observer_ != nullptr) {
		bandwidth_observer_->OnReceivedEstimatedBitrate(bitrate);
	}
}

void RTCAudioSendStreamRouter::OnReceivedRtcpReceiverReport(const ReportBlockList& report_blocks,
	int64_t rtt, int64_t now_ms) {
	if (bandwidth_observer_ != nullptr) {
		if (!report_blocks.empty() || rtt != 0)
			bandwidth_observer_->OnReceivedRtcpReceiverReport(report_blocks, rtt, now_ms);
	}
}

/////////////////////////////////////////////////////////////////////////
// TransportFeedbackObserver
void RTCAudioSendStreamRouter::OnAddPacket(const RtpPacketSendInfo& packet_info) {
	if (feedback_observer_ != nullptr) {
		feedback_observer_->OnAddPacket(packet_info);
	}
}

void RTCAudioSendStreamRouter::OnTransportFeedback(const rtcp::TransportFeedback& feedback) {
	if (feedback_observer_ != nullptr) {
		feedback_observer_->OnTransportFeedback(feedback);
	}
}

// StreamFeedbackObserver
void RTCAudioSendStreamRouter::OnPacketFeedbackVector(std::vector<StreamPacketInfo> packet_feedback_vector) {
}

}