//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#include "rtc_base/logging.h"
#include "rtc_base/checks.h"
#include "rtc_base/time_utils.h"

#include "api/rtp_headers.h"

#include "modules/rtp_rtcp/source/rtp_packet_received.h"
#include "modules/rtp_rtcp/source/rtp_header_extensions.h"

#include "../rtc_pc_src/_deprecate_defines.h"		// deprecate

#include "RTCChannelRtpPacketReceiver.h"

namespace webrtc {

constexpr int kAbsSendTimeInterArrivalUpshift = 8;
static const double kTimestampToMs = 1000.0 / static_cast<double>(1 << RTPHeaderExtension::kAbsSendTimeFraction + kAbsSendTimeInterArrivalUpshift);

namespace voe {

std::unique_ptr<RTCRtpRtcpRecvImpl>  CreateRtpRtcpModule(
	Clock* clock,
	ReceiveStatistics* receive_statistics,
	Transport* outgoing_transport,
	const RTCAudioReceiveStream::Config &config) {

	RTCRtpRtcpInterface::Configuration configuration;
	configuration.clock = clock;
	configuration.audio = true;
	configuration.receiver_only = true;
	configuration.receive_statistics = receive_statistics;
	configuration.outgoing_transport = outgoing_transport;
	configuration.local_media_ssrc = config.rtp.local_ssrc;

	std::unique_ptr<RTCRtpRtcpRecvImpl> rtp_rtcp =
		RTCRtpRtcpRecvImpl::Create(configuration);
	rtp_rtcp->SetRTCPStatus(RtcpMode::kCompound);

	return rtp_rtcp;
}

RTCChannelRtpPacketReceiver::RTCChannelRtpPacketReceiver(Clock* clock, const RTCAudioReceiveStream::Config &config)
	: clock_(clock),
	config_(config),
	ntp_estimator_(clock),
	local_ssrc_(config.rtp.local_ssrc),
	remote_ssrc_(config.rtp.remote_ssrc),
	rtp_receive_statistics_(ReceiveStatistics::Create(clock_)),
	rtp_rtcp_(CreateRtpRtcpModule(clock, rtp_receive_statistics_.get(),
								  config.rtcp_send_transport, config)),
	sink_(NULL) {
	rtp_rtcp_->SetRTCPStatus(RtcpMode::kCompound);
	rtp_rtcp_->SetRemoteSSRC(config.rtp.remote_ssrc);
}

RTCChannelRtpPacketReceiver::~RTCChannelRtpPacketReceiver() {

}

void RTCChannelRtpPacketReceiver::SetSink(RTCAudioRtpPacketListenSinkInterface *sink) {
	RTC_DCHECK(worker_thread_checker_.IsCurrent());
	MutexLock lock(&callback_mutex_);
	sink_ = sink;
}

void RTCChannelRtpPacketReceiver::SetReceiveCodecs(
	const std::map<int, SdpAudioFormat>& codecs) {
	RTC_DCHECK(worker_thread_checker_.IsCurrent());
	for (const auto& kv : codecs) {
		RTC_DCHECK_GE(kv.second.clockrate_hz, 1000);
		payload_type_frequencies_[kv.first] = kv.second.clockrate_hz;
	}
}

absl::optional<std::pair<int32_t, SdpAudioFormat>>
	RTCChannelRtpPacketReceiver::GetReceiveCodec() const {
	RTC_DCHECK(worker_thread_checker_.IsCurrent());

	return absl::nullopt;
}

void RTCChannelRtpPacketReceiver::OnRtpPacket(const RtpPacketReceived& packet) {
	int64_t debug_nowMs = rtc::TimeMillis();	// kimi

	const auto& it = payload_type_frequencies_.find(packet.PayloadType());
	if (it == payload_type_frequencies_.end())
		return;

	(const_cast<RtpPacketReceived*>(&packet))->set_payload_type_frequency(it->second);
	(const_cast<RtpPacketReceived*>(&packet))->set_first_packet_in_frame();

	{
		MutexLock lock(&callback_mutex_);
		if (sink_) {
			sink_->OnRtpPacket(packet);
		}
	}

	rtp_receive_statistics_->OnRtpPacket(packet);

	// for test
	uint32_t abs_time = 0;
	packet.GetExtension<AbsoluteSendTime>(&abs_time);
	uint32_t timestamp = abs_time << kAbsSendTimeInterArrivalUpshift;
	int64_t send_time_ms = static_cast<int64_t>(timestamp)* kTimestampToMs;
	int64_t arrive_time_ms = packet.arrival_time_ms();

	_debug_last_recvMs = debug_nowMs;
	_debug_abs_ms = send_time_ms;
	_debug_arrive_time_ms = arrive_time_ms;
}

void RTCChannelRtpPacketReceiver::ReceivedRTCPPacket(const uint8_t* data, size_t length) {
	rtp_rtcp_->IncomingRtcpPacket(data, length);

	absl::optional<TimeDelta> rtt = rtp_rtcp_->LastRtt();
	if (!rtt.has_value()) {
		// Waiting for valid RTT.
		return;
	}

	absl::optional<RTCRtpRtcpInterface::RTCSenderReportStats> last_sr =
		rtp_rtcp_->GetSenderReportStats();
	if (!last_sr.has_value()) {
		// Waiting for RTCP.
		return;
	}

	// Don't use old SRs to estimate time.
	ntp_estimator_.UpdateRtcpTimestamp(*rtt, last_sr->last_remote_timestamp,
		last_sr->last_remote_rtp_timestamp);
}

void RTCChannelRtpPacketReceiver::Start() {
	MutexLock lock(&callback_mutex_);
	if (sink_) {
		sink_->OnStart();
	}
}

void RTCChannelRtpPacketReceiver::Stop() {
	MutexLock lock(&callback_mutex_);
	if (sink_) {
		sink_->OnStop();
	}
}

void RTCChannelRtpPacketReceiver::RegisterReceiverCongestionControlObjects(
	RTCPacketRouter* packet_router) {
	RTC_DCHECK(packet_router);
	RTC_DCHECK(!packet_router_);
	packet_router->AddReceiveRtpModule(rtp_rtcp_.get(), false);
	packet_router_ = packet_router;
}

void RTCChannelRtpPacketReceiver::ResetReceiverCongestionControlObjects() {
	RTC_DCHECK(packet_router_);
	packet_router_->RemoveReceiveRtpModule(rtp_rtcp_.get());
	packet_router_ = nullptr;
}

std::unique_ptr<RTCChannelRtpPacketReceiverInterface> CreateRTCChannelRtpPacketReceiver(Clock* clock,
	const RTCAudioReceiveStream::Config &config) {

	return std::make_unique<RTCChannelRtpPacketReceiver>(clock, config);
}

}

}	// namespace webrtc