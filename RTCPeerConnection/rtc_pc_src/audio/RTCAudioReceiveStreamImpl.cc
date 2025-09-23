//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /audio/audio_receive_stream.cc
//
//////////////////////////////////////////////////////////////////////////

#include <string>
#include <utility>

#include "absl/memory/memory.h"
#include "api/array_view.h"

#include "call/rtp_config.h"
#include "call/rtp_stream_receiver_controller_interface.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "rtc_base/strings/string_builder.h"
#include "rtc_base/time_utils.h"

#include "../api/audio/RTCAudioRtpPacketListenSinkInterface.h"
#include "../modules/pacing/RTCPacketRouter.h"
#include "../call/RTCRtpTransportControllerSendInterface.h"
#include "RTCChannelRtpPacketReceiverInterface.h"
#include "RTCAudioReceiveStreamImpl.h"


namespace webrtc {

std::string RTCAudioReceiveStream::Config::Rtp::ToString() const {
	char ss_buf[1024];
	rtc::SimpleStringBuilder ss(ss_buf);
	ss << "{remote_ssrc: " << remote_ssrc;
	ss << ", local_ssrc: " << local_ssrc;
	ss << ", transport_cc: " << (transport_cc ? "on" : "off");
	ss << ", nack: " << nack.ToString();
	ss << ", extensions: [";
	for (size_t i = 0; i < extensions.size(); ++i) {
		ss << extensions[i].ToString();
		if (i != extensions.size() - 1) {
			ss << ", ";
		}
	}
	ss << ']';
	ss << '}';
	return ss.str();
}

std::string RTCAudioReceiveStream::Config::ToString() const {
	char ss_buf[1024];
	rtc::SimpleStringBuilder ss(ss_buf);
	ss << "{rtp: " << rtp.ToString();
	ss << ", rtcp_send_transport: "
		<< (rtcp_send_transport ? "(Transport)" : "null");
	ss << '}';
	return ss.str();
}

std::unique_ptr<voe::RTCChannelRtpPacketReceiverInterface> CreateRTCChannelRtpPacketReceiver(
	Clock* clock,
	const RTCAudioReceiveStream::Config& config) {
	return voe::CreateRTCChannelRtpPacketReceiver(clock, config);
}


RTCAudioReceiveStreamImpl::RTCAudioReceiveStreamImpl(
	Clock* clock,
	RTCRtpTransportControllerSendInterface* rtp_transport,
	RtpStreamReceiverControllerInterface* receiver_controller, 
	const RTCAudioReceiveStream::Config& config,
	RTCAudioRtpPacketListenSinkInterface *audio_rtp_packet_listen_sink)
	: RTCAudioReceiveStreamImpl(clock,
								rtp_transport,
								receiver_controller,
								config,
								audio_rtp_packet_listen_sink,
								CreateRTCChannelRtpPacketReceiver(clock, config)) {
}

RTCAudioReceiveStreamImpl::RTCAudioReceiveStreamImpl(
	Clock* clock,
	RTCRtpTransportControllerSendInterface* rtp_transport,
	RtpStreamReceiverControllerInterface* receiver_controller,
	const RTCAudioReceiveStream::Config& config,
	RTCAudioRtpPacketListenSinkInterface *audio_rtp_packet_listen_sink,
	std::unique_ptr<voe::RTCChannelRtpPacketReceiverInterface> channel_rtp_packet_receive)
	: source_tracker_(clock),
	  audio_rtp_packet_listen_sink_(audio_rtp_packet_listen_sink),
	  channel_rtp_packet_receive_(std::move(channel_rtp_packet_receive)) {

	RTC_CHECK(receiver_controller);

	channel_rtp_packet_receive_->RegisterReceiverCongestionControlObjects(rtp_transport->packet_router());
	channel_rtp_packet_receive_->SetSink(audio_rtp_packet_listen_sink_);

	rtp_stream_receiver_ = receiver_controller->CreateReceiver(
		config.rtp.remote_ssrc, channel_rtp_packet_receive_.get());

	ConfigureStream(this, config, true);	
}

RTCAudioReceiveStreamImpl::~RTCAudioReceiveStreamImpl() {
	RTC_DCHECK_RUN_ON(&worker_thread_checker_);

	Stop();
	channel_rtp_packet_receive_->ResetReceiverCongestionControlObjects();
}

void RTCAudioReceiveStreamImpl::Reconfigure(
	const RTCAudioReceiveStream::Config& config) {
	RTC_DCHECK(worker_thread_checker_.IsCurrent());
	config_ = config;		
	ConfigureStream(this, config, false);
}

void RTCAudioReceiveStreamImpl::Start() {
	RTC_DCHECK_RUN_ON(&worker_thread_checker_);
	if (playing_) {
		return;
	}

	channel_rtp_packet_receive_->Start();			
	playing_ = true;
}


void RTCAudioReceiveStreamImpl::Stop() {
	RTC_DCHECK_RUN_ON(&worker_thread_checker_);
	if (!playing_) {
		return;
	}

	channel_rtp_packet_receive_->Stop();			
	playing_ = false;
}

bool RTCAudioReceiveStreamImpl::IsRunning() const {
	RTC_DCHECK_RUN_ON(&worker_thread_checker_);
	return playing_;
}

std::vector<RtpSource> RTCAudioReceiveStreamImpl::GetSources() const {
	RTC_DCHECK_RUN_ON(&worker_thread_checker_);
	return source_tracker_.GetSources();
}

void RTCAudioReceiveStreamImpl::DeliverRtcp(const uint8_t* packet, size_t length) {
	channel_rtp_packet_receive_->ReceivedRTCPPacket(packet, length);
}

const RTCAudioReceiveStream::Config& RTCAudioReceiveStreamImpl::config() const {
	RTC_DCHECK_RUN_ON(&worker_thread_checker_);
	return config_;
}

void RTCAudioReceiveStreamImpl::ConfigureStream(
	RTCAudioReceiveStreamImpl* stream,
	const Config& new_config,
	bool first_time) {
	RTC_LOG(LS_INFO) << "RTCAudioReceiveStreamImpl::ConfigureStream: "
		<< new_config.ToString();

	const auto& channel_rtp_packet_receive = stream->channel_rtp_packet_receive_;
	
	const auto& old_config = stream->config_;

	// Configuration parameters which cannot be changed.
	RTC_DCHECK(first_time ||
		old_config.rtp.remote_ssrc == new_config.rtp.remote_ssrc);

	if (!first_time) {
		// SSRC can't be changed mid-stream.
		RTC_DCHECK_EQ(old_config.rtp.local_ssrc, new_config.rtp.local_ssrc);
		RTC_DCHECK_EQ(old_config.rtp.remote_ssrc, new_config.rtp.remote_ssrc);
	}

	// TODO(solenberg): Config NACK history window (which is a packet count),
	// using the actual packet size for the configured codec.

	if (first_time || old_config.decoder_map != new_config.decoder_map) {
		channel_rtp_packet_receive->SetReceiveCodecs(new_config.decoder_map);
	}


	stream->config_ = new_config;
}

}