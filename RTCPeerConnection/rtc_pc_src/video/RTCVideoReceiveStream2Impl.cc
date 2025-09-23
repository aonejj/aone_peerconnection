//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /video/video_receive_stream2.cc
//
//////////////////////////////////////////////////////////////////////////

#include "../rtc_pc_src/_deprecate_defines.h"

#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <memory>
#include <set>
#include <string>
#include <utility>

#include "absl/algorithm/container.h"
#include "absl/types/optional.h"
#include "api/array_view.h"

#include "rtc_base/checks.h"
#include "rtc_base/location.h"
#include "rtc_base/logging.h"
#include "rtc_base/strings/string_builder.h"
#include "rtc_base/system/thread_registry.h"
#include "rtc_base/time_utils.h"
#include "modules/rtp_rtcp/include/receive_statistics.h"
#include "media/base/media_constants.h"

#include "../../src_update/api/video_codecs/_video_codec.h"
#include "RTCVideoReceiveStream2Impl.h"


namespace webrtc {

RTCVideoReceiveStream2Impl::RTCVideoReceiveStream2Impl(
	TaskQueueBase* current_queue,
	RTCRtpTransportControllerSendInterface* rtp_transport,
	RtpStreamReceiverControllerInterface* receiver_controller,
	RTCVideoReceiveStream::Config config,
	CallStats* call_stats,
	Clock* clock,
	RTCVideoRtpPacketListenSinkInterface *video_rtp_packet_listen_sink)
	: transport_adapter_(config.rtcp_send_transport),
	config_(std::move(config)),
	worker_thread_(current_queue),
	clock_(clock),
	call_stats_(call_stats),
	source_tracker_(clock_),
	rtp_receive_statistics_(ReceiveStatistics::Create(clock_)),	
	rtp_video_stream_receiver_(worker_thread_,
							   clock_,
							   rtp_transport,
							   &transport_adapter_,
							   call_stats->AsRtcpRttStats(),
							   &config_,
							   rtp_receive_statistics_.get(),
							   this,
							   nullptr),
	video_rtp_packet_listen_sink_(video_rtp_packet_listen_sink) {

	RTC_DCHECK(worker_thread_);
	RTC_DCHECK(call_stats_);

	rtp_video_stream_receiver_.SetSink(this);

	media_receiver_ = receiver_controller->CreateReceiver(
		config_.rtp.remote_ssrc, &rtp_video_stream_receiver_);

	if (config_.rtp.rtx_ssrc) {
		rtx_receive_stream_ = std::make_unique<RTCRtxReceiveStream>(
			&rtp_video_stream_receiver_, config.rtp.rtx_associated_payload_types,
			config_.rtp.remote_ssrc, rtp_receive_statistics_.get());

		rtx_receiver_ = receiver_controller->CreateReceiver(
			config_.rtp.rtx_ssrc, rtx_receive_stream_.get());
	}
	else {
		rtp_receive_statistics_->EnableRetransmitDetection(config.rtp.remote_ssrc,
			true);
	}

	for (const Decoder& decoder : config_.decoders) {
		rtp_video_stream_receiver_.AddReceiveCodecInfo(decoder.payload_type,
			PayloadStringToCodecType(decoder.video_format.name),
			decoder.video_format.parameters);
	}
}

RTCVideoReceiveStream2Impl::~RTCVideoReceiveStream2Impl() {
	RTC_DCHECK_RUN_ON(&worker_sequence_checker_);
	Stop();
}

void RTCVideoReceiveStream2Impl::SignalNetworkState(NetworkState state) {
	RTC_DCHECK_RUN_ON(&worker_sequence_checker_);
}

bool RTCVideoReceiveStream2Impl::DeliverRtcp(const uint8_t* packet, size_t length) {
	return rtp_video_stream_receiver_.DeliverRtcp(packet, length);
}

void RTCVideoReceiveStream2Impl::Stop() {
	RTC_DCHECK_RUN_ON(&worker_sequence_checker_);
	
	rtp_video_stream_receiver_.StopReceive();
	call_stats_->DeregisterStatsObserver(this);
	transport_adapter_.Disable();
}

void RTCVideoReceiveStream2Impl::Start() {
	RTC_DCHECK_RUN_ON(&worker_sequence_checker_);

	transport_adapter_.Enable();
	call_stats_->RegisterStatsObserver(this);
	rtp_video_stream_receiver_.StartReceive();
}

RTCVideoReceiveStream::Stats RTCVideoReceiveStream2Impl::GetStats() const {
	RTC_DCHECK_RUN_ON(&worker_sequence_checker_);
	RTCVideoReceiveStream::Stats stats;
	
	return stats;
}

std::vector<RtpSource> RTCVideoReceiveStream2Impl::GetSources() const {
	return source_tracker_.GetSources();
}

void RTCVideoReceiveStream2Impl::AddSecondarySink(RtpPacketSinkInterface* sink) {

}

void RTCVideoReceiveStream2Impl::RemoveSecondarySink(
		const RtpPacketSinkInterface* sink) {
}

void RTCVideoReceiveStream2Impl::SendNack(
	const std::vector<uint16_t>& sequence_numbers,
	bool buffering_allowed) {
	RTC_DCHECK_RUN_ON(&worker_sequence_checker_);
	RTC_DCHECK(buffering_allowed);

	rtp_video_stream_receiver_.RequestPacketRetransmit(sequence_numbers);
}

void RTCVideoReceiveStream2Impl::RequestKeyFrame(int64_t timestamp_ms, bool is_fir /*= true*/) {
	if (is_fir) {
		rtp_video_stream_receiver_.RequestKeyFrame();
	}
	else {
		rtp_video_stream_receiver_.RequestKeyFramePli();
	}
}

void RTCVideoReceiveStream2Impl::OnRttUpdate(int64_t avg_rtt_ms, int64_t max_rtt_ms) {
	RTC_DCHECK_RUN_ON(&worker_sequence_checker_);
	rtp_video_stream_receiver_.UpdateRtt(max_rtt_ms);
}

void RTCVideoReceiveStream2Impl::GenerateKeyFrame() {
	RTC_DCHECK_RUN_ON(&worker_sequence_checker_);
	RequestKeyFrame(clock_->TimeInMilliseconds());
	keyframe_generation_requested_ = true;
}

void RTCVideoReceiveStream2Impl::GenerateKeyFramePli() {
	RTC_DCHECK_RUN_ON(&worker_sequence_checker_);
	RequestKeyFrame(clock_->TimeInMilliseconds(), false);
}

void RTCVideoReceiveStream2Impl::OnStart() {
	video_rtp_packet_listen_sink_->OnStart();
}

void RTCVideoReceiveStream2Impl::OnStop() {
	video_rtp_packet_listen_sink_->OnStop();
}

void RTCVideoReceiveStream2Impl::OnRtpPacket(const RtpPacketReceived& packet) {
	video_rtp_packet_listen_sink_->OnRtpPacket(packet);
}

void  RTCVideoReceiveStream2Impl::OnRtxRtpPacket(const RtpPacketReceived& packet) {
	video_rtp_packet_listen_sink_->OnRtxRtpPacket(packet);
}

uint32_t RTCVideoReceiveStream2Impl::RemoteSsrc() {
	return config_.rtp.remote_ssrc;
}

VideoCodecType RTCVideoReceiveStream2Impl::_get_rtp_codec_type(const RtpPacketReceived& packet) {
	RTPHeader hdr;
	packet.GetHeader(&hdr);

	std::vector<RTCVideoReceiveStream::Decoder>::const_iterator it;
	it = config_.decoders.begin();

	while (it != config_.decoders.end()) {
		if (hdr.payloadType == it->payload_type) {
			if (!it->video_format.name.compare(cricket::kH264CodecName)) {
				return VideoCodecType::kVideoCodecH264;
			}
			else if (!it->video_format.name.compare(cricket::kVp8CodecName)) {
				return VideoCodecType::kVideoCodecVP8;
			}
			else if (!it->video_format.name.compare(cricket::kVp9CodecName)) {
				return VideoCodecType::kVideoCodecVP9;
			}
		}

		it++;
	}

	return VideoCodecType::kVideoCodecGeneric;
}

}