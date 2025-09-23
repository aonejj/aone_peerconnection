//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /call/video_send_stream.cc
//
//////////////////////////////////////////////////////////////////////////

#include <utility>

#include "rtc_base/strings/string_builder.h"

#include "RTCVideoSendStream.h"

namespace webrtc {

const char* StreamTypeToString(RTCVideoSendStream::StreamStats::StreamType type) {
	switch (type) {
	case RTCVideoSendStream::StreamStats::StreamType::kMedia:
		return "media";
	case RTCVideoSendStream::StreamStats::StreamType::kRtx:
		return "rtx";
	case RTCVideoSendStream::StreamStats::StreamType::kFlexfec:
		return "flexfec";
	}
	RTC_CHECK_NOTREACHED();
}

RTCVideoSendStream::StreamStats::StreamStats() = default;
RTCVideoSendStream::StreamStats::~StreamStats() = default;


std::string RTCVideoSendStream::StreamStats::ToString() const {
	char buf[1024];
	rtc::SimpleStringBuilder ss(buf);
	ss << "type: " << StreamTypeToString(type);
	if (referenced_media_ssrc.has_value())
		ss << " (for: " << referenced_media_ssrc.value() << ")";
	ss << ", ";
	ss << "width: " << width << ", ";
	ss << "height: " << height << ", ";
	ss << "key: " << frame_counts.key_frames << ", ";
	ss << "delta: " << frame_counts.delta_frames << ", ";
	ss << "total_bps: " << total_bitrate_bps << ", ";
	ss << "retransmit_bps: " << retransmit_bitrate_bps << ", ";
	ss << "avg_delay_ms: " << avg_delay_ms << ", ";
	ss << "max_delay_ms: " << max_delay_ms << ", ";
	ss << "cum_loss: " << rtcp_stats.packets_lost << ", ";
	ss << "max_ext_seq: " << rtcp_stats.extended_highest_sequence_number << ", ";
	ss << "nack: " << rtcp_packet_type_counts.nack_packets << ", ";
	ss << "fir: " << rtcp_packet_type_counts.fir_packets << ", ";
	ss << "pli: " << rtcp_packet_type_counts.pli_packets;
	return ss.str();
}

RTCVideoSendStream::Stats::Stats() = default;
RTCVideoSendStream::Stats::~Stats() = default;

std::string RTCVideoSendStream::Stats::ToString(int64_t time_ms) const {
	char buf[2048];
	rtc::SimpleStringBuilder ss(buf);
	ss << "VideoSendStream stats: " << time_ms << ", {";
	ss << "suspended: " << (suspended ? "true" : "false") << ", ";
	ss << '}';
	for (const auto& substream : substreams) {
		if (substream.second.type ==
			RTCVideoSendStream::StreamStats::StreamType::kMedia) {
			ss << " {ssrc: " << substream.first << ", ";
			ss << substream.second.ToString();
			ss << '}';
		}
	}
	return ss.str();
}

RTCVideoSendStream::Config::Config(const Config&) = default;
RTCVideoSendStream::Config::Config(Config&&) = default;
RTCVideoSendStream::Config::Config(Transport* send_transport)
	: rtp(),
	send_transport(send_transport) {}

RTCVideoSendStream::Config& RTCVideoSendStream::Config::operator=(Config&&) = default;
RTCVideoSendStream::Config::Config::~Config() = default;

std::string RTCVideoSendStream::Config::ToString() const {
	char buf[2 * 1024];
	rtc::SimpleStringBuilder ss(buf);
	ss << ", rtp: " << rtp.ToString();
	ss << ", rtcp_report_interval_ms: " << rtcp_report_interval_ms;
	ss << ", send_transport: " << (send_transport ? "(Transport)" : "nullptr");
	ss << '}';
	return ss.str();
}

}