//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /call/video_receive_stream.cc
//
//////////////////////////////////////////////////////////////////////////

#include "rtc_base/strings/string_builder.h"

#include "RTCVideoReceiveStream.h"


namespace webrtc {

RTCVideoReceiveStream::Decoder::Decoder() : video_format("Unset") {}
RTCVideoReceiveStream::Decoder::Decoder(const Decoder&) = default;
RTCVideoReceiveStream::Decoder::~Decoder() = default;

std::string RTCVideoReceiveStream::Decoder::ToString() const {
	char buf[1024];
	rtc::SimpleStringBuilder ss(buf);
	ss << "{payload_type: " << payload_type;
	ss << ", payload_name: " << video_format.name;
	ss << ", codec_params: {";
	for (auto it = video_format.parameters.begin();
		it != video_format.parameters.end(); ++it) {
		if (it != video_format.parameters.begin()) {
			ss << ", ";
		}
		ss << it->first << ": " << it->second;
	}
	ss << '}';
	ss << '}';

	return ss.str();
}

RTCVideoReceiveStream::Stats::Stats() = default;
RTCVideoReceiveStream::Stats::~Stats() = default;

std::string RTCVideoReceiveStream::Stats::ToString(int64_t time_ms) const {
	char buf[2048];
	rtc::SimpleStringBuilder ss(buf);
	ss << "RTCVideoReceiveStream stats: " << time_ms << ", {ssrc: " << ssrc << ", ";
	ss << "width: " << width << ", ";
	ss << "height: " << height << ", ";
	ss << "key: " << frame_counts.key_frames << ", ";
	ss << "delta: " << frame_counts.delta_frames << ", ";
	ss << "cum_loss: " << rtp_stats.packets_lost << ", ";
	ss << "nack: " << rtcp_packet_type_counts.nack_packets << ", ";
	ss << "fir: " << rtcp_packet_type_counts.fir_packets << ", ";
	ss << "pli: " << rtcp_packet_type_counts.pli_packets;
	ss << '}';
	return ss.str();
}

RTCVideoReceiveStream::Config::Config(const Config&) = default;
RTCVideoReceiveStream::Config::Config(Config&&) = default;
RTCVideoReceiveStream::Config::Config(Transport* rtcp_send_transport)
	: rtcp_send_transport(rtcp_send_transport) {}

RTCVideoReceiveStream::Config& RTCVideoReceiveStream::Config::operator=(Config&&) =
	default;
RTCVideoReceiveStream::Config::Config::~Config() = default;

std::string RTCVideoReceiveStream::Config::ToString() const {
	char buf[4 * 1024];
	rtc::SimpleStringBuilder ss(buf);
	ss << "{ rtp: " << rtp.ToString();
	ss << '}';

	return ss.str();
}

RTCVideoReceiveStream::Config::Rtp::Rtp() = default;
RTCVideoReceiveStream::Config::Rtp::Rtp(const Rtp&) = default;
RTCVideoReceiveStream::Config::Rtp::~Rtp() = default;

std::string RTCVideoReceiveStream::Config::Rtp::ToString() const {
	char buf[2 * 1024];
	rtc::SimpleStringBuilder ss(buf);
	ss << "{remote_ssrc: " << remote_ssrc;
	ss << ", local_ssrc: " << local_ssrc;
	ss << ", rtcp_mode: "
		<< (rtcp_mode == RtcpMode::kCompound ? "RtcpMode::kCompound"
		: "RtcpMode::kReducedSize");
	ss << ", rtcp_xr: ";
	ss << "{receiver_reference_time_report: "
		<< (rtcp_xr.receiver_reference_time_report ? "on" : "off");
	ss << '}';
	ss << ", transport_cc: " << (transport_cc ? "on" : "off");
	ss << ", lntf: {enabled: " << (lntf.enabled ? "true" : "false") << '}';
	ss << ", nack: {rtp_history_ms: " << nack.rtp_history_ms << '}';
	ss << ", ulpfec_payload_type: " << ulpfec_payload_type;
	ss << ", red_type: " << red_payload_type;
	ss << ", rtx_ssrc: " << rtx_ssrc;
	ss << ", rtx_payload_types: {";
	for (auto& kv : rtx_associated_payload_types) {
		ss << kv.first << " (pt) -> " << kv.second << " (apt), ";
	}
	ss << '}';
	ss << ", raw_payload_types: {";
	for (const auto& pt : raw_payload_types) {
		ss << pt << ", ";
	}
	ss << '}';
	ss << ", extensions: [";
	for (size_t i = 0; i < extensions.size(); ++i) {
		ss << extensions[i].ToString();
		if (i != extensions.size() - 1)
			ss << ", ";
	}
	ss << ']';
	ss << '}';
	return ss.str();
}

}  // namespace webrtc