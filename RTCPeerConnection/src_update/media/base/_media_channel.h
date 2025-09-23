//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /media/base/media_channel.h
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_UPDATE_MEDIA_BASE_MEDIA_CHANNEL_H__
#define __RTC_UPDATE_MEDIA_BASE_MEDIA_CHANNEL_H__

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/types/optional.h"

#include "api/rtc_error.h"
#include "api/rtp_parameters.h"
#include "api/media_types.h"
#include "api/video/video_content_type.h"
#include "api/audio_options.h"
#include "api/video/video_timing.h"
#include "api/transport/rtp/rtp_source.h"
#include "api/call/audio_sink.h"
#include "common_video/include/quality_limitation_reason.h"
#include "media/base/audio_source.h"
#include "media/base/media_config.h"
#include "media/base/stream_params.h"
#include "media/base/delayable.h"
#include "media/base/codec.h"
#include "rtc_base/socket.h"
#include "rtc_base/async_packet_socket.h"
#include "rtc_base/copy_on_write_buffer.h"
#include "rtc_base/network_route.h"
#include "rtc_base/thread_annotations.h"
#include "rtc_base/synchronization/mutex.h"
#include "rtc_base/third_party/sigslot/sigslot.h"
#include "modules/rtp_rtcp/include/report_block_data.h"
#include "modules/rtp_rtcp/source/rtp_packet_received.h"

#include "../../../rtc_pc_src/api/video/RTCVideoPacketRouteSourceInterface.h"
#include "../../../rtc_pc_src/api/audio/RTCAudioPacketRouteSourceInterface.h"


namespace rtc {
	class Timing;
}

namespace cricket {

template <class T>
static std::string ToStringIfSet(const char* key,
	const absl::optional<T>& val) {
	std::string str;
	if (val) {
		str = key;
		str += ": ";
		str += val ? rtc::ToString(*val) : "";
		str += ", ";
	}
	return str;
}

template <class T>
static std::string VectorToString(const std::vector<T>& vals) {
	rtc::StringBuilder ost;  // no-presubmit-check TODO(webrtc:8982)
	ost << "[";
	for (size_t i = 0; i < vals.size(); ++i) {
		if (i > 0) {
			ost << ", ";
		}
		ost << vals[i].ToString();
	}
	ost << "]";
	return ost.Release();
}

class MediaChannel : public sigslot::has_slots<> {
public:
	class NetworkInterface {
	public:
		enum SocketType { ST_RTP, ST_RTCP };
		virtual bool SendPacket(rtc::CopyOnWriteBuffer* packet,
			const rtc::PacketOptions& options) = 0;
		virtual bool SendRtcp(rtc::CopyOnWriteBuffer* packet,
			const rtc::PacketOptions& options) = 0;
		virtual int SetOption(SocketType type,
			rtc::Socket::Option opt,
			int option) = 0;
		virtual ~NetworkInterface() {}
	};

	explicit MediaChannel(const MediaConfig& config);
	MediaChannel();
	~MediaChannel() override;

	virtual cricket::MediaType media_type() const = 0;

	virtual void SetInterface(NetworkInterface* iface) RTC_LOCKS_EXCLUDED(network_interface_mutex_);

	virtual void OnPacketReceived(rtc::CopyOnWriteBuffer packet,
		int64_t packet_time_us) = 0;

	virtual void OnReadyToSend(bool ready) = 0;

	virtual void OnNetworkRouteChanged(
		const std::string& transport_name,
		const rtc::NetworkRoute& network_route) = 0;

	virtual bool AddSendStream(const StreamParams& sp) = 0;

	virtual bool RemoveSendStream(uint32_t ssrc) = 0;

	virtual bool AddRecvStream(const StreamParams& sp) = 0;

	virtual bool RemoveRecvStream(uint32_t ssrc) = 0;

	virtual void ResetUnsignaledRecvStream() = 0;

	virtual int GetRtpSendTimeExtnId() const;

	virtual void SetVideoCodecSwitchingEnabled(bool enabled);	

	bool SendPacket(rtc::CopyOnWriteBuffer* packet,
		const rtc::PacketOptions& options) {
		return DoSendPacket(packet, false, options);
	}

	bool SendRtcp(rtc::CopyOnWriteBuffer* packet,
		const rtc::PacketOptions& options) {
		return DoSendPacket(packet, true, options);
	}

	int SetOption(NetworkInterface::SocketType type,
		rtc::Socket::Option opt,
		int option) RTC_LOCKS_EXCLUDED(network_interface_mutex_) {
		webrtc::MutexLock lock(&network_interface_mutex_);
		return SetOptionLocked(type, opt, option);
	}

	void SetExtmapAllowMixed(bool extmap_allow_mixed) {
		extmap_allow_mixed_ = extmap_allow_mixed;
	}

	bool ExtmapAllowMixed() const { return extmap_allow_mixed_; }

	virtual webrtc::RtpParameters GetRtpSendParameters(uint32_t ssrc) const = 0;
	virtual webrtc::RTCError SetRtpSendParameters(
		uint32_t ssrc,
		const webrtc::RtpParameters& parameters) = 0;

protected:
	int SetOptionLocked(NetworkInterface::SocketType type,
		rtc::Socket::Option opt,
		int option)
		RTC_EXCLUSIVE_LOCKS_REQUIRED(network_interface_mutex_) {
		if (!network_interface_)
			return -1;
		return network_interface_->SetOption(type, opt, option);
	}

	bool DscpEnabled() const { return enable_dscp_; }

	rtc::DiffServCodePoint PreferredDscp() const
		RTC_LOCKS_EXCLUDED(network_interface_mutex_) {
		webrtc::MutexLock lock(&network_interface_mutex_);
		return preferred_dscp_;
	}

	int SetPreferredDscp(rtc::DiffServCodePoint preferred_dscp)
		RTC_LOCKS_EXCLUDED(network_interface_mutex_) {
		webrtc::MutexLock lock(&network_interface_mutex_);
		if (preferred_dscp == preferred_dscp_) {
			return 0;
		}
		preferred_dscp_ = preferred_dscp;
		return UpdateDscp();
	}

private:
	int UpdateDscp() RTC_EXCLUSIVE_LOCKS_REQUIRED(network_interface_mutex_) {
		rtc::DiffServCodePoint value =
			enable_dscp_ ? preferred_dscp_ : rtc::DSCP_DEFAULT;
		int ret =
			SetOptionLocked(NetworkInterface::ST_RTP, rtc::Socket::OPT_DSCP, value);
		if (ret == 0) {
			ret = SetOptionLocked(NetworkInterface::ST_RTCP, rtc::Socket::OPT_DSCP,
				value);
		}
		return ret;
	}

	bool DoSendPacket(rtc::CopyOnWriteBuffer* packet,
		bool rtcp,
		const rtc::PacketOptions& options)
		RTC_LOCKS_EXCLUDED(network_interface_mutex_) {
		webrtc::MutexLock lock(&network_interface_mutex_);

		if (!network_interface_)
			return false;

		return (!rtcp) ? network_interface_->SendPacket(packet, options)
			: network_interface_->SendRtcp(packet, options);
	}

private:
	const bool enable_dscp_;
	mutable webrtc::Mutex network_interface_mutex_;
	NetworkInterface* network_interface_
		RTC_GUARDED_BY(network_interface_mutex_) = nullptr;
	rtc::DiffServCodePoint preferred_dscp_
		RTC_GUARDED_BY(network_interface_mutex_) = rtc::DSCP_DEFAULT;
	bool extmap_allow_mixed_ = false;
};

struct SsrcSenderInfo {
	uint32_t ssrc = 0;
	double timestamp = 0.0;  // NTP timestamp, represented as seconds since epoch.
};

struct SsrcReceiverInfo {
	uint32_t ssrc = 0;
	double timestamp = 0.0;
};

struct MediaSenderInfo {
	MediaSenderInfo();
	~MediaSenderInfo();
	void add_ssrc(const SsrcSenderInfo& stat) { local_stats.push_back(stat); }
	// Temporary utility function for call sites that only provide SSRC.
	// As more info is added into SsrcSenderInfo, this function should go away.
	void add_ssrc(uint32_t ssrc) {
		SsrcSenderInfo stat;
		stat.ssrc = ssrc;
		add_ssrc(stat);
	}
	// Utility accessor for clients that are only interested in ssrc numbers.
	std::vector<uint32_t> ssrcs() const {
		std::vector<uint32_t> retval;
		for (std::vector<SsrcSenderInfo>::const_iterator it = local_stats.begin();
			it != local_stats.end(); ++it) {
			retval.push_back(it->ssrc);
		}
		return retval;
	}
	// Returns true if the media has been connected.
	bool connected() const { return local_stats.size() > 0; }
	// Utility accessor for clients that make the assumption only one ssrc
	// exists per media.
	// This will eventually go away.
	// Call sites that compare this to zero should use connected() instead.
	// https://bugs.webrtc.org/8694
	uint32_t ssrc() const {
		if (connected()) {
			return local_stats[0].ssrc;
		}
		else {
			return 0;
		}
	}
	// https://w3c.github.io/webrtc-stats/#dom-rtcsentrtpstreamstats-bytessent
	int64_t payload_bytes_sent = 0;
	// https://w3c.github.io/webrtc-stats/#dom-rtcoutboundrtpstreamstats-headerbytessent
	int64_t header_and_padding_bytes_sent = 0;
	// https://w3c.github.io/webrtc-stats/#dom-rtcoutboundrtpstreamstats-retransmittedbytessent
	uint64_t retransmitted_bytes_sent = 0;
	int packets_sent = 0;
	// https://w3c.github.io/webrtc-stats/#dom-rtcoutboundrtpstreamstats-retransmittedpacketssent
	uint64_t retransmitted_packets_sent = 0;
	int packets_lost = 0;
	float fraction_lost = 0.0f;
	int64_t rtt_ms = 0;
	std::string codec_name;
	absl::optional<int> codec_payload_type;
	std::vector<SsrcSenderInfo> local_stats;
	std::vector<SsrcReceiverInfo> remote_stats;
	// A snapshot of the most recent Report Block with additional data of interest
	// to statistics. Used to implement RTCRemoteInboundRtpStreamStats. Within
	// this list, the ReportBlockData::RTCPReportBlock::source_ssrc(), which is
	// the SSRC of the corresponding outbound RTP stream, is unique.
	std::vector<webrtc::ReportBlockData> report_block_datas;	
};

struct MediaReceiverInfo {
	MediaReceiverInfo();
	~MediaReceiverInfo();
	void add_ssrc(const SsrcReceiverInfo& stat) { local_stats.push_back(stat); }
	// Temporary utility function for call sites that only provide SSRC.
	// As more info is added into SsrcSenderInfo, this function should go away.
	void add_ssrc(uint32_t ssrc) {
		SsrcReceiverInfo stat;
		stat.ssrc = ssrc;
		add_ssrc(stat);
	}
	std::vector<uint32_t> ssrcs() const {
		std::vector<uint32_t> retval;
		for (std::vector<SsrcReceiverInfo>::const_iterator it = local_stats.begin();
			it != local_stats.end(); ++it) {
			retval.push_back(it->ssrc);
		}
		return retval;
	}
	// Returns true if the media has been connected.
	bool connected() const { return local_stats.size() > 0; }
	// Utility accessor for clients that make the assumption only one ssrc
	// exists per media.
	// This will eventually go away.
	// Call sites that compare this to zero should use connected();
	// https://bugs.webrtc.org/8694
	uint32_t ssrc() const {
		if (connected()) {
			return local_stats[0].ssrc;
		}
		else {
			return 0;
		}
	}

	// https://w3c.github.io/webrtc-stats/#dom-rtcinboundrtpstreamstats-bytesreceived
	int64_t payload_bytes_rcvd = 0;
	// https://w3c.github.io/webrtc-stats/#dom-rtcinboundrtpstreamstats-headerbytesreceived
	int64_t header_and_padding_bytes_rcvd = 0;
	int packets_rcvd = 0;
	int packets_lost = 0;
	// The timestamp at which the last packet was received, i.e. the time of the
	// local clock when it was received - not the RTP timestamp of that packet.
	// https://w3c.github.io/webrtc-stats/#dom-rtcinboundrtpstreamstats-lastpacketreceivedtimestamp
	absl::optional<int64_t> last_packet_received_timestamp_ms;
	// https://w3c.github.io/webrtc-stats/#dom-rtcinboundrtpstreamstats-estimatedplayouttimestamp
	absl::optional<int64_t> estimated_playout_ntp_timestamp_ms;
	std::string codec_name;
	absl::optional<int> codec_payload_type;
	std::vector<SsrcReceiverInfo> local_stats;
	std::vector<SsrcSenderInfo> remote_stats;
};

struct VoiceSenderInfo : public MediaSenderInfo {
	VoiceSenderInfo();
	~VoiceSenderInfo();
	int jitter_ms = 0;
	// Current audio level, expressed linearly [0,32767].
	int audio_level = 0;
	// See description of "totalAudioEnergy" in the WebRTC stats spec:
	// https://w3c.github.io/webrtc-stats/#dom-rtcmediastreamtrackstats-totalaudioenergy
	double total_input_energy = 0.0;
	double total_input_duration = 0.0;
	bool typing_noise_detected = false;
};

struct VoiceReceiverInfo : public MediaReceiverInfo {
	VoiceReceiverInfo();
	~VoiceReceiverInfo();
	int jitter_ms = 0;
	int jitter_buffer_ms = 0;
	int jitter_buffer_preferred_ms = 0;
	int delay_estimate_ms = 0;
	int audio_level = 0;
	// Stats below correspond to similarly-named fields in the WebRTC stats spec.
	// https://w3c.github.io/webrtc-stats/#dom-rtcmediastreamtrackstats
	double total_output_energy = 0.0;
	uint64_t total_samples_received = 0;
	double total_output_duration = 0.0;
	uint64_t concealed_samples = 0;
	uint64_t silent_concealed_samples = 0;
	uint64_t concealment_events = 0;
	double jitter_buffer_delay_seconds = 0.0;
	uint64_t jitter_buffer_emitted_count = 0;
	double jitter_buffer_target_delay_seconds = 0.0;
	uint64_t inserted_samples_for_deceleration = 0;
	uint64_t removed_samples_for_acceleration = 0;
	uint64_t fec_packets_received = 0;
	uint64_t fec_packets_discarded = 0;
	// Stats below DO NOT correspond directly to anything in the WebRTC stats
	// fraction of synthesized audio inserted through expansion.
	float expand_rate = 0.0f;
	// fraction of synthesized speech inserted through expansion.
	float speech_expand_rate = 0.0f;
	// fraction of data out of secondary decoding, including FEC and RED.
	float secondary_decoded_rate = 0.0f;
	// Fraction of secondary data, including FEC and RED, that is discarded.
	// Discarding of secondary data can be caused by the reception of the primary
	// data, obsoleting the secondary data. It can also be caused by early
	// or late arrival of secondary data. This metric is the percentage of
	// discarded secondary data since last query of receiver info.
	float secondary_discarded_rate = 0.0f;
	// Fraction of data removed through time compression.
	float accelerate_rate = 0.0f;
	// Fraction of data inserted through time stretching.
	float preemptive_expand_rate = 0.0f;
	int decoding_calls_to_silence_generator = 0;
	int decoding_calls_to_neteq = 0;
	int decoding_normal = 0;
	// TODO(alexnarest): Consider decoding_neteq_plc for consistency
	int decoding_plc = 0;
	int decoding_codec_plc = 0;
	int decoding_cng = 0;
	int decoding_plc_cng = 0;
	int decoding_muted_output = 0;
	// Estimated capture start time in NTP time in ms.
	int64_t capture_start_ntp_time_ms = -1;
	// Count of the number of buffer flushes.
	uint64_t jitter_buffer_flushes = 0;
	// Number of samples expanded due to delayed packets.
	uint64_t delayed_packet_outage_samples = 0;
	// Arrival delay of received audio packets.
	double relative_packet_arrival_delay_seconds = 0.0;
	// Count and total duration of audio interruptions (loss-concealement periods
	// longer than 150 ms).
	int32_t interruption_count = 0;
	int32_t total_interruption_duration_ms = 0;
};

struct VideoSenderInfo : public MediaSenderInfo {
	VideoSenderInfo();
	~VideoSenderInfo();
	std::vector<SsrcGroup> ssrc_groups;
	std::string encoder_implementation_name;
	int firs_rcvd = 0;
	int plis_rcvd = 0;
	int nacks_rcvd = 0;
	int send_frame_width = 0;
	int send_frame_height = 0;
	int framerate_input = 0;
	int framerate_sent = 0;
	int aggregated_framerate_sent = 0;
	int nominal_bitrate = 0;
	int adapt_reason = 0;
	int adapt_changes = 0;
	// https://w3c.github.io/webrtc-stats/#dom-rtcoutboundrtpstreamstats-qualitylimitationreason
	webrtc::QualityLimitationReason quality_limitation_reason =
		webrtc::QualityLimitationReason::kNone;
	// https://w3c.github.io/webrtc-stats/#dom-rtcoutboundrtpstreamstats-qualitylimitationdurations
	std::map<webrtc::QualityLimitationReason, int64_t>
		quality_limitation_durations_ms;
	// https://w3c.github.io/webrtc-stats/#dom-rtcoutboundrtpstreamstats-qualitylimitationresolutionchanges
	uint32_t quality_limitation_resolution_changes = 0;
	int avg_encode_ms = 0;
	int encode_usage_percent = 0;
	uint32_t frames_encoded = 0;
	uint32_t key_frames_encoded = 0;
	// https://w3c.github.io/webrtc-stats/#dom-rtcoutboundrtpstreamstats-totalencodetime
	uint64_t total_encode_time_ms = 0;
	// https://w3c.github.io/webrtc-stats/#dom-rtcoutboundrtpstreamstats-totalencodedbytestarget
	uint64_t total_encoded_bytes_target = 0;
	uint64_t total_packet_send_delay_ms = 0;
	bool has_entered_low_resolution = false;
	absl::optional<uint64_t> qp_sum;
	webrtc::VideoContentType content_type = webrtc::VideoContentType::UNSPECIFIED;
	uint32_t frames_sent = 0;
	// https://w3c.github.io/webrtc-stats/#dom-rtcvideosenderstats-hugeframessent
	uint32_t huge_frames_sent = 0;
	uint32_t aggregated_huge_frames_sent = 0;
	absl::optional<std::string> rid;
};

struct VideoReceiverInfo : public MediaReceiverInfo {
	VideoReceiverInfo();
	~VideoReceiverInfo();
	std::vector<SsrcGroup> ssrc_groups;
	std::string decoder_implementation_name;
	int packets_concealed = 0;
	int firs_sent = 0;
	int plis_sent = 0;
	int nacks_sent = 0;
	int frame_width = 0;
	int frame_height = 0;
	int framerate_rcvd = 0;
	int framerate_decoded = 0;
	int framerate_output = 0;
	// Framerate as sent to the renderer.
	int framerate_render_input = 0;
	// Framerate that the renderer reports.
	int framerate_render_output = 0;
	uint32_t frames_received = 0;
	uint32_t frames_dropped = 0;
	uint32_t frames_decoded = 0;
	uint32_t key_frames_decoded = 0;
	uint32_t frames_rendered = 0;
	absl::optional<uint64_t> qp_sum;
	// https://w3c.github.io/webrtc-stats/#dom-rtcinboundrtpstreamstats-totaldecodetime
	uint64_t total_decode_time_ms = 0;
	double total_inter_frame_delay = 0;
	double total_squared_inter_frame_delay = 0;
	int64_t interframe_delay_max_ms = -1;
	uint32_t freeze_count = 0;
	uint32_t pause_count = 0;
	uint32_t total_freezes_duration_ms = 0;
	uint32_t total_pauses_duration_ms = 0;
	uint32_t total_frames_duration_ms = 0;
	double sum_squared_frame_durations = 0.0;

	webrtc::VideoContentType content_type = webrtc::VideoContentType::UNSPECIFIED;

	// All stats below are gathered per-VideoReceiver, but some will be correlated
	// across MediaStreamTracks.  NOTE(hta): when sinking stats into per-SSRC
	// structures, reflect this in the new layout.

	// Current frame decode latency.
	int decode_ms = 0;
	// Maximum observed frame decode latency.
	int max_decode_ms = 0;
	// Jitter (network-related) latency.
	int jitter_buffer_ms = 0;
	// Jitter (network-related) latency (cumulative).
	// https://w3c.github.io/webrtc-stats/#dom-rtcvideoreceiverstats-jitterbufferdelay
	double jitter_buffer_delay_seconds = 0;
	// Number of observations for cumulative jitter latency.
	// https://w3c.github.io/webrtc-stats/#dom-rtcvideoreceiverstats-jitterbufferemittedcount
	uint64_t jitter_buffer_emitted_count = 0;
	// Requested minimum playout latency.
	int min_playout_delay_ms = 0;
	// Requested latency to account for rendering delay.
	int render_delay_ms = 0;
	// Target overall delay: network+decode+render, accounting for
	// min_playout_delay_ms.
	int target_delay_ms = 0;
	// Current overall delay, possibly ramping towards target_delay_ms.
	int current_delay_ms = 0;

	// Estimated capture start time in NTP time in ms.
	int64_t capture_start_ntp_time_ms = -1;

	// First frame received to first frame decoded latency.
	int64_t first_frame_received_to_decoded_ms = -1;

	// Timing frame info: all important timestamps for a full lifetime of a
	// single 'timing frame'.
	absl::optional<webrtc::TimingFrameInfo> timing_frame_info;
};

struct DataSenderInfo : public MediaSenderInfo {
	uint32_t ssrc = 0;
};

struct DataReceiverInfo : public MediaReceiverInfo {
	uint32_t ssrc = 0;
};

struct BandwidthEstimationInfo {
	int available_send_bandwidth = 0;
	int available_recv_bandwidth = 0;
	int target_enc_bitrate = 0;
	int actual_enc_bitrate = 0;
	int retransmit_bitrate = 0;
	int transmit_bitrate = 0;
	int64_t bucket_delay = 0;
};

typedef std::map<int, webrtc::RtpCodecParameters> RtpCodecParametersMap;

struct VoiceMediaInfo {
	VoiceMediaInfo();
	~VoiceMediaInfo();
	void Clear() {
		senders.clear();
		receivers.clear();
		send_codecs.clear();
		receive_codecs.clear();
	}
	std::vector<VoiceSenderInfo> senders;
	std::vector<VoiceReceiverInfo> receivers;
	RtpCodecParametersMap send_codecs;
	RtpCodecParametersMap receive_codecs;
	int32_t device_underrun_count = 0;
};

struct VideoMediaInfo {
	VideoMediaInfo();
	~VideoMediaInfo();
	void Clear() {
		senders.clear();
		aggregated_senders.clear();
		receivers.clear();
		send_codecs.clear();
		receive_codecs.clear();
	}
	// Each sender info represents one "outbound-rtp" stream.In non - simulcast,
	// this means one info per RtpSender but if simulcast is used this means
	// one info per simulcast layer.
	std::vector<VideoSenderInfo> senders;
	// Used for legacy getStats() API's "ssrc" stats and modern getStats() API's
	// "track" stats. If simulcast is used, instead of having one sender info per
	// simulcast layer, the metrics of all layers of an RtpSender are aggregated
	// into a single sender info per RtpSender.
	std::vector<VideoSenderInfo> aggregated_senders;
	std::vector<VideoReceiverInfo> receivers;
	RtpCodecParametersMap send_codecs;
	RtpCodecParametersMap receive_codecs;
};

struct DataMediaInfo {
	DataMediaInfo();
	~DataMediaInfo();
	void Clear() {
		senders.clear();
		receivers.clear();
	}
	std::vector<DataSenderInfo> senders;
	std::vector<DataReceiverInfo> receivers;
};

struct RtcpParameters {
	bool reduced_size = false;
	bool remote_estimate = false;
};

template <class Codec>
struct RtpParameters {
	virtual ~RtpParameters() = default;

	std::vector<Codec> codecs;
	std::vector<webrtc::RtpExtension> extensions;
	// For a send stream this is true if we've neogtiated a send direction,
	// for a receive stream this is true if we've negotiated a receive direction.
	bool is_stream_active = true;

	// TODO(pthatcher): Add streams.
	RtcpParameters rtcp;

	std::string ToString() const {
		rtc::StringBuilder ost;
		ost << "{";
		const char* separator = "";
		for (const auto& entry : ToStringMap()) {
			ost << separator << entry.first << ": " << entry.second;
			separator = ", ";
		}
		ost << "}";
		return ost.Release();
	}

protected:
	virtual std::map<std::string, std::string> ToStringMap() const {
		return{ { "codecs", VectorToString(codecs) },
		{ "extensions", VectorToString(extensions) } };
	}
};

template <class Codec>
struct RtpSendParameters : RtpParameters<Codec> {
	int max_bandwidth_bps = -1;
	// This is the value to be sent in the MID RTP header extension (if the header
	// extension in included in the list of extensions).
	std::string mid;
	bool extmap_allow_mixed = false;

protected:
	std::map<std::string, std::string> ToStringMap() const override {
		auto params = RtpParameters<Codec>::ToStringMap();
		params["max_bandwidth_bps"] = rtc::ToString(max_bandwidth_bps);
		params["mid"] = (mid.empty() ? "<not set>" : mid);
		params["extmap-allow-mixed"] = extmap_allow_mixed ? "true" : "false";
		return params;
	}
};

struct AudioSendParameters : RtpSendParameters<AudioCodec> {
	AudioSendParameters();
	~AudioSendParameters() override;
	AudioOptions options;

protected:
	std::map<std::string, std::string> ToStringMap() const override;
};

struct AudioRecvParameters : RtpParameters<AudioCodec> {};

class VoiceMediaChannel : public MediaChannel, public Delayable {
public:
	VoiceMediaChannel() {}
	explicit VoiceMediaChannel(const MediaConfig& config)
		: MediaChannel(config) {}
	~VoiceMediaChannel() override {}

	cricket::MediaType media_type() const override;
	virtual bool SetSendParameters(const AudioSendParameters& params) = 0;
	virtual bool SetRecvParameters(const AudioRecvParameters& params) = 0;
	// Get the receive parameters for the incoming stream identified by |ssrc|.
	virtual webrtc::RtpParameters GetRtpReceiveParameters(
		uint32_t ssrc) const = 0;
	// Retrieve the receive parameters for the default receive
	// stream, which is used when SSRCs are not signaled.
	virtual webrtc::RtpParameters GetDefaultRtpReceiveParameters() const = 0;
	// Starts or stops playout of received audio.
	virtual void SetPlayout(bool playout) = 0;		
	// Starts or stops sending (and potentially capture) of local audio.
	virtual void SetSend(bool send) = 0;
	// Configure stream for sending.

	virtual bool SetAudioSend(uint32_t ssrc,
		rtc::RTCAudioPacketRouteSourceInterface<std::unique_ptr<webrtc::RtpPacketReceived>>* source) = 0;

	// Gets quality stats for the channel.
	virtual bool GetStats(VoiceMediaInfo* info,
		bool get_and_clear_legacy_stats) = 0;

	virtual std::vector<webrtc::RtpSource> GetSources(uint32_t ssrc) const = 0;
};

struct VideoSendParameters : RtpSendParameters<VideoCodec> {
	VideoSendParameters();
	~VideoSendParameters() override;
	// Use conference mode? This flag comes from the remote
	// description's SDP line 'a=x-google-flag:conference', copied over
	// by VideoChannel::SetRemoteContent_w, and ultimately used by
	// conference mode screencast logic in
	// WebRtcVideoChannel::WebRtcVideoSendStream::CreateVideoEncoderConfig.
	// The special screencast behaviour is disabled by default.
	bool conference_mode = false;

protected:
	std::map<std::string, std::string> ToStringMap() const override;
};

struct VideoRecvParameters : RtpParameters<VideoCodec> {};

class VideoMediaChannel : public MediaChannel, public Delayable {
public:
	VideoMediaChannel() {}
	explicit VideoMediaChannel(const MediaConfig& config)
		: MediaChannel(config) {}
	~VideoMediaChannel() override {}

	cricket::MediaType media_type() const override;
	virtual bool SetSendParameters(const VideoSendParameters& params) = 0;
	virtual bool SetRecvParameters(const VideoRecvParameters& params) = 0;
	// Get the receive parameters for the incoming stream identified by |ssrc|.
	virtual webrtc::RtpParameters GetRtpReceiveParameters(
		uint32_t ssrc) const = 0;
	// Retrieve the receive parameters for the default receive
	// stream, which is used when SSRCs are not signaled.
	virtual webrtc::RtpParameters GetDefaultRtpReceiveParameters() const = 0;
	// Gets the currently set codecs/payload types to be used for outgoing media.
	virtual bool GetSendCodec(VideoCodec* send_codec) = 0;
	// Starts or stops transmission (and potentially capture) of local video.
	virtual bool SetSend(bool send) = 0;
	// Configure stream for sending and register a source.
	// The |ssrc| must correspond to a registered send stream.

	virtual bool SetVideoSend(
		uint32_t ssrc,
		rtc::RTCVideoPacketRouteSourceInterface<std::unique_ptr<webrtc::RtpPacketReceived>>* source) = 0;

	virtual bool SetPacketSink(uint32_t ssrc) = 0;

	// Gets quality stats for the channel.
	virtual bool GetStats(VideoMediaInfo* info) = 0;

	virtual void GenerateKeyFrame(uint32_t ssrc) = 0;
	virtual std::vector<webrtc::RtpSource> GetSources(uint32_t ssrc) const = 0;
};

enum DataMessageType {
	// Chrome-Internal use only.  See SctpDataMediaChannel for the actual PPID
	// values.
	DMT_NONE = 0,
	DMT_CONTROL = 1,
	DMT_BINARY = 2,
	DMT_TEXT = 3,
};

// Info about data received in DataMediaChannel.  For use in
// DataMediaChannel::SignalDataReceived and in all of the signals that
// signal fires, on up the chain.
struct ReceiveDataParams {
	// The in-packet stream indentifier.
	// RTP data channels use SSRCs, SCTP data channels use SIDs.
	union {
		uint32_t ssrc;
		int sid = 0;
	};
	// The type of message (binary, text, or control).
	DataMessageType type = DMT_TEXT;
	// A per-stream value incremented per packet in the stream.
	int seq_num = 0;
	// A per-stream value monotonically increasing with time.
	int timestamp = 0;
};

struct SendDataParams {
	// The in-packet stream indentifier.
	// RTP data channels use SSRCs, SCTP data channels use SIDs.
	union {
		uint32_t ssrc;
		int sid = 0;
	};
	// The type of message (binary, text, or control).
	DataMessageType type = DMT_TEXT;

	// TODO(pthatcher): Make |ordered| and |reliable| true by default?
	// For SCTP, whether to send messages flagged as ordered or not.
	// If false, messages can be received out of order.
	bool ordered = false;
	// For SCTP, whether the messages are sent reliably or not.
	// If false, messages may be lost.
	bool reliable = false;
	// For SCTP, if reliable == false, provide partial reliability by
	// resending up to this many times.  Either count or millis
	// is supported, not both at the same time.
	int max_rtx_count = 0;
	// For SCTP, if reliable == false, provide partial reliability by
	// resending for up to this many milliseconds.  Either count or millis
	// is supported, not both at the same time.
	int max_rtx_ms = 0;
};

enum SendDataResult { SDR_SUCCESS, SDR_ERROR, SDR_BLOCK };

struct DataSendParameters : RtpSendParameters<DataCodec> {};

struct DataRecvParameters : RtpParameters<DataCodec> {};

class DataMediaChannel : public MediaChannel {
public:
	DataMediaChannel();
	explicit DataMediaChannel(const MediaConfig& config);
	~DataMediaChannel() override;

	cricket::MediaType media_type() const override;
	virtual bool SetSendParameters(const DataSendParameters& params) = 0;
	virtual bool SetRecvParameters(const DataRecvParameters& params) = 0;

	// RtpParameter methods are not supported for Data channel.
	webrtc::RtpParameters GetRtpSendParameters(uint32_t ssrc) const override;
	webrtc::RTCError SetRtpSendParameters(
		uint32_t ssrc,
		const webrtc::RtpParameters& parameters) override;

	// TODO(pthatcher): Implement this.
	virtual bool GetStats(DataMediaInfo* info);

	virtual bool SetSend(bool send) = 0;
	virtual bool SetReceive(bool receive) = 0;

	void OnNetworkRouteChanged(const std::string& transport_name,
		const rtc::NetworkRoute& network_route) override {}

	virtual bool SendData(const SendDataParams& params,
		const rtc::CopyOnWriteBuffer& payload,
		SendDataResult* result = NULL) = 0;
	// Signals when data is received (params, data, len)
	sigslot::signal3<const ReceiveDataParams&, const char*, size_t>
		SignalDataReceived;
	// Signal when the media channel is ready to send the stream. Arguments are:
	//     writable(bool)
	sigslot::signal1<bool> SignalReadyToSend;
};


}







#endif // __RTC_UPDATE_MEDIA_BASE_MEDIA_CHANNEL_H__
