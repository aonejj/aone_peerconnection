//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /media/engine/webrtc_voice_engine.cc
//
//////////////////////////////////////////////////////////////////////////

#include "../rtc_pc_src/_deprecate_defines.h"		// deprecate

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/strings/match.h"

#include "api/audio_codecs/audio_codec_pair_id.h"	
#include "api/transport/bitrate_settings.h"
#include "api/call/audio_sink.h"
#include "media/base/audio_source.h"
#include "media/base/media_constants.h"
#include "media/base/stream_params.h"
#include "media/engine/payload_type_mapper.h"
#include "rtc_base/arraysize.h"
#include "rtc_base/byte_order.h"
#include "rtc_base/experiments/field_trial_parser.h"
#include "rtc_base/experiments/field_trial_units.h"
#include "rtc_base/experiments/struct_parameters_parser.h"
#include "rtc_base/helpers.h"
#include "rtc_base/ignore_wundef.h"
#include "rtc_base/logging.h"
#include "rtc_base/race_checker.h"
#include "rtc_base/strings/audio_format_to_string.h"
#include "rtc_base/strings/string_builder.h"
#include "rtc_base/strings/string_format.h"
#include "rtc_base/task_utils/pending_task_safety_flag.h"
#include "rtc_base/task_utils/to_queued_task.h"
#include "rtc_base/third_party/base64/base64.h"
#include "rtc_base/thread.h"
#include "system_wrappers/include/metrics.h"

//#include "../../../src_update/media/engine/_webrtc_media_engine.h"
#include "../../../src_update/media/engine/_webrtc_media_engine_info.h"
#include "../../api/audio/RTCAudioRtpPacketListenSinkInterface.h"
#include "RTCWebRTCVoiceEngine.h"


namespace cricket {

constexpr size_t kMaxUnsignaledRecvStreams = 4;

constexpr int kNackRtpHistoryMs = 5000;

const int kMinTelephoneEventCode = 0;  // RFC4733 (Section 2.3.1)
const int kMaxTelephoneEventCode = 255;

const int kMinPayloadType = 0;
const int kMaxPayloadType = 127;

class ProxySink : public webrtc::AudioSinkInterface {
public:
	explicit ProxySink(AudioSinkInterface* sink) : sink_(sink) {
		RTC_DCHECK(sink);
	}

	void OnData(const Data& audio) override { sink_->OnData(audio); }

private:
	webrtc::AudioSinkInterface* sink_;
};

bool ValidateStreamParams(const StreamParams& sp) {
	if (sp.ssrcs.empty()) {
		RTC_DLOG(LS_ERROR) << "No SSRCs in stream parameters: " << sp.ToString();
		return false;
	}
	if (sp.ssrcs.size() > 1) {
		RTC_DLOG(LS_ERROR) << "Multiple SSRCs in stream parameters: "
			<< sp.ToString();
		return false;
	}
	return true;
}

// Dumps an AudioCodec in RFC 2327-ish format.
std::string ToString(const AudioCodec& codec) {
	rtc::StringBuilder ss;
	ss << codec.name << "/" << codec.clockrate << "/" << codec.channels;
	if (!codec.params.empty()) {
		ss << " {";
		for (const auto& param : codec.params) {
			ss << " " << param.first << "=" << param.second;
		}
		ss << " }";
	}
	ss << " (" << codec.id << ")";
	return ss.Release();
}

bool IsCodec(const AudioCodec& codec, const char* ref_name) {
	return absl::EqualsIgnoreCase(codec.name, ref_name);
}

bool FindCodec(const std::vector<AudioCodec>& codecs,
	const AudioCodec& codec,
	AudioCodec* found_codec) {
	for (const AudioCodec& c : codecs) {
		if (c.Matches(codec)) {
			if (found_codec != NULL) {
				*found_codec = c;
			}
			return true;
		}
	}
	return false;
}

bool VerifyUniquePayloadTypes(const std::vector<AudioCodec>& codecs) {
	if (codecs.empty()) {
		return true;
	}
	std::vector<int> payload_types;
	absl::c_transform(codecs, std::back_inserter(payload_types),
		[](const AudioCodec& codec) { return codec.id; });
	absl::c_sort(payload_types);
	return absl::c_adjacent_find(payload_types) == payload_types.end();
}

// Returns its smallest positive argument. If neither argument is positive,
// returns an arbitrary nonpositive value.
int MinPositive(int a, int b) {
	if (a <= 0) {
		return b;
	}
	if (b <= 0) {
		return a;
	}
	return std::min(a, b);
}

// |max_send_bitrate_bps| is the bitrate from "b=" in SDP.
// |rtp_max_bitrate_bps| is the bitrate from RtpSender::SetParameters.
absl::optional<int> ComputeSendBitrate(int max_send_bitrate_bps,
	absl::optional<int> rtp_max_bitrate_bps,
	const webrtc::AudioCodecSpec& spec) {
	// If application-configured bitrate is set, take minimum of that and SDP
	// bitrate.
	const int bps = rtp_max_bitrate_bps
		? MinPositive(max_send_bitrate_bps, *rtp_max_bitrate_bps)
		: max_send_bitrate_bps;
	if (bps <= 0) {
		return spec.info.default_bitrate_bps;
	}

	if (bps < spec.info.min_bitrate_bps) {
		// If codec is not multi-rate and |bps| is less than the fixed bitrate then
		// fail. If codec is not multi-rate and |bps| exceeds or equal the fixed
		// bitrate then ignore.
		RTC_LOG(LS_ERROR) << "Failed to set codec " << spec.format.name
			<< " to bitrate " << bps
			<< " bps"
			", requires at least "
			<< spec.info.min_bitrate_bps << " bps.";
		return absl::nullopt;
	}

	if (spec.info.HasFixedBitrate()) {
		return spec.info.default_bitrate_bps;
	}
	else {
		// If codec is multi-rate then just set the bitrate.
		return std::min(bps, spec.info.max_bitrate_bps);
	}
}

struct AdaptivePtimeConfig {
	bool enabled = false;
	webrtc::DataRate min_payload_bitrate = webrtc::DataRate::KilobitsPerSec(16);
	webrtc::DataRate min_encoder_bitrate = webrtc::DataRate::KilobitsPerSec(12);
	bool use_slow_adaptation = true;

	absl::optional<std::string> audio_network_adaptor_config;

	std::unique_ptr<webrtc::StructParametersParser> Parser() {
		return webrtc::StructParametersParser::Create(    //
			"enabled", &enabled,                          //
			"min_payload_bitrate", &min_payload_bitrate,  //
			"min_encoder_bitrate", &min_encoder_bitrate,  //
			"use_slow_adaptation", &use_slow_adaptation);
	}

	AdaptivePtimeConfig() {}
};


RTCWebRtcVoiceEngine::RTCWebRtcVoiceEngine(
	webrtc::TaskQueueFactory* task_queue_factory,
	std::vector<AudioCodec>	audio_decoder_codecs,		
	std::vector<AudioCodec>	audio_encoder_codecs		
	)
	: task_queue_factory_(task_queue_factory),
	recv_codecs_(audio_decoder_codecs),				
	send_codecs_(audio_encoder_codecs) 				
   {
	worker_thread_checker_.Detach();
	signal_thread_checker_.Detach();
	RTC_LOG(LS_INFO) << "RTCWebRtcVoiceEngine::RTCWebRtcVoiceEngine";
	RTC_DCHECK(!recv_codecs_.empty());	
	RTC_DCHECK(!send_codecs_.empty());	
}
	
RTCWebRtcVoiceEngine::~RTCWebRtcVoiceEngine() {
	RTC_DCHECK_RUN_ON(&worker_thread_checker_);
	RTC_LOG(LS_INFO) << "RTCWebRtcVoiceEngine::~RTCWebRtcVoiceEngine";
}

void RTCWebRtcVoiceEngine::Init() {
	RTC_DCHECK_RUN_ON(&worker_thread_checker_);
	RTC_LOG(LS_INFO) << "RTCWebRtcVoiceEngine::Init";

	// TaskQueue expects to be created/destroyed on the same thread.
	low_priority_worker_queue_.reset(
		new rtc::TaskQueue(task_queue_factory_->CreateTaskQueue(
		"rtc-low-prio", webrtc::TaskQueueFactory::Priority::LOW)));

	// Load our audio codec lists.
	RTC_LOG(LS_VERBOSE) << "Supported send codecs in order of preference:";
	for (const AudioCodec& codec : send_codecs_) {
		RTC_LOG(LS_VERBOSE) << ToString(codec);
	}

	RTC_LOG(LS_VERBOSE) << "Supported recv codecs in order of preference:";
	for (const AudioCodec& codec : recv_codecs_) {
		RTC_LOG(LS_VERBOSE) << ToString(codec);
	}

	initialized_ = true;
}

VoiceMediaChannel* RTCWebRtcVoiceEngine::CreateMediaChannel(
	webrtc::RTCCall* call,	
	const MediaConfig& config,
	const AudioOptions& options,
	const webrtc::CryptoOptions& crypto_options,
	rtc::RTCThreadManagerInterface* rtc_thread_manager
	) {
	RTC_DCHECK_RUN_ON(&worker_thread_checker_);

	return new RTCWebRtcVoiceMediaChannel(this, config, options, 
				crypto_options,	call,
				rtc_thread_manager
				);
}

bool RTCWebRtcVoiceEngine::ApplyOptions(const AudioOptions& options_in) {
	RTC_DCHECK_RUN_ON(&worker_thread_checker_);
	RTC_LOG(LS_INFO) << "RTCWebRtcVoiceEngine::ApplyOptions: "
		<< options_in.ToString();
	AudioOptions options = options_in;  // The options are modified below.

	bool use_mobile_software_aec = false;

	options.echo_cancellation = false;
	options.auto_gain_control = false;
	options.noise_suppression = false;
	options.typing_detection = false;
	options.experimental_ns = false;
	options.highpass_filter = false;
	options.stereo_swapping = false;

	if (options.audio_jitter_buffer_max_packets) {
		RTC_LOG(LS_INFO) << "NetEq capacity is "
			<< *options.audio_jitter_buffer_max_packets;
		audio_jitter_buffer_max_packets_ =
			std::max(20, *options.audio_jitter_buffer_max_packets);
	}
	if (options.audio_jitter_buffer_fast_accelerate) {
		RTC_LOG(LS_INFO) << "NetEq fast mode? "
			<< *options.audio_jitter_buffer_fast_accelerate;
		audio_jitter_buffer_fast_accelerate_ =
			*options.audio_jitter_buffer_fast_accelerate;
	}
	if (options.audio_jitter_buffer_min_delay_ms) {
		RTC_LOG(LS_INFO) << "NetEq minimum delay is "
			<< *options.audio_jitter_buffer_min_delay_ms;
		audio_jitter_buffer_min_delay_ms_ =
			*options.audio_jitter_buffer_min_delay_ms;
	}
	if (options.audio_jitter_buffer_enable_rtx_handling) {
		RTC_LOG(LS_INFO) << "NetEq handle reordered packets? "
			<< *options.audio_jitter_buffer_enable_rtx_handling;
		audio_jitter_buffer_enable_rtx_handling_ =
			*options.audio_jitter_buffer_enable_rtx_handling;
	}

	return true;
}

const std::vector<AudioCodec>& RTCWebRtcVoiceEngine::send_codecs() const {
	RTC_DCHECK(signal_thread_checker_.IsCurrent());
	return send_codecs_;
}

const std::vector<AudioCodec>& RTCWebRtcVoiceEngine::recv_codecs() const {
	RTC_DCHECK(signal_thread_checker_.IsCurrent());
	return recv_codecs_;
}

std::vector<webrtc::RtpHeaderExtensionCapability>
RTCWebRtcVoiceEngine::GetRtpHeaderExtensions() const {
	RTC_DCHECK(signal_thread_checker_.IsCurrent());
	std::vector<webrtc::RtpHeaderExtensionCapability> result;
	int id = 1;
	for (const auto& uri :
	{ webrtc::RtpExtension::kAudioLevelUri,
	webrtc::RtpExtension::kAbsSendTimeUri,
	webrtc::RtpExtension::kTransportSequenceNumberUri,
	webrtc::RtpExtension::kMidUri/*		
	, webrtc::RtpExtension::kRidUri,
	webrtc::RtpExtension::kRepairedRidUri */ }) {
		result.emplace_back(uri, id++, webrtc::RtpTransceiverDirection::kSendRecv);
	}
	return result;
}

void RTCWebRtcVoiceEngine::RegisterChannel(RTCWebRtcVoiceMediaChannel* channel) {
	RTC_DCHECK_RUN_ON(&worker_thread_checker_);
	RTC_DCHECK(channel);
	channels_.push_back(channel);
}

void RTCWebRtcVoiceEngine::UnregisterChannel(RTCWebRtcVoiceMediaChannel* channel) {
	RTC_DCHECK_RUN_ON(&worker_thread_checker_);
	auto it = absl::c_find(channels_, channel);
	RTC_DCHECK(it != channels_.end());
	channels_.erase(it);
}

std::vector<AudioCodec> RTCWebRtcVoiceEngine::CollectCodecs(
	const std::vector<webrtc::AudioCodecSpec>& specs) const {
	PayloadTypeMapper mapper;
	std::vector<AudioCodec> out;

	// Only generate CN payload types for these clockrates:
	std::map<int, bool, std::greater<int>> generate_cn = {
		{ 8000, false }, { 16000, false }, { 32000, false } };
	// Only generate telephone-event payload types for these clockrates:
	std::map<int, bool, std::greater<int>> generate_dtmf = {
		{ 8000, false }, { 16000, false }, { 32000, false }, { 48000, false } };

	auto map_format = [&mapper](const webrtc::SdpAudioFormat& format,
		std::vector<AudioCodec>* out) {
		absl::optional<AudioCodec> opt_codec = mapper.ToAudioCodec(format);
		if (opt_codec) {
			if (out) {
				out->push_back(*opt_codec);
			}
		}
		else {
			RTC_LOG(LS_ERROR) << "Unable to assign payload type to format: "
				<< rtc::ToString(format);
		}

		return opt_codec;
	};

	for (const auto& spec : specs) {
		// We need to do some extra stuff before adding the main codecs to out.
		absl::optional<AudioCodec> opt_codec = map_format(spec.format, nullptr);
		if (opt_codec) {
			AudioCodec& codec = *opt_codec;
			if (spec.info.supports_network_adaption) {
				codec.AddFeedbackParam(
					FeedbackParam(kRtcpFbParamTransportCc, kParamValueEmpty));
			}

			if (spec.info.allow_comfort_noise) {
				// Generate a CN entry if the decoder allows it and we support the
				// clockrate.
				auto cn = generate_cn.find(spec.format.clockrate_hz);
				if (cn != generate_cn.end()) {
					cn->second = true;
				}
			}

			// Generate a telephone-event entry if we support the clockrate.
			auto dtmf = generate_dtmf.find(spec.format.clockrate_hz);
			if (dtmf != generate_dtmf.end()) {
				dtmf->second = true;
			}

			out.push_back(codec);
		}
	}

	// Add CN codecs after "proper" audio codecs.
	for (const auto& cn : generate_cn) {
		if (cn.second) {
			map_format({ kCnCodecName, cn.first, 1 }, &out);
		}
	}

	// Add telephone-event codecs last.
	for (const auto& dtmf : generate_dtmf) {
		if (dtmf.second) {
			map_format({ kDtmfCodecName, dtmf.first, 1 }, &out);
		}
	}

	return out;
}

class RTCWebRtcVoiceMediaChannel::RTCWebRtcAudioSendStream
	/*: public AudioSource::Sink */ {
public:
	RTCWebRtcAudioSendStream(
		uint32_t ssrc,
		const std::string& mid,
		const std::string& c_name,
		const std::string track_id,
 		const absl::optional<webrtc::RTCAudioSendStream::Config::SendCodecSpec>&
 		send_codec_spec,
		bool extmap_allow_mixed,
		const std::vector<webrtc::RtpExtension>& extensions,
		int max_send_bitrate_bps,
		int rtcp_report_interval_ms,
		webrtc::RTCCall* call,
		webrtc::Transport* send_transport,
		const webrtc::CryptoOptions& crypto_options)
		: call_(call),	
		config_(send_transport),
		max_send_bitrate_bps_(max_send_bitrate_bps),
		rtp_parameters_(CreateRtpParametersWithOneEncoding()) {
		
		RTC_DCHECK(call);	
		
		config_.rtp.ssrc = ssrc;
		config_.rtp.mid = mid;
		config_.rtp.c_name = c_name;
		config_.rtp.extmap_allow_mixed = extmap_allow_mixed;
		config_.rtp.extensions = extensions;
		config_.has_dscp =
			rtp_parameters_.encodings[0].network_priority != webrtc::Priority::kLow;
		config_.track_id = track_id;
		config_.crypto_options = crypto_options;
		config_.rtcp_report_interval_ms = rtcp_report_interval_ms;
		config_.send_codec_spec = send_codec_spec;

		rtp_parameters_.encodings[0].ssrc = ssrc;
		rtp_parameters_.rtcp.cname = c_name;
		rtp_parameters_.header_extensions = extensions;

		stream_ = call_->CreateAudioSendStream(config_);		
	}

	RTCWebRtcAudioSendStream() = delete;
	RTCWebRtcAudioSendStream(const RTCWebRtcAudioSendStream&) = delete;
	RTCWebRtcAudioSendStream& operator=(const RTCWebRtcAudioSendStream&) = delete;

	~RTCWebRtcAudioSendStream() {
		RTC_DCHECK_RUN_ON(&worker_thread_checker_);
		ClearSource();

		call_->DestroyAudioSendStream(stream_);
	}

	void SetRtpExtensions(const std::vector<webrtc::RtpExtension>& extensions) {
		RTC_DCHECK_RUN_ON(&worker_thread_checker_);
		config_.rtp.extensions = extensions;
		rtp_parameters_.header_extensions = extensions;
		ReconfigureAudioSendStream();	
	}

	void SetExtmapAllowMixed(bool extmap_allow_mixed) {
		config_.rtp.extmap_allow_mixed = extmap_allow_mixed;
		ReconfigureAudioSendStream();	
	}

	void SetMid(const std::string& mid) {
		RTC_DCHECK_RUN_ON(&worker_thread_checker_);
		if (config_.rtp.mid == mid) {
			return;
		}
		config_.rtp.mid = mid;
		ReconfigureAudioSendStream();	
	}

	void SetSend(bool send) {
		RTC_DCHECK_RUN_ON(&worker_thread_checker_);
		send_ = send;
		UpdateSendState();
	}

	webrtc::RTCAudioSendStream::Stats GetStats(bool has_remote_tracks) const {
		RTC_DCHECK_RUN_ON(&worker_thread_checker_);
		RTC_DCHECK(stream_);
		return stream_->GetStats();
	}

	void SetSource(rtc::RTCAudioPacketRouteSourceInterface<std::unique_ptr<webrtc::RtpPacketReceived>>* source) {
		RTC_DCHECK_RUN_ON(&worker_thread_checker_);
		RTC_DCHECK(source);
		if (source_) {
			RTC_DCHECK(source_ == source);
			return;
		}
		source_ = source;
		stream_->SetSource(source_);
		UpdateSendState();
	}

	void ClearSource() {
		RTC_DCHECK_RUN_ON(&worker_thread_checker_);
		if (source_) {
			stream_->SetSource(nullptr);
			source_ = nullptr;
		}
		UpdateSendState();
	}

	const webrtc::RtpParameters& rtp_parameters() const {
		return rtp_parameters_;
	}

	webrtc::RTCError SetRtpParameters(const webrtc::RtpParameters& parameters) {
		webrtc::RTCError error = CheckRtpParametersInvalidModificationAndValues(
			rtp_parameters_, parameters);
		if (!error.ok()) {
			return error;
		}

		absl::optional<int> send_rate;
		if (audio_codec_spec_) {
			send_rate = ComputeSendBitrate(max_send_bitrate_bps_,
				parameters.encodings[0].max_bitrate_bps,
				*audio_codec_spec_);
			if (!send_rate) {
				return webrtc::RTCError(webrtc::RTCErrorType::INTERNAL_ERROR);
			}
		}

		const absl::optional<int> old_rtp_max_bitrate =
			rtp_parameters_.encodings[0].max_bitrate_bps;
		double old_priority = rtp_parameters_.encodings[0].bitrate_priority;
		webrtc::Priority old_dscp = rtp_parameters_.encodings[0].network_priority;
		bool old_adaptive_ptime = rtp_parameters_.encodings[0].adaptive_ptime;
		rtp_parameters_ = parameters;
		config_.bitrate_priority = rtp_parameters_.encodings[0].bitrate_priority;
		config_.has_dscp = (rtp_parameters_.encodings[0].network_priority !=
			webrtc::Priority::kLow);

		bool reconfigure_send_stream =
			(rtp_parameters_.encodings[0].max_bitrate_bps != old_rtp_max_bitrate) ||
			(rtp_parameters_.encodings[0].bitrate_priority != old_priority) ||
			(rtp_parameters_.encodings[0].network_priority != old_dscp) ||
			(rtp_parameters_.encodings[0].adaptive_ptime != old_adaptive_ptime);


		if (reconfigure_send_stream) {
			// Changing adaptive_ptime may update the audio network adaptor config
			// used.
			ReconfigureAudioSendStream();
		}

		rtp_parameters_.rtcp.cname = config_.rtp.c_name;
		rtp_parameters_.rtcp.reduced_size = false;

		// parameters.encodings[0].active could have changed.
		UpdateSendState();
		return webrtc::RTCError::OK();
	}

private:
	void UpdateSendState() {
		RTC_DCHECK_RUN_ON(&worker_thread_checker_);
		RTC_DCHECK(stream_);
		RTC_DCHECK_EQ(1UL, rtp_parameters_.encodings.size());

		if (send_ && source_ != nullptr && rtp_parameters_.encodings[0].active) {
			stream_->Start();
		}
		else {  // !send || source_ = nullptr
			stream_->Stop();
		}
	}


	void ReconfigureAudioSendStream() {
		RTC_DCHECK_RUN_ON(&worker_thread_checker_);
		RTC_DCHECK(stream_);
	}

private:
	rtc::ThreadChecker worker_thread_checker_;
	webrtc::RTCCall* call_ = nullptr;
	webrtc::RTCAudioSendStream::Config config_;
	// The stream is owned by WebRtcAudioSendStream and may be reallocated if
	// configuration changes.
	webrtc::RTCAudioSendStream* stream_ = nullptr;

	// Raw pointer to AudioSource owned by LocalAudioTrackHandler.
	// PeerConnection will make sure invalidating the pointer before the object
	// goes away.
	rtc::RTCAudioPacketRouteSourceInterface<std::unique_ptr<webrtc::RtpPacketReceived>>* source_ = nullptr;
	bool send_ = false;
	bool muted_ = false;
	int max_send_bitrate_bps_;
	webrtc::RtpParameters rtp_parameters_;
	absl::optional<webrtc::AudioCodecSpec> audio_codec_spec_;
	// TODO(webrtc:11717): Remove this once audio_network_adaptor in AudioOptions
	// has been removed.
};


class RTCWebRtcVoiceMediaChannel::RTCWebRtcAudioReceiveStream {
public:
	RTCWebRtcAudioReceiveStream(
		uint32_t remote_ssrc,
		uint32_t local_ssrc,
		bool use_transport_cc,
		bool use_nack,
		const std::vector<std::string>& stream_ids,
		const std::vector<webrtc::RtpExtension>& extensions,
		webrtc::RTCCall* call,
		webrtc::Transport* rtcp_send_transport,
		const std::map<int, webrtc::SdpAudioFormat>& decoder_map,
		size_t jitter_buffer_max_packets,
		bool jitter_buffer_fast_accelerate,
		int jitter_buffer_min_delay_ms,
		bool jitter_buffer_enable_rtx_handling,
		const webrtc::CryptoOptions& crypto_options)
		: call_(call),
		  config_() {

		RTC_DCHECK(call);
		config_.rtp.remote_ssrc = remote_ssrc;
		config_.rtp.local_ssrc = local_ssrc;
		config_.rtp.transport_cc = use_transport_cc;
		config_.rtp.nack.rtp_history_ms = use_nack ? kNackRtpHistoryMs : 0;
		config_.rtp.extensions = extensions;
		config_.rtcp_send_transport = rtcp_send_transport;
		config_.decoder_map = decoder_map;			
		RecreateAudioReceiveStream();
	}

	RTCWebRtcAudioReceiveStream() = delete;
	RTCWebRtcAudioReceiveStream(const RTCWebRtcAudioReceiveStream&) = delete;
	RTCWebRtcAudioReceiveStream& operator=(const RTCWebRtcAudioReceiveStream&) = delete;

	~RTCWebRtcAudioReceiveStream() {
		RTC_DCHECK_RUN_ON(&worker_thread_checker_);

		call_->DestroyAudioReceiveStream(stream_);
	}

	void SetLocalSsrc(uint32_t local_ssrc) {
		RTC_DCHECK_RUN_ON(&worker_thread_checker_);
		if (local_ssrc != config_.rtp.local_ssrc) {
			config_.rtp.local_ssrc = local_ssrc;
			RecreateAudioReceiveStream();
		}
	}

	void SetUseTransportCcAndRecreateStream(bool use_transport_cc,
		bool use_nack) {
		RTC_DCHECK_RUN_ON(&worker_thread_checker_);
		config_.rtp.transport_cc = use_transport_cc;
		config_.rtp.nack.rtp_history_ms = use_nack ? kNackRtpHistoryMs : 0;
		ReconfigureAudioReceiveStream();
	}

	void SetRtpExtensionsAndRecreateStream(
		const std::vector<webrtc::RtpExtension>& extensions) {
		RTC_DCHECK_RUN_ON(&worker_thread_checker_);
		config_.rtp.extensions = extensions;
		RecreateAudioReceiveStream();
	}

	// Set a new payload type -> decoder map.
	void SetDecoderMap(const std::map<int, webrtc::SdpAudioFormat>& decoder_map) {
		RTC_DCHECK_RUN_ON(&worker_thread_checker_);
 		config_.decoder_map = decoder_map;	
	}

	void MaybeRecreateAudioReceiveStream(
		const std::vector<std::string>& stream_ids) {
		RTC_DCHECK_RUN_ON(&worker_thread_checker_);
	}

	void SetRawAudioSink(std::unique_ptr<webrtc::AudioSinkInterface> sink) {
		RTC_DCHECK_RUN_ON(&worker_thread_checker_);
		// Need to update the stream's sink first; once raw_audio_sink_ is
		// reassigned, whatever was in there before is destroyed.
	}

	void SetOutputVolume(double volume) {
		RTC_DCHECK_RUN_ON(&worker_thread_checker_);
		output_volume_ = volume;
	}

	void SetPlayout(bool playout) {
		RTC_DCHECK_RUN_ON(&worker_thread_checker_);
		RTC_DCHECK(stream_);
		if (playout) {
			stream_->Start();
		}
		else {
			stream_->Stop();
		}
	}

	bool SetBaseMinimumPlayoutDelayMs(int delay_ms) {
		RTC_DCHECK_RUN_ON(&worker_thread_checker_);
		RTC_DCHECK(stream_);
		return true;
	}

	int GetBaseMinimumPlayoutDelayMs() const {
		RTC_DCHECK_RUN_ON(&worker_thread_checker_);
		RTC_DCHECK(stream_);
		return 0;
	}

	std::vector<webrtc::RtpSource> GetSources() {
		RTC_DCHECK_RUN_ON(&worker_thread_checker_);
		RTC_DCHECK(stream_);
		return stream_->GetSources();
	}

	webrtc::RtpParameters GetRtpParameters() const {
		RTC_LOG(LS_VERBOSE) << "RTCWebRtcAudioReceiveStream::GetRtpParameters";
		webrtc::RtpParameters rtp_parameters;
		rtp_parameters.encodings.emplace_back();
		rtp_parameters.encodings[0].ssrc = config_.rtp.remote_ssrc;
		rtp_parameters.header_extensions = config_.rtp.extensions;

		return rtp_parameters;
	}

private:
	void RecreateAudioReceiveStream() {
		RTC_DCHECK_RUN_ON(&worker_thread_checker_);
		bool was_running = false;
		if (stream_) {
			was_running = stream_->IsRunning();
			call_->DestroyAudioReceiveStream(stream_);
		}
		stream_ = call_->CreateAudioReceiveStream(config_);
		RTC_CHECK(stream_);


		if (was_running)
			SetPlayout(was_running);
	}

	void ReconfigureAudioReceiveStream() {
		RTC_DCHECK_RUN_ON(&worker_thread_checker_);
		RTC_DCHECK(stream_);
		stream_->Reconfigure(config_);
	}

private:
	rtc::ThreadChecker worker_thread_checker_;
	webrtc::RTCCall* call_ = nullptr;	
	webrtc::RTCAudioReceiveStream::Config config_;
	// The stream is owned by WebRtcAudioReceiveStream and may be reallocated if
	// configuration changes.
	webrtc::RTCAudioReceiveStream* stream_ = nullptr;
	float output_volume_ = 1.0;
	std::unique_ptr<webrtc::AudioSinkInterface> raw_audio_sink_;
};


RTCWebRtcVoiceMediaChannel::RTCWebRtcVoiceMediaChannel(
	RTCWebRtcVoiceEngine* engine,
	const MediaConfig& config,
	const AudioOptions& options,
	const webrtc::CryptoOptions& crypto_options,
	webrtc::RTCCall* call
	, rtc::RTCThreadManagerInterface* rtc_thread_manager
	)
	: VoiceMediaChannel(config),
	worker_thread_(rtc::Thread::Current(rtc_thread_manager)),
	engine_(engine),
	call_(call),
	audio_config_(config.audio),
	crypto_options_(crypto_options) {

	RTC_DCHECK_RUN_ON(worker_thread_);
	network_thread_checker_.Detach();
	engine->RegisterChannel(this);
}

RTCWebRtcVoiceMediaChannel::~RTCWebRtcVoiceMediaChannel() {
	RTC_DCHECK_RUN_ON(worker_thread_);
	// TODO(solenberg): Should be able to delete the streams directly, without
	//                  going through RemoveNnStream(), once stream objects handle
	//                  all (de)configuration.
	while (!send_streams_.empty()) {
		RemoveSendStream(send_streams_.begin()->first);
	}
	while (!recv_streams_.empty()) {
		RemoveRecvStream(recv_streams_.begin()->first);
	}
	engine()->UnregisterChannel(this);
}

bool RTCWebRtcVoiceMediaChannel::SetSendParameters(
	const AudioSendParameters& params) {
	RTC_DCHECK_RUN_ON(worker_thread_);
	RTC_LOG(LS_INFO) << "RTCWebRtcVoiceMediaChannel::SetSendParameters: "
		<< params.ToString();
	// TODO(pthatcher): Refactor this to be more clean now that we have
	// all the information at once.

	if (!SetSendCodecs(params.codecs)) {
		return false;
	}

	if (!ValidateRtpExtensions(params.extensions)) {
		return false;
	}

	if (ExtmapAllowMixed() != params.extmap_allow_mixed) {
		SetExtmapAllowMixed(params.extmap_allow_mixed);
		for (auto& it : send_streams_) {
			it.second->SetExtmapAllowMixed(params.extmap_allow_mixed);
		}
	}

	std::vector<webrtc::RtpExtension> filtered_extensions = FilterRtpExtensions(
		params.extensions, webrtc::RtpExtension::IsSupportedForAudio, true);
	if (send_rtp_extensions_ != filtered_extensions) {
		send_rtp_extensions_.swap(filtered_extensions);
		for (auto& it : send_streams_) {
			it.second->SetRtpExtensions(send_rtp_extensions_);
		}
	}
	if (!params.mid.empty()) {
		mid_ = params.mid;
		for (auto& it : send_streams_) {
			it.second->SetMid(params.mid);
		}
	}

	return true;
}

bool RTCWebRtcVoiceMediaChannel::SetRecvParameters(
	const AudioRecvParameters& params) {
	RTC_DCHECK_RUN_ON(worker_thread_);
	RTC_LOG(LS_INFO) << "RTCWebRtcVoiceMediaChannel::SetRecvParameters: "
		<< params.ToString();
	// TODO(pthatcher): Refactor this to be more clean now that we have
	// all the information at once.

	if (!SetRecvCodecs(params.codecs)) {
		return false;
	}

	if (!ValidateRtpExtensions(params.extensions)) {
		return false;
	}
	std::vector<webrtc::RtpExtension> filtered_extensions = FilterRtpExtensions(
		params.extensions, webrtc::RtpExtension::IsSupportedForAudio, false);
	if (recv_rtp_extensions_ != filtered_extensions) {
		recv_rtp_extensions_.swap(filtered_extensions);
		for (auto& it : recv_streams_) {
			it.second->SetRtpExtensionsAndRecreateStream(recv_rtp_extensions_);
		}
	}
	return true;
}

webrtc::RtpParameters RTCWebRtcVoiceMediaChannel::GetRtpSendParameters(
	uint32_t ssrc) const {
	RTC_DCHECK_RUN_ON(worker_thread_);
	auto it = send_streams_.find(ssrc);
	if (it == send_streams_.end()) {
		RTC_LOG(LS_WARNING) << "Attempting to get RTP send parameters for stream "
			"with ssrc "
			<< ssrc << " which doesn't exist.";
		return webrtc::RtpParameters();
	}

	webrtc::RtpParameters rtp_params = it->second->rtp_parameters();
	// Need to add the common list of codecs to the send stream-specific
	// RTP parameters.
	for (const AudioCodec& codec : send_codecs_) {
		rtp_params.codecs.push_back(codec.ToCodecParameters());
	}
	return rtp_params;
}

webrtc::RTCError RTCWebRtcVoiceMediaChannel::SetRtpSendParameters(
		uint32_t ssrc,
		const webrtc::RtpParameters& parameters) {
	RTC_DCHECK_RUN_ON(worker_thread_);
	auto it = send_streams_.find(ssrc);
	if (it == send_streams_.end()) {
		RTC_LOG(LS_WARNING) << "Attempting to set RTP send parameters for stream "
			"with ssrc "
			<< ssrc << " which doesn't exist.";
		return webrtc::RTCError(webrtc::RTCErrorType::INTERNAL_ERROR);
	}

	// TODO(deadbeef): Handle setting parameters with a list of codecs in a
	// different order (which should change the send codec).
	webrtc::RtpParameters current_parameters = GetRtpSendParameters(ssrc);
	if (current_parameters.codecs != parameters.codecs) {
		RTC_DLOG(LS_ERROR) << "Using SetParameters to change the set of codecs "
			"is not currently supported.";
		return webrtc::RTCError(webrtc::RTCErrorType::UNSUPPORTED_PARAMETER);
	}

	if (!parameters.encodings.empty()) {
		// Note that these values come from:
		// https://tools.ietf.org/html/draft-ietf-tsvwg-rtcweb-qos-16#section-5
		rtc::DiffServCodePoint new_dscp = rtc::DSCP_DEFAULT;
		switch (parameters.encodings[0].network_priority) {
		case webrtc::Priority::kVeryLow:
			new_dscp = rtc::DSCP_CS1;
			break;
		case webrtc::Priority::kLow:
			new_dscp = rtc::DSCP_DEFAULT;
			break;
		case webrtc::Priority::kMedium:
			new_dscp = rtc::DSCP_EF;
			break;
		case webrtc::Priority::kHigh:
			new_dscp = rtc::DSCP_EF;
			break;
		}
		SetPreferredDscp(new_dscp);
	}

	// TODO(minyue): The following legacy actions go into
	// |WebRtcAudioSendStream::SetRtpParameters()| which is called at the end,
	// though there are two difference:
	// 1. |WebRtcVoiceMediaChannel::SetChannelSendParameters()| only calls
	// |SetSendCodec| while |WebRtcAudioSendStream::SetRtpParameters()| calls
	// |SetSendCodecs|. The outcome should be the same.
	// 2. AudioSendStream can be recreated.

	// Codecs are handled at the WebRtcVoiceMediaChannel level.
	webrtc::RtpParameters reduced_params = parameters;
	reduced_params.codecs.clear();
	return it->second->SetRtpParameters(reduced_params);
}

webrtc::RtpParameters RTCWebRtcVoiceMediaChannel::GetRtpReceiveParameters(
		uint32_t ssrc) const {
	RTC_DCHECK_RUN_ON(worker_thread_);
	RTC_LOG(LS_VERBOSE) << "RTCWebRtcVoiceMediaChannel::GetRtpReceiveParameters";
	webrtc::RtpParameters rtp_params;
	auto it = recv_streams_.find(ssrc);
	if (it == recv_streams_.end()) {
		RTC_LOG(LS_WARNING)
			<< "Attempting to get RTP receive parameters for stream "
			"with ssrc "
			<< ssrc << " which doesn't exist.";
		return webrtc::RtpParameters();
	}
	rtp_params = it->second->GetRtpParameters();

	for (const AudioCodec& codec : recv_codecs_) {
		rtp_params.codecs.push_back(codec.ToCodecParameters());
	}
	return rtp_params;
}

webrtc::RtpParameters RTCWebRtcVoiceMediaChannel::GetDefaultRtpReceiveParameters()
		const {
	RTC_DCHECK_RUN_ON(worker_thread_);
	webrtc::RtpParameters rtp_params;

	rtp_params.encodings.emplace_back();

	for (const AudioCodec& codec : recv_codecs_) {
		rtp_params.codecs.push_back(codec.ToCodecParameters());
	}
	return rtp_params;
}

bool RTCWebRtcVoiceMediaChannel::SetRecvCodecs(
		const std::vector<AudioCodec>& codecs) {
	RTC_DCHECK_RUN_ON(worker_thread_);

	// Set the payload types to be used for incoming media.
	RTC_LOG(LS_INFO) << "Setting receive voice codecs.";

	if (!VerifyUniquePayloadTypes(codecs)) {
		RTC_LOG(LS_ERROR) << "Codec payload types overlap.";
		return false;
	}

	// Create a payload type -> SdpAudioFormat map with all the decoders. Fail
	// unless the factory claims to support all decoders.
	std::map<int, webrtc::SdpAudioFormat> decoder_map;
	for (const AudioCodec& codec : codecs) {
		// Log a warning if a codec's payload type is changing. This used to be
		// treated as an error. It's abnormal, but not really illegal.
		AudioCodec old_codec;
		if (FindCodec(recv_codecs_, codec, &old_codec) &&
			old_codec.id != codec.id) {
			RTC_LOG(LS_WARNING) << codec.name << " mapped to a second payload type ("
				<< codec.id << ", was already mapped to "
				<< old_codec.id << ")";
		}
		auto format = AudioCodecToSdpAudioFormat(codec);
		auto existing = decoder_map_.find(codec.id);
		if (existing != decoder_map_.end() && !existing->second.Matches(format)) {
			RTC_LOG(LS_ERROR) << "Attempting to use payload type " << codec.id
				<< " for " << codec.name
				<< ", but it is already used for "
				<< existing->second.name;
			return false;
		}
		decoder_map.insert({ codec.id, std::move(format) });
	}

	if (decoder_map == decoder_map_) {
		// There's nothing new to configure.
		return true;
	}

	bool playout_enabled = playout_;
	// Receive codecs can not be changed while playing. So we temporarily
	// pause playout.
	SetPlayout(false);
	RTC_DCHECK(!playout_);

	decoder_map_ = std::move(decoder_map);
	for (auto& kv : recv_streams_) {
		kv.second->SetDecoderMap(decoder_map_);
	}

	recv_codecs_ = codecs;

	SetPlayout(playout_enabled);
	RTC_DCHECK_EQ(playout_, playout_enabled);

	return true;
}

bool RTCWebRtcVoiceMediaChannel::SetSendCodecs(
	const std::vector<AudioCodec>& codecs) {
	RTC_DCHECK_RUN_ON(worker_thread_);
// Scan through the list to figure out the codec to use for sending.
	absl::optional<webrtc::RTCAudioSendStream::Config::SendCodecSpec> send_codec_spec;
	for (const AudioCodec& voice_codec : codecs) {
		if (!(IsCodec(voice_codec, kCnCodecName) ||
			IsCodec(voice_codec, kDtmfCodecName) ||
			IsCodec(voice_codec, kRedCodecName))) {
			webrtc::SdpAudioFormat format(voice_codec.name, voice_codec.clockrate,
				voice_codec.channels, voice_codec.params);

			send_codec_spec = webrtc::RTCAudioSendStream::Config::SendCodecSpec(
				voice_codec.id, format);
			if (voice_codec.bitrate > 0) {
				send_codec_spec->target_bitrate_bps = voice_codec.bitrate;
			}
			send_codec_spec->transport_cc_enabled = HasTransportCc(voice_codec);
			send_codec_spec->nack_enabled = HasNack(voice_codec);
			break;
		}
	}

	if (!send_codec_spec) {
		return false;
	}

	send_codec_spec_ = std::move(send_codec_spec);

	recv_transport_cc_enabled_ = send_codec_spec->transport_cc_enabled;
	return true;
}

void RTCWebRtcVoiceMediaChannel::SetPlayout(bool playout) {
	RTC_DCHECK_RUN_ON(worker_thread_);
	if (playout_ == playout) {
		return;
	}

	for (const auto& kv : recv_streams_) {
		kv.second->SetPlayout(playout);
	}
	playout_ = playout;
}

void RTCWebRtcVoiceMediaChannel::SetSend(bool send) {
	if (send_ == send) {
		return;
	}

	// Apply channel specific options, and initialize the ADM for recording (this
	// may take time on some platforms, e.g. Android).
	if (send) {
		engine()->ApplyOptions(options_);
	}

	// Change the settings on each send channel.
	for (auto& kv : send_streams_) {
		kv.second->SetSend(send);
	}

	send_ = send;
}


bool RTCWebRtcVoiceMediaChannel::SetAudioSend(uint32_t ssrc,
	rtc::RTCAudioPacketRouteSourceInterface<std::unique_ptr<webrtc::RtpPacketReceived>>* source) {
	RTC_DCHECK_RUN_ON(worker_thread_);
	if (!SetLocalSource(ssrc, source)) {
		return false;
	}

	return true;
}

bool RTCWebRtcVoiceMediaChannel::AddSendStream(const StreamParams& sp) {
	RTC_DCHECK_RUN_ON(worker_thread_);

	uint32_t ssrc = sp.first_ssrc();
	RTC_DCHECK(0 != ssrc);

	if (send_streams_.find(ssrc) != send_streams_.end()) {
		RTC_LOG(LS_ERROR) << "Stream already exists with ssrc " << ssrc;
		return false;
	}

	RTCWebRtcAudioSendStream* stream = new RTCWebRtcAudioSendStream(
		ssrc, mid_, sp.cname, sp.id,
		send_codec_spec_,		
		ExtmapAllowMixed(),
		send_rtp_extensions_, 
		max_send_bitrate_bps_,
		audio_config_.rtcp_report_interval_ms, 
		call_,
		this, 
		crypto_options_);
	
	send_streams_.insert(std::make_pair(ssrc, stream));

	// At this point the stream's local SSRC has been updated. If it is the first
	// send stream, make sure that all the receive streams are updated with the
	// same SSRC in order to send receiver reports.
	if (send_streams_.size() == 1) {
		receiver_reports_ssrc_ = ssrc;
		for (const auto& kv : recv_streams_) {
			// TODO(solenberg): Allow applications to set the RTCP SSRC of receive
			// streams instead, so we can avoid reconfiguring the streams here.
			kv.second->SetLocalSsrc(ssrc);	// �̰� �� �ϴ°���...
		}
	}
	send_streams_[ssrc]->SetSend(send_);
	return true;
}

bool RTCWebRtcVoiceMediaChannel::RemoveSendStream(uint32_t ssrc) {
	RTC_DCHECK_RUN_ON(worker_thread_);
	RTC_LOG(LS_INFO) << "RemoveSendStream: " << ssrc;

	auto it = send_streams_.find(ssrc);
	if (it == send_streams_.end()) {
		RTC_LOG(LS_WARNING) << "Try to remove stream with ssrc " << ssrc
			<< " which doesn't exist.";
		return false;
	}

	it->second->SetSend(false);

	// TODO(solenberg): If we're removing the receiver_reports_ssrc_ stream, find
	// the first active send stream and use that instead, reassociating receive
	// streams.

	delete it->second;
	send_streams_.erase(it);
	if (send_streams_.empty()) {
		SetSend(false);
	}
	return true;
}

bool RTCWebRtcVoiceMediaChannel::AddRecvStream(const StreamParams& sp) {
	RTC_DCHECK_RUN_ON(worker_thread_);
	RTC_LOG(LS_INFO) << "AddRecvStream: " << sp.ToString();

	if (!sp.has_ssrcs()) {
		// This is a StreamParam with unsignaled SSRCs. Store it, so it can be used
		// later when we know the SSRCs on the first packet arrival.
		unsignaled_stream_params_ = sp;
		return true;
	}

	if (!ValidateStreamParams(sp)) {
		return false;
	}

	const uint32_t ssrc = sp.first_ssrc();

	// If this stream was previously received unsignaled, we promote it, possibly
	// recreating the AudioReceiveStream, if stream ids have changed.
	if (MaybeDeregisterUnsignaledRecvStream(ssrc)) {
		recv_streams_[ssrc]->MaybeRecreateAudioReceiveStream(sp.stream_ids());
		return true;
	}

	if (recv_streams_.find(ssrc) != recv_streams_.end()) {
		RTC_LOG(LS_ERROR) << "Stream already exists with ssrc " << ssrc;
		return false;
	}

	// Create a new channel for receiving audio data.
	recv_streams_.insert(std::make_pair(
		ssrc, new RTCWebRtcAudioReceiveStream(
		ssrc, receiver_reports_ssrc_, recv_transport_cc_enabled_,
		recv_nack_enabled_, sp.stream_ids(), recv_rtp_extensions_,
		call_,
		this, 
		decoder_map_,
		engine()->audio_jitter_buffer_max_packets_,
		engine()->audio_jitter_buffer_fast_accelerate_,
		engine()->audio_jitter_buffer_min_delay_ms_,
		engine()->audio_jitter_buffer_enable_rtx_handling_,
		crypto_options_)));
	recv_streams_[ssrc]->SetPlayout(playout_);

	return true;
}

bool RTCWebRtcVoiceMediaChannel::RemoveRecvStream(uint32_t ssrc) {
	RTC_DCHECK_RUN_ON(worker_thread_);

	const auto it = recv_streams_.find(ssrc);
	if (it == recv_streams_.end()) {
		RTC_LOG(LS_WARNING) << "Try to remove stream with ssrc " << ssrc
			<< " which doesn't exist.";
		return false;
	}

	MaybeDeregisterUnsignaledRecvStream(ssrc);

	it->second->SetRawAudioSink(nullptr);
	delete it->second;
	recv_streams_.erase(it);
	return true;
}

void RTCWebRtcVoiceMediaChannel::ResetUnsignaledRecvStream() {
	RTC_DCHECK_RUN_ON(worker_thread_);
	RTC_LOG(LS_INFO) << "ResetUnsignaledRecvStream.";
	unsignaled_stream_params_ = StreamParams();
	// Create a copy since RemoveRecvStream will modify |unsignaled_recv_ssrcs_|.
	std::vector<uint32_t> to_remove = unsignaled_recv_ssrcs_;
	for (uint32_t ssrc : to_remove) {
		RemoveRecvStream(ssrc);
	}
}

bool RTCWebRtcVoiceMediaChannel::SetLocalSource(
	uint32_t ssrc,
	rtc::RTCAudioPacketRouteSourceInterface<std::unique_ptr<webrtc::RtpPacketReceived>>* source) {

	auto it = send_streams_.find(ssrc);
	if (it == send_streams_.end()) {
		if (source) {
			// Return an error if trying to set a valid source with an invalid ssrc.
			RTC_LOG(LS_ERROR) << "SetLocalSource failed with ssrc " << ssrc;
			return false;
		}

		// The channel likely has gone away, do nothing.
		return true;
	}

	if (source) {
		it->second->SetSource(source);	// send stream�� local source ����
	}
	else {
		it->second->ClearSource();
	}

	return true;
}

bool RTCWebRtcVoiceMediaChannel::SetBaseMinimumPlayoutDelayMs(uint32_t ssrc,
	int delay_ms) {
	RTC_DCHECK_RUN_ON(worker_thread_);
	std::vector<uint32_t> ssrcs(1, ssrc);
	// SSRC of 0 represents the default receive stream.
	if (ssrc == 0) {
		default_recv_base_minimum_delay_ms_ = delay_ms;
		ssrcs = unsignaled_recv_ssrcs_;
	}
	for (uint32_t ssrc : ssrcs) {
		const auto it = recv_streams_.find(ssrc);
		if (it == recv_streams_.end()) {
			RTC_LOG(LS_WARNING) << "SetBaseMinimumPlayoutDelayMs: no recv stream "
				<< ssrc;
			return false;
		}
		it->second->SetBaseMinimumPlayoutDelayMs(delay_ms);
		RTC_LOG(LS_INFO) << "SetBaseMinimumPlayoutDelayMs() to " << delay_ms
			<< " for recv stream with ssrc " << ssrc;
	}
	return true;
}

absl::optional<int> RTCWebRtcVoiceMediaChannel::GetBaseMinimumPlayoutDelayMs(
	uint32_t ssrc) const {
	// SSRC of 0 represents the default receive stream.
	if (ssrc == 0) {
		return default_recv_base_minimum_delay_ms_;
	}

	const auto it = recv_streams_.find(ssrc);

	if (it != recv_streams_.end()) {
		return it->second->GetBaseMinimumPlayoutDelayMs();
	}
	return absl::nullopt;
}

void RTCWebRtcVoiceMediaChannel::OnPacketReceived(rtc::CopyOnWriteBuffer packet,
												  int64_t packet_time_us) {
	RTC_DCHECK_RUN_ON(&network_thread_checker_);
	// TODO(bugs.webrtc.org/11993): This code is very similar to what
	// WebRtcVideoChannel::OnPacketReceived does. For maintainability and
	// consistency it would be good to move the interaction with call_->Receiver()
	// to a common implementation and provide a callback on the worker thread
	// for the exception case (DELIVERY_UNKNOWN_SSRC) and how retry is attempted.
#ifdef __ROUTING_PACKET_PATH_USING_NETWORK_THREAD__
	webrtc::PacketReceiver::DeliveryStatus delivery_result =
		call_->Receiver()->DeliverPacket(webrtc::MediaType::AUDIO, packet,
		packet_time_us);

	if (delivery_result != webrtc::PacketReceiver::DELIVERY_UNKNOWN_SSRC) {
		return;
	}

	// Create an unsignaled receive stream for this previously not received
	// ssrc. If there already is N unsignaled receive streams, delete the
	// oldest. See: https://bugs.chromium.org/p/webrtc/issues/detail?id=5208
	uint32_t ssrc = 0;
	if (!GetRtpSsrc(packet.cdata(), packet.size(), &ssrc)) {
		return;
	}
	RTC_DCHECK(!absl::c_linear_search(unsignaled_recv_ssrcs_, ssrc));

	// Add new stream.
	StreamParams sp = unsignaled_stream_params_;
	sp.ssrcs.push_back(ssrc);
	if (!AddRecvStream(sp)) {
		RTC_LOG(LS_WARNING) << "Could not create unsignaled receive stream.";
		return;
	}
	unsignaled_recv_ssrcs_.push_back(ssrc);
	RTC_HISTOGRAM_COUNTS_LINEAR("WebRTC.Audio.NumOfUnsignaledStreams",
		unsignaled_recv_ssrcs_.size(), 1, 100, 101);

	// Remove oldest unsignaled stream, if we have too many.
	if (unsignaled_recv_ssrcs_.size() > kMaxUnsignaledRecvStreams) {
		uint32_t remove_ssrc = unsignaled_recv_ssrcs_.front();
		RTC_DLOG(LS_INFO) << "Removing unsignaled receive stream with SSRC="
			<< remove_ssrc;
		RemoveRecvStream(remove_ssrc);
	}
	RTC_DCHECK_GE(kMaxUnsignaledRecvStreams, unsignaled_recv_ssrcs_.size());

	SetBaseMinimumPlayoutDelayMs(ssrc, default_recv_base_minimum_delay_ms_);

	delivery_result = call_->Receiver()->DeliverPacket(webrtc::MediaType::AUDIO,
		packet, packet_time_us);
	RTC_DCHECK_NE(webrtc::PacketReceiver::DELIVERY_UNKNOWN_SSRC,
		delivery_result);
#else
	worker_thread_->PostTask(SafeTask(task_safety_.flag(), [this, packet,
		packet_time_us] {
		RTC_DCHECK_RUN_ON(worker_thread_);

		webrtc::PacketReceiver::DeliveryStatus delivery_result =
			call_->Receiver()->DeliverPacket(webrtc::MediaType::AUDIO, packet,
			packet_time_us);

		if (delivery_result != webrtc::PacketReceiver::DELIVERY_UNKNOWN_SSRC) {
			return;
		}

		// Create an unsignaled receive stream for this previously not received
		// ssrc. If there already is N unsignaled receive streams, delete the
		// oldest. See: https://bugs.chromium.org/p/webrtc/issues/detail?id=5208
		uint32_t ssrc = 0;
		if (!GetRtpSsrc(packet.cdata(), packet.size(), &ssrc)) {
			return;
		}
		RTC_DCHECK(!absl::c_linear_search(unsignaled_recv_ssrcs_, ssrc));

		// Add new stream.
		StreamParams sp = unsignaled_stream_params_;
		sp.ssrcs.push_back(ssrc);
		if (!AddRecvStream(sp)) {
			RTC_LOG(LS_WARNING) << "Could not create unsignaled receive stream.";
			return;
		}
		unsignaled_recv_ssrcs_.push_back(ssrc);
		RTC_HISTOGRAM_COUNTS_LINEAR("WebRTC.Audio.NumOfUnsignaledStreams",
			unsignaled_recv_ssrcs_.size(), 1, 100, 101);

		// Remove oldest unsignaled stream, if we have too many.
		if (unsignaled_recv_ssrcs_.size() > kMaxUnsignaledRecvStreams) {
			uint32_t remove_ssrc = unsignaled_recv_ssrcs_.front();
			RTC_DLOG(LS_INFO) << "Removing unsignaled receive stream with SSRC="
				<< remove_ssrc;
			RemoveRecvStream(remove_ssrc);
		}
		RTC_DCHECK_GE(kMaxUnsignaledRecvStreams, unsignaled_recv_ssrcs_.size());

		SetBaseMinimumPlayoutDelayMs(ssrc, default_recv_base_minimum_delay_ms_);

		delivery_result = call_->Receiver()->DeliverPacket(webrtc::MediaType::AUDIO,
			packet, packet_time_us);
		RTC_DCHECK_NE(webrtc::PacketReceiver::DELIVERY_UNKNOWN_SSRC,
			delivery_result);

	}));
#endif
}

void RTCWebRtcVoiceMediaChannel::OnNetworkRouteChanged(
	const std::string& transport_name,
	const rtc::NetworkRoute& network_route) {
	RTC_DCHECK_RUN_ON(worker_thread_);

	call_->OnAudioTransportOverheadChanged(network_route.packet_overhead);
}

bool RTCWebRtcVoiceMediaChannel::MuteStream(uint32_t ssrc, bool muted) {
	RTC_DCHECK_RUN_ON(worker_thread_);
	const auto it = send_streams_.find(ssrc);
	if (it == send_streams_.end()) {
		RTC_LOG(LS_WARNING) << "The specified ssrc " << ssrc << " is not in use.";
		return false;
	}

	return true;
}

void RTCWebRtcVoiceMediaChannel::OnReadyToSend(bool ready) {
	RTC_DCHECK_RUN_ON(worker_thread_);
	RTC_LOG(LS_VERBOSE) << "OnReadyToSend: " << (ready ? "Ready." : "Not ready.");

	call_->SignalChannelNetworkState(
		webrtc::MediaType::AUDIO,
		ready ? webrtc::kNetworkUp : webrtc::kNetworkDown);
}

bool RTCWebRtcVoiceMediaChannel::GetStats(VoiceMediaInfo* info,
										  bool get_and_clear_legacy_stats) {
	RTC_DCHECK_RUN_ON(worker_thread_);
	RTC_DCHECK(info);

	// Get SSRC and stats for each sender.
	RTC_DCHECK_EQ(info->senders.size(), 0U);
	for (const auto& stream : send_streams_) {
		webrtc::RTCAudioSendStream::Stats stats =
			stream.second->GetStats(recv_streams_.size() > 0);
		VoiceSenderInfo sinfo;
		sinfo.add_ssrc(stats.local_ssrc);
		sinfo.payload_bytes_sent = stats.payload_bytes_sent;
		sinfo.header_and_padding_bytes_sent = stats.header_and_padding_bytes_sent;
		sinfo.retransmitted_bytes_sent = stats.retransmitted_bytes_sent;
		sinfo.packets_sent = stats.packets_sent;
		sinfo.retransmitted_packets_sent = stats.retransmitted_packets_sent;
		sinfo.packets_lost = stats.packets_lost;
		sinfo.fraction_lost = stats.fraction_lost;
		sinfo.codec_name = stats.codec_name;
		sinfo.codec_payload_type = stats.codec_payload_type;
		sinfo.jitter_ms = stats.jitter_ms;
		sinfo.rtt_ms = stats.rtt_ms;
		sinfo.audio_level = stats.audio_level;
		sinfo.total_input_energy = stats.total_input_energy;
		sinfo.total_input_duration = stats.total_input_duration;
		sinfo.typing_noise_detected = (send_ ? stats.typing_noise_detected : false);
		sinfo.report_block_datas = std::move(stats.report_block_datas);
		info->senders.push_back(sinfo);
	}

	// Get SSRC and stats for each receiver.
	RTC_DCHECK_EQ(info->receivers.size(), 0U);

	// Get codec info
	for (const AudioCodec& codec : send_codecs_) {
		webrtc::RtpCodecParameters codec_params = codec.ToCodecParameters();
		info->send_codecs.insert(
			std::make_pair(codec_params.payload_type, std::move(codec_params)));
	}
	for (const AudioCodec& codec : recv_codecs_) {
		webrtc::RtpCodecParameters codec_params = codec.ToCodecParameters();
		info->receive_codecs.insert(
			std::make_pair(codec_params.payload_type, std::move(codec_params)));
	}

	return true;
}

std::vector<webrtc::RtpSource> RTCWebRtcVoiceMediaChannel::GetSources(
	uint32_t ssrc) const {
	auto it = recv_streams_.find(ssrc);
	if (it == recv_streams_.end()) {
		RTC_LOG(LS_ERROR) << "Attempting to get contributing sources for SSRC:"
			<< ssrc << " which doesn't exist.";
		return std::vector<webrtc::RtpSource>();
	}
	return it->second->GetSources();
}

bool RTCWebRtcVoiceMediaChannel::MaybeDeregisterUnsignaledRecvStream(
	uint32_t ssrc) {
	RTC_DCHECK_RUN_ON(worker_thread_);
	auto it = absl::c_find(unsignaled_recv_ssrcs_, ssrc);
	if (it != unsignaled_recv_ssrcs_.end()) {
		unsignaled_recv_ssrcs_.erase(it);
		return true;
	}
	return false;
}

} // namespace cricket
