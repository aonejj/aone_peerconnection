//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /media/engine/webrtc_video_engine.cc
//
//////////////////////////////////////////////////////////////////////////

#include <stdio.h>

#include <algorithm>
#include <set>
#include <string>
#include <utility>

#include <vector>
#include <syscall.h>

#include "absl/algorithm/container.h"
#include "absl/strings/match.h"

#include "api/rtc_error.h"
#include "rtc_base/logging.h"
#include "media/base/rtp_utils.h"
#include "media/engine/constants.h"

#include "RTCWebRTCVideoEngine.h"
//#include "../../../src_update/media/engine/_webrtc_media_engine.h"
#include "../../../src_update/media/engine/_webrtc_media_engine_info.h"
#include "../../../src_update/api/video_codecs/_video_codec.h"



namespace cricket {

static const int kNackHistoryMs = 1000;
static const int kDefaultRtcpReceiverReportSsrc = 1;

void AddDefaultFeedbackParams(VideoCodec* codec) {
	// Don't add any feedback params for RED and ULPFEC.
	if (codec->name == kRedCodecName || codec->name == kUlpfecCodecName)
		return;
	codec->AddFeedbackParam(FeedbackParam(kRtcpFbParamRemb, kParamValueEmpty));
	codec->AddFeedbackParam(
		FeedbackParam(kRtcpFbParamTransportCc, kParamValueEmpty));
	// Don't add any more feedback params for FLEXFEC.
	if (codec->name == kFlexfecCodecName)
		return;
	codec->AddFeedbackParam(FeedbackParam(kRtcpFbParamCcm, kRtcpFbCcmParamFir));
	codec->AddFeedbackParam(FeedbackParam(kRtcpFbParamNack, kParamValueEmpty));
	codec->AddFeedbackParam(FeedbackParam(kRtcpFbParamNack, kRtcpFbNackParamPli));
	if (codec->name == kVp8CodecName /* &&
										IsEnabled(trials, "WebRTC-RtcpLossNotification") */) {
		codec->AddFeedbackParam(FeedbackParam(kRtcpFbParamLntf, kParamValueEmpty));
	}
}

bool IsTemporalLayersSupported(const std::string& codec_name) {
	return absl::EqualsIgnoreCase(codec_name, kVp8CodecName) ||
		absl::EqualsIgnoreCase(codec_name, kVp9CodecName);
}

static std::string CodecVectorToString(const std::vector<VideoCodec>& codecs) {
	rtc::StringBuilder out;
	out << "{";
	for (size_t i = 0; i < codecs.size(); ++i) {
		out << codecs[i].ToString();
		if (i != codecs.size() - 1) {
			out << ", ";
		}
	}
	out << "}";
	return out.Release();
}

static bool ValidateCodecFormats(const std::vector<VideoCodec>& codecs) {
	bool has_video = false;
	for (size_t i = 0; i < codecs.size(); ++i) {
		if (!codecs[i].ValidateCodecFormat()) {
			return false;
		}
		if (codecs[i].GetCodecType() == VideoCodec::CODEC_VIDEO) {
			has_video = true;
		}
	}
	if (!has_video) {
		RTC_LOG(LS_ERROR) << "Setting codecs without a video codec is invalid: "
			<< CodecVectorToString(codecs);
		return false;
	}
	return true;
}

static bool ValidateStreamParams(const StreamParams& sp) {
	if (sp.ssrcs.empty()) {
		RTC_LOG(LS_ERROR) << "No SSRCs in stream parameters: " << sp.ToString();
		return false;
	}

	std::vector<uint32_t> primary_ssrcs;
	sp.GetPrimarySsrcs(&primary_ssrcs);
	std::vector<uint32_t> rtx_ssrcs;
	sp.GetFidSsrcs(primary_ssrcs, &rtx_ssrcs);
	for (uint32_t rtx_ssrc : rtx_ssrcs) {
		bool rtx_ssrc_present = false;
		for (uint32_t sp_ssrc : sp.ssrcs) {
			if (sp_ssrc == rtx_ssrc) {
				rtx_ssrc_present = true;
				break;
			}
		}
		if (!rtx_ssrc_present) {
			RTC_LOG(LS_ERROR) << "RTX SSRC '" << rtx_ssrc
				<< "' missing from StreamParams ssrcs: "
				<< sp.ToString();
			return false;
		}
	}
	if (!rtx_ssrcs.empty() && primary_ssrcs.size() != rtx_ssrcs.size()) {
		RTC_LOG(LS_ERROR)
			<< "RTX SSRCs exist, but don't cover all SSRCs (unsupported): "
			<< sp.ToString();
		return false;
	}

	return true;
}

bool IsLayerActive(const webrtc::RtpEncodingParameters& layer) {
	return layer.active &&
		(!layer.max_bitrate_bps || *layer.max_bitrate_bps > 0) &&
		(!layer.max_framerate || *layer.max_framerate > 0);
}

DefaultUnsignalledSsrcHandler::DefaultUnsignalledSsrcHandler() {}

UnsignalledSsrcHandler::Action DefaultUnsignalledSsrcHandler::OnUnsignalledSsrc(
	RTCWebRtcVideoChannel* channel,
	uint32_t ssrc) {
	absl::optional<uint32_t> default_recv_ssrc =
		channel->GetDefaultReceiveStreamSsrc();

	if (default_recv_ssrc) {
		RTC_LOG(LS_INFO) << "Destroying old default receive stream for SSRC="
			<< ssrc << ".";
		channel->RemoveRecvStream(*default_recv_ssrc);
	}

	StreamParams sp = channel->unsignaled_stream_params();
	sp.ssrcs.push_back(ssrc);

	RTC_LOG(LS_INFO) << "Creating default receive stream for SSRC=" << ssrc
		<< ".";
	if (!channel->AddRecvStream(sp, /*default_stream=*/true)) {
		RTC_LOG(LS_WARNING) << "Could not create default receive stream.";
	}

	// SSRC 0 returns default_recv_base_minimum_delay_ms.
	const int unsignaled_ssrc = 0;
	int default_recv_base_minimum_delay_ms =
		channel->GetBaseMinimumPlayoutDelayMs(unsignaled_ssrc).value_or(0);
	// Set base minimum delay if it was set before for the default receive stream.
	channel->SetBaseMinimumPlayoutDelayMs(ssrc,
		default_recv_base_minimum_delay_ms);
	return kDeliverPacket;
}

RTCWebRtcVideoEngine::RTCWebRtcVideoEngine(
	std::vector<VideoCodec> video_decoder_codecs,
	std::vector<VideoCodec> video_encoder_codecs) 
	: recv_codecs_(video_encoder_codecs),
	  send_codecs_(video_encoder_codecs) {
	RTC_LOG(LS_INFO) << "WebRtcVideoEngine::WebRtcVideoEngine()";
}

RTCWebRtcVideoEngine::~RTCWebRtcVideoEngine() {
	RTC_LOG(LS_INFO) << "WebRtcVideoEngine::~WebRtcVideoEngine";
}

VideoMediaChannel* RTCWebRtcVideoEngine::CreateMediaChannel(
	webrtc::RTCCall* call,
	const MediaConfig& config,
	const webrtc::CryptoOptions& crypto_options,
	rtc::RTCThreadManagerInterface* rtc_thread_manager
	) {
	RTC_LOG(LS_INFO) << "CreateMediaChannel";
	return new RTCWebRtcVideoChannel(call, config, crypto_options, 
								  recv_codecs_, send_codecs_
								  , rtc_thread_manager
								  );
}

std::vector<VideoCodec> RTCWebRtcVideoEngine::send_codecs() const {
	return send_codecs_;
}

std::vector<VideoCodec> RTCWebRtcVideoEngine::recv_codecs() const {
	return recv_codecs_;
}

std::vector<webrtc::RtpHeaderExtensionCapability>
RTCWebRtcVideoEngine::GetRtpHeaderExtensions() const {
	std::vector<webrtc::RtpHeaderExtensionCapability> result;
	int id = 1;
	for (const auto& uri :
		{ webrtc::RtpExtension::kTimestampOffsetUri,
		webrtc::RtpExtension::kAbsSendTimeUri,
		webrtc::RtpExtension::kVideoRotationUri,
		webrtc::RtpExtension::kTransportSequenceNumberUri,
		webrtc::RtpExtension::kPlayoutDelayUri,
		webrtc::RtpExtension::kVideoContentTypeUri,
		webrtc::RtpExtension::kVideoTimingUri,
		webrtc::RtpExtension::kColorSpaceUri, webrtc::RtpExtension::kMidUri,
		webrtc::RtpExtension::kRidUri, webrtc::RtpExtension::kRepairedRidUri }) {
		result.emplace_back(uri, id++, webrtc::RtpTransceiverDirection::kSendRecv);
	}
	result.emplace_back(
		webrtc::RtpExtension::kGenericFrameDescriptorUri00, id++,
		webrtc::RtpTransceiverDirection::kStopped);
	result.emplace_back(
		webrtc::RtpExtension::kDependencyDescriptorUri, id++,
		webrtc::RtpTransceiverDirection::kStopped);
	result.emplace_back(
		webrtc::RtpExtension::kVideoLayersAllocationUri, id++,
		webrtc::RtpTransceiverDirection::kStopped);

	return result;
}

RTCWebRtcVideoChannel::RTCWebRtcVideoChannel(
	webrtc::RTCCall* call,
	const MediaConfig& config,
	const webrtc::CryptoOptions& crypto_options,
	std::vector<VideoCodec> video_decoder_codecs,
	std::vector<VideoCodec> video_encoder_codecs
	, rtc::RTCThreadManagerInterface* rtc_thread_manager
	)
	: VideoMediaChannel(config),
	worker_thread_(rtc::Thread::Current(rtc_thread_manager)),
	call_(call),
	unsignalled_ssrc_handler_(&default_unsignalled_ssrc_handler_),
	discard_unknown_ssrc_packets_(true),
	crypto_options_(crypto_options),
//	unknown_ssrc_packet_buffer_(new UnhandledPacketsBuffer()),	// kimi  unused
	video_decoder_codecs_(video_decoder_codecs),
	video_encoder_codecs_(video_encoder_codecs)
	, _rtc_thread_manager(rtc_thread_manager)
{

	RTC_DCHECK_RUN_ON(&thread_checker_);
	network_thread_checker_.Detach();

	rtcp_receiver_report_ssrc_ = kDefaultRtcpReceiverReportSsrc;
	sending_ = false;

	recv_codecs_ = MapCodecs(video_decoder_codecs);

	recv_flexfec_payload_type_ =
		recv_codecs_.empty() ? 0 : recv_codecs_.front().flexfec_payload_type;
}

RTCWebRtcVideoChannel::~RTCWebRtcVideoChannel() {
	RTC_LOG(LS_VERBOSE) << "RTCWebRtcVideoChannel::~RTCWebRtcVideoChannel";
	for (auto& kv : send_streams_)
		delete kv.second;
	for (auto& kv : receive_streams_)
		delete kv.second;
}

std::vector<RTCWebRtcVideoChannel::VideoCodecSettings>
RTCWebRtcVideoChannel::SelectSendVideoCodecs(
	const std::vector<VideoCodecSettings>& remote_mapped_codecs) const {
	std::vector<webrtc::SdpVideoFormat> sdp_formats = std::vector<webrtc::SdpVideoFormat>();

	for (const VideoCodec &codec : video_encoder_codecs_) {
		webrtc::SdpVideoFormat format(codec.name, codec.params);
		sdp_formats.push_back(format);
	}

	// The returned vector holds the VideoCodecSettings in term of preference.
	// They are orderd by receive codec preference first and local implementation
	// preference second.
	std::vector<VideoCodecSettings> encoders;
	for (const VideoCodecSettings& remote_codec : remote_mapped_codecs) {
		for (auto format_it = sdp_formats.begin();
			format_it != sdp_formats.end();) {
			// For H264, we will limit the encode level to the remote offered level
			// regardless if level asymmetry is allowed or not. This is strictly not
			// following the spec in https://tools.ietf.org/html/rfc6184#section-8.2.2
			// since we should limit the encode level to the lower of local and remote
			// level when level asymmetry is not allowed.
			if (IsSameCodec(format_it->name, format_it->parameters,
				remote_codec.codec.name, remote_codec.codec.params)) {
				encoders.push_back(remote_codec);

				// To allow the VideoEncoderFactory to keep information about which
				// implementation to instantitate when CreateEncoder is called the two
				// parmeter sets are merged.
				encoders.back().codec.params.insert(format_it->parameters.begin(),
					format_it->parameters.end());

				format_it = sdp_formats.erase(format_it);
			}
			else {
				++format_it;
			}
		}
	}

	return encoders;
}

bool RTCWebRtcVideoChannel::SetSendParameters(const VideoSendParameters& params) {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	RTC_LOG(LS_INFO) << "SetSendParameters: " << params.ToString();

	ChangedSendParameters changed_params;
	if (!GetChangedSendParameters(params, &changed_params)) {
		return false;
	}

	if (changed_params.negotiated_codecs) {
		for (const auto& send_codec : *changed_params.negotiated_codecs)
			RTC_LOG(LS_INFO) << "Negotiated codec: " << send_codec.codec.ToString();
	}

	send_params_ = params;

	return ApplyChangedParams(changed_params);
}

bool RTCWebRtcVideoChannel::ApplyChangedParams(
	const ChangedSendParameters& changed_params) {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	if (changed_params.negotiated_codecs)
		negotiated_codecs_ = *changed_params.negotiated_codecs;

	if (changed_params.send_codec)
		send_codec_ = changed_params.send_codec;

	if (changed_params.extmap_allow_mixed) {
		SetExtmapAllowMixed(*changed_params.extmap_allow_mixed);
	}
	if (changed_params.rtp_header_extensions) {
		send_rtp_extensions_ = changed_params.rtp_header_extensions;
	}

	for (auto& kv : send_streams_) {
		kv.second->SetSendParameters(changed_params);
	}
	if (changed_params.send_codec || changed_params.rtcp_mode) {
		// Update receive feedback parameters from new codec or RTCP mode.
		RTC_LOG(LS_INFO)
			<< "SetFeedbackOptions on all the receive streams because the send "
			"codec or RTCP mode has changed.";
		for (auto& kv : receive_streams_) {
			RTC_DCHECK(kv.second != nullptr);
			kv.second->SetFeedbackParameters(
				HasLntf(send_codec_->codec), HasNack(send_codec_->codec),
				HasTransportCc(send_codec_->codec),
				send_params_.rtcp.reduced_size ? webrtc::RtcpMode::kReducedSize
				: webrtc::RtcpMode::kCompound);
		}
	}
	return true;
}

bool RTCWebRtcVideoChannel::GetChangedSendParameters(
	const VideoSendParameters& params,
	ChangedSendParameters* changed_params) const {
	if (!ValidateCodecFormats(params.codecs) ||
		!ValidateRtpExtensions(params.extensions)) {
		return false;
	}

	std::vector<VideoCodecSettings> negotiated_codecs =
		SelectSendVideoCodecs(MapCodecs(params.codecs));

	// We should only fail here if send direction is enabled.
	if (params.is_stream_active && negotiated_codecs.empty()) {
		RTC_LOG(LS_ERROR) << "No video codecs supported.";
		return false;
	}

	if (negotiated_codecs_ != negotiated_codecs) {
		if (negotiated_codecs.empty()) {
			changed_params->send_codec = absl::nullopt;
		}
		else if (send_codec_ != negotiated_codecs.front()) {
			changed_params->send_codec = negotiated_codecs.front();
		}
		changed_params->negotiated_codecs = std::move(negotiated_codecs);
	}

	// Handle RTP header extensions.
	if (params.extmap_allow_mixed != ExtmapAllowMixed()) {
		changed_params->extmap_allow_mixed = params.extmap_allow_mixed;
	}
	std::vector<webrtc::RtpExtension> filtered_extensions = FilterRtpExtensions(
		params.extensions, webrtc::RtpExtension::IsSupportedForVideo, true);
	if (!send_rtp_extensions_ || (*send_rtp_extensions_ != filtered_extensions)) {
		changed_params->rtp_header_extensions =
			absl::optional<std::vector<webrtc::RtpExtension>>(filtered_extensions);
	}

	if (params.mid != send_params_.mid) {
		changed_params->mid = params.mid;
	}

	// Handle max bitrate.
	if (params.max_bandwidth_bps != send_params_.max_bandwidth_bps &&
		params.max_bandwidth_bps >= -1) {
		// 0 or -1 uncaps max bitrate.
		// TODO(pbos): Reconsider how 0 should be treated. It is not mentioned as a
		// special value and might very well be used for stopping sending.
		changed_params->max_bandwidth_bps =
			params.max_bandwidth_bps == 0 ? -1 : params.max_bandwidth_bps;
	}

	// Handle conference mode.
	if (params.conference_mode != send_params_.conference_mode) {
		changed_params->conference_mode = params.conference_mode;
	}

	// Handle RTCP mode.
	if (params.rtcp.reduced_size != send_params_.rtcp.reduced_size) {
		changed_params->rtcp_mode = params.rtcp.reduced_size
			? webrtc::RtcpMode::kReducedSize
			: webrtc::RtcpMode::kCompound;
	}

	return true;
}

webrtc::RtpParameters RTCWebRtcVideoChannel::GetRtpSendParameters(
	uint32_t ssrc) const {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	auto it = send_streams_.find(ssrc);
	if (it == send_streams_.end()) {
		RTC_LOG(LS_WARNING) << "Attempting to get RTP send parameters for stream "
			"with ssrc "
			<< ssrc << " which doesn't exist.";
		return webrtc::RtpParameters();
	}

	webrtc::RtpParameters rtp_params = it->second->GetRtpParameters();
	// Need to add the common list of codecs to the send stream-specific
	// RTP parameters.
	for (const VideoCodec& codec : send_params_.codecs) {
		rtp_params.codecs.push_back(codec.ToCodecParameters());
	}

	return rtp_params;
}

webrtc::RTCError RTCWebRtcVideoChannel::SetRtpSendParameters(
	uint32_t ssrc,
	const webrtc::RtpParameters& parameters) {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	auto it = send_streams_.find(ssrc);
	if (it == send_streams_.end()) {
		RTC_LOG(LS_ERROR) << "Attempting to set RTP send parameters for stream "
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
		return webrtc::RTCError(webrtc::RTCErrorType::INTERNAL_ERROR);
	}

	if (!parameters.encodings.empty()) {
		// Note that these values come from:
		// https://tools.ietf.org/html/draft-ietf-tsvwg-rtcweb-qos-16#section-5
		// TODO(deadbeef): Change values depending on whether we are sending a
		// keyframe or non-keyframe.
		rtc::DiffServCodePoint new_dscp = rtc::DSCP_DEFAULT;
		switch (parameters.encodings[0].network_priority) {
		case webrtc::Priority::kVeryLow:
			new_dscp = rtc::DSCP_CS1;
			break;
		case webrtc::Priority::kLow:
			new_dscp = rtc::DSCP_DEFAULT;
			break;
		case webrtc::Priority::kMedium:
			new_dscp = rtc::DSCP_AF42;
			break;
		case webrtc::Priority::kHigh:
			new_dscp = rtc::DSCP_AF41;
			break;
		}
		SetPreferredDscp(new_dscp);
	}

	return it->second->SetRtpParameters(parameters);
}

webrtc::RtpParameters RTCWebRtcVideoChannel::GetRtpReceiveParameters(
	uint32_t ssrc) const {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	webrtc::RtpParameters rtp_params;
	auto it = receive_streams_.find(ssrc);
	if (it == receive_streams_.end()) {
		RTC_LOG(LS_WARNING)
			<< "Attempting to get RTP receive parameters for stream "
			"with SSRC "
			<< ssrc << " which doesn't exist.";
		return webrtc::RtpParameters();
	}

	 rtp_params = it->second->GetRtpParameters();

	// Add codecs, which any stream is prepared to receive.
	for (const VideoCodec& codec : recv_params_.codecs) {
		rtp_params.codecs.push_back(codec.ToCodecParameters());
	}

	return rtp_params;
}

webrtc::RtpParameters RTCWebRtcVideoChannel::GetDefaultRtpReceiveParameters()
	const {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	webrtc::RtpParameters rtp_params;

	rtp_params.encodings.emplace_back();

	// Add codecs, which any stream is prepared to receive.
	for (const VideoCodec& codec : recv_params_.codecs) {
		rtp_params.codecs.push_back(codec.ToCodecParameters());
	}

	return rtp_params;
}

bool RTCWebRtcVideoChannel::NonFlexfecReceiveCodecsHaveChanged(
	std::vector<VideoCodecSettings> before,
	std::vector<VideoCodecSettings> after) {
	// The receive codec order doesn't matter, so we sort the codecs before
	// comparing. This is necessary because currently the
	// only way to change the send codec is to munge SDP, which causes
	// the receive codec list to change order, which causes the streams
	// to be recreates which causes a "blink" of black video.  In order
	// to support munging the SDP in this way without recreating receive
	// streams, we ignore the order of the received codecs so that
	// changing the order doesn't cause this "blink".
	auto comparison = [](const VideoCodecSettings& codec1,
		const VideoCodecSettings& codec2) {
		return codec1.codec.id > codec2.codec.id;
	};
	absl::c_sort(before, comparison);
	absl::c_sort(after, comparison);

	// Changes in FlexFEC payload type are handled separately in
	// WebRtcVideoChannel::GetChangedRecvParameters, so disregard FlexFEC in the
	// comparison here.
	return !absl::c_equal(before, after,
		VideoCodecSettings::EqualsDisregardingFlexfec);
}

bool RTCWebRtcVideoChannel::GetChangedRecvParameters(
	const VideoRecvParameters& params,
	ChangedRecvParameters* changed_params) const {
	if (!ValidateCodecFormats(params.codecs) ||
		!ValidateRtpExtensions(params.extensions)) {
		return false;
	}

	// Handle receive codecs.
	const std::vector<VideoCodecSettings> mapped_codecs =
		MapCodecs(params.codecs);
	if (mapped_codecs.empty()) {
		RTC_LOG(LS_ERROR) << "SetRecvParameters called without any video codecs.";
		return false;
	}

	// Verify that every mapped codec is supported locally.
	if (params.is_stream_active) {
		const std::vector<VideoCodec> local_supported_codecs = video_decoder_codecs_;
		for (const VideoCodecSettings& mapped_codec : mapped_codecs) {
			if (!FindMatchingCodec(local_supported_codecs, mapped_codec.codec)) {
				RTC_LOG(LS_ERROR)
					<< "SetRecvParameters called with unsupported video codec: "
					<< mapped_codec.codec.ToString();
				return false;
			}
		}
	}

	if (NonFlexfecReceiveCodecsHaveChanged(recv_codecs_, mapped_codecs)) {
		changed_params->codec_settings =
			absl::optional<std::vector<VideoCodecSettings>>(mapped_codecs);
	}

	// Handle RTP header extensions.
	std::vector<webrtc::RtpExtension> filtered_extensions = FilterRtpExtensions(
		params.extensions, webrtc::RtpExtension::IsSupportedForVideo, false);
	if (filtered_extensions != recv_rtp_extensions_) {
		changed_params->rtp_header_extensions =
			absl::optional<std::vector<webrtc::RtpExtension>>(filtered_extensions);
	}

	int flexfec_payload_type = mapped_codecs.front().flexfec_payload_type;
	if (flexfec_payload_type != recv_flexfec_payload_type_) {
		changed_params->flexfec_payload_type = flexfec_payload_type;
	}

	return true;
}

bool RTCWebRtcVideoChannel::SetRecvParameters(const VideoRecvParameters& params) {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	RTC_LOG(LS_INFO) << "SetRecvParameters: " << params.ToString();
	ChangedRecvParameters changed_params;
	if (!GetChangedRecvParameters(params, &changed_params)) {
		return false;
	}
	if (changed_params.flexfec_payload_type) {
		RTC_LOG(LS_INFO) << "Changing FlexFEC payload type (recv) from "
			<< recv_flexfec_payload_type_ << " to "
			<< *changed_params.flexfec_payload_type;
		recv_flexfec_payload_type_ = *changed_params.flexfec_payload_type;
	}
	if (changed_params.rtp_header_extensions) {
		recv_rtp_extensions_ = *changed_params.rtp_header_extensions;
	}
	if (changed_params.codec_settings) {
		RTC_LOG(LS_INFO) << "Changing recv codecs from "
			<< CodecSettingsVectorToString(recv_codecs_) << " to "
			<< CodecSettingsVectorToString(
			*changed_params.codec_settings);
		recv_codecs_ = *changed_params.codec_settings;
	}

	for (auto& kv : receive_streams_) {
		kv.second->SetRecvParameters(changed_params);
	}
	recv_params_ = params;
	return true;
}

std::string RTCWebRtcVideoChannel::CodecSettingsVectorToString(
	const std::vector<VideoCodecSettings>& codecs) {
	rtc::StringBuilder out;
	out << "{";
	for (size_t i = 0; i < codecs.size(); ++i) {
		out << codecs[i].codec.ToString();
		if (i != codecs.size() - 1) {
			out << ", ";
		}
	}
	out << "}";
	return out.Release();
}

bool RTCWebRtcVideoChannel::GetSendCodec(VideoCodec* codec) {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	if (!send_codec_) {
		RTC_LOG(LS_VERBOSE) << "GetSendCodec: No send codec set.";
		return false;
	}
	*codec = send_codec_->codec;
	return true;
}

bool RTCWebRtcVideoChannel::SetSend(bool send) {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	if (send && !send_codec_) {
		RTC_DLOG(LS_ERROR) << "SetSend(true) called before setting codec.";
		return false;
	}
	for (const auto& kv : send_streams_) {
		kv.second->SetSend(send);
	}
	sending_ = send;
	return true;
}

bool RTCWebRtcVideoChannel::SetVideoSend(
	uint32_t ssrc,
	rtc::RTCVideoPacketRouteSourceInterface<std::unique_ptr<webrtc::RtpPacketReceived>>* source) {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	RTC_DCHECK(ssrc != 0);

	const auto& kv = send_streams_.find(ssrc);
	if (kv == send_streams_.end()) {
		// Allow unknown ssrc only if source is null.
		RTC_LOG(LS_ERROR) << "No sending stream on ssrc " << ssrc;
		return false;
	}

	return kv->second->SetVideoSend(source);
}

bool RTCWebRtcVideoChannel::ValidateSendSsrcAvailability(
		const StreamParams& sp) const {
	for (uint32_t ssrc : sp.ssrcs) {
		if (send_ssrcs_.find(ssrc) != send_ssrcs_.end()) {
			RTC_LOG(LS_ERROR) << "Send stream with SSRC '" << ssrc
				<< "' already exists.";
			return false;
		}
	}
	return true;
}

bool RTCWebRtcVideoChannel::ValidateReceiveSsrcAvailability(
	const StreamParams& sp) const {
	for (uint32_t ssrc : sp.ssrcs) {
		if (receive_ssrcs_.find(ssrc) != receive_ssrcs_.end()) {
			RTC_LOG(LS_ERROR) << "Receive stream with SSRC '" << ssrc
				<< "' already exists.";
			return false;
		}
	}
	return true;
}

bool RTCWebRtcVideoChannel::AddSendStream(const StreamParams& sp) {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	RTC_LOG(LS_INFO) << "AddSendStream: " << sp.ToString();

	if (!ValidateStreamParams(sp))
		return false;

	if (!ValidateSendSsrcAvailability(sp))
		return false;

	for (uint32_t used_ssrc : sp.ssrcs)
		send_ssrcs_.insert(used_ssrc);

	webrtc::RTCVideoSendStream::Config config(this);

	for (const RidDescription& rid : sp.rids()) {
		config.rtp.rids.push_back(rid.rid);
	}

	config.crypto_options = crypto_options_;
	config.rtp.extmap_allow_mixed = ExtmapAllowMixed();
	config.rtcp_report_interval_ms = video_config_.rtcp_report_interval_ms;

	RTCWebRtcVideoSendStream* stream = new RTCWebRtcVideoSendStream(
		call_, sp, std::move(config), 
		send_codec_, send_rtp_extensions_, send_params_
		, _rtc_thread_manager
		);

	uint32_t ssrc = sp.first_ssrc();
	RTC_DCHECK(ssrc != 0);
	send_streams_[ssrc] = stream;

	if (rtcp_receiver_report_ssrc_ == kDefaultRtcpReceiverReportSsrc) {
		rtcp_receiver_report_ssrc_ = ssrc;
		RTC_LOG(LS_INFO)
			<< "SetLocalSsrc on all the receive streams because we added "
			"a send stream.";

		for (auto& kv : receive_streams_)
			kv.second->SetLocalSsrc(ssrc);
	}
	if (sending_) {
		stream->SetSend(true);
	}

	return true;
}

bool RTCWebRtcVideoChannel::RemoveSendStream(uint32_t ssrc) {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	RTC_LOG(LS_INFO) << "RemoveSendStream: " << ssrc;

	RTCWebRtcVideoSendStream* removed_stream;
	std::map<uint32_t, RTCWebRtcVideoSendStream*>::iterator it =
		send_streams_.find(ssrc);
	if (it == send_streams_.end()) {
		return false;
	}

	for (uint32_t old_ssrc : it->second->GetSsrcs())
		send_ssrcs_.erase(old_ssrc);

	removed_stream = it->second;
	send_streams_.erase(it);

	if (rtcp_receiver_report_ssrc_ == ssrc) {
		rtcp_receiver_report_ssrc_ = send_streams_.empty()
			? kDefaultRtcpReceiverReportSsrc
			: send_streams_.begin()->first;
		RTC_LOG(LS_INFO) << "SetLocalSsrc on all the receive streams because the "
			"previous local SSRC was removed.";

		for (auto& kv : receive_streams_) {
			kv.second->SetLocalSsrc(rtcp_receiver_report_ssrc_);
		}
	}

	delete removed_stream;

	return true;
}

void RTCWebRtcVideoChannel::DeleteReceiveStream(
	RTCWebRtcVideoChannel::RTCWebRtcVideoReceiveStream* stream) {
	for (uint32_t old_ssrc : stream->GetSsrcs())
		receive_ssrcs_.erase(old_ssrc);
	delete stream;
}

bool RTCWebRtcVideoChannel::AddRecvStream(const StreamParams& sp) {
	return AddRecvStream(sp, false);
}

bool RTCWebRtcVideoChannel::AddRecvStream(const StreamParams& sp,
			bool default_stream) {
	RTC_DCHECK_RUN_ON(&thread_checker_);

	RTC_LOG(LS_INFO) << "AddRecvStream"
		<< (default_stream ? " (default stream)" : "") << ": "
		<< sp.ToString();

	if (!sp.has_ssrcs()) {
		// This is a StreamParam with unsignaled SSRCs. Store it, so it can be used
		// later when we know the SSRC on the first packet arrival.
		unsignaled_stream_params_ = sp;
		return true;
	}

	if (!ValidateStreamParams(sp))
		return false;

	uint32_t ssrc = sp.first_ssrc();

	// Remove running stream if this was a default stream.
	const auto& prev_stream = receive_streams_.find(ssrc);
	if (prev_stream != receive_streams_.end()) {

		if (default_stream || !prev_stream->second->IsDefaultStream()) {
			RTC_LOG(LS_ERROR) << "Receive stream for SSRC '" << ssrc
				<< "' already exists.";
			return false;
		}
		DeleteReceiveStream(prev_stream->second);
		receive_streams_.erase(prev_stream);
	}

	if (!ValidateReceiveSsrcAvailability(sp))
		return false;

	for (uint32_t used_ssrc : sp.ssrcs)
		receive_ssrcs_.insert(used_ssrc);

	webrtc::RTCVideoReceiveStream::Config config(this);
	webrtc::FlexfecReceiveStream::Config flexfec_config(this);
	ConfigureReceiverRtp(&config, &flexfec_config, sp);

	config.crypto_options = crypto_options_;

	if (!sp.stream_ids().empty()) {
		config.sync_group = sp.stream_ids()[0];
	}

	receive_streams_[ssrc] = new RTCWebRtcVideoReceiveStream(
		this, call_, sp, std::move(config), 
		default_stream, recv_codecs_, flexfec_config);

	return true;
}

void RTCWebRtcVideoChannel::ConfigureReceiverRtp(
	webrtc::RTCVideoReceiveStream::Config* config,
	webrtc::FlexfecReceiveStream::Config* flexfec_config,
	const StreamParams& sp) const {
	uint32_t ssrc = sp.first_ssrc();

	config->rtp.remote_ssrc = ssrc;
	config->rtp.local_ssrc = rtcp_receiver_report_ssrc_;

	// TODO(pbos): This protection is against setting the same local ssrc as
	// remote which is not permitted by the lower-level API. RTCP requires a
	// corresponding sender SSRC. Figure out what to do when we don't have
	// (receive-only) or know a good local SSRC.
	if (config->rtp.remote_ssrc == config->rtp.local_ssrc) {
		if (config->rtp.local_ssrc != kDefaultRtcpReceiverReportSsrc) {
			config->rtp.local_ssrc = kDefaultRtcpReceiverReportSsrc;
		}
		else {
			config->rtp.local_ssrc = kDefaultRtcpReceiverReportSsrc + 1;
		}
	}

	// Whether or not the receive stream sends reduced size RTCP is determined
	// by the send params.
	// TODO(deadbeef): Once we change "send_params" to "sender_params" and
	// "recv_params" to "receiver_params", we should get this out of
	// receiver_params_.
	config->rtp.rtcp_mode = send_params_.rtcp.reduced_size
		? webrtc::RtcpMode::kReducedSize
		: webrtc::RtcpMode::kCompound;

	config->rtp.transport_cc =
		send_codec_ ? HasTransportCc(send_codec_->codec) : false;

	sp.GetFidSsrc(ssrc, &config->rtp.rtx_ssrc);

	config->rtp.extensions = recv_rtp_extensions_;

	// TODO(brandtr): Generalize when we add support for multistream protection.
	flexfec_config->payload_type = recv_flexfec_payload_type_;
}

bool RTCWebRtcVideoChannel::RemoveRecvStream(uint32_t ssrc) {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	RTC_LOG(LS_INFO) << "RemoveRecvStream: " << ssrc;

	std::map<uint32_t, RTCWebRtcVideoReceiveStream*>::iterator stream =
		receive_streams_.find(ssrc);
	if (stream == receive_streams_.end()) {
		RTC_LOG(LS_ERROR) << "Stream not found for ssrc: " << ssrc;
		return false;
	}
	DeleteReceiveStream(stream->second);
	receive_streams_.erase(stream);

	return true;
}

void RTCWebRtcVideoChannel::ResetUnsignaledRecvStream() {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	RTC_LOG(LS_INFO) << "ResetUnsignaledRecvStream.";
	unsignaled_stream_params_ = StreamParams();

	// Delete any created default streams. This is needed to avoid SSRC collisions
	// in Call's RtpDemuxer, in the case that |this| has created a default video
	// receiver, and then some other WebRtcVideoChannel gets the SSRC signaled
	// in the corresponding Unified Plan "m=" section.
	auto it = receive_streams_.begin();
	while (it != receive_streams_.end()) {

		if (it->second->IsDefaultStream()) {
			DeleteReceiveStream(it->second);
			receive_streams_.erase(it++);
		}
		else {
			++it;
		}
	}
}

bool RTCWebRtcVideoChannel::SetPacketSink(uint32_t ssrc) {
	return true;
}

bool RTCWebRtcVideoChannel::GetStats(VideoMediaInfo* info) {
	return false;
}

void RTCWebRtcVideoChannel::FillSenderStats(VideoMediaInfo* video_media_info,
	bool log_stats) {
	for (std::map<uint32_t, RTCWebRtcVideoSendStream*>::iterator it =
		send_streams_.begin();
		it != send_streams_.end(); ++it) {
		auto infos = it->second->GetPerLayerVideoSenderInfos(log_stats);
		if (infos.empty())
			continue;
		video_media_info->aggregated_senders.push_back(
			it->second->GetAggregatedVideoSenderInfo(infos));
		for (auto&& info : infos) {
			video_media_info->senders.push_back(info);
		}
	}
}

void RTCWebRtcVideoChannel::FillReceiverStats(VideoMediaInfo* video_media_info,
	bool log_stats) {
	for (std::map<uint32_t, RTCWebRtcVideoReceiveStream*>::iterator it =
		receive_streams_.begin();
		it != receive_streams_.end(); ++it) {
		video_media_info->receivers.push_back(
			it->second->GetVideoReceiverInfo(log_stats));
	}
}

void RTCWebRtcVideoChannel::FillSendAndReceiveCodecStats(
	VideoMediaInfo* video_media_info) {
	for (const VideoCodec& codec : send_params_.codecs) {
		webrtc::RtpCodecParameters codec_params = codec.ToCodecParameters();
		video_media_info->send_codecs.insert(
			std::make_pair(codec_params.payload_type, std::move(codec_params)));
	}
	for (const VideoCodec& codec : recv_params_.codecs) {
		webrtc::RtpCodecParameters codec_params = codec.ToCodecParameters();
		video_media_info->receive_codecs.insert(
			std::make_pair(codec_params.payload_type, std::move(codec_params)));
	}
}

void RTCWebRtcVideoChannel::OnPacketReceived(rtc::CopyOnWriteBuffer packet,
		int64_t packet_time_us) {
	RTC_DCHECK_RUN_ON(&network_thread_checker_);

#ifdef __ROUTING_PACKET_PATH_USING_NETWORK_THREAD__
	const webrtc::PacketReceiver::DeliveryStatus delivery_result =
		call_->Receiver()->DeliverPacket(webrtc::MediaType::VIDEO, packet,
		packet_time_us);
	switch (delivery_result) {
	case webrtc::PacketReceiver::DELIVERY_OK:
		return;
	case webrtc::PacketReceiver::DELIVERY_PACKET_ERROR:
		return;
	case webrtc::PacketReceiver::DELIVERY_UNKNOWN_SSRC:
		break;
	}

	uint32_t ssrc = 0;
	if (!GetRtpSsrc(packet.cdata(), packet.size(), &ssrc)) {
		return;
	}

	/* kimi unused 
	if (unknown_ssrc_packet_buffer_) {
		unknown_ssrc_packet_buffer_->AddPacket(ssrc, packet_time_us, packet);
		return;
	}
	*/

	if (discard_unknown_ssrc_packets_) {
		return;
	}

	int payload_type = 0;
	if (!GetRtpPayloadType(packet.cdata(), packet.size(), &payload_type)) {
		return;
	}

	// See if this payload_type is registered as one that usually gets its own
	// SSRC (RTX) or at least is safe to drop either way (FEC). If it is, and
	// it wasn't handled above by DeliverPacket, that means we don't know what
	// stream it associates with, and we shouldn't ever create an implicit channel
	// for these.
	for (auto& codec : recv_codecs_) {
		if (payload_type == codec.rtx_payload_type ||
			payload_type == codec.ulpfec.red_rtx_payload_type ||
			payload_type == codec.ulpfec.ulpfec_payload_type) {
			return;
		}
	}
	if (payload_type == recv_flexfec_payload_type_) {
		return;
	}

	worker_thread_->PostTask(
		SafeTask(task_safety_.flag(), [this, packet, packet_time_us, ssrc] {
		switch (unsignalled_ssrc_handler_->OnUnsignalledSsrc(this, ssrc)) {
		case UnsignalledSsrcHandler::kDropPacket:
			return;
		case UnsignalledSsrcHandler::kDeliverPacket:
			break;
		}

		if (call_->Receiver()->DeliverPacket(webrtc::MediaType::VIDEO, packet,
			packet_time_us) !=
			webrtc::PacketReceiver::DELIVERY_OK) {
			RTC_LOG(LS_WARNING) << "Failed to deliver RTP packet on re-delivery.";
			return;
		}
	}));
#else
	worker_thread_->PostTask(
		SafeTask(task_safety_.flag(), [this, packet, packet_time_us] {
		RTC_DCHECK_RUN_ON(&thread_checker_);
		const webrtc::PacketReceiver::DeliveryStatus delivery_result =
			call_->Receiver()->DeliverPacket(webrtc::MediaType::VIDEO, packet,
			packet_time_us);
		switch (delivery_result) {
		case webrtc::PacketReceiver::DELIVERY_OK:
			return;
		case webrtc::PacketReceiver::DELIVERY_PACKET_ERROR:
			return;
		case webrtc::PacketReceiver::DELIVERY_UNKNOWN_SSRC:
			break;
		}

		uint32_t ssrc = 0;
		if (!GetRtpSsrc(packet.cdata(), packet.size(), &ssrc)) {
			return;
		}

		/* kimi unused
		if (unknown_ssrc_packet_buffer_) {
			unknown_ssrc_packet_buffer_->AddPacket(ssrc, packet_time_us, packet);
			return;
		}
		*/

		if (discard_unknown_ssrc_packets_) {
			return;
		}

		int payload_type = 0;
		if (!GetRtpPayloadType(packet.cdata(), packet.size(), &payload_type)) {
			return;
		}

		// See if this payload_type is registered as one that usually gets its own
		// SSRC (RTX) or at least is safe to drop either way (FEC). If it is, and
		// it wasn't handled above by DeliverPacket, that means we don't know what
		// stream it associates with, and we shouldn't ever create an implicit channel
		// for these.
		for (auto& codec : recv_codecs_) {
			if (payload_type == codec.rtx_payload_type ||
				payload_type == codec.ulpfec.red_rtx_payload_type ||
				payload_type == codec.ulpfec.ulpfec_payload_type) {
				return;
			}
		}
		if (payload_type == recv_flexfec_payload_type_) {
			return;
		}

		switch (unsignalled_ssrc_handler_->OnUnsignalledSsrc(this, ssrc)) {
		case UnsignalledSsrcHandler::kDropPacket:
			return;
		case UnsignalledSsrcHandler::kDeliverPacket:
			break;
		}
		if (call_->Receiver()->DeliverPacket(webrtc::MediaType::VIDEO, packet,
			packet_time_us) !=
			webrtc::PacketReceiver::DELIVERY_OK) {
			RTC_LOG(LS_WARNING) << "Failed to deliver RTP packet on re-delivery.";
			return;
		}
	}));
#endif
}

#if 0 // kimi unused
void RTCWebRtcVideoChannel::BackfillBufferedPackets(
	rtc::ArrayView<const uint32_t> ssrcs) {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	if (!unknown_ssrc_packet_buffer_) {
		return;
	}

	int delivery_ok_cnt = 0;
	int delivery_unknown_ssrc_cnt = 0;
	int delivery_packet_error_cnt = 0;

	webrtc::PacketReceiver* receiver = this->call_->Receiver();
	unknown_ssrc_packet_buffer_->BackfillPackets(
		ssrcs, [&](uint32_t /*ssrc*/, int64_t packet_time_us,
		rtc::CopyOnWriteBuffer packet) {
		switch (receiver->DeliverPacket(webrtc::MediaType::VIDEO, packet,
			packet_time_us)) {
		case webrtc::PacketReceiver::DELIVERY_OK:
			delivery_ok_cnt++;
			break;
		case webrtc::PacketReceiver::DELIVERY_UNKNOWN_SSRC:
			delivery_unknown_ssrc_cnt++;
			break;
		case webrtc::PacketReceiver::DELIVERY_PACKET_ERROR:
			delivery_packet_error_cnt++;
			break;
		}
	});

	rtc::StringBuilder out;
	out << "[ ";
	for (uint32_t ssrc : ssrcs) {
		out << std::to_string(ssrc) << " ";
	}
	out << "]";
	auto level = rtc::LS_INFO;
	if (delivery_unknown_ssrc_cnt > 0 || delivery_packet_error_cnt > 0) {
		level = rtc::LS_ERROR;
	}
	int total =
		delivery_ok_cnt + delivery_unknown_ssrc_cnt + delivery_packet_error_cnt;
	RTC_LOG_V(level) << "Backfilled " << total
		<< " packets for ssrcs: " << out.Release()
		<< " ok: " << delivery_ok_cnt
		<< " error: " << delivery_packet_error_cnt
		<< " unknown: " << delivery_unknown_ssrc_cnt;
}
#endif

RTCWebRtcVideoChannel::RTCWebRtcVideoReceiveStream*
	RTCWebRtcVideoChannel::FindReceiveStream(uint32_t ssrc) {
	if (ssrc == 0) {
		absl::optional<uint32_t> default_ssrc = GetDefaultReceiveStreamSsrc();
		if (!default_ssrc) {
			return nullptr;
		}
		ssrc = *default_ssrc;
	}
	auto it = receive_streams_.find(ssrc);
	if (it != receive_streams_.end()) {
		return it->second;
	}
	return nullptr;
}

void RTCWebRtcVideoChannel::GenerateKeyFrame(uint32_t ssrc) {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	RTCWebRtcVideoReceiveStream* stream = FindReceiveStream(ssrc);
	if (stream) {
		stream->GenerateKeyFrame();
	}
	else {
		RTC_LOG(LS_ERROR)
			<< "Absent receive stream; ignoring key frame generation for ssrc "
			<< ssrc;
	}
}

void RTCWebRtcVideoChannel::OnReadyToSend(bool ready) {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	RTC_LOG(LS_VERBOSE) << "OnReadyToSend: " << (ready ? "Ready." : "Not ready.");

	call_->SignalChannelNetworkState(
		webrtc::MediaType::VIDEO,
		ready ? webrtc::kNetworkUp : webrtc::kNetworkDown);
}

void RTCWebRtcVideoChannel::OnNetworkRouteChanged(
	const std::string& transport_name,
	const rtc::NetworkRoute& network_route) {
	RTC_DCHECK_RUN_ON(&thread_checker_);

}

void RTCWebRtcVideoChannel::SetInterface(NetworkInterface* iface) {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	MediaChannel::SetInterface(iface);
	// Set the RTP recv/send buffer to a bigger size.

	// The group should be a positive integer with an explicit size, in
	// which case that is used as UDP recevie buffer size. All other values shall
	// result in the default value being used.

	const std::string group_name_recv_buf_size;

	int recv_buffer_size = kVideoRtpRecvBufferSize;
	if (!group_name_recv_buf_size.empty() &&
		(sscanf(group_name_recv_buf_size.c_str(), "%d", &recv_buffer_size) != 1 ||
		recv_buffer_size <= 0)) {
		RTC_LOG(LS_WARNING) << "Invalid receive buffer size: "
			<< group_name_recv_buf_size;
		recv_buffer_size = kVideoRtpRecvBufferSize;
	}

	MediaChannel::SetOption(NetworkInterface::ST_RTP, rtc::Socket::OPT_RCVBUF,
		recv_buffer_size);

	// Speculative change to increase the outbound socket buffer size.
	// In b/15152257, we are seeing a significant number of packets discarded
	// due to lack of socket buffer space, although it's not yet clear what the
	// ideal value should be.

	const std::string group_name_send_buf_size;
	int send_buffer_size = kVideoRtpSendBufferSize;
	if (!group_name_send_buf_size.empty() &&
		(sscanf(group_name_send_buf_size.c_str(), "%d", &send_buffer_size) != 1 ||
		send_buffer_size <= 0)) {
		RTC_LOG(LS_WARNING) << "Invalid send buffer size: "
			<< group_name_send_buf_size;
		send_buffer_size = kVideoRtpSendBufferSize;
	}

	MediaChannel::SetOption(NetworkInterface::ST_RTP, rtc::Socket::OPT_SNDBUF,
		send_buffer_size);
}

absl::optional<uint32_t> RTCWebRtcVideoChannel::GetDefaultReceiveStreamSsrc() {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	absl::optional<uint32_t> ssrc;
	for (auto it = receive_streams_.begin(); it != receive_streams_.end(); ++it) {
		if (it->second->IsDefaultStream()) {
			ssrc.emplace(it->first);
			break;
		}
	}
	return ssrc;
}

std::vector<webrtc::RtpSource> RTCWebRtcVideoChannel::GetSources(
	uint32_t ssrc) const {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	auto it = receive_streams_.find(ssrc);
	if (it == receive_streams_.end()) {
		// TODO(bugs.webrtc.org/9781): Investigate standard compliance
		// with sources for streams that has been removed.
		RTC_LOG(LS_ERROR) << "Attempting to get contributing sources for SSRC:"
			<< ssrc << " which doesn't exist.";
		return{};
	}

	return it->second->GetSources();
}

bool RTCWebRtcVideoChannel::SetBaseMinimumPlayoutDelayMs(uint32_t ssrc,
	int delay_ms) {
	return true;
}

absl::optional<int> RTCWebRtcVideoChannel::GetBaseMinimumPlayoutDelayMs(
	uint32_t ssrc) const {
	return absl::nullopt;
}


bool RTCWebRtcVideoChannel::SendRtp(const uint8_t* data,
	size_t len,
	const webrtc::PacketOptions& options) {
	rtc::CopyOnWriteBuffer packet(data, len, kMaxRtpPacketLen);
	rtc::PacketOptions rtc_options;
	rtc_options.packet_id = options.packet_id;
	if (DscpEnabled()) {
		rtc_options.dscp = PreferredDscp();
	}
	rtc_options.info_signaled_after_sent.included_in_feedback =
		options.included_in_feedback;
	rtc_options.info_signaled_after_sent.included_in_allocation =
		options.included_in_allocation;

	return MediaChannel::SendPacket(&packet, rtc_options);
}

bool RTCWebRtcVideoChannel::SendRtcp(const uint8_t* data, size_t len) {
	rtc::CopyOnWriteBuffer packet(data, len, kMaxRtpPacketLen);
	rtc::PacketOptions rtc_options;
	if (DscpEnabled()) {
		rtc_options.dscp = PreferredDscp();
	}
	return MediaChannel::SendRtcp(&packet, rtc_options);
}

RTCWebRtcVideoChannel::VideoCodecSettings::VideoCodecSettings()
	: flexfec_payload_type(-1), rtx_payload_type(-1) {}

bool RTCWebRtcVideoChannel::VideoCodecSettings::operator==(
	const RTCWebRtcVideoChannel::VideoCodecSettings& other) const {
	return codec == other.codec && ulpfec == other.ulpfec &&
		flexfec_payload_type == other.flexfec_payload_type &&
		rtx_payload_type == other.rtx_payload_type;
}

bool RTCWebRtcVideoChannel::VideoCodecSettings::EqualsDisregardingFlexfec(
	const RTCWebRtcVideoChannel::VideoCodecSettings& a,
	const RTCWebRtcVideoChannel::VideoCodecSettings& b) {
	return a.codec == b.codec && a.ulpfec == b.ulpfec &&
		a.rtx_payload_type == b.rtx_payload_type;
}

bool RTCWebRtcVideoChannel::VideoCodecSettings::operator!=(
	const RTCWebRtcVideoChannel::VideoCodecSettings& other) const {
	return !(*this == other);
}


std::vector<RTCWebRtcVideoChannel::VideoCodecSettings>
RTCWebRtcVideoChannel::MapCodecs(const std::vector<VideoCodec>& codecs) {
	if (codecs.empty()) {
		return{};
	}

	std::vector<VideoCodecSettings> video_codecs;
	std::map<int, VideoCodec::CodecType> payload_codec_type;
	// |rtx_mapping| maps video payload type to rtx payload type.
	std::map<int, int> rtx_mapping;

	webrtc::UlpfecConfig ulpfec_config;
	absl::optional<int> flexfec_payload_type;

	for (const VideoCodec& in_codec : codecs) {
		const int payload_type = in_codec.id;

		if (payload_codec_type.find(payload_type) != payload_codec_type.end()) {
			RTC_LOG(LS_ERROR) << "Payload type already registered: "
				<< in_codec.ToString();
			return{};
		}
		payload_codec_type[payload_type] = in_codec.GetCodecType();

		switch (in_codec.GetCodecType()) {
		case VideoCodec::CODEC_RED: {
			if (ulpfec_config.red_payload_type != -1) {
				RTC_LOG(LS_ERROR)
					<< "Duplicate RED codec: ignoring PT=" << payload_type
					<< " in favor of PT=" << ulpfec_config.red_payload_type
					<< " which was specified first.";
				break;
			}
			ulpfec_config.red_payload_type = payload_type;
			break;
		}

		case VideoCodec::CODEC_ULPFEC: {
			if (ulpfec_config.ulpfec_payload_type != -1) {
				RTC_LOG(LS_ERROR)
					<< "Duplicate ULPFEC codec: ignoring PT=" << payload_type
					<< " in favor of PT=" << ulpfec_config.ulpfec_payload_type
					<< " which was specified first.";
				break;
			}
			ulpfec_config.ulpfec_payload_type = payload_type;
			break;
		}

		case VideoCodec::CODEC_FLEXFEC: {
			if (flexfec_payload_type) {
				RTC_LOG(LS_ERROR)
					<< "Duplicate FLEXFEC codec: ignoring PT=" << payload_type
					<< " in favor of PT=" << *flexfec_payload_type
					<< " which was specified first.";
				break;
			}
			flexfec_payload_type = payload_type;
			break;
		}

		case VideoCodec::CODEC_RTX: {
			int associated_payload_type;
			if (!in_codec.GetParam(kCodecParamAssociatedPayloadType,
				&associated_payload_type) ||
				!IsValidRtpPayloadType(associated_payload_type)) {
				RTC_LOG(LS_ERROR)
					<< "RTX codec with invalid or no associated payload type: "
					<< in_codec.ToString();
				return{};
			}
			rtx_mapping[associated_payload_type] = payload_type;
			break;
		}

		case VideoCodec::CODEC_VIDEO: {
			video_codecs.emplace_back();
			video_codecs.back().codec = in_codec;
			break;
		}
		}
	}

	// One of these codecs should have been a video codec. Only having FEC
	// parameters into this code is a logic error.
	RTC_DCHECK(!video_codecs.empty());

	for (const auto& entry : rtx_mapping) {
		const int associated_payload_type = entry.first;
		const int rtx_payload_type = entry.second;
		auto it = payload_codec_type.find(associated_payload_type);
		if (it == payload_codec_type.end()) {
			RTC_LOG(LS_ERROR) << "RTX codec (PT=" << rtx_payload_type
				<< ") mapped to PT=" << associated_payload_type
				<< " which is not in the codec list.";
			return{};
		}
		const VideoCodec::CodecType associated_codec_type = it->second;
		if (associated_codec_type != VideoCodec::CODEC_VIDEO &&
			associated_codec_type != VideoCodec::CODEC_RED) {
			RTC_LOG(LS_ERROR)
				<< "RTX PT=" << rtx_payload_type
				<< " not mapped to regular video codec or RED codec (PT="
				<< associated_payload_type << ").";
			return{};
		}

		if (associated_payload_type == ulpfec_config.red_payload_type) {
			ulpfec_config.red_rtx_payload_type = rtx_payload_type;
		}
	}

	for (VideoCodecSettings& codec_settings : video_codecs) {
		const int payload_type = codec_settings.codec.id;
		codec_settings.ulpfec = ulpfec_config;
		codec_settings.flexfec_payload_type = flexfec_payload_type.value_or(-1);
		auto it = rtx_mapping.find(payload_type);
		if (it != rtx_mapping.end()) {
			const int rtx_payload_type = it->second;
			codec_settings.rtx_payload_type = rtx_payload_type;
		}
	}

	return video_codecs;
}

RTCWebRtcVideoChannel::RTCWebRtcVideoSendStream::VideoSendStreamParameters::
	VideoSendStreamParameters(
		webrtc::RTCVideoSendStream::Config config,
		const absl::optional<VideoCodecSettings>& codec_settings)
: config(std::move(config)),
  conference_mode(false),
  codec_settings(codec_settings) {
}

RTCWebRtcVideoChannel::RTCWebRtcVideoSendStream::RTCWebRtcVideoSendStream(
	webrtc::RTCCall* call,
	const StreamParams& sp,
	webrtc::RTCVideoSendStream::Config config,
	const absl::optional<VideoCodecSettings>& codec_settings,
	const absl::optional<std::vector<webrtc::RtpExtension>>& rtp_extensions,
	const VideoSendParameters& send_params
	, rtc::RTCThreadManagerInterface* rtc_thread_manager
	) 
	: 
	worker_thread_(rtc::Thread::Current(rtc_thread_manager)),
	ssrcs_(sp.ssrcs), 
	ssrc_groups_(sp.ssrc_groups),
	call_(call), 
	stream_(nullptr),
	parameters_(std::move(config),  codec_settings),
	rtp_parameters_(CreateRtpParametersWithEncodings(sp)),
	source_(nullptr),
	sending_(false) {

	parameters_.config.rtp.max_packet_size =
		std::min<size_t>(parameters_.config.rtp.max_packet_size, kVideoMtu);
	parameters_.conference_mode = send_params.conference_mode;

	sp.GetPrimarySsrcs(&parameters_.config.rtp.ssrcs);

	// ValidateStreamParams should prevent this from happening.
	RTC_CHECK(!parameters_.config.rtp.ssrcs.empty());
	rtp_parameters_.encodings[0].ssrc = parameters_.config.rtp.ssrcs[0];

	// RTX.
	sp.GetFidSsrcs(parameters_.config.rtp.ssrcs,
		&parameters_.config.rtp.rtx.ssrcs);

	parameters_.config.rtp.c_name = sp.cname;
	if (rtp_extensions) {
		parameters_.config.rtp.extensions = *rtp_extensions;
		rtp_parameters_.header_extensions = *rtp_extensions;
	}
	parameters_.config.rtp.rtcp_mode = send_params.rtcp.reduced_size
		? webrtc::RtcpMode::kReducedSize
		: webrtc::RtcpMode::kCompound;
	parameters_.config.rtp.mid = send_params.mid;
	rtp_parameters_.rtcp.reduced_size = send_params.rtcp.reduced_size;

	if (codec_settings) {
		SetCodec(*codec_settings);
	}
}

RTCWebRtcVideoChannel::RTCWebRtcVideoSendStream::~RTCWebRtcVideoSendStream() {
	if (stream_ != NULL) {
		call_->DestroyVideoSendStream(stream_);
	}
}

bool RTCWebRtcVideoChannel::RTCWebRtcVideoSendStream::SetVideoSend(
	rtc::RTCVideoPacketRouteSourceInterface<std::unique_ptr<webrtc::RtpPacketReceived>>* source) {
	if (source_ && stream_) {
		stream_->SetSource(nullptr);
	}

	source_ = source;

	if (source_ && stream_) {
		stream_->SetSource(source_);
	}

	return true;
}

void RTCWebRtcVideoChannel::RTCWebRtcVideoSendStream::SetSendParameters(
	const ChangedSendParameters& params) {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	// |recreate_stream| means construction-time parameters have changed and the
	// sending stream needs to be reset with the new config.
	bool recreate_stream = false;
	if (params.rtcp_mode) {
		parameters_.config.rtp.rtcp_mode = *params.rtcp_mode;
		rtp_parameters_.rtcp.reduced_size =
			parameters_.config.rtp.rtcp_mode == webrtc::RtcpMode::kReducedSize;
		recreate_stream = true;
	}
	if (params.extmap_allow_mixed) {
		parameters_.config.rtp.extmap_allow_mixed = *params.extmap_allow_mixed;
		recreate_stream = true;
	}
	if (params.rtp_header_extensions) {
		parameters_.config.rtp.extensions = *params.rtp_header_extensions;
		rtp_parameters_.header_extensions = *params.rtp_header_extensions;
		recreate_stream = true;
	}
	if (params.mid) {
		parameters_.config.rtp.mid = *params.mid;
		recreate_stream = true;
	}

	if (params.conference_mode) {
		parameters_.conference_mode = *params.conference_mode;
	}

	// Set codecs and options.
	if (params.send_codec) {
		SetCodec(*params.send_codec);
		recreate_stream = false;  // SetCodec has already recreated the stream.
	}
	else if (params.conference_mode && parameters_.codec_settings) {
		SetCodec(*parameters_.codec_settings);
		recreate_stream = false;  // SetCodec has already recreated the stream.
	}
	if (recreate_stream) {
		RTC_LOG(LS_INFO)
			<< "RecreateWebRtcStream (send) because of SetSendParameters";
		RecreateWebRtcStream();
	}
}

webrtc::RTCError 
RTCWebRtcVideoChannel::RTCWebRtcVideoSendStream::SetRtpParameters(
	const webrtc::RtpParameters& new_parameters) {
	return webrtc::RTCError::OK();
}

webrtc::RtpParameters 
RTCWebRtcVideoChannel::RTCWebRtcVideoSendStream::GetRtpParameters() const {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	return rtp_parameters_;
}


void RTCWebRtcVideoChannel::RTCWebRtcVideoSendStream::SetSend(bool send) {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	sending_ = send;
	UpdateSendState();
}

const std::vector<uint32_t>& 
RTCWebRtcVideoChannel::RTCWebRtcVideoSendStream::GetSsrcs() const {
	return ssrcs_;
}

std::vector<VideoSenderInfo> 
RTCWebRtcVideoChannel::RTCWebRtcVideoSendStream::GetPerLayerVideoSenderInfos(bool log_stats) {
	return{};
}

VideoSenderInfo RTCWebRtcVideoChannel::RTCWebRtcVideoSendStream::GetAggregatedVideoSenderInfo(
	const std::vector<VideoSenderInfo>& infos) const {
	return VideoSenderInfo();
}

void RTCWebRtcVideoChannel::RTCWebRtcVideoSendStream::SetCodec(
	const VideoCodecSettings& codec_settings) {

	RTC_DCHECK_RUN_ON(&thread_checker_);
	parameters_.encoder_config = CreateVideoEncoderConfig(codec_settings.codec);
	RTC_DCHECK_GT(parameters_.encoder_config.number_of_streams, 0);

	parameters_.config.rtp.payload_name = codec_settings.codec.name;
	parameters_.config.rtp.payload_type = codec_settings.codec.id;
	parameters_.config.rtp.raw_payload =
		codec_settings.codec.packetization == kPacketizationParamRaw;
	parameters_.config.rtp.ulpfec = codec_settings.ulpfec;
	parameters_.config.rtp.flexfec.payload_type =
		codec_settings.flexfec_payload_type;

	// Set RTX payload type if RTX is enabled.
	if (!parameters_.config.rtp.rtx.ssrcs.empty()) {
		if (codec_settings.rtx_payload_type == -1) {
			RTC_LOG(LS_WARNING)
				<< "RTX SSRCs configured but there's no configured RTX "
				"payload type. Ignoring.";
			parameters_.config.rtp.rtx.ssrcs.clear();
		}
		else {
			parameters_.config.rtp.rtx.payload_type = 
				codec_settings.rtx_payload_type;
		}
	}

	const bool has_lntf = HasLntf(codec_settings.codec);
	parameters_.config.rtp.lntf.enabled = has_lntf;
	parameters_.config.rtp.nack.rtp_history_ms =
		HasNack(codec_settings.codec) ? kNackHistoryMs : 0;
	parameters_.codec_settings = codec_settings;

	// TODO(nisse): Avoid recreation, it should be enough to call
	// ReconfigureEncoder.
	RTC_LOG(LS_INFO) << "RecreateWebRtcStream (send) because of SetCodec.";
	RecreateWebRtcStream();
}

webrtc::VideoEncoderConfig 
RTCWebRtcVideoChannel::RTCWebRtcVideoSendStream::CreateVideoEncoderConfig(
			const VideoCodec& codec) const {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	webrtc::VideoEncoderConfig encoder_config;
	encoder_config.codec_type = webrtc::PayloadStringToCodecType(codec.name);
	encoder_config.video_format =
		webrtc::SdpVideoFormat(codec.name, codec.params);

	encoder_config.min_transmit_bitrate_bps = 0;
	encoder_config.content_type =
		webrtc::VideoEncoderConfig::ContentType::kRealtimeVideo;
	encoder_config.number_of_streams = 1;
	// Application-controlled state is held in the encoder_config's
	// simulcast_layers. Currently this is used to control which simulcast layers
	// are active and for configuring the min/max bitrate and max framerate.
	// The encoder_config's simulcast_layers is also used for non-simulcast (when
	// there is a single layer).
	RTC_DCHECK_GE(rtp_parameters_.encodings.size(),
		encoder_config.number_of_streams);
	RTC_DCHECK_GT(encoder_config.number_of_streams, 0);

	encoder_config.legacy_conference_mode = parameters_.conference_mode;

	return encoder_config;
}

void RTCWebRtcVideoChannel::RTCWebRtcVideoSendStream::RecreateWebRtcStream() {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	if (stream_ != NULL) {
		call_->DestroyVideoSendStream(stream_);
	}

	RTC_CHECK(parameters_.codec_settings);

	webrtc::RTCVideoSendStream::Config config = parameters_.config.Copy();
	if (!config.rtp.rtx.ssrcs.empty() && config.rtp.rtx.payload_type == -1) {
		RTC_LOG(LS_WARNING) << "RTX SSRCs configured but there's no configured RTX "
			"payload type the set codec. Ignoring RTX.";
		config.rtp.rtx.ssrcs.clear();
	}
	if (parameters_.encoder_config.number_of_streams == 1) {
		// SVC is used instead of simulcast. Remove unnecessary SSRCs.
		if (config.rtp.ssrcs.size() > 1) {
			config.rtp.ssrcs.resize(1);
			if (config.rtp.rtx.ssrcs.size() > 1) {
				config.rtp.rtx.ssrcs.resize(1);
			}
		}
	}


	stream_ = call_->CreateVideoSendStream(std::move(config),
										   parameters_.encoder_config.Copy());

	parameters_.encoder_config.encoder_specific_settings = NULL;

	if (source_) {
		stream_->SetSource(source_);
	}

	UpdateSendState();

}

void RTCWebRtcVideoChannel::RTCWebRtcVideoSendStream::UpdateSendState() {
	RTC_DCHECK_RUN_ON(&thread_checker_);
	if (sending_) {
		RTC_DCHECK(stream_ != nullptr);
		size_t num_layers = rtp_parameters_.encodings.size();
		if (parameters_.encoder_config.number_of_streams == 1) {
			// SVC is used. Only one simulcast layer is present.
			num_layers = 1;
		}
		std::vector<bool> active_layers(num_layers);
		for (size_t i = 0; i < num_layers; ++i) {
			active_layers[i] = IsLayerActive(rtp_parameters_.encodings[i]);
		}
		if (parameters_.encoder_config.number_of_streams == 1 &&
			rtp_parameters_.encodings.size() > 1) {
			// SVC is used.
			// The only present simulcast layer should be active if any of the
			// configured SVC layers is active.
			active_layers[0] =
				absl::c_any_of(rtp_parameters_.encodings,
				[](const auto& encoding) { return encoding.active; });
		}
		// This updates what simulcast layers are sending, and possibly starts
		// or stops the VideoSendStream.
		stream_->Start();
	}
	else {
		if (stream_ != nullptr) {
			stream_->Stop();
		}
	}
}

webrtc::DegradationPreference 
RTCWebRtcVideoChannel::RTCWebRtcVideoSendStream::GetDegradationPreference() const {
	return webrtc::DegradationPreference::DISABLED;
}


RTCWebRtcVideoChannel::RTCWebRtcVideoReceiveStream::RTCWebRtcVideoReceiveStream(
	RTCWebRtcVideoChannel* channel,
	webrtc::RTCCall* call,
	const StreamParams& sp,
	webrtc::RTCVideoReceiveStream::Config config,
	bool default_stream,
	const std::vector<VideoCodecSettings>& recv_codecs,
	const webrtc::FlexfecReceiveStream::Config& flexfec_config)
	: channel_(channel),
	call_(call),
	stream_params_(sp),
	stream_(NULL),
	default_stream_(default_stream),
	config_(std::move(config)),
	flexfec_config_(flexfec_config),
	flexfec_stream_(nullptr),
	first_frame_timestamp_(-1),
	estimated_remote_start_ntp_time_ms_(0) {

	ConfigureCodecs(recv_codecs);
	ConfigureFlexfecCodec(flexfec_config.payload_type);
	MaybeRecreateWebRtcFlexfecStream();

	RecreateWebRtcVideoStream();
}

RTCWebRtcVideoChannel::RTCWebRtcVideoReceiveStream::~RTCWebRtcVideoReceiveStream() {
	if (flexfec_stream_) {
		MaybeDissociateFlexfecFromVideo();
		call_->DestroyFlexfecReceiveStream(flexfec_stream_);
	}
	call_->DestroyVideoReceiveStream(stream_);
}

void RTCWebRtcVideoChannel::RTCWebRtcVideoReceiveStream::ConfigureCodecs(
	const std::vector<VideoCodecSettings>& recv_codecs) {
	RTC_DCHECK(!recv_codecs.empty());
	config_.decoders.clear();
	config_.rtp.rtx_associated_payload_types.clear();
	config_.rtp.raw_payload_types.clear();
	for (const auto& recv_codec : recv_codecs) {
		webrtc::SdpVideoFormat video_format(recv_codec.codec.name,
			recv_codec.codec.params);

		webrtc::RTCVideoReceiveStream::Decoder decoder;
		decoder.video_format = video_format;
		decoder.payload_type = recv_codec.codec.id;
		decoder.video_format =
			webrtc::SdpVideoFormat(recv_codec.codec.name, recv_codec.codec.params);
		config_.decoders.push_back(decoder);
		config_.rtp.rtx_associated_payload_types[recv_codec.rtx_payload_type] =
			recv_codec.codec.id;
		if (recv_codec.codec.packetization == kPacketizationParamRaw) {
			config_.rtp.raw_payload_types.insert(recv_codec.codec.id);
		}
	}

	const auto& codec = recv_codecs.front();
	config_.rtp.ulpfec_payload_type = codec.ulpfec.ulpfec_payload_type;
	config_.rtp.red_payload_type = codec.ulpfec.red_payload_type;

	config_.rtp.lntf.enabled = HasLntf(codec.codec);
	config_.rtp.nack.rtp_history_ms = HasNack(codec.codec) ? kNackHistoryMs : 0;
	config_.rtp.rtcp_xr.receiver_reference_time_report = HasRrtr(codec.codec);
	if (codec.ulpfec.red_rtx_payload_type != -1) {
		config_.rtp
			.rtx_associated_payload_types[codec.ulpfec.red_rtx_payload_type] =
			codec.ulpfec.red_payload_type;
	}
}

void RTCWebRtcVideoChannel::RTCWebRtcVideoReceiveStream::ConfigureFlexfecCodec(
	int flexfec_payload_type) {
	flexfec_config_.payload_type = flexfec_payload_type;
}


const std::vector<uint32_t>& 
RTCWebRtcVideoChannel::RTCWebRtcVideoReceiveStream::GetSsrcs() const {
	return stream_params_.ssrcs;
}

std::vector<webrtc::RtpSource> 
RTCWebRtcVideoChannel::RTCWebRtcVideoReceiveStream::GetSources() {
	RTC_DCHECK(stream_);
	return stream_->GetSources();
}

// Does not return codecs, they are filled by the owning WebRtcVideoChannel.
webrtc::RtpParameters 
RTCWebRtcVideoChannel::RTCWebRtcVideoReceiveStream::GetRtpParameters() const {
	webrtc::RtpParameters rtp_parameters;

	std::vector<uint32_t> primary_ssrcs;
	stream_params_.GetPrimarySsrcs(&primary_ssrcs);
	for (uint32_t ssrc : primary_ssrcs) {
		rtp_parameters.encodings.emplace_back();
		rtp_parameters.encodings.back().ssrc = ssrc;
		uint32_t fid_ssrc;
		if (stream_params_.GetFidSsrc(ssrc, &fid_ssrc)) {
			rtp_parameters.rtxs.emplace_back();
			rtp_parameters.rtxs.back().ssrc = fid_ssrc;
		}
	}

	rtp_parameters.header_extensions = config_.rtp.extensions;
	rtp_parameters.rtcp.reduced_size =
		config_.rtp.rtcp_mode == webrtc::RtcpMode::kReducedSize;

	return rtp_parameters;
}

void RTCWebRtcVideoChannel::RTCWebRtcVideoReceiveStream::SetLocalSsrc(uint32_t local_ssrc) {
	if (local_ssrc == config_.rtp.local_ssrc) {
		RTC_DLOG(LS_INFO) << "Ignoring call to SetLocalSsrc because parameters are "
			"unchanged; local_ssrc="
			<< local_ssrc;
		return;
	}

	config_.rtp.local_ssrc = local_ssrc;
	flexfec_config_.local_ssrc = local_ssrc;
	RTC_LOG(LS_INFO)
		<< "RecreateWebRtcVideoStream (recv) because of SetLocalSsrc; local_ssrc="
		<< local_ssrc;
	MaybeRecreateWebRtcFlexfecStream();

	RecreateWebRtcVideoStream();
}

// TODO(deadbeef): Move these feedback parameters into the recv parameters.
void RTCWebRtcVideoChannel::RTCWebRtcVideoReceiveStream::SetFeedbackParameters(bool lntf_enabled,
	bool nack_enabled,
	bool transport_cc_enabled,
	webrtc::RtcpMode rtcp_mode) {
	int nack_history_ms = nack_enabled ? kNackHistoryMs : 0;
	if (config_.rtp.lntf.enabled == lntf_enabled &&
		config_.rtp.nack.rtp_history_ms == nack_history_ms &&
		config_.rtp.transport_cc == transport_cc_enabled &&
		config_.rtp.rtcp_mode == rtcp_mode) {
		RTC_LOG(LS_INFO)
			<< "Ignoring call to SetFeedbackParameters because parameters are "
			"unchanged; lntf="
			<< lntf_enabled << ", nack=" << nack_enabled
			<< ", transport_cc=" << transport_cc_enabled;
		return;
	}
	config_.rtp.lntf.enabled = lntf_enabled;
	config_.rtp.nack.rtp_history_ms = nack_history_ms;
	config_.rtp.transport_cc = transport_cc_enabled;
	config_.rtp.rtcp_mode = rtcp_mode;
	// TODO(brandtr): We should be spec-compliant and set |transport_cc| here
	// based on the rtcp-fb for the FlexFEC codec, not the media codec.
	flexfec_config_.transport_cc = config_.rtp.transport_cc;
	flexfec_config_.rtcp_mode = config_.rtp.rtcp_mode;
	RTC_LOG(LS_INFO) << "RecreateWebRtcVideoStream (recv) because of "
		"SetFeedbackParameters; nack="
		<< nack_enabled << ", transport_cc=" << transport_cc_enabled;
	MaybeRecreateWebRtcFlexfecStream();

	RecreateWebRtcVideoStream();
}

void RTCWebRtcVideoChannel::RTCWebRtcVideoReceiveStream::SetRecvParameters(
	const ChangedRecvParameters& params) {
	bool video_needs_recreation = false;
	bool flexfec_needs_recreation = false;
	if (params.codec_settings) {
		ConfigureCodecs(*params.codec_settings);
		video_needs_recreation = true;
	}
	if (params.rtp_header_extensions) {
		config_.rtp.extensions = *params.rtp_header_extensions;
		flexfec_config_.rtp_header_extensions = *params.rtp_header_extensions;
		video_needs_recreation = true;
		flexfec_needs_recreation = true;
	}
	if (params.flexfec_payload_type) {
		ConfigureFlexfecCodec(*params.flexfec_payload_type);
		flexfec_needs_recreation = true;
	}
	if (flexfec_needs_recreation) {
		RTC_LOG(LS_INFO) << "MaybeRecreateWebRtcFlexfecStream (recv) because of "
			"SetRecvParameters";
		MaybeRecreateWebRtcFlexfecStream();
	}
	if (video_needs_recreation) {
		RTC_LOG(LS_INFO)
			<< "RecreateWebRtcVideoStream (recv) because of SetRecvParameters";

		RecreateWebRtcVideoStream();
	}
}

bool RTCWebRtcVideoChannel::RTCWebRtcVideoReceiveStream::IsDefaultStream() const {
	return default_stream_;
}

VideoReceiverInfo 
RTCWebRtcVideoChannel::RTCWebRtcVideoReceiveStream::GetVideoReceiverInfo(bool log_stats) {
	return VideoReceiverInfo();
}

void RTCWebRtcVideoChannel::RTCWebRtcVideoReceiveStream::GenerateKeyFrame() {
	if (stream_) {
		stream_->GenerateKeyFrame();
	}
	else {
		RTC_LOG(LS_ERROR)
			<< "Absent receive stream; ignoring key frame generation request.";
	}
}

void RTCWebRtcVideoChannel::RTCWebRtcVideoReceiveStream::RecreateWebRtcVideoStream() {
	if (stream_) {
		MaybeDissociateFlexfecFromVideo();
		call_->DestroyVideoReceiveStream(stream_);
		stream_ = nullptr;
	}

	webrtc::RTCVideoReceiveStream::Config config = config_.Copy();

	config.rtp.protected_by_flexfec = (flexfec_stream_ != nullptr);
	config.stream_id = stream_params_.id;
	stream_ = call_->CreateVideoReceiveStream(std::move(config));
	MaybeAssociateFlexfecWithVideo();

	stream_->Start();
}

void RTCWebRtcVideoChannel::RTCWebRtcVideoReceiveStream::MaybeRecreateWebRtcFlexfecStream() {
	if (flexfec_stream_) {
		MaybeDissociateFlexfecFromVideo();
		call_->DestroyFlexfecReceiveStream(flexfec_stream_);
		flexfec_stream_ = nullptr;
	}
	if (flexfec_config_.IsCompleteAndEnabled()) {
		flexfec_stream_ = call_->CreateFlexfecReceiveStream(flexfec_config_);
		MaybeAssociateFlexfecWithVideo();
	}
}

void RTCWebRtcVideoChannel::RTCWebRtcVideoReceiveStream::MaybeAssociateFlexfecWithVideo() {
	if (stream_ && flexfec_stream_) {
		stream_->AddSecondarySink(flexfec_stream_);
	}
}

void RTCWebRtcVideoChannel::RTCWebRtcVideoReceiveStream::MaybeDissociateFlexfecFromVideo() {
	if (stream_ && flexfec_stream_) {
		stream_->RemoveSecondarySink(flexfec_stream_);
	}
}

std::string 
RTCWebRtcVideoChannel::RTCWebRtcVideoReceiveStream::GetCodecNameFromPayloadType(
	int payload_type) {
	for (const webrtc::RTCVideoReceiveStream::Decoder& decoder : config_.decoders) {
		if (decoder.payload_type == payload_type) {
			return decoder.video_format.name;
		}
	}
	return "";
}


}