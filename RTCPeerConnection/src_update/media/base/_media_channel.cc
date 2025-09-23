//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /media/base/media_channel.cc update
//
//////////////////////////////////////////////////////////////////////////

#include "_media_channel.h"


namespace cricket {

MediaChannel::MediaChannel(const MediaConfig& config)
	: enable_dscp_(config.enable_dscp) {}

MediaChannel::MediaChannel() : enable_dscp_(false) {}

MediaChannel::~MediaChannel() {}

void MediaChannel::SetInterface(NetworkInterface* iface) {
	webrtc::MutexLock lock(&network_interface_mutex_);
	network_interface_ = iface;
	UpdateDscp();
}

int MediaChannel::GetRtpSendTimeExtnId() const {
	return -1;
}

void MediaChannel::SetVideoCodecSwitchingEnabled(bool enabled) {}

MediaSenderInfo::MediaSenderInfo() = default;
MediaSenderInfo::~MediaSenderInfo() = default;

MediaReceiverInfo::MediaReceiverInfo() = default;
MediaReceiverInfo::~MediaReceiverInfo() = default;

VoiceSenderInfo::VoiceSenderInfo() = default;
VoiceSenderInfo::~VoiceSenderInfo() = default;

VoiceReceiverInfo::VoiceReceiverInfo() = default;
VoiceReceiverInfo::~VoiceReceiverInfo() = default;

VideoSenderInfo::VideoSenderInfo() = default;
VideoSenderInfo::~VideoSenderInfo() = default;

VideoReceiverInfo::VideoReceiverInfo() = default;
VideoReceiverInfo::~VideoReceiverInfo() = default;

VoiceMediaInfo::VoiceMediaInfo() = default;
VoiceMediaInfo::~VoiceMediaInfo() = default;

VideoMediaInfo::VideoMediaInfo() = default;
VideoMediaInfo::~VideoMediaInfo() = default;

DataMediaInfo::DataMediaInfo() = default;
DataMediaInfo::~DataMediaInfo() = default;

AudioSendParameters::AudioSendParameters() = default;
AudioSendParameters::~AudioSendParameters() = default;

std::map<std::string, std::string> AudioSendParameters::ToStringMap() const {
	auto params = RtpSendParameters<AudioCodec>::ToStringMap();
	params["options"] = options.ToString();
	return params;
}

cricket::MediaType VoiceMediaChannel::media_type() const {
	return cricket::MediaType::MEDIA_TYPE_AUDIO;
}

VideoSendParameters::VideoSendParameters() = default;
VideoSendParameters::~VideoSendParameters() = default;

std::map<std::string, std::string> VideoSendParameters::ToStringMap() const {
	auto params = RtpSendParameters<VideoCodec>::ToStringMap();
	params["conference_mode"] = (conference_mode ? "yes" : "no");
	return params;
}

cricket::MediaType VideoMediaChannel::media_type() const {
	return cricket::MediaType::MEDIA_TYPE_VIDEO;
}

DataMediaChannel::DataMediaChannel() = default;
DataMediaChannel::DataMediaChannel(const MediaConfig& config)
	: MediaChannel(config) {}
DataMediaChannel::~DataMediaChannel() = default;

webrtc::RtpParameters DataMediaChannel::GetRtpSendParameters(
	uint32_t ssrc) const {
	// GetRtpSendParameters is not supported for DataMediaChannel.
	RTC_NOTREACHED();
	return webrtc::RtpParameters();
}
webrtc::RTCError DataMediaChannel::SetRtpSendParameters(
	uint32_t ssrc,
	const webrtc::RtpParameters& parameters) {
	// SetRtpSendParameters is not supported for DataMediaChannel.
	RTC_NOTREACHED();
	return webrtc::RTCError(webrtc::RTCErrorType::UNSUPPORTED_OPERATION);
}

cricket::MediaType DataMediaChannel::media_type() const {
	return cricket::MediaType::MEDIA_TYPE_DATA;
}

bool DataMediaChannel::GetStats(DataMediaInfo* info) {
	return true;
}

}