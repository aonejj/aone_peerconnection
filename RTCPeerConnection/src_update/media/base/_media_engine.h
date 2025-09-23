//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /media/base/media_engine.h
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_UPDATE_MEDIA_BASE_MEDIA_ENGINE_H__
#define __RTC_UPDATE_MEDIA_BASE_MEDIA_ENGINE_H__

#include <memory>
#include <string>
#include <vector>

#include "../rtc_pc_src/_deprecate_defines.h"

#include "api/rtc_error.h"
#include "api/rtp_parameters.h"
#include "api/audio_options.h"
#include "api/crypto/crypto_options.h"
#include "media/base/media_config.h"
#include "media/base/codec.h"
#include "media/base/stream_params.h"
#include "rtc_base/constructor_magic.h"

#include "_media_channel.h"
#include "../../../rtc_pc_src/call/RTCCall.h"

#include "../rtc_pc_src/RTCThreadManagerInterface.h"


namespace cricket {

webrtc::RTCError CheckRtpParametersValues(
	const webrtc::RtpParameters& new_parameters);

webrtc::RTCError CheckRtpParametersInvalidModificationAndValues(
	const webrtc::RtpParameters& old_parameters,
	const webrtc::RtpParameters& new_parameters);

struct RtpCapabilities {
	RtpCapabilities();
	~RtpCapabilities();
	std::vector<webrtc::RtpExtension> header_extensions;
};

class RtpHeaderExtensionQueryInterface {
public:
	virtual ~RtpHeaderExtensionQueryInterface() = default;

	// Returns a vector of RtpHeaderExtensionCapability, whose direction is
	// kStopped if the extension is stopped (not used) by default.
	virtual std::vector<webrtc::RtpHeaderExtensionCapability>
		GetRtpHeaderExtensions() const = 0;
};

class VoiceEngineInterface : public RtpHeaderExtensionQueryInterface {
public:
	VoiceEngineInterface() = default;
	virtual ~VoiceEngineInterface() = default;
	RTC_DISALLOW_COPY_AND_ASSIGN(VoiceEngineInterface);

	// Initialization
	// Starts the engine.
	virtual void Init() = 0;

	// MediaChannel creation
	// Creates a voice media channel. Returns NULL on failure.
	virtual VoiceMediaChannel* CreateMediaChannel(
		webrtc::RTCCall* call,
		const MediaConfig& config,
		const AudioOptions& options,
		const webrtc::CryptoOptions& crypto_options,
		rtc::RTCThreadManagerInterface* rtc_thread_manager
		) = 0;

	virtual const std::vector<AudioCodec>& send_codecs() const = 0;
	virtual const std::vector<AudioCodec>& recv_codecs() const = 0;
};

class VideoEngineInterface : public RtpHeaderExtensionQueryInterface {
public:
	VideoEngineInterface() = default;
	virtual ~VideoEngineInterface() = default;
	RTC_DISALLOW_COPY_AND_ASSIGN(VideoEngineInterface);

	// Creates a video media channel, paired with the specified voice channel.
	// Returns NULL on failure.
	virtual VideoMediaChannel* CreateMediaChannel(
		webrtc::RTCCall* call,
		const MediaConfig& config,
		const webrtc::CryptoOptions& crypto_options,
		rtc::RTCThreadManagerInterface* rtc_thread_manager
		) = 0;

	virtual std::vector<VideoCodec> send_codecs() const = 0;
	virtual std::vector<VideoCodec> recv_codecs() const = 0;
};

class MediaEngineInterface {
public:
	virtual ~MediaEngineInterface() {}

	// Initialization
	// Starts the engine.
	virtual bool Init() = 0;
	virtual VoiceEngineInterface& voice() = 0;
	virtual VideoEngineInterface& video() = 0;
	virtual const VoiceEngineInterface& voice() const = 0;
	virtual const VideoEngineInterface& video() const = 0;
};

class CompositeMediaEngine : public MediaEngineInterface {
public:
	CompositeMediaEngine(std::unique_ptr<VoiceEngineInterface> audio_engine,
		std::unique_ptr<VideoEngineInterface> video_engine);
	~CompositeMediaEngine() override;
	bool Init() override;

	VoiceEngineInterface& voice() override;
	VideoEngineInterface& video() override;
	const VoiceEngineInterface& voice() const override;
	const VideoEngineInterface& video() const override;

private:
	std::unique_ptr<VoiceEngineInterface> voice_engine_;
	std::unique_ptr<VideoEngineInterface> video_engine_;
};

enum DataChannelType {
	DCT_NONE = 0,
	DCT_RTP = 1,
	DCT_SCTP = 2,
};

class DataEngineInterface {
public:
	virtual ~DataEngineInterface() {}
	virtual DataMediaChannel* CreateChannel(const MediaConfig& config) = 0;
	virtual const std::vector<DataCodec>& data_codecs() = 0;
};

webrtc::RtpParameters CreateRtpParametersWithOneEncoding();
webrtc::RtpParameters CreateRtpParametersWithEncodings(StreamParams sp);

std::vector<webrtc::RtpExtension> GetDefaultEnabledRtpHeaderExtensions(
	const RtpHeaderExtensionQueryInterface& query_interface);
}


#endif	// __RTC_UPDATE_MEDIA_BASE_MEDIA_ENGINE_H__