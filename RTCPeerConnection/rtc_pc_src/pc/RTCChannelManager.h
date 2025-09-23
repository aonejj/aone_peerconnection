//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /pc/channel_manager.h
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_CHANNEL_MANAGER_H__
#define __RTC_CHANNEL_MANAGER_H__

#include <stdint.h>

#include <memory>
#include <string>
#include <vector>

#include "../rtc_pc_src/_deprecate_defines.h"

#include "api/rtp_parameters.h"
#include "api/crypto/crypto_options.h"
#include "media/base/codec.h"
#include "pc/session_description.h"
#include "rtc_base/thread.h"

#include "../../src_update/media/base/_media_engine.h"
#include "RTCChannel.h"
#include "../call/RTCCall.h"

#include "../rtc_pc_src/RTCThreadManagerInterface.h"

namespace cricket {

class RTCChannelManager final {
public:
	RTCChannelManager(std::unique_ptr<MediaEngineInterface> media_engine,
				   std::unique_ptr<DataEngineInterface> data_engine,
				   rtc::Thread* worker_thread,
				   rtc::Thread* network_thread,
				   rtc::RTCThreadManagerInterface* rtc_thread_manager
				   );
	~RTCChannelManager();

public:
	rtc::Thread* worker_thread() const { return worker_thread_; }
	bool set_worker_thread(rtc::Thread* thread) {
		if (initialized_) {
			return false;
		}
		worker_thread_ = thread;
		return true;
	}
	rtc::Thread* network_thread() const { return network_thread_; }
	bool set_network_thread(rtc::Thread* thread) {
		if (initialized_) {
			return false;
		}
		network_thread_ = thread;
		return true;
	}

	MediaEngineInterface* media_engine() { return media_engine_.get(); }

	// Retrieves the list of supported audio & video codec types.
	// Can be called before starting the media engine.
	void GetSupportedAudioSendCodecs(std::vector<AudioCodec>* codecs) const;
	void GetSupportedAudioReceiveCodecs(std::vector<AudioCodec>* codecs) const;
	void GetSupportedVideoSendCodecs(std::vector<VideoCodec>* codecs) const;
	void GetSupportedVideoReceiveCodecs(std::vector<VideoCodec>* codecs) const;
	void GetSupportedDataCodecs(std::vector<DataCodec>* codecs) const;
	RtpHeaderExtensions GetDefaultEnabledAudioRtpHeaderExtensions() const;
	std::vector<webrtc::RtpHeaderExtensionCapability>
		GetSupportedAudioRtpHeaderExtensions() const;
	RtpHeaderExtensions GetDefaultEnabledVideoRtpHeaderExtensions() const;
	std::vector<webrtc::RtpHeaderExtensionCapability>
		GetSupportedVideoRtpHeaderExtensions() const;

	// Indicates whether the media engine is started.
	bool initialized() const { return initialized_; }
	// Starts up the media engine.
	bool Init();
	// Shuts down the media engine.
	void Terminate();


	RTCVoiceChannel* CreateVoiceChannel(
		webrtc::RTCCall* call,		
		const cricket::MediaConfig& media_config,
		webrtc::RtpTransportInternal* rtp_transport,
		rtc::Thread* signaling_thread,
		const std::string& content_name,
		bool srtp_required,
		const webrtc::CryptoOptions& crypto_options,
		rtc::UniqueRandomIdGenerator* ssrc_generator,
		const AudioOptions& options);
	// Destroys a voice channel created by CreateVoiceChannel.
	void DestroyVoiceChannel(RTCVoiceChannel* voice_channel);

	// Creates a video channel, synced with the specified voice channel, and
	// associated with the specified session.
	// Version of the above that takes PacketTransportInternal.
	RTCVideoChannel* CreateVideoChannel(
		webrtc::RTCCall* call,			
		const cricket::MediaConfig& media_config,
		webrtc::RtpTransportInternal* rtp_transport,
		rtc::Thread* signaling_thread,
		const std::string& content_name,
		bool srtp_required,
		const webrtc::CryptoOptions& crypto_options,
		rtc::UniqueRandomIdGenerator* ssrc_generator /*,
		const VideoOptions& options,
		webrtc::VideoBitrateAllocatorFactory* video_bitrate_allocator_factory*/);
	// Destroys a video channel created by CreateVideoChannel.
	void DestroyVideoChannel(RTCVideoChannel* video_channel);

	RTCRtpDataChannel* CreateRtpDataChannel(
		const cricket::MediaConfig& media_config,
		webrtc::RtpTransportInternal* rtp_transport,
		rtc::Thread* signaling_thread,
		const std::string& content_name,
		bool srtp_required,
		const webrtc::CryptoOptions& crypto_options,
		rtc::UniqueRandomIdGenerator* ssrc_generator);
	// Destroys a data channel created by CreateRtpDataChannel.
	void DestroyRtpDataChannel(RTCRtpDataChannel* data_channel);

	// Indicates whether any channels exist.
	bool has_channels() const {
		return (!voice_channels_.empty() || !video_channels_.empty() ||
			!data_channels_.empty());
	}

	// RTX will be enabled/disabled in engines that support it. The supporting
	// engines will start offering an RTX codec. Must be called before Init().
	bool SetVideoRtxEnabled(bool enable);

	// Starts/stops the local microphone and enables polling of the input level.
	bool capturing() const { return capturing_; }

private:
	std::unique_ptr<MediaEngineInterface> media_engine_;  // Nullable.
	std::unique_ptr<DataEngineInterface> data_engine_;    // Non-null.
	bool initialized_ = false;
	rtc::Thread* main_thread_;
	rtc::Thread* worker_thread_;
	rtc::Thread* network_thread_;

	// Vector contents are non-null.
	std::vector<std::unique_ptr<RTCVoiceChannel>> voice_channels_;
	std::vector<std::unique_ptr<RTCVideoChannel>> video_channels_;
	std::vector<std::unique_ptr<RTCRtpDataChannel>> data_channels_;

	bool enable_rtx_ = false;
	bool capturing_ = false;

	rtc::RTCThreadManagerInterface *_rtc_thread_manager;
};

}


#endif	// __RTC_CHANNEL_MANAGER_H__
