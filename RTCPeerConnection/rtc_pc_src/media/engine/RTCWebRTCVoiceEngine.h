//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /media/engine/webrtc_voice_engine.h
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_MEDIA_ENGINE_WEBRTC_VOICE_ENGINE_H__
#define __RTC_MEDIA_ENGINE_WEBRTC_VOICE_ENGINE_H__

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "../rtc_pc_src/_deprecate_defines.h"

#include "api/audio_codecs/audio_codec_pair_id.h"
#include "api/scoped_refptr.h"
#include "api/task_queue/task_queue_factory.h"
#include "api/transport/rtp/rtp_source.h"
#include "media/base/rtp_utils.h"
#include "rtc_base/buffer.h"
#include "rtc_base/network_route.h"
#include "rtc_base/task_queue.h"
#include "rtc_base/task_utils/pending_task_safety_flag.h"
#include "rtc_base/thread_checker.h"

#include "../../../src_update/media/base/_media_channel.h"
#include "../../../src_update/media/base/_media_engine.h"
#include "../../call/RTCAudioSendStream.h"
#include "../../call/RTCAudioReceiveStream.h"

#include "../rtc_pc_src/RTCThreadManagerInterface.h"

namespace cricket {

class AudioSource;
class RTCWebRtcVoiceMediaChannel;
class RTCAudioRtpPacketListenSinkInterface;

class RTCWebRtcVoiceEngine final : public VoiceEngineInterface {
	friend class RTCWebRtcVoiceMediaChannel;

public:
	RTCWebRtcVoiceEngine(
		webrtc::TaskQueueFactory* task_queue_factory,
		std::vector<AudioCodec>	_audio_decoder_codecs,
		std::vector<AudioCodec> _audio_encoder_codecs
		);

	RTCWebRtcVoiceEngine() = delete;
	RTCWebRtcVoiceEngine(const RTCWebRtcVoiceEngine&) = delete;
	RTCWebRtcVoiceEngine& operator=(const RTCWebRtcVoiceEngine&) = delete;

	~RTCWebRtcVoiceEngine() override;

	void Init() override;

	VoiceMediaChannel* CreateMediaChannel(
		webrtc::RTCCall* call,
		const MediaConfig& config,
		const AudioOptions& options,
		const webrtc::CryptoOptions& crypto_options,
		rtc::RTCThreadManagerInterface* rtc_thread_manager
		) override;

	const std::vector<AudioCodec>& send_codecs() const override;
	const std::vector<AudioCodec>& recv_codecs() const override;
	std::vector<webrtc::RtpHeaderExtensionCapability> GetRtpHeaderExtensions()
		const override;

	void RegisterChannel(RTCWebRtcVoiceMediaChannel* channel);
	void UnregisterChannel(RTCWebRtcVoiceMediaChannel* channel);

private:
	bool ApplyOptions(const AudioOptions& options);
	int CreateVoEChannel();

	std::vector<AudioCodec> CollectCodecs(
		const std::vector<webrtc::AudioCodecSpec>& specs) const;

private:
	webrtc::TaskQueueFactory* const task_queue_factory_;
	std::unique_ptr<rtc::TaskQueue> low_priority_worker_queue_;

	rtc::ThreadChecker signal_thread_checker_;
	rtc::ThreadChecker worker_thread_checker_;

	std::vector<AudioCodec> audio_decoder_codecs_;
	std::vector<AudioCodec> send_codecs_;
	std::vector<AudioCodec> recv_codecs_;
	std::vector<RTCWebRtcVoiceMediaChannel*> channels_;

	bool initialized_ = false;

	size_t audio_jitter_buffer_max_packets_ = 200;
	bool audio_jitter_buffer_fast_accelerate_ = false;
	int audio_jitter_buffer_min_delay_ms_ = 0;
	bool audio_jitter_buffer_enable_rtx_handling_ = false;

	RTCAudioRtpPacketListenSinkInterface *rtp_packet_listen_sink_;
};

class RTCWebRtcVoiceMediaChannel final : public VoiceMediaChannel,
										 public webrtc::Transport {
public:
	RTCWebRtcVoiceMediaChannel(RTCWebRtcVoiceEngine* engine,
		const MediaConfig& config,
		const AudioOptions& options,
		const webrtc::CryptoOptions& crypto_options,
		webrtc::RTCCall* call,
		rtc::RTCThreadManagerInterface* rtc_thread_manager
		);

	RTCWebRtcVoiceMediaChannel() = delete;
	RTCWebRtcVoiceMediaChannel(const RTCWebRtcVoiceMediaChannel&) = delete;
	RTCWebRtcVoiceMediaChannel& operator=(const RTCWebRtcVoiceMediaChannel&) = delete;

	~RTCWebRtcVoiceMediaChannel() override;

	const AudioOptions& options() const { return options_; }

	bool SetSendParameters(const AudioSendParameters& params) override;
	bool SetRecvParameters(const AudioRecvParameters& params) override;
	webrtc::RtpParameters GetRtpSendParameters(uint32_t ssrc) const override;
	webrtc::RTCError SetRtpSendParameters(
		uint32_t ssrc,
		const webrtc::RtpParameters& parameters) override;
	webrtc::RtpParameters GetRtpReceiveParameters(uint32_t ssrc) const override;
	webrtc::RtpParameters GetDefaultRtpReceiveParameters() const override;

	void SetPlayout(bool playout) override;	
	void SetSend(bool send) override;

	bool SetAudioSend(uint32_t ssrc,
		rtc::RTCAudioPacketRouteSourceInterface<std::unique_ptr<webrtc::RtpPacketReceived>>* source) override;

	bool AddSendStream(const StreamParams& sp) override;
	bool RemoveSendStream(uint32_t ssrc) override;
	bool AddRecvStream(const StreamParams& sp) override;
	bool RemoveRecvStream(uint32_t ssrc) override;
	void ResetUnsignaledRecvStream() override;

	bool SetBaseMinimumPlayoutDelayMs(uint32_t ssrc, int delay_ms) override;

	absl::optional<int> GetBaseMinimumPlayoutDelayMs(
		uint32_t ssrc) const override;

	void OnPacketReceived(rtc::CopyOnWriteBuffer packet,
		int64_t packet_time_us) override;
	void OnNetworkRouteChanged(const std::string& transport_name,
		const rtc::NetworkRoute& network_route) override;
	void OnReadyToSend(bool ready) override;
	bool GetStats(VoiceMediaInfo* info, bool get_and_clear_legacy_stats) override;

	std::vector<webrtc::RtpSource> GetSources(uint32_t ssrc) const override;

	bool SendRtp(const uint8_t* data,
		size_t len,
		const webrtc::PacketOptions& options) override {
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
		return VoiceMediaChannel::SendPacket(&packet, rtc_options);
	}

	bool SendRtcp(const uint8_t* data, size_t len) override {
		rtc::CopyOnWriteBuffer packet(data, len, kMaxRtpPacketLen);
		rtc::PacketOptions rtc_options;
		if (DscpEnabled()) {
			rtc_options.dscp = PreferredDscp();
		}

		return VoiceMediaChannel::SendRtcp(&packet, rtc_options);
	}

private:
	bool SetRecvCodecs(const std::vector<AudioCodec>& codecs);
	bool SetSendCodecs(const std::vector<AudioCodec>& codecs);

	bool SetLocalSource(uint32_t ssrc, rtc::RTCAudioPacketRouteSourceInterface<std::unique_ptr<webrtc::RtpPacketReceived>>* source);

	bool MuteStream(uint32_t ssrc, bool mute);

	RTCWebRtcVoiceEngine* engine() { return engine_; }
	int CreateVoEChannel();
	bool DeleteVoEChannel(int channel);
	void SetupRecording();
	// Check if 'ssrc' is an unsignaled stream, and if so mark it as not being
	// unsignaled anymore (i.e. it is now removed, or signaled), and return true.
	bool MaybeDeregisterUnsignaledRecvStream(uint32_t ssrc);

private:
	webrtc::TaskQueueBase* const worker_thread_;
	webrtc::ScopedTaskSafety task_safety_;
	rtc::ThreadChecker network_thread_checker_;

	RTCWebRtcVoiceEngine* const engine_ = nullptr;
	std::vector<AudioCodec> send_codecs_;

	// TODO(kwiberg): decoder_map_ and recv_codecs_ store the exact same
	// information, in slightly different formats. Eliminate recv_codecs_.
	std::map<int, webrtc::SdpAudioFormat> decoder_map_;
	std::vector<AudioCodec> recv_codecs_;

	int max_send_bitrate_bps_ = 0;
	AudioOptions options_;
	bool recv_transport_cc_enabled_ = false;
	bool recv_nack_enabled_ = false;
	bool playout_ = false;
	bool send_ = false;

	webrtc::RTCCall* const call_ = nullptr;

	const MediaConfig::Audio audio_config_;
	std::vector<uint32_t> unsignaled_recv_ssrcs_;

	StreamParams unsignaled_stream_params_;

	double default_recv_volume_ = 1.0;
	int default_recv_base_minimum_delay_ms_ = 0;

	uint32_t receiver_reports_ssrc_ = 0xFA17FA17u;

	class RTCWebRtcAudioSendStream;
	std::map<uint32_t, RTCWebRtcAudioSendStream*> send_streams_;
	std::vector<webrtc::RtpExtension> send_rtp_extensions_;
	std::string mid_;

	class RTCWebRtcAudioReceiveStream;
	std::map<uint32_t, RTCWebRtcAudioReceiveStream*> recv_streams_;
	std::vector<webrtc::RtpExtension> recv_rtp_extensions_;


 	absl::optional<webrtc::RTCAudioSendStream::Config::SendCodecSpec> send_codec_spec_;

	const webrtc::AudioCodecPairId codec_pair_id_ = webrtc::AudioCodecPairId::Create();			

	const webrtc::CryptoOptions crypto_options_;
};

}



#endif	// __RTC_MEDIA_ENGINE_WEBRTC_VOICE_ENGINE_H__
