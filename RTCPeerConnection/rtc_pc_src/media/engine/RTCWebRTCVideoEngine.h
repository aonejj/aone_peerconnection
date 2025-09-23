//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /media/engine/webrtc_video_engine.h
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_MEDIA_ENGINE_WEBRTC_VIDEO_ENGEINE_H__
#define __RTC_MEDIA_ENGINE_WEBRTC_VIDEO_ENGEINE_H__

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "../../_deprecate_defines.h"		// deprecate

#include "absl/types/optional.h"

#include "api/crypto/crypto_options.h"
#include "api/call/transport.h"
#include "api/video_codecs/video_encoder_config.h"
#include "rtc_base/thread.h"
#include "rtc_base/thread_checker.h"
#include "call/rtp_config.h"
#include "media/base/media_config.h"
#include "media/engine/unhandled_packets_buffer.h"

#include "../../../src_update/media/base/_media_channel.h"
#include "../../../src_update/media/base/_media_engine.h"
#include "../../call/RTCCall.h"
#include "../../call/RTCVideoReceiveStream.h"
#include "../../call/RTCVideoSendStream.h"

#include "../rtc_pc_src/RTCThreadManagerInterface.h"

namespace cricket {

class RTCWebRtcVideoChannel;

class UnsignalledSsrcHandler {
public:
	enum Action {
		kDropPacket,
		kDeliverPacket,
	};
	virtual Action OnUnsignalledSsrc(RTCWebRtcVideoChannel* channel,
		uint32_t ssrc) = 0;
	virtual ~UnsignalledSsrcHandler() = default;
};

class DefaultUnsignalledSsrcHandler : public UnsignalledSsrcHandler {
public:
	DefaultUnsignalledSsrcHandler();
	Action OnUnsignalledSsrc(RTCWebRtcVideoChannel* channel, uint32_t ssrc) override;
};

class RTCWebRtcVideoEngine : public VideoEngineInterface {
public:
	// These video codec factories represents all video codecs, i.e. both software
	// and external hardware codecs.
	RTCWebRtcVideoEngine(
		std::vector<VideoCodec> video_decoder_codecs,
		std::vector<VideoCodec> video_encoder_codecs);

	~RTCWebRtcVideoEngine() override;

	VideoMediaChannel* CreateMediaChannel(
		webrtc::RTCCall* call,
		const MediaConfig& config,
		const webrtc::CryptoOptions& crypto_options
		, rtc::RTCThreadManagerInterface* rtc_thread_manager
		)
		override;

	std::vector<VideoCodec> send_codecs() const override;
	std::vector<VideoCodec> recv_codecs() const override;
	std::vector<webrtc::RtpHeaderExtensionCapability>
		GetRtpHeaderExtensions()const override;

private:
	std::vector<VideoCodec>	recv_codecs_;	
	std::vector<VideoCodec>	send_codecs_;	
};

class RTCWebRtcVideoChannel : public VideoMediaChannel,
	public webrtc::Transport {
public:
	RTCWebRtcVideoChannel(
		webrtc::RTCCall* call,
		const MediaConfig& config,
		const webrtc::CryptoOptions& crypto_options,
		std::vector<VideoCodec> video_decoder_codecs,
		std::vector<VideoCodec> video_encoder_codecs,
		rtc::RTCThreadManagerInterface* rtc_thread_manager
		);
	~RTCWebRtcVideoChannel() override;

	// VideoMediaChannel implementation
	bool SetSendParameters(const VideoSendParameters& params) override;
	bool SetRecvParameters(const VideoRecvParameters& params) override;
	webrtc::RtpParameters GetRtpSendParameters(uint32_t ssrc) const override;
	webrtc::RTCError SetRtpSendParameters(
		uint32_t ssrc,
		const webrtc::RtpParameters& parameters) override;
	webrtc::RtpParameters GetRtpReceiveParameters(uint32_t ssrc) const override;
	webrtc::RtpParameters GetDefaultRtpReceiveParameters() const override;
	bool GetSendCodec(VideoCodec* send_codec) override;
	bool SetSend(bool send) override;
	bool SetVideoSend(uint32_t ssrc, rtc::RTCVideoPacketRouteSourceInterface<std::unique_ptr<webrtc::RtpPacketReceived>>* source) override;
	bool AddSendStream(const StreamParams& sp) override;
	bool RemoveSendStream(uint32_t ssrc) override;
	bool AddRecvStream(const StreamParams& sp) override;
	bool AddRecvStream(const StreamParams& sp, bool default_stream);
	bool RemoveRecvStream(uint32_t ssrc) override;
	void ResetUnsignaledRecvStream() override;

	bool SetPacketSink(uint32_t ssrc) override;	
	bool GetStats(VideoMediaInfo* info) override;

	void OnPacketReceived(rtc::CopyOnWriteBuffer packet,
		int64_t packet_time_us) override;

	void OnReadyToSend(bool ready) override;
	void OnNetworkRouteChanged(const std::string& transport_name,
		const rtc::NetworkRoute& network_route) override;
	void SetInterface(NetworkInterface* iface) override;

	absl::optional<uint32_t> GetDefaultReceiveStreamSsrc();

	StreamParams unsignaled_stream_params() {
		RTC_DCHECK_RUN_ON(&thread_checker_);
		return unsignaled_stream_params_;
	}

	std::vector<webrtc::RtpSource> GetSources(uint32_t ssrc) const override;

	virtual bool SetBaseMinimumPlayoutDelayMs(uint32_t ssrc, int delay_ms) override;

	virtual absl::optional<int> GetBaseMinimumPlayoutDelayMs(uint32_t ssrc) const override;

//	void BackfillBufferedPackets(rtc::ArrayView<const uint32_t> ssrcs);		// kimi unused

	void GenerateKeyFrame(uint32_t ssrc) override;

private:
	class RTCWebRtcVideoReceiveStream;

	RTCWebRtcVideoReceiveStream* FindReceiveStream(uint32_t ssrc)
		RTC_EXCLUSIVE_LOCKS_REQUIRED(thread_checker_);

	struct VideoCodecSettings {
		VideoCodecSettings();

		// Checks if all members of |*this| are equal to the corresponding members
		// of |other|.
		bool operator==(const VideoCodecSettings& other) const;
		bool operator!=(const VideoCodecSettings& other) const;

		// Checks if all members of |a|, except |flexfec_payload_type|, are equal
		// to the corresponding members of |b|.
		static bool EqualsDisregardingFlexfec(const VideoCodecSettings& a,
			const VideoCodecSettings& b);

		VideoCodec codec;
		webrtc::UlpfecConfig ulpfec;
		int flexfec_payload_type;  // -1 if absent.
		int rtx_payload_type;      // -1 if absent.
	};

	struct ChangedSendParameters {
		// These optionals are unset if not changed.
		absl::optional<VideoCodecSettings> send_codec;
		absl::optional<std::vector<VideoCodecSettings>> negotiated_codecs;
		absl::optional<std::vector<webrtc::RtpExtension>> rtp_header_extensions;
		absl::optional<std::string> mid;
		absl::optional<bool> extmap_allow_mixed;
		absl::optional<int> max_bandwidth_bps;
		absl::optional<bool> conference_mode;
		absl::optional<webrtc::RtcpMode> rtcp_mode;
	};

	struct ChangedRecvParameters {
		// These optionals are unset if not changed.
		absl::optional<std::vector<VideoCodecSettings>> codec_settings;
		absl::optional<std::vector<webrtc::RtpExtension>> rtp_header_extensions;
		// Keep track of the FlexFEC payload type separately from |codec_settings|.
		// This allows us to recreate the FlexfecReceiveStream separately from the
		// VideoReceiveStream when the FlexFEC payload type is changed.
		absl::optional<int> flexfec_payload_type;
	};

	bool GetChangedSendParameters(const VideoSendParameters& params,
			ChangedSendParameters* changed_params) const
			RTC_EXCLUSIVE_LOCKS_REQUIRED(thread_checker_);
	bool ApplyChangedParams(const ChangedSendParameters& changed_params);
	bool GetChangedRecvParameters(const VideoRecvParameters& params,
		ChangedRecvParameters* changed_params) const
		RTC_EXCLUSIVE_LOCKS_REQUIRED(thread_checker_);

	void ConfigureReceiverRtp(
		webrtc::RTCVideoReceiveStream::Config* config,
		webrtc::FlexfecReceiveStream::Config* flexfec_config,
		const StreamParams& sp) const;

	bool ValidateSendSsrcAvailability(const StreamParams& sp) const
		RTC_EXCLUSIVE_LOCKS_REQUIRED(thread_checker_);
	bool ValidateReceiveSsrcAvailability(const StreamParams& sp) const
		RTC_EXCLUSIVE_LOCKS_REQUIRED(thread_checker_);
	void DeleteReceiveStream(RTCWebRtcVideoReceiveStream* stream)
		RTC_EXCLUSIVE_LOCKS_REQUIRED(thread_checker_);

	static std::string CodecSettingsVectorToString(
		const std::vector<VideoCodecSettings>& codecs);

	
	class RTCWebRtcVideoSendStream {
	public:
		RTCWebRtcVideoSendStream(webrtc::RTCCall* call,
			const StreamParams& sp,
			webrtc::RTCVideoSendStream::Config config,
			const absl::optional<VideoCodecSettings>& codec_settings,
			const absl::optional<std::vector<webrtc::RtpExtension>>& rtp_extensions, 
			const VideoSendParameters& send_params,
			rtc::RTCThreadManagerInterface* rtc_thread_manager
			);
		~RTCWebRtcVideoSendStream();

	public:
		void SetSendParameters(const ChangedSendParameters& send_params);
		webrtc::RTCError SetRtpParameters(const webrtc::RtpParameters& parameters);
		webrtc::RtpParameters GetRtpParameters() const;

		bool SetVideoSend(rtc::RTCVideoPacketRouteSourceInterface<std::unique_ptr<webrtc::RtpPacketReceived>>* source);

		void SetSend(bool send);

		const std::vector<uint32_t>& GetSsrcs() const;
		std::vector<VideoSenderInfo> GetPerLayerVideoSenderInfos(bool log_stats);
		VideoSenderInfo GetAggregatedVideoSenderInfo(
			const std::vector<VideoSenderInfo>& infos) const;

	private:
		struct VideoSendStreamParameters {
			VideoSendStreamParameters(
				webrtc::RTCVideoSendStream::Config config,
				const absl::optional<VideoCodecSettings>& codec_settings);
			webrtc::RTCVideoSendStream::Config config;
			bool conference_mode;
			absl::optional<VideoCodecSettings> codec_settings;
			webrtc::VideoEncoderConfig encoder_config;
		};


		void SetCodec(const VideoCodecSettings& codec);
		void RecreateWebRtcStream();
		webrtc::VideoEncoderConfig CreateVideoEncoderConfig(
			const VideoCodec& codec) const;
		void UpdateSendState();

		webrtc::DegradationPreference GetDegradationPreference() const
			RTC_EXCLUSIVE_LOCKS_REQUIRED(&thread_checker_);

	private:
		rtc::ThreadChecker thread_checker_;
		rtc::Thread* worker_thread_;
		const std::vector<uint32_t> ssrcs_ RTC_GUARDED_BY(&thread_checker_);
		const std::vector<SsrcGroup> ssrc_groups_ RTC_GUARDED_BY(&thread_checker_);
		webrtc::RTCCall* const call_;

		webrtc::RTCVideoSendStream* stream_ RTC_GUARDED_BY(&thread_checker_);
		VideoSendStreamParameters parameters_ RTC_GUARDED_BY(&thread_checker_);

		webrtc::RtpParameters rtp_parameters_ RTC_GUARDED_BY(&thread_checker_);

		rtc::RTCVideoPacketRouteSourceInterface<std::unique_ptr<webrtc::RtpPacketReceived>>* source_
			RTC_GUARDED_BY(&thread_checker_);

		bool sending_ RTC_GUARDED_BY(&thread_checker_);
	};

	class RTCWebRtcVideoReceiveStream {
	public:
		RTCWebRtcVideoReceiveStream(
			RTCWebRtcVideoChannel* channel,
			webrtc::RTCCall* call,
			const StreamParams& sp,
			webrtc::RTCVideoReceiveStream::Config config, 
			bool default_stream,
			const std::vector<VideoCodecSettings>& recv_codecs,
			const webrtc::FlexfecReceiveStream::Config& flexfec_config);

		~RTCWebRtcVideoReceiveStream();

	public:
		const std::vector<uint32_t>& GetSsrcs() const;

		std::vector<webrtc::RtpSource> GetSources();

		// Does not return codecs, they are filled by the owning WebRtcVideoChannel.
		webrtc::RtpParameters GetRtpParameters() const;

		void SetLocalSsrc(uint32_t local_ssrc);
		// TODO(deadbeef): Move these feedback parameters into the recv parameters.
		void SetFeedbackParameters(bool lntf_enabled,
			bool nack_enabled,
			bool transport_cc_enabled,
			webrtc::RtcpMode rtcp_mode);
		void SetRecvParameters(const ChangedRecvParameters& recv_params);
		bool IsDefaultStream() const;

		VideoReceiverInfo GetVideoReceiverInfo(bool log_stats);
		void GenerateKeyFrame();

	private:
		void RecreateWebRtcVideoStream();
		void MaybeRecreateWebRtcFlexfecStream();

		void MaybeAssociateFlexfecWithVideo();
		void MaybeDissociateFlexfecFromVideo();

		void ConfigureCodecs(const std::vector<VideoCodecSettings>& recv_codecs);
		void ConfigureFlexfecCodec(int flexfec_payload_type);


		std::string GetCodecNameFromPayloadType(int payload_type);

	private:
		RTCWebRtcVideoChannel* const channel_;
		webrtc::RTCCall* const call_;
		const StreamParams stream_params_;

		// Both |stream_| and |flexfec_stream_| are managed by |this|. They are
		// destroyed by calling call_->DestroyVideoReceiveStream and
		// call_->DestroyFlexfecReceiveStream, respectively.
		webrtc::RTCVideoReceiveStream* stream_;
		const bool default_stream_;
		webrtc::RTCVideoReceiveStream::Config config_;
		webrtc::FlexfecReceiveStream::Config flexfec_config_;
		webrtc::FlexfecReceiveStream* flexfec_stream_;


		webrtc::Mutex sink_lock_;
		// Expands remote RTP timestamps to int64_t to be able to estimate how long
		// the stream has been running.
		rtc::TimestampWrapAroundHandler timestamp_wraparound_handler_
			RTC_GUARDED_BY(sink_lock_);
		int64_t first_frame_timestamp_ RTC_GUARDED_BY(sink_lock_);
		// Start NTP time is estimated as current remote NTP time (estimated from
		// RTCP) minus the elapsed time, as soon as remote NTP time is available.
		int64_t estimated_remote_start_ntp_time_ms_ RTC_GUARDED_BY(sink_lock_);

	};

	void Construct(webrtc::RTCCall* call, RTCWebRtcVideoEngine *engine);	

	bool SendRtp(const uint8_t* data,
		size_t len,
		const webrtc::PacketOptions& options) override;
	bool SendRtcp(const uint8_t* data, size_t len) override;

	static std::vector<VideoCodecSettings> MapCodecs(
		const std::vector<VideoCodec>& codecs);


	std::vector<VideoCodecSettings> SelectSendVideoCodecs(
			const std::vector<VideoCodecSettings>& remote_mapped_codecs) const
			RTC_EXCLUSIVE_LOCKS_REQUIRED(thread_checker_);

	static bool NonFlexfecReceiveCodecsHaveChanged(
		std::vector<VideoCodecSettings> before,
		std::vector<VideoCodecSettings> after);

	void FillSenderStats(VideoMediaInfo* info, bool log_stats)
		RTC_EXCLUSIVE_LOCKS_REQUIRED(thread_checker_);
	void FillReceiverStats(VideoMediaInfo* info, bool log_stats)
		RTC_EXCLUSIVE_LOCKS_REQUIRED(thread_checker_);
	void FillSendAndReceiveCodecStats(VideoMediaInfo* video_media_info)
		RTC_EXCLUSIVE_LOCKS_REQUIRED(thread_checker_);

private:
	rtc::Thread* const worker_thread_;
	webrtc::ScopedTaskSafety task_safety_;
	rtc::ThreadChecker network_thread_checker_;
	rtc::ThreadChecker thread_checker_;


	uint32_t rtcp_receiver_report_ssrc_ RTC_GUARDED_BY(thread_checker_);
	bool sending_ RTC_GUARDED_BY(thread_checker_);

	DefaultUnsignalledSsrcHandler default_unsignalled_ssrc_handler_
		RTC_GUARDED_BY(thread_checker_);
	UnsignalledSsrcHandler* const unsignalled_ssrc_handler_
		RTC_GUARDED_BY(thread_checker_);

	int default_recv_base_minimum_delay_ms_ RTC_GUARDED_BY(thread_checker_) = 0;

	const MediaConfig::Video video_config_ RTC_GUARDED_BY(thread_checker_);

	// Using primary-ssrc (first ssrc) as key.
	std::map<uint32_t, RTCWebRtcVideoSendStream*> send_streams_
		RTC_GUARDED_BY(thread_checker_);
	std::map<uint32_t, RTCWebRtcVideoReceiveStream*> receive_streams_
		RTC_GUARDED_BY(thread_checker_);

	std::set<uint32_t> send_ssrcs_ RTC_GUARDED_BY(thread_checker_);
	std::set<uint32_t> receive_ssrcs_ RTC_GUARDED_BY(thread_checker_);

	absl::optional<VideoCodecSettings> send_codec_
		RTC_GUARDED_BY(thread_checker_);
	std::vector<VideoCodecSettings> negotiated_codecs_
		RTC_GUARDED_BY(thread_checker_);

	absl::optional<std::vector<webrtc::RtpExtension>> send_rtp_extensions_
		RTC_GUARDED_BY(thread_checker_);

	std::vector<VideoCodecSettings> recv_codecs_ RTC_GUARDED_BY(thread_checker_);
	std::vector<webrtc::RtpExtension> recv_rtp_extensions_
		RTC_GUARDED_BY(thread_checker_);

#ifdef __ROUTING_PACKET_PATH_USING_NETWORK_THREAD__
	int recv_flexfec_payload_type_;
#else
	int recv_flexfec_payload_type_ RTC_GUARDED_BY(thread_checker_);
#endif

	VideoSendParameters send_params_ RTC_GUARDED_BY(thread_checker_);
	VideoRecvParameters recv_params_ RTC_GUARDED_BY(thread_checker_);

#ifdef __ROUTING_PACKET_PATH_USING_NETWORK_THREAD__
	const bool discard_unknown_ssrc_packets_;
#else
	const bool discard_unknown_ssrc_packets_ RTC_GUARDED_BY(thread_checker_);
#endif

	StreamParams unsignaled_stream_params_ RTC_GUARDED_BY(thread_checker_);

	const webrtc::CryptoOptions crypto_options_ RTC_GUARDED_BY(thread_checker_);

	/* // kimi unused 
	std::unique_ptr<UnhandledPacketsBuffer> unknown_ssrc_packet_buffer_
		RTC_GUARDED_BY(thread_checker_);
	*/

	webrtc::RTCCall* const call_ RTC_GUARDED_BY(thread_checker_);

	std::vector<VideoCodec> video_decoder_codecs_;
	std::vector<VideoCodec> video_encoder_codecs_;

	rtc::RTCThreadManagerInterface* _rtc_thread_manager;
};

}

#endif	// __RTC_MEDIA_ENGINE_WEBRTC_VIDEO_ENGEINE_H__