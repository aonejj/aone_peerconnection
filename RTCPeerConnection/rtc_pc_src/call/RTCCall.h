//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /call/call.h
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_CALL_CALL_H__
#define __RTC_CALL_CALL_H__

#include <atomic>

#include "../_deprecate_defines.h"

#include "api/media_types.h"
#include "api/task_queue/task_queue_factory.h"
#include "api/transport/network_control.h"
#include "api/scoped_refptr.h"
#include "api/video_codecs/video_encoder_config.h"
#include "call/packet_receiver.h"
#include "call/flexfec_receive_stream.h"
#include "call/rtp_stream_receiver_controller.h"
#include "call/receive_time_calculator.h"
#include "modules/rtp_rtcp/include/rtp_rtcp_defines.h"
#include "modules/rtp_rtcp/include/rtp_header_extension_map.h"
#include "rtc_base/task_utils/pending_task_safety_flag.h"
#include "rtc_base/constructor_magic.h"
#include "rtc_base/network/sent_packet.h"
#include "rtc_base/task_utils/repeating_task.h"

#include "system_wrappers/include/clock.h"
#include "video/stats_counter.h"
#include "video/call_stats2.h"

#include "RTCAudioSendStream.h"
#include "RTCVideoSendStream.h"
#include "RTCRtpTransportControllerSend.h"
#include "../api/call/RTCTransportSequenceNumberGenerator.h"
#include "../api/call/RTCGetNetworkEstimateInterface.h"
#include "../api/audio/RTCAudioRtpPacketListenSinkInterface.h"
#include "../api/video/RTCVideoRtpPacketListenSinkInterface.h"
#include "../audio/RTCAudioReceiveStreamImpl.h"
#include "../video/RTCVideoReceiveStream2Impl.h"
#include "../modules/congestion_controller/include/RTCReceiveSideCongestionController.h"

#ifdef __ROUTING_PACKET_PATH_USING_NETWORK_THREAD__
#include "RTCRtpStreamReceiverController.h"
#endif

#include "../RTCThreadManagerInterface.h"

namespace webrtc {

class RTCVideeRtpPacketListenerProxyGetInterface {
public:
	RTCVideeRtpPacketListenerProxyGetInterface() = default;
	~RTCVideeRtpPacketListenerProxyGetInterface() = default;

public:
	virtual RTCVideoRtpPacketListenSinkInterface* GetVideoRtpPacketProxyInterface(uint32_t ssrc) = 0;
};

class RTCCall final : public PacketReceiver,
					  public RecoveredPacketReceiver,
					  public TargetTransferRateObserver,
					  public RTCTransportSequenceNumberGenerator {
public:
	static RTCCall* Create(
		TaskQueueFactory* task_queue_factory,
		RTCAudioRtpPacketListenSinkInterface* audio_rtp_packet_listener,
		RTCVideeRtpPacketListenerProxyGetInterface* video_rtp_packet_listener_proxy_get_interface,
		rtc::RTCThreadManagerInterface* rtc_thread_manager
		);

	~RTCCall();

public:
	struct Stats {
		std::string ToString(int64_t time_ms) const;

		int send_bandwidth_bps = 0;       // Estimated available send bandwidth.
		int max_padding_bitrate_bps = 0;  // Cumulative configured max padding.
		int recv_bandwidth_bps = 0;       // Estimated available receive bandwidth.
		int64_t pacer_delay_ms = 0;
		int64_t rtt_ms = -1;
	};

public:

	PacketReceiver* Receiver();

	RTCAudioSendStream* CreateAudioSendStream(
		const RTCAudioSendStream::Config& config);
	void DestroyAudioSendStream(RTCAudioSendStream* send_stream);

	RTCAudioReceiveStream* CreateAudioReceiveStream(
		RTCAudioReceiveStream::Config& config);
	void DestroyAudioReceiveStream(RTCAudioReceiveStream* receive_stream);

	RTCVideoSendStream* CreateVideoSendStream(
		RTCVideoSendStream::Config config,
		VideoEncoderConfig encoder_config);
	void DestroyVideoSendStream(RTCVideoSendStream* send_stream);

	RTCVideoReceiveStream* CreateVideoReceiveStream(
		RTCVideoReceiveStream::Config config);
	void DestroyVideoReceiveStream(RTCVideoReceiveStream* receive_stream);

	FlexfecReceiveStream* CreateFlexfecReceiveStream(
		const FlexfecReceiveStream::Config& config);
	void DestroyFlexfecReceiveStream(
		FlexfecReceiveStream* receive_stream);

	RTCRtpTransportControllerSendInterface* GetTransportControllerSend();

	Stats GetStats() const;

	void GenerateKeyFrame();
	void GenerateKeyFramePli(uint32_t ssrc);

public:
	// Implements PacketReceiver.
	DeliveryStatus DeliverPacket(MediaType media_type,
		rtc::CopyOnWriteBuffer packet,
		int64_t packet_time_us) override;
	void DeliverPacketAsync(MediaType media_type,
		rtc::CopyOnWriteBuffer packet,
		int64_t packet_time_us,
		PacketCallback callback) override;

	// Implements RecoveredPacketReceiver.
	void OnRecoveredPacket(const uint8_t* packet, size_t length) override;

	void SignalChannelNetworkState(MediaType media, NetworkState state);

	void OnAudioTransportOverheadChanged(
			int transport_overhead_per_packet);

	void OnSentPacket(const rtc::SentPacket& sent_packet);

	// Implements TargetTransferRateObserver,
	void OnTargetTransferRate(TargetTransferRate msg) override;
	void OnStartRateUpdate(DataRate start_rate) override;

	// Implement RTCTransportSequenceNumberGenerator
	uint64_t GetTransportSequenceNumber() override;
private:
	RTCCall(Clock* clock,
		TaskQueueFactory* task_queue_factory,
		std::unique_ptr<RTCRtpTransportControllerSendInterface> transport_send,
		RTCAudioRtpPacketListenSinkInterface *audio_rtp_packet_listener,
		RTCVideeRtpPacketListenerProxyGetInterface* video_rtp_packet_listener_proxy_get_interface,
		rtc::RTCThreadManagerInterface* rtc_thread_manager
		);


private:
	DeliveryStatus DeliverRtcp(MediaType media_type,
		const uint8_t* packet,
		size_t length)
		RTC_EXCLUSIVE_LOCKS_REQUIRED(worker_thread_);
	DeliveryStatus DeliverRtp(MediaType media_type,
		rtc::CopyOnWriteBuffer packet,
		int64_t packet_time_us)
		RTC_EXCLUSIVE_LOCKS_REQUIRED(worker_thread_);

	void NotifyBweOfReceivedPacket(const RtpPacketReceived& packet,
		MediaType media_type)
		RTC_SHARED_LOCKS_REQUIRED(worker_thread_);

	void UpdateAggregateNetworkState();

	void EnsureStarted() RTC_EXCLUSIVE_LOCKS_REQUIRED(worker_thread_);

private:
	Clock* const clock_;
	TaskQueueFactory* const task_queue_factory_;
	TaskQueueBase* const worker_thread_;
	TaskQueueBase* const network_thread_;

	const std::unique_ptr<CallStats> call_stats_;

	NetworkState audio_network_state_;
	NetworkState video_network_state_;

	bool aggregate_network_up_ RTC_GUARDED_BY(worker_thread_);

	// Audio, Video, and FlexFEC receive streams are owned by the client that
	// creates them.
	std::set<RTCAudioReceiveStreamImpl*> audio_receive_streams_
		RTC_GUARDED_BY(worker_thread_);
	std::set<RTCVideoReceiveStream2Impl*> video_receive_streams_
		RTC_GUARDED_BY(worker_thread_);

	std::map<std::string, RTCAudioReceiveStreamImpl*> sync_stream_mapping_
		RTC_GUARDED_BY(worker_thread_);

#ifdef __ROUTING_PACKET_PATH_USING_NETWORK_THREAD__
	RTCRtpStreamReceiverController audio_receiver_controller_;
	RTCRtpStreamReceiverController video_receiver_controller_;
#else
	RtpStreamReceiverController audio_receiver_controller_;
	RtpStreamReceiverController video_receiver_controller_;
#endif

	struct ReceiveRtpConfig {
		explicit ReceiveRtpConfig(const RTCAudioReceiveStream::Config& config)
			: extensions(config.rtp.extensions) {}

		explicit ReceiveRtpConfig(const RTCVideoReceiveStream::Config& config)
			: extensions(config.rtp.extensions) {}

		explicit ReceiveRtpConfig(const FlexfecReceiveStream::Config& config)
			: extensions(config.rtp_header_extensions) {}

		// Registered RTP header extensions for each stream. Note that RTP header
		// extensions are negotiated per track ("m= line") in the SDP, but we have
		// no notion of tracks at the Call level. We therefore store the RTP header
		// extensions per SSRC instead, which leads to some storage overhead.
		const RtpHeaderExtensionMap extensions;
	};

#ifdef __ROUTING_PACKET_PATH_USING_NETWORK_THREAD__
	std::map<uint32_t, ReceiveRtpConfig> receive_rtp_config_;
	mutable Mutex recv_rtc_config_mutex_;
#else
	std::map<uint32_t, ReceiveRtpConfig> receive_rtp_config_
		RTC_GUARDED_BY(worker_thread_);
#endif

	// Audio and Video send streams are owned by the client that creates them.
	std::map<uint32_t, RTCAudioSendStream*> audio_send_ssrcs_
		RTC_GUARDED_BY(worker_thread_);
	std::map<uint32_t, RTCVideoSendStream*> video_send_ssrcs_
		RTC_GUARDED_BY(worker_thread_);
	std::set<RTCVideoSendStream*> video_send_streams_ RTC_GUARDED_BY(worker_thread_);

	using RtpStateMap = std::map<uint32_t, RtpState>;
	RtpStateMap suspended_audio_send_ssrcs_ RTC_GUARDED_BY(worker_thread_);
	RtpStateMap suspended_video_send_ssrcs_ RTC_GUARDED_BY(worker_thread_);

	RateCounter received_bytes_per_second_counter_;
	RateCounter received_audio_bytes_per_second_counter_;
	RateCounter received_video_bytes_per_second_counter_;
	RateCounter received_rtcp_bytes_per_second_counter_;
	absl::optional<int64_t> first_received_rtp_audio_ms_;
	absl::optional<int64_t> last_received_rtp_audio_ms_;
	absl::optional<int64_t> first_received_rtp_video_ms_;
	absl::optional<int64_t> last_received_rtp_video_ms_;

	uint32_t last_bandwidth_bps_ RTC_GUARDED_BY(worker_thread_);

	RTCReceiveSideCongestionController receive_side_cc_;
	RepeatingTaskHandle receive_side_cc_periodic_task_;

	const std::unique_ptr<ReceiveTimeCalculator> receive_time_calculator_;
	const int64_t start_ms_;

	ScopedTaskSafety task_safety_;

	bool is_started_ RTC_GUARDED_BY(worker_thread_) = false;

	RTCAudioRtpPacketListenSinkInterface *audio_rtp_packet_listener_;

	RTCVideeRtpPacketListenerProxyGetInterface* video_rtp_packet_listener_proxy_get_interface_;

	RTCRtpTransportControllerSendInterface* const transport_send_ptr_;
	std::unique_ptr<RTCRtpTransportControllerSendInterface> transport_send_;

	std::atomic<uint64_t> transport_sequence_number_;

	volatile mutable uint32_t _recv_network_estimate_bps = 0;
	volatile mutable uint32_t _send_network_estimate_bps = 0;

//	uint32_t _rr_estimate_ssrc = 0;		

	RTC_NO_UNIQUE_ADDRESS SequenceChecker sent_packet_sequence_checker_;
	absl::optional<rtc::SentPacket> last_sent_packet_
		RTC_GUARDED_BY(sent_packet_sequence_checker_);

	// kimi for debug...
	bool debug_diff_prev_ = false;
	int64_t debug_abs_send_prev_time_ms_ = 0;
	int64_t debug_arrival_prev_time_ms_ = 0;
	int64_t debug_recv_calc_prev_time_ms_ = 0;
	uint64_t debug_twcc_seq_prev_;

	Unwrapper<uint16_t> debug_twcc_unwrapper_;
	Unwrapper<uint16_t> debug_abs_unwrapper_;

	rtc::RTCThreadManagerInterface *_rtc_thread_manager;

private:
	RTC_DISALLOW_COPY_AND_ASSIGN(RTCCall);
};

}


#endif	// __RTC_CALL_CALL_H__
