//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : peerconnection lite version for rtc media server.
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_CONNECTION_H__
#define __RTC_CONNECTION_H__

#include <stdint.h>
#include <memory>
#include <string>
#include <vector>
#include <functional>

#include "../_deprecate_defines.h"

#include "api/rtc_error.h"
#include "api/scoped_refptr.h"
#include "api/make_ref_counted.h"
#include "api/async_resolver_factory.h"
#include "api/video_codecs/sdp_video_format.h"
#include "api/rtp_parameters.h"
#include "api/transport/sctp_transport_factory_interface.h"
#include "media/base/codec.h"
#include "media/base/h264_profile_level_id.h"
#include "p2p/base/port_allocator.h"
#include "p2p/base/basic_packet_socket_factory.h"
#include "pc/jsep_transport_controller.h"
#include "pc/channel_interface.h"
#include "rtc_base/network.h"
#include "rtc_base/rtc_certificate.h"
#include "rtc_base/rtc_certificate_generator.h"
#include "rtc_base/ssl_stream_adapter.h"
#include "rtc_base/third_party/sigslot/sigslot.h"
#include "rtc_base/thread.h"
#include "rtc_base/task_utils/pending_task_safety_flag.h"
#include "rtc_base/weak_ptr.h"

#include "../../src_update/api/_peer_connection_interface_defs.h"
#include "../../src_update/media/engine/_webrtc_media_engine.h"
#include "../../src_update/media/base/_media_engine.h"

#include "RTCConnectionInternal.h"

#include "../api/ThreadsInfoInterface.h"
#include "RTCConnectionMessageHandler.h"
#include "RTCChannelManager.h"
#include "RTCRtpTransmissionManager.h"
#include "RTCDataChannelController.h"
#include "../call/RTCCall.h"

#include "../RTCThreadManagerInterface.h"

namespace webrtc {

class RTCSdpOfferAnswerHandler;
class RTCRtpSenderInterface;

class RTCConnection : public RTCConnectionInternal,
					  public JsepTransportController::Observer,
					  public sigslot::has_slots<> {
public:
	static rtc::scoped_refptr<RTCConnection> Create(
		ThreadsInfoInterface *threadsInfo, 
		std::unique_ptr<RTCCall> call, 
		TaskQueueFactory *task_queue_factory,
		PeerConnectionInterfaceDefs::RTCConfiguration &rtcConfigure,
		RTCConnectionObserver *observer,
		rtc::RTCThreadManagerInterface *rtc_thread_manager
		);
	
	//////////////////////////////////////////////////////////////////////////
	// RTCConnectionInterface method override
	void CreateOffer(CreateSessionDescriptionObserver *observer, 
		const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions& options) override;
	void CreateAnswer(CreateSessionDescriptionObserver *observer,
		const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions& options) override;

	void SetLocalDescription(SetSessionDescriptionObserver *observer, 
		SessionDescriptionInterface *desc_ptr) override;
	void SetLocalDescription(rtc::scoped_refptr<SetSessionDescriptionObserver> observer,
		std::unique_ptr<SessionDescriptionInterface> desc) override;
	
	void SetRemoteDescription(SetSessionDescriptionObserver *observer, 
		SessionDescriptionInterface *desc_ptr) override;
	void SetRemoteDescription(rtc::scoped_refptr<SetSessionDescriptionObserver> observer,
		std::unique_ptr<SessionDescriptionInterface> desc) override;

	bool ShouldFireNegotiationNeededEvent(uint32_t event_id) override;

	bool AddIceCandidate(const IceCandidateInterface* candidate) override;
	void AddIceCandidate(
		std::unique_ptr<IceCandidateInterface> candidate,
		std::function<void(RTCError)> callback) override;

	bool RemoveIceCandidates(
		const std::vector<cricket::Candidate>& candidates) override;
	PeerConnectionInterfaceDefs::IceConnectionState ice_connection_state() override;

	RTCErrorOr<rtc::scoped_refptr<RTCRtpSenderInterface>> AddTrack(
		rtc::scoped_refptr<MediaStreamTrackInterface> track,
		const std::vector<std::string>& stream_ids) override;
	RTCError RemoveTrack(rtc::scoped_refptr<RTCRtpSenderInterface> sender) override;


	RTCErrorOr<rtc::scoped_refptr<RTCRtpTransceiverInterface>> AddTransceiver(
		rtc::scoped_refptr<MediaStreamTrackInterface> track) override;
	RTCErrorOr<rtc::scoped_refptr<RTCRtpTransceiverInterface>> AddTransceiver(
		rtc::scoped_refptr<MediaStreamTrackInterface> track,
		const RTCRtpTransceiverInit& init) override;
	RTCErrorOr<rtc::scoped_refptr<RTCRtpTransceiverInterface>> AddTransceiver(
		cricket::MediaType media_type) override;
	RTCErrorOr<rtc::scoped_refptr<RTCRtpTransceiverInterface>> AddTransceiver(
		cricket::MediaType media_type,
		const RTCRtpTransceiverInit& init) override;

	std::vector<rtc::scoped_refptr<RTCRtpSenderInterface>> GetSenders() const override;
	std::vector<rtc::scoped_refptr<RTCRtpReceiverInterface>> GetReceivers() const  override;
	std::vector<rtc::scoped_refptr<RTCRtpTransceiverInterface>> GetTransceivers() const  override;

	rtc::scoped_refptr<DataChannelInterface> CreateDataChannel(
		const std::string& label,
		const DataChannelInit* config) override;
	void GenerateKeyFrame() override;
	void GenerateKeyFramePli(uint32_t ssrc) override;

	const SessionDescriptionInterface* local_description() const override;
	const SessionDescriptionInterface* remote_description() const override;
	const SessionDescriptionInterface* current_local_description() const override;
	const SessionDescriptionInterface* current_remote_description() const override;
	const SessionDescriptionInterface* pending_local_description() const override;
	const SessionDescriptionInterface* pending_remote_description() const override;
	void Close() override;
	RTCCall::Stats GetCallStats() override;

	std::vector<DataChannelStats> GetDataChannelStats() const override;
	absl::optional<std::string> sctp_transport_name() const override;
	//////////////////////////////////////////////////////////////////////////

	bool IsClosed() const override;

	bool GetSctpSslRole(rtc::SSLRole* role) override;
	void OnSctpDataChannelClosed(DataChannelInterface* channel);

	RTCDataChannelController* data_channel_controller() override {
		RTC_DCHECK_RUN_ON(signaling_thread());
		return &data_channel_controller_;
	}

	absl::optional<std::string> sctp_mid() override {
		RTC_DCHECK_RUN_ON(signaling_thread());
		return sctp_mid_s_;
	}
	
	absl::optional<std::string> GetDataMid() const override;

	void SetSctpDataMid(const std::string& mid) override {
		RTC_DCHECK_RUN_ON(signaling_thread());
		sctp_mid_s_ = mid;
	}

	void ResetSctpDataMid() override {
		RTC_DCHECK_RUN_ON(signaling_thread());
		sctp_mid_s_.reset();
	}

	cricket::DataChannelType data_channel_type() const override;

	bool dtls_enabled() const override {
		RTC_DCHECK_RUN_ON(signaling_thread());
		return _dtls_enabled;
	}

	void SetIceConnectionState(PeerConnectionInterfaceDefs::IceConnectionState new_state) override;

	RTCErrorOr<rtc::scoped_refptr<RTCRtpTransceiverInterface>> AddTransceiver(
		cricket::MediaType media_type,
		rtc::scoped_refptr<MediaStreamTrackInterface> track,
		const RTCRtpTransceiverInit& init,
		bool fire_callback = true) override;


	// Returns the observer. Will crash on CHECK if the observer is removed.
	RTCConnectionObserver* Observer() const override;

	JsepTransportController* transport_controller() override {
		return _transport_controller.get();
	}

	rtc::Thread* signaling_thread() { return _signaling_thread; }

	std::string session_id() const override {
		RTC_DCHECK_RUN_ON(signaling_thread());
		return _session_id;
	}

	bool IsUnifiedPlan() const override {
		RTC_DCHECK_RUN_ON(signaling_thread());
		return is_unified_plan_;
	}

	bool IsDisableEncryption() const override {
		RTC_DCHECK_RUN_ON(signaling_thread());
		return false;
	}

	const std::vector<cricket::AudioCodec>& GetSendAudioCodecs() const override {
		return _send_audio_codecs;
	}
	const std::vector<cricket::AudioCodec>& GetRecvAudioCodecs() const override {
		return _recv_audio_codecs;
	}
	const std::vector<cricket::VideoCodec>& GetSendVideoCodecs() const override {
		return _send_video_codecs;
	}
	const std::vector<cricket::VideoCodec>& GetRecvVideoCodecs() const override {
		return _recv_video_codecs;
	}

	RTCConnectionMessageHandler* message_handler() override {
		return &_message_handler;
	}

	cricket::PortAllocator* port_allocator() override { return _allocator.get(); }

	RTCCall* call_ptr() override { return call_ptr_; }

	const PeerConnectionInterfaceDefs::RTCConfiguration* configuration() const override {
		RTC_DCHECK_RUN_ON(signaling_thread());
		return &_rtcConfigure;
	}

	// Returns the CryptoOptions for this PeerConnection. This will always
	// return the RTCConfiguration.crypto_options if set and will only default
	// back to the PeerConnectionFactory settings if nothing was set.
	CryptoOptions GetCryptoOptions() override;

	std::vector<RtpHeaderExtensionCapability> GetAudioRtpHeaderExtensions() const override;
	std::vector<RtpHeaderExtensionCapability> GetVideoRtpHeaderExtensions() const override;

	bool ValidateBundleSettings(const cricket::SessionDescription* desc) override;

	// RTCConnectionInternal override method
	rtc::Thread* network_thread() const final { return _network_thread; }
	rtc::Thread* signaling_thread() const final { return _signaling_thread; }
	rtc::Thread* worker_thread() const final { return _worker_thread; }
	void BaseChannelConnectSentPacketSignal(cricket::RTCBaseChannel *channel) override;
	void SctpDataChannelConnectClosedSignal(SctpDataChannel *channel) override;

	RTCRtpTransmissionManager* rtp_manager() override { return rtp_manager_.get(); }
	const RTCRtpTransmissionManager* rtp_manager() const override {
		return rtp_manager_.get();
	}

	cricket::RTCChannelManager* channel_manager() const override {
		return _channel_manager.get();
	}

	cricket::ChannelInterface* GetChannel(const std::string& content_name) override;

	bool SrtpRequired() const RTC_RUN_ON(signaling_thread()) override;

	void OnSentPacket_w(const rtc::SentPacket& sent_packet);

	bool SetupDataChannelTransport_n(const std::string& mid) override RTC_RUN_ON(network_thread());

	void TeardownDataChannelTransport_n() override RTC_RUN_ON(network_thread());

	// Returns rtp transport, result can not be nullptr.
	RtpTransportInternal* GetRtpTransport(const std::string& mid) override {
		RTC_DCHECK_RUN_ON(signaling_thread());
		auto rtp_transport = _transport_controller->GetRtpTransport(mid);
		RTC_DCHECK(rtp_transport);
		return rtp_transport;
	}

protected:
	RTCConnection(ThreadsInfoInterface *threadsInfo, 
				  std::unique_ptr<RTCCall> call,
				  TaskQueueFactory *task_queue_factory,
				  RTCConnectionObserver *observer, 
				  bool is_unified_plan,
				  rtc::RTCThreadManagerInterface *rtc_thread_manager
				  );
	~RTCConnection() override;

	// Called when first configuring the port allocator.
	struct InitializePortAllocatorResult {
		bool enable_ipv6;
	};

	void SetConnectionState(
		PeerConnectionInterfaceDefs::PeerConnectionState new_state)
		RTC_RUN_ON(signaling_thread());

	void SetStandardizedIceConnectionState(
		PeerConnectionInterfaceDefs::IceConnectionState new_state)
		RTC_RUN_ON(signaling_thread());

	std::function<void(const rtc::CopyOnWriteBuffer& packet,
		int64_t packet_time_us)>
		InitializeRtcpCallback();

private:
	RTCError _initialize(PeerConnectionInterfaceDefs::RTCConfiguration &rtcConfigure);

	InitializePortAllocatorResult _initialize_port_allocator_n(
		const cricket::ServerAddresses& stun_servers,
		const std::vector<cricket::RelayServerConfig>& turn_servers,
		const PeerConnectionInterfaceDefs::RTCConfiguration &rtcConfigure);

	rtc::scoped_refptr<RTCRtpTransceiverProxyWithInternal<RTCRtpTransceiver>>
		FindTransceiverBySender(rtc::scoped_refptr<RTCRtpSenderInterface> sender)
		RTC_RUN_ON(signaling_thread());

	// JsepTransportController::Observer override.
	//
	// Called by |transport_controller_| when processing transport information
	// from a session description, and the mapping from m= sections to transports
	// changed (as a result of BUNDLE negotiation, or m= sections being
	// rejected).
	bool OnTransportChanged(const std::string& mid,
							RtpTransportInternal* rtp_transport,
							rtc::scoped_refptr<DtlsTransport> dtls_transport,
							DataChannelTransportInterface* data_channel_transport) override;

	// JsepTransportController signal handlers.
	void OnTransportControllerConnectionState(cricket::IceConnectionState state)
		RTC_RUN_ON(signaling_thread());
	void OnTransportControllerGatheringState(cricket::IceGatheringState state)
		RTC_RUN_ON(signaling_thread());
	void OnTransportControllerCandidatesGathered(
		const std::string& transport_name,
		const std::vector<cricket::Candidate>& candidates)
		RTC_RUN_ON(signaling_thread());
	void OnTransportControllerCandidateError(
		const cricket::IceCandidateErrorEvent& event)
		RTC_RUN_ON(signaling_thread());
	void OnTransportControllerCandidatesRemoved(
		const std::vector<cricket::Candidate>& candidates)
		RTC_RUN_ON(signaling_thread());
	void OnTransportControllerCandidateChanged(
		const cricket::CandidatePairChangeEvent& event)
		RTC_RUN_ON(signaling_thread());
	void OnTransportControllerDtlsHandshakeError(rtc::SSLHandshakeError error);


	// Called any time the IceGatheringState changes.
	void OnIceGatheringChange(PeerConnectionInterfaceDefs::IceGatheringState new_state)
		RTC_RUN_ON(signaling_thread());
	// New ICE candidate has been gathered.
	void OnIceCandidate(std::unique_ptr<IceCandidateInterface> candidate)
		RTC_RUN_ON(signaling_thread());
	// Gathering of an ICE candidate failed.
	void OnIceCandidateError(const std::string& address,
		int port,
		const std::string& url,
		int error_code,
		const std::string& error_text)
		RTC_RUN_ON(signaling_thread());

	// Some local ICE candidates have been removed.
	void OnIceCandidatesRemoved(const std::vector<cricket::Candidate>& candidates)
		RTC_RUN_ON(signaling_thread());

	void OnSelectedCandidatePairChanged(
		const cricket::CandidatePairChangeEvent& event)
		RTC_RUN_ON(signaling_thread());

	void _create_codecs_info();
	void _create_audio_encoder_codecs_info();
	void _create_audio_decoder_codecs_info();
	void _create_video_encoder_codecs_info();
	void _create_video_decoder_codecs_info();
	std::vector<SdpVideoFormat> _supported_H264_codecs();
	SdpVideoFormat _create_h264_format(H264::Profile profile, H264::Level level, const std::string& packetization_mode);
	void _add_video_default_feedback_params(cricket::VideoCodec* codec);


	bool GetLocalCandidateMediaIndex(const std::string& content_name,
									 int* sdp_mline_index) RTC_RUN_ON(signaling_thread());

	SctpDataChannel* FindDataChannelBySid(int sid) const RTC_RUN_ON(signaling_thread());

	cricket::RTCRtpDataChannel* rtp_data_channel() const {
		return data_channel_controller_.rtp_data_channel();
	}

private:
	rtc::Thread* const _signaling_thread;
	rtc::Thread* const _worker_thread;
	rtc::Thread* const _network_thread;
	const bool is_unified_plan_;

	std::unique_ptr<rtc::BasicNetworkManager> _default_network_manager RTC_GUARDED_BY(_signaling_thread);
	std::unique_ptr<cricket::PortAllocator> _allocator;
	std::unique_ptr<rtc::BasicPacketSocketFactory> _packet_socket_factory RTC_GUARDED_BY(_signaling_thread);
	std::unique_ptr<AsyncResolverFactory> _async_resolver_factory RTC_GUARDED_BY(_signaling_thread);
	std::unique_ptr<IceTransportFactory> _ice_transport_factory;
	std::unique_ptr<rtc::SSLCertificateVerifier> _tls_cert_verifier;
	std::unique_ptr<JsepTransportController> _transport_controller;
	std::unique_ptr<cricket::RTCChannelManager> _channel_manager;

	std::string _session_id;

	bool _dtls_enabled RTC_GUARDED_BY(signaling_thread()) = false;

	PeerConnectionInterfaceDefs::IceConnectionState ice_connection_state_ 
		RTC_GUARDED_BY(signaling_thread()) = PeerConnectionInterfaceDefs::kIceConnectionNew;
	PeerConnectionInterfaceDefs::PeerConnectionState connection_state_ 
		RTC_GUARDED_BY(signaling_thread()) = PeerConnectionInterfaceDefs::PeerConnectionState::kNew;
	PeerConnectionInterfaceDefs::IceConnectionState standardized_ice_connection_state_
		RTC_GUARDED_BY(signaling_thread()) = PeerConnectionInterfaceDefs::kIceConnectionNew;
	PeerConnectionInterfaceDefs::IceGatheringState ice_gathering_state_ RTC_GUARDED_BY(signaling_thread()) =
		PeerConnectionInterfaceDefs::kIceGatheringNew;

	PeerConnectionInterfaceDefs::RTCConfiguration _rtcConfigure RTC_GUARDED_BY(signaling_thread());

	RTCConnectionObserver* _observer RTC_GUARDED_BY(signaling_thread()) =
		nullptr;

	// The machinery for handling offers and answers. Const after initialization.
	std::unique_ptr<RTCSdpOfferAnswerHandler> _sdp_handler
		RTC_GUARDED_BY(signaling_thread());

	//////////////////////////////////////////////////////////////////////////
	// for test and to move other file...
	std::vector<cricket::AudioCodec> _send_audio_codecs;
	std::vector<cricket::AudioCodec> _recv_audio_codecs;

	std::vector<cricket::VideoCodec> _send_video_codecs;
	std::vector<cricket::VideoCodec> _recv_video_codecs;
	//////////////////////////////////////////////////////////////////////////

	RTCConnectionMessageHandler	_message_handler;

	std::unique_ptr<RTCRtpTransmissionManager> rtp_manager_;

	std::unique_ptr<RTCCall> call_ RTC_GUARDED_BY(worker_thread());
	std::unique_ptr<ScopedTaskSafety> call_safety_
		RTC_GUARDED_BY(worker_thread());
	RTCCall* const call_ptr_;

	TaskQueueFactory *task_queue_factory_;

	rtc::WeakPtrFactory<RTCConnection> _weak_factory;

	RTCDataChannelController data_channel_controller_;

	absl::optional<std::string> sctp_mid_s_ RTC_GUARDED_BY(signaling_thread());
	absl::optional<std::string> sctp_mid_n_ RTC_GUARDED_BY(network_thread());

	std::unique_ptr<SctpTransportFactoryInterface> sctp_factory_;

	rtc::RTCThreadManagerInterface *_rtc_thread_manager;
};

}

#endif //	__RTC_CONNECTION_H__