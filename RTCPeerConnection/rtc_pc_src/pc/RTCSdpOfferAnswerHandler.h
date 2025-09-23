//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : Replace SdpOfferAnswerHandler to RTCSdpOfferAnswerHandler class for rtc media server.
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_SDP_OFFER_ANSWER_HANDLER_H__
#define __RTC_SDP_OFFER_ANSWER_HANDLER_H__

#include <string>
#include <memory>

#include "../rtc_pc_src/_deprecate_defines.h"

#include "absl/types/optional.h"

#include "api/jsep.h"
#include "api/rtp_transceiver_direction.h"
#include "api/set_local_description_observer_interface.h"
#include "api/set_remote_description_observer_interface.h"
#include "api/jsep_ice_candidate.h"
#include "rtc_base/thread.h"
#include "rtc_base/ssl_stream_adapter.h"
#include "rtc_base/third_party/sigslot/sigslot.h"
#include "rtc_base/synchronization/sequence_checker.h"
#include "rtc_base/unique_id_generator.h"
#include "rtc_base/operations_chain.h"
#include "rtc_base/weak_ptr.h"
#include "pc/sdp_state_provider.h"
#include "pc/webrtc_session_description_factory.h"
#include "pc/stream_collection.h"
#include "pc/media_stream_observer.h"

#include "pc/jsep_transport_controller.h"

#include "../../src_update/api/_peer_connection_interface_defs.h"

#include "RTCConnectionInternal.h"
#include "RTCRtpTransceiver.h"
#include "RTCTransceiverList.h"
#include "RTCRtpTransmissionManager.h"

#include "RTCChannelManager.h"
#include "RTCDataChannelController.h"

#include "../rtc_pc_src/RTCThreadManagerInterface.h"

namespace webrtc {

class RTCSdpOfferAnswerHandler : public SdpStateProvider,
								 public sigslot::has_slots<> {
public:
	~RTCSdpOfferAnswerHandler();

	static std::unique_ptr<RTCSdpOfferAnswerHandler> Create(
		RTCConnectionInternal *rtc_connection,
		const PeerConnectionInterfaceDefs::RTCConfiguration& configuration,
		std::unique_ptr<rtc::RTCCertificateGeneratorInterface> cert_generator,
		rtc::RTCThreadManagerInterface* rtc_thread_manager
		);

	void ResetSessionDescFactory() {
		RTC_DCHECK_RUN_ON(signaling_thread());
		webrtc_session_desc_factory_.reset();
	}

	const WebRtcSessionDescriptionFactory* webrtc_session_desc_factory() const {
		RTC_DCHECK_RUN_ON(signaling_thread());
		return webrtc_session_desc_factory_.get();
	}

	void Close();

	void PrepareForShutdown();

	void CreateOffer(CreateSessionDescriptionObserver* observer,
		const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions& options);

	void CreateAnswer(CreateSessionDescriptionObserver* observer,
		const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions& options);

	void SetLocalDescription(SetSessionDescriptionObserver* observer,
							 SessionDescriptionInterface* desc);

	void SetLocalDescription(rtc::scoped_refptr<SetSessionDescriptionObserver> observer,
							 std::unique_ptr<SessionDescriptionInterface> desc);

	void SetRemoteDescription(SetSessionDescriptionObserver* observer,
							  SessionDescriptionInterface* desc);

	void SetRemoteDescription(rtc::scoped_refptr<SetSessionDescriptionObserver> observer,
							  std::unique_ptr<SessionDescriptionInterface> desc);

	bool AddIceCandidate(const IceCandidateInterface* candidate);
	void AddIceCandidate(std::unique_ptr<IceCandidateInterface> candidate,
						 std::function<void(RTCError)> callback);
	bool RemoveIceCandidates(const std::vector<cricket::Candidate>& candidates);

	void AddLocalIceCandidate(const JsepIceCandidate* candidate);

	void RemoveLocalIceCandidates(
		const std::vector<cricket::Candidate>& candidates);

	bool ShouldFireNegotiationNeededEvent(uint32_t event_id);

	const cricket::ContentInfo* FindMediaSectionForTransceiver(
		rtc::scoped_refptr<RTCRtpTransceiverProxyWithInternal<RTCRtpTransceiver>>
		transceiver,
		const SessionDescriptionInterface* sdesc) const;

	// Destroys all BaseChannels and destroys the SCTP data channel, if present.
	void DestroyAllChannels();

	void SetHavePendingRtpDataChannel() {				
		RTC_DCHECK_RUN_ON(signaling_thread());
		have_pending_rtp_data_channel_ = true;
	}

	class LocalIceCredentialsToReplace;

public:	

	explicit RTCSdpOfferAnswerHandler(RTCConnectionInternal *rtc_connection, rtc::RTCThreadManagerInterface* rtc_thread_manager);

	PeerConnectionInterfaceDefs::SignalingState signaling_state() const override;

	const SessionDescriptionInterface* local_description() const override;
	const SessionDescriptionInterface* remote_description() const override;
	const SessionDescriptionInterface* current_local_description() const override;
	const SessionDescriptionInterface* current_remote_description()
		const override;
	const SessionDescriptionInterface* pending_local_description() const override;
	const SessionDescriptionInterface* pending_remote_description()
		const override;

	bool NeedsIceRestart(const std::string& content_name) const override;
	bool IceRestartPending(const std::string& content_name) const override;
	absl::optional<rtc::SSLRole> GetDtlsRole(
		const std::string& mid) const override;

	void RestartIce();

	void UpdateNegotiationNeeded();

	absl::optional<bool> is_caller();

private:
	class SetSessionDescriptionObserverAdapter;

	friend class SetSessionDescriptionObserverAdapter;

	enum class SessionError {
		kNone,       // No error.
		kContent,    // Error in BaseChannel SetLocalContent/SetRemoteContent.
		kTransport,  // Error from the underlying transport.
	};

	void Initialize(const PeerConnectionInterfaceDefs::RTCConfiguration& configuration, 
		std::unique_ptr<rtc::RTCCertificateGeneratorInterface> cert_generator);

	rtc::Thread* signaling_thread() const;

	SessionDescriptionInterface* mutable_local_description()
		RTC_RUN_ON(signaling_thread()) {
		return pending_local_description_ ? pending_local_description_.get()
			: current_local_description_.get();
	}
	SessionDescriptionInterface* mutable_remote_description()
		RTC_RUN_ON(signaling_thread()) {
		return pending_remote_description_ ? pending_remote_description_.get()
			: current_remote_description_.get();
	}

	JsepTransportController* transport_controller();

	const JsepTransportController* transport_controller() const;

//	bool _is_unified_plan() const RTC_RUN_ON(signaling_thread());

//	bool _is_disable_encryption() const RTC_RUN_ON(signaling_thread());
	RTCError Rollback(SdpType desc_type);
	void OnOperationsChainEmpty();

	void SetAssociatedRemoteStreams(
		rtc::scoped_refptr<RTCRtpReceiverInternal> receiver,
		const std::vector<std::string>& stream_ids,
		std::vector<rtc::scoped_refptr<MediaStreamInterface>>* added_streams,
		std::vector<rtc::scoped_refptr<MediaStreamInterface>>* removed_streams);

	void DoCreateOffer(
		const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions& options,
		rtc::scoped_refptr<CreateSessionDescriptionObserver> observer);
	void DoCreateAnswer(
		const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions& options,
		rtc::scoped_refptr<CreateSessionDescriptionObserver> observer);

	void DoSetLocalDescription(
		std::unique_ptr<SessionDescriptionInterface> desc,
		rtc::scoped_refptr<SetLocalDescriptionObserverInterface> observer);

	void DoSetRemoteDescription(
		std::unique_ptr<SessionDescriptionInterface> desc,
		rtc::scoped_refptr<SetRemoteDescriptionObserverInterface> observer);

	void ChangeSignalingState(
		PeerConnectionInterfaceDefs::SignalingState signaling_state);


	RTCError UpdateSessionState(SdpType type,
		cricket::ContentSource source,
		const cricket::SessionDescription* description);

	rtc::scoped_refptr<RTCRtpTransceiverProxyWithInternal<RTCRtpTransceiver>>
		FindAvailableTransceiverToReceive(cricket::MediaType media_type) const;

	// Returns a MediaSessionOptions struct with options decided by |options|,
	// the local MediaStreams and DataChannels.
	void GetOptionsForOffer(const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions&
		offer_answer_options,
		cricket::MediaSessionOptions* session_options);

	void GetOptionsForAnswer(const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions&
		offer_answer_options,
		cricket::MediaSessionOptions* session_options);

	void GetOptionsForUnifiedPlanOffer(
		const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions&
		offer_answer_options,
		cricket::MediaSessionOptions* session_options)
		RTC_RUN_ON(signaling_thread());

	void GetOptionsForPlanBOffer(
		const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions&
		offer_answer_options,
		cricket::MediaSessionOptions* session_options)
		RTC_RUN_ON(signaling_thread());

	void GetOptionsForPlanBAnswer(
		const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions&
		offer_answer_options,
		cricket::MediaSessionOptions* session_options)
		RTC_RUN_ON(signaling_thread());

	void GetOptionsForUnifiedPlanAnswer(
		const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions&
		offer_answer_options,
		cricket::MediaSessionOptions* session_options)
		RTC_RUN_ON(signaling_thread());

	void GenerateMediaDescriptionOptions(
		const SessionDescriptionInterface* session_desc,
		RtpTransceiverDirection audio_direction,
		RtpTransceiverDirection video_direction,
		absl::optional<size_t>* audio_index,
		absl::optional<size_t>* video_index,
		absl::optional<size_t>* data_index,
		cricket::MediaSessionOptions* session_options);

	bool UpdatePayloadTypeDemuxingState(cricket::ContentSource source);


	bool IsUnifiedPlan() const RTC_RUN_ON(signaling_thread());

	cricket::PortAllocator* port_allocator();
	const cricket::PortAllocator* port_allocator() const;

	bool HasNewIceCredentials();

	const char* SessionErrorToString(SessionError error) const;
	std::string GetSessionErrorMsg();

	SessionError session_error() const {
		RTC_DCHECK_RUN_ON(signaling_thread());
		return session_error_;
	}
	const std::string& session_error_desc() const { return session_error_desc_; }

	// Updates the error state, signaling if necessary.
	void SetSessionError(SessionError error, const std::string& error_desc);

	RTCError HandleLegacyOfferOptions(
		const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions& options);
	void RemoveRecvDirectionFromReceivingTransceiversOfType(
		cricket::MediaType media_type) RTC_RUN_ON(signaling_thread());
	void AddUpToOneReceivingTransceiverOfType(cricket::MediaType media_type);
	std::vector<
		rtc::scoped_refptr<RTCRtpTransceiverProxyWithInternal<RTCRtpTransceiver>>>
		GetReceivingTransceiversOfType(cricket::MediaType media_type)
		RTC_RUN_ON(signaling_thread());

	void ProcessRemovalOfRemoteTrack(
		rtc::scoped_refptr<RTCRtpTransceiverProxyWithInternal<RTCRtpTransceiver>>
		transceiver,
		std::vector<rtc::scoped_refptr<RTCRtpTransceiverInterface>>* remove_list,
		std::vector<rtc::scoped_refptr<MediaStreamInterface>>* removed_streams);

	bool CheckIfNegotiationIsNeeded();
	void GenerateNegotiationNeededEvent();

	RTCError ValidateSessionDescription(const SessionDescriptionInterface* sdesc,
		cricket::ContentSource source) RTC_RUN_ON(signaling_thread());

	RTCError UpdateTransceiversAndDataChannels(
		cricket::ContentSource source,
		const SessionDescriptionInterface& new_session,
		const SessionDescriptionInterface* old_local_description,
		const SessionDescriptionInterface* old_remote_description);

	RTCErrorOr<
		rtc::scoped_refptr<RTCRtpTransceiverProxyWithInternal<RTCRtpTransceiver>>>
		AssociateTransceiver(cricket::ContentSource source,
		SdpType type,
		size_t mline_index,
		const cricket::ContentInfo& content,
		const cricket::ContentInfo* old_local_content,
		const cricket::ContentInfo* old_remote_content)
		RTC_RUN_ON(signaling_thread());


	RTCErrorOr<const cricket::ContentGroup*> GetEarlyBundleGroup(
		const cricket::SessionDescription& desc) const
		RTC_RUN_ON(signaling_thread());

	RTCError UpdateTransceiverChannel(
		rtc::scoped_refptr<RTCRtpTransceiverProxyWithInternal<RTCRtpTransceiver>>
			transceiver,
		const cricket::ContentInfo& content,
		const cricket::ContentGroup* bundle_group) RTC_RUN_ON(signaling_thread());

	RTCError UpdateDataChannel(cricket::ContentSource source,
								const cricket::ContentInfo& content,
								const cricket::ContentGroup* bundle_group)
		RTC_RUN_ON(signaling_thread());

	bool ExpectSetLocalDescription(SdpType type);
	bool ExpectSetRemoteDescription(SdpType type);

	RTCError ApplyLocalDescription(
		std::unique_ptr<SessionDescriptionInterface> desc);
	RTCError ApplyRemoteDescription(
		std::unique_ptr<SessionDescriptionInterface> desc);

	RTCError PushdownTransportDescription(cricket::ContentSource source,
		SdpType type);

	RTCError CreateChannels(const cricket::SessionDescription& desc);
	void RemoveUnusedChannels(const cricket::SessionDescription* desc);

	cricket::RTCChannelManager* channel_manager() const;

	RTCTransceiverList* transceivers();
	const RTCTransceiverList* transceivers() const;

	RTCDataChannelController* data_channel_controller();				
	const RTCDataChannelController* data_channel_controller() const;	

	bool UseCandidatesInSessionDescription(
		const SessionDescriptionInterface* remote_desc);

	bool ReadyToUseRemoteCandidate(const IceCandidateInterface* candidate,
		const SessionDescriptionInterface* remote_desc,
		bool* valid);

	RTCErrorOr<const cricket::ContentInfo*> FindContentInfo(
		const SessionDescriptionInterface* description,
		const IceCandidateInterface* candidate) RTC_RUN_ON(signaling_thread());

	const std::string GetTransportName(const std::string& content_name);

	bool UseCandidate(const IceCandidateInterface* candidate);

	void RemoveRemoteStreamsIfEmpty(
		const std::vector<rtc::scoped_refptr<MediaStreamInterface>>&
		remote_streams,
		std::vector<rtc::scoped_refptr<MediaStreamInterface>>* removed_streams);

	void RemoveSenders(cricket::MediaType media_type);

	void UpdateLocalSenders(const std::vector<cricket::StreamParams>& streams,
							cricket::MediaType media_type);

	void UpdateRemoteSendersList(
		const std::vector<cricket::StreamParams>& streams,
		bool default_track_needed,
		cricket::MediaType media_type,
		StreamCollection* new_streams);

	void EnableSending();

	RTCError PushdownMediaDescription(SdpType type,
		cricket::ContentSource source);

	RTCRtpTransmissionManager* rtp_manager();
	const RTCRtpTransmissionManager* rtp_manager() const;

	void RemoveStoppedTransceivers();

	void FillInMissingRemoteMids(cricket::SessionDescription* remote_description);

	void UpdateEndedRemoteMediaStreams();

	void DestroyTransceiverChannel(
		rtc::scoped_refptr<RTCRtpTransceiverProxyWithInternal<RTCRtpTransceiver>>
		transceiver);

	// Destroys the given ChannelInterface.
	// The channel cannot be accessed after this method is called.
	void DestroyChannelInterface(cricket::ChannelInterface* channel);

	cricket::MediaDescriptionOptions GetMediaDescriptionOptionsForActiveData(
		const std::string& mid) const;		
	cricket::MediaDescriptionOptions GetMediaDescriptionOptionsForRejectedData(
		const std::string& mid) const;		

	cricket::RTCVoiceChannel* CreateVoiceChannel(const std::string& mid);
	cricket::RTCVideoChannel* CreateVideoChannel(const std::string& mid);
	bool CreateDataChannel(const std::string& mid);	

	const cricket::AudioOptions& audio_options() { return audio_options_; }

	void DestroyDataChannelTransport();	

private:
	RTCConnectionInternal* const _rtc_connection;

	std::unique_ptr<WebRtcSessionDescriptionFactory> webrtc_session_desc_factory_ 
		RTC_GUARDED_BY(signaling_thread());
	std::unique_ptr<SessionDescriptionInterface> current_local_description_
		RTC_GUARDED_BY(signaling_thread());
	std::unique_ptr<SessionDescriptionInterface> pending_local_description_
		RTC_GUARDED_BY(signaling_thread());
	std::unique_ptr<SessionDescriptionInterface> current_remote_description_
		RTC_GUARDED_BY(signaling_thread());
	std::unique_ptr<SessionDescriptionInterface> pending_remote_description_
		RTC_GUARDED_BY(signaling_thread());

	PeerConnectionInterfaceDefs::SignalingState signaling_state_
		RTC_GUARDED_BY(signaling_thread()) = PeerConnectionInterfaceDefs::kStable;

	// MIDs will be generated using this generator which will keep track of
	// all the MIDs that have been seen over the life of the PeerConnection.
	rtc::UniqueStringGenerator mid_generator_ RTC_GUARDED_BY(signaling_thread());

	// List of content names for which the remote side triggered an ICE restart.
	std::set<std::string> pending_ice_restarts_
		RTC_GUARDED_BY(signaling_thread());

	// This object should be used to generate any SSRC that is not explicitly
	// specified by the user (or by the remote party).
	// The generator is not used directly, instead it is passed on to the
	// channel manager and the session description factory.
	rtc::UniqueRandomIdGenerator ssrc_generator_
		RTC_GUARDED_BY(signaling_thread());

	// The operations chain is used by the offer/answer exchange methods to ensure
	// they are executed in the right order. For example, if
	// SetRemoteDescription() is invoked while CreateOffer() is still pending, the
	// SRD operation will not start until CreateOffer() has completed. See
	// https://w3c.github.io/webrtc-pc/#dfn-operations-chain.
	rtc::scoped_refptr<rtc::OperationsChain> operations_chain_
		RTC_GUARDED_BY(signaling_thread());

	const std::string rtcp_cname_;

	std::unique_ptr<LocalIceCredentialsToReplace>
		local_ice_credentials_to_replace_ RTC_GUARDED_BY(signaling_thread());

	bool remote_peer_supports_msid_ RTC_GUARDED_BY(signaling_thread()) = false;
	bool is_negotiation_needed_ RTC_GUARDED_BY(signaling_thread()) = false;
	uint32_t negotiation_needed_event_id_ = 0;
	bool update_negotiation_needed_on_empty_chain_
		RTC_GUARDED_BY(signaling_thread()) = false;


	SessionError session_error_ RTC_GUARDED_BY(signaling_thread()) =
		SessionError::kNone;

	std::string session_error_desc_ RTC_GUARDED_BY(signaling_thread());

	absl::optional<bool> is_caller_ RTC_GUARDED_BY(signaling_thread());

	const rtc::scoped_refptr<StreamCollection> local_streams_
		RTC_GUARDED_BY(signaling_thread());
	// Streams created as a result of SetRemoteDescription.
	const rtc::scoped_refptr<StreamCollection> remote_streams_
		RTC_GUARDED_BY(signaling_thread());

	std::vector<std::unique_ptr<MediaStreamObserver>> stream_observers_
		RTC_GUARDED_BY(signaling_thread());

	// Member variables for caching global options.
	cricket::AudioOptions audio_options_ RTC_GUARDED_BY(signaling_thread());

	rtc::WeakPtrFactory<RTCSdpOfferAnswerHandler> weak_ptr_factory_;

	rtc::scoped_refptr<MediaStreamInterface> missing_msid_default_stream_
		RTC_GUARDED_BY(signaling_thread());

	bool have_pending_rtp_data_channel_ RTC_GUARDED_BY(signaling_thread()) = false;

	rtc::RTCThreadManagerInterface* _rtc_thread_manager;
};

}



#endif // __RTC_SDP_OFFER_ANSWER_HANDLER_H__