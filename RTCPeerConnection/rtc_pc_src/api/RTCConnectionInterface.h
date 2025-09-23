//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : peerconnection interface lite version for rtc media server.
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_CONNECTION_INTERFACE_H__
#define __RTC_CONNECTION_INTERFACE_H__

#include <vector>

#include "rtc_base/ref_count.h"
#include "rtc_base/system/rtc_export.h"
#include "api/rtc_error.h"
#include "api/jsep.h"
#include "api/data_channel_interface.h"
#include "api/sctp_transport_interface.h"
#include "pc/rtp_data_channel.h"
#include "pc/sctp_data_channel.h"
#include "p2p/base/port.h"

#include "../../src_update/api/_peer_connection_interface_defs.h"
#include "../call/RTCCall.h"
#include "../pc/RTCChannel.h"
#include "RTCRtpReceiverInterface.h"
#include "RTCRtpTransceiverInterface.h"

namespace webrtc {

class RtpHeaderExtensionMap;

class StreamCollectionInterface : public rtc::RefCountInterface {
public:
	// TODO(ronghuawu): Update the function names to c++ style, e.g. find -> Find.
	virtual size_t count() = 0;
	virtual MediaStreamInterface* at(size_t index) = 0;
	virtual MediaStreamInterface* find(const std::string& label) = 0;
	virtual MediaStreamTrackInterface* FindAudioTrack(const std::string& id) = 0;
	virtual MediaStreamTrackInterface* FindVideoTrack(const std::string& id) = 0;

protected:
	// Dtor protected as objects shouldn't be deleted via this interface.
	~StreamCollectionInterface() override = default;
};

class RTC_EXPORT RTCConnectionInterface : public rtc::RefCountInterface {
public:
	virtual void CreateOffer(CreateSessionDescriptionObserver *observer, const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions& options) = 0;
	virtual void CreateAnswer(CreateSessionDescriptionObserver *observer, const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions& options) = 0;

	virtual void SetLocalDescription(SetSessionDescriptionObserver *observer, SessionDescriptionInterface *desc_ptr) = 0;
	virtual void SetLocalDescription(rtc::scoped_refptr<SetSessionDescriptionObserver> observer,
									 std::unique_ptr<SessionDescriptionInterface> desc) = 0;
	virtual void SetRemoteDescription(SetSessionDescriptionObserver *observer, SessionDescriptionInterface *desc_ptr) = 0;
	virtual void SetRemoteDescription(rtc::scoped_refptr<SetSessionDescriptionObserver> observer, 
									  std::unique_ptr<SessionDescriptionInterface> desc) = 0;
	
	virtual bool ShouldFireNegotiationNeededEvent(uint32_t event_id) {
		return true;
	}

	virtual bool AddIceCandidate(const IceCandidateInterface* candidate) = 0;
	virtual void AddIceCandidate(std::unique_ptr<IceCandidateInterface> candidate,
								 std::function<void(RTCError)> callback) {}

	virtual bool RemoveIceCandidates(
		const std::vector<cricket::Candidate>& candidates) = 0;

	virtual PeerConnectionInterfaceDefs::IceConnectionState ice_connection_state() = 0;

	virtual RTCErrorOr<rtc::scoped_refptr<RTCRtpSenderInterface>> AddTrack(
								rtc::scoped_refptr<MediaStreamTrackInterface> track,
								const std::vector<std::string>& stream_ids) = 0;
	virtual RTCError RemoveTrack(rtc::scoped_refptr<RTCRtpSenderInterface> sender) = 0;

	virtual RTCErrorOr<rtc::scoped_refptr<RTCRtpTransceiverInterface>>
		AddTransceiver(rtc::scoped_refptr<MediaStreamTrackInterface> track) = 0;
	virtual RTCErrorOr<rtc::scoped_refptr<RTCRtpTransceiverInterface>>
		AddTransceiver(rtc::scoped_refptr<MediaStreamTrackInterface> track,
		const RTCRtpTransceiverInit& init) = 0;

	virtual RTCErrorOr<rtc::scoped_refptr<RTCRtpTransceiverInterface>>
		AddTransceiver(cricket::MediaType media_type) = 0;
	virtual RTCErrorOr<rtc::scoped_refptr<RTCRtpTransceiverInterface>>
		AddTransceiver(cricket::MediaType media_type,
		const RTCRtpTransceiverInit& init) = 0;

	virtual std::vector<rtc::scoped_refptr<RTCRtpSenderInterface>> GetSenders() const = 0;
	virtual std::vector<rtc::scoped_refptr<RTCRtpReceiverInterface>> GetReceivers() const = 0;
	virtual std::vector<rtc::scoped_refptr<RTCRtpTransceiverInterface>> GetTransceivers() const = 0;

	virtual rtc::scoped_refptr<DataChannelInterface> CreateDataChannel(
		const std::string& label,
		const DataChannelInit* config) = 0;

	virtual void GenerateKeyFrame() = 0;
	virtual void GenerateKeyFramePli(uint32_t ssrc) = 0;

	virtual const SessionDescriptionInterface* local_description() const = 0;
	virtual const SessionDescriptionInterface* remote_description() const = 0;
	virtual const SessionDescriptionInterface* current_local_description() const = 0;
	virtual const SessionDescriptionInterface* current_remote_description()
		const = 0;
	virtual const SessionDescriptionInterface* pending_local_description() const = 0;
	virtual const SessionDescriptionInterface* pending_remote_description()
		const = 0;
	virtual void Close() = 0;

	virtual RTCCall::Stats GetCallStats() = 0;

	virtual std::vector<DataChannelStats> GetDataChannelStats() const {
		return{};
	}
	virtual absl::optional<std::string> sctp_transport_name() const {
		return absl::nullopt;
	}
};

class RTCConnectionObserver {
public:
	virtual ~RTCConnectionObserver() = default;

	// Triggered when the SignalingState changed.
	virtual void OnSignalingChange(
		PeerConnectionInterfaceDefs::SignalingState new_state) = 0;

	// Triggered when media is received on a new stream from remote peer.
	virtual void OnAddStream(rtc::scoped_refptr<MediaStreamInterface> stream) {}

	// Triggered when a remote peer closes a stream.
	virtual void OnRemoveStream(rtc::scoped_refptr<MediaStreamInterface> stream) {
	}

	// Triggered when a remote peer opens a data channel.
	virtual void OnDataChannel(
		rtc::scoped_refptr<DataChannelInterface> data_channel) = 0;

	// Triggered when renegotiation is needed. For example, an ICE restart
	// has begun.
	// TODO(hbos): Delete in favor of OnNegotiationNeededEvent() when downstream
	// projects have migrated.
	virtual void OnRenegotiationNeeded() {}

	// Used to fire spec-compliant onnegotiationneeded events, which should only
	// fire when the Operations Chain is empty. The observer is responsible for
	// queuing a task (e.g. Chromium: jump to main thread) to maybe fire the
	// event. The event identified using |event_id| must only fire if
	// PeerConnection::ShouldFireNegotiationNeededEvent() returns true since it is
	// possible for the event to become invalidated by operations subsequently
	// chained.
	virtual void OnNegotiationNeededEvent(uint32_t event_id) {}

	// Called any time the legacy IceConnectionState changes.
	//
	// Note that our ICE states lag behind the standard slightly. The most
	// notable differences include the fact that "failed" occurs after 15
	// seconds, not 30, and this actually represents a combination ICE + DTLS
	// state, so it may be "failed" if DTLS fails while ICE succeeds.
	//
	// TODO(jonasolsson): deprecate and remove this.
	virtual void OnIceConnectionChange(
		PeerConnectionInterfaceDefs::IceConnectionState new_state) {}

	// Called any time the standards-compliant IceConnectionState changes.
	virtual void OnStandardizedIceConnectionChange(
		PeerConnectionInterfaceDefs::IceConnectionState new_state) {}

	// Called any time the PeerConnectionState changes.
	virtual void OnConnectionChange(
		PeerConnectionInterfaceDefs::PeerConnectionState new_state) {}

	// Called any time the IceGatheringState changes.
	virtual void OnIceGatheringChange(
		PeerConnectionInterfaceDefs::IceGatheringState new_state) = 0;

	// A new ICE candidate has been gathered.
	virtual void OnIceCandidate(const IceCandidateInterface* candidate) = 0;

	// Gathering of an ICE candidate failed.
	// See https://w3c.github.io/webrtc-pc/#event-icecandidateerror
	// |host_candidate| is a stringified socket address.
	virtual void OnIceCandidateError(const std::string& host_candidate,
		const std::string& url,
		int error_code,
		const std::string& error_text) {}

	// Gathering of an ICE candidate failed.
	// See https://w3c.github.io/webrtc-pc/#event-icecandidateerror
	virtual void OnIceCandidateError(const std::string& address,
		int port,
		const std::string& url,
		int error_code,
		const std::string& error_text) {}

	// Ice candidates have been removed.
	// TODO(honghaiz): Make this a pure virtual method when all its subclasses
	// implement it.
	virtual void OnIceCandidatesRemoved(
		const std::vector<cricket::Candidate>& candidates) {}

	// Called when the ICE connection receiving status changes.
	virtual void OnIceConnectionReceivingChange(bool receiving) {}

	// Called when the selected candidate pair for the ICE connection changes.
	virtual void OnIceSelectedCandidatePairChanged(
		const cricket::CandidatePairChangeEvent& event) {}

	virtual void OnAddTrack(
		rtc::scoped_refptr<RTCRtpReceiverInterface> receiver,
		const std::vector<rtc::scoped_refptr<MediaStreamInterface>>& streams) {}

	virtual void OnAddSimulcastTrack(std::vector<rtc::scoped_refptr<RTCRtpReceiverInterface>> &receivers,
		const std::vector<rtc::scoped_refptr<MediaStreamInterface>>& streams) {}

	virtual void OnTrack(
		rtc::scoped_refptr<RTCRtpTransceiverInterface> transceiver) {}

	virtual void OnRemoveTrack(
		rtc::scoped_refptr<RTCRtpReceiverInterface> receiver) {}

};

}

#endif	// __RTC_CONNECTION_INTERFACE_H__