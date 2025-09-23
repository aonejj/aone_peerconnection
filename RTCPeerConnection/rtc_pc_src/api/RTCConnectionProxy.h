//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : peerconnection proxy lite version for rtc media server.
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_CONNECTION_PROXY_H__
#define __RTC_CONNECTION_PROXY_H__

#include "api/proxy.h"
#include "api/jsep.h"
#include "api/make_ref_counted.h"

#include "RTCConnectionInterface.h"
#include "../../src_update/api/_peer_connection_interface_defs.h"

namespace webrtc {

BEGIN_PRIMARY_PROXY_MAP(RTCConnection)
PROXY_PRIMARY_THREAD_DESTRUCTOR()
PROXY_METHOD2(void,
			  CreateOffer,
			  CreateSessionDescriptionObserver*,
			  const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions&)
PROXY_METHOD2(void, 
			  CreateAnswer,
			  CreateSessionDescriptionObserver*,
			  const PeerConnectionInterfaceDefs::RTCOfferAnswerOptions&)
PROXY_METHOD2(void, 
			  SetLocalDescription,
			  SetSessionDescriptionObserver*,
			  SessionDescriptionInterface*)
PROXY_METHOD2(void,
			  SetLocalDescription,
			  rtc::scoped_refptr<SetSessionDescriptionObserver>,
			  std::unique_ptr<SessionDescriptionInterface>)
PROXY_METHOD2(void, 
			  SetRemoteDescription,
			  SetSessionDescriptionObserver*,
			  SessionDescriptionInterface*)
PROXY_METHOD2(void, 
			  SetRemoteDescription,
			  rtc::scoped_refptr<SetSessionDescriptionObserver>,
			  std::unique_ptr<SessionDescriptionInterface>)
PROXY_METHOD1(bool, ShouldFireNegotiationNeededEvent, uint32_t)
PROXY_METHOD1(bool,
			  AddIceCandidate,
			  const IceCandidateInterface*)
PROXY_METHOD2(void,
			  AddIceCandidate,
			  std::unique_ptr<IceCandidateInterface>, 
			  std::function<void(RTCError)>)
PROXY_METHOD1(bool,
			  RemoveIceCandidates,
			  const std::vector<cricket::Candidate>&)
PROXY_METHOD0(PeerConnectionInterfaceDefs::IceConnectionState, 
			  ice_connection_state)
PROXY_METHOD2(RTCErrorOr<rtc::scoped_refptr<RTCRtpSenderInterface>>,
			  AddTrack,
			  rtc::scoped_refptr<MediaStreamTrackInterface>,
			  const std::vector<std::string>&)
PROXY_METHOD1(RTCError,
			  RemoveTrack,
			  rtc::scoped_refptr<RTCRtpSenderInterface>)
PROXY_METHOD1(RTCErrorOr<rtc::scoped_refptr<RTCRtpTransceiverInterface>>,
			  AddTransceiver,
			  rtc::scoped_refptr<MediaStreamTrackInterface>)
PROXY_METHOD2(RTCErrorOr<rtc::scoped_refptr<RTCRtpTransceiverInterface>>,
			  AddTransceiver,
			  rtc::scoped_refptr<MediaStreamTrackInterface>,
			  const RTCRtpTransceiverInit&)
PROXY_METHOD1(RTCErrorOr<rtc::scoped_refptr<RTCRtpTransceiverInterface>>,
			  AddTransceiver,
			  cricket::MediaType)
PROXY_METHOD2(RTCErrorOr<rtc::scoped_refptr<RTCRtpTransceiverInterface>>,
			  AddTransceiver,
			  cricket::MediaType,
			  const RTCRtpTransceiverInit&)
PROXY_CONSTMETHOD0(std::vector<rtc::scoped_refptr<RTCRtpSenderInterface>>,
				   GetSenders)
PROXY_CONSTMETHOD0(std::vector<rtc::scoped_refptr<RTCRtpReceiverInterface>>,
				   GetReceivers)
PROXY_CONSTMETHOD0(std::vector<rtc::scoped_refptr<RTCRtpTransceiverInterface>>,
				   GetTransceivers)
PROXY_METHOD2(rtc::scoped_refptr<DataChannelInterface>,
			  CreateDataChannel,
			  const std::string&,
			  const DataChannelInit*)
PROXY_METHOD0(void, GenerateKeyFrame)
PROXY_METHOD1(void, GenerateKeyFramePli, uint32_t)
PROXY_CONSTMETHOD0(const SessionDescriptionInterface*, local_description)
PROXY_CONSTMETHOD0(const SessionDescriptionInterface*, remote_description)
PROXY_CONSTMETHOD0(const SessionDescriptionInterface*, current_local_description)
PROXY_CONSTMETHOD0(const SessionDescriptionInterface*, current_remote_description)
PROXY_CONSTMETHOD0(const SessionDescriptionInterface*, pending_local_description)
PROXY_CONSTMETHOD0(const SessionDescriptionInterface*, pending_remote_description)
PROXY_METHOD0(void, Close)
PROXY_METHOD0(RTCCall::Stats, GetCallStats)
END_PROXY_MAP(RTCConnection)

}

#endif // __RTC_CONNECTION_PROXY_H__