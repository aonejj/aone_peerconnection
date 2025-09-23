#ifndef __RTC_NODE_OBSERVER_H__
#define __RTC_NODE_OBSERVER_H__

#include "rtc_base/ref_count.h"
#include "api/rtc_error.h"
#include "api/jsep.h"
#include "api/data_channel_interface.h"

#include "../../src_update/api/_peer_connection_interface_defs.h"
#include "RTCRtpReceiverInterface.h"
#include "RTCRtpTransceiverInterface.h"



namespace rtc_media_server {

class RTCNodeObserver {
public:
	virtual void OnAddStream(
		rtc::scoped_refptr<webrtc::MediaStreamInterface> stream) {}
	virtual void OnRemoveStream(
		rtc::scoped_refptr<webrtc::MediaStreamInterface> stream) {}

	virtual void OnIceConnectionChange(
		webrtc::PeerConnectionInterfaceDefs::IceConnectionState new_state) {}

	virtual void OnIceCandidate(
		const webrtc::IceCandidateInterface *candidate) = 0;

	virtual void OnAddTrack(
		rtc::scoped_refptr<webrtc::RTCRtpReceiverInterface> receiver,
		const std::vector<rtc::scoped_refptr<webrtc::MediaStreamInterface>>& streams) {}

	virtual void OnAddSimulcastTrack(std::vector<rtc::scoped_refptr<webrtc::RTCRtpReceiverInterface>>& receivers,
		const std::vector<rtc::scoped_refptr<webrtc::MediaStreamInterface>>& streams) {}

	virtual void OnTrack(
		rtc::scoped_refptr<webrtc::RTCRtpTransceiverInterface> transceiver) {}

	virtual void OnRemoveTrack(
		rtc::scoped_refptr<webrtc::RTCRtpReceiverInterface> receiver) {}

	virtual void OnDataChannel(rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel) {}

	virtual void OnSdpCreateSuccess(webrtc::SessionDescriptionInterface *desc) = 0;
	virtual void OnSdpCreateFailure(webrtc::RTCError error) = 0;

	virtual void OnSdpSetSuccess(bool is_local, webrtc::SessionDescriptionInterface* desc) = 0;
	virtual void OnSdpSetFailure(webrtc::RTCError error, bool is_local) = 0;
};

}

#endif	// __RTC_NODE_OBSERVER_H__