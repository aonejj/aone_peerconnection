//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /api/rtp_receiver_interface.cc
//
//////////////////////////////////////////////////////////////////////////

#include "RTCRtpReceiverInterface.h"


namespace webrtc {

std::vector<std::string> RTCRtpReceiverInterface::stream_ids() const {
	return{};
}

std::vector<rtc::scoped_refptr<MediaStreamInterface>>
RTCRtpReceiverInterface::streams() const {
	return{};
}

std::vector<RtpSource> RTCRtpReceiverInterface::GetSources() const {
	return{};
}

rtc::scoped_refptr<DtlsTransportInterface>
RTCRtpReceiverInterface::dtls_transport() const {
	return nullptr;
}

}  // namespace webrtc
