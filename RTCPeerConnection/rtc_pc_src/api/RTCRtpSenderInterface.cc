//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /api/rtp_sender_interface.cc
//
//////////////////////////////////////////////////////////////////////////

#include "RTCRtpSenderInterface.h"


namespace webrtc {

std::vector<RtpEncodingParameters> RTCRtpSenderInterface::init_send_encodings()
	const {
	return{};
}

rtc::scoped_refptr<DtlsTransportInterface> RTCRtpSenderInterface::dtls_transport()
	const {
	return nullptr;
}

}  // namespace webrtc
