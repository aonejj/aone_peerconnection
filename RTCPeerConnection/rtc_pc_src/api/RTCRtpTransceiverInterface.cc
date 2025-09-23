//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : Replace RtpTransceiverInterface to RTCRtpTransceiverInterface class for rtc media server.
//
//////////////////////////////////////////////////////////////////////////

#include "rtc_base/checks.h"

#include "RTCRtpTransceiverInterface.h"


namespace webrtc {

RTCRtpTransceiverInit::RTCRtpTransceiverInit() = default;

RTCRtpTransceiverInit::RTCRtpTransceiverInit(const RTCRtpTransceiverInit& rhs) = default;

RTCRtpTransceiverInit::~RTCRtpTransceiverInit() = default;


absl::optional<RtpTransceiverDirection>
RTCRtpTransceiverInterface::fired_direction() const {
	return absl::nullopt;
}

bool RTCRtpTransceiverInterface::stopping() const {
	return false;
}

void RTCRtpTransceiverInterface::Stop() {
	StopInternal();
}

RTCError RTCRtpTransceiverInterface::StopStandard() {
	RTC_NOTREACHED() << "DEBUG: RtpTransceiverInterface::StopStandard called";
	return RTCError::OK();
}

void RTCRtpTransceiverInterface::StopInternal() {
	RTC_NOTREACHED() << "DEBUG: RtpTransceiverInterface::StopInternal called";
}

RTCError RTCRtpTransceiverInterface::SetCodecPreferences(
	rtc::ArrayView<RtpCodecCapability>) {
	RTC_NOTREACHED() << "Not implemented";
	return{};
}

std::vector<RtpCodecCapability> RTCRtpTransceiverInterface::codec_preferences()
	const {
	return{};
}

std::vector<RtpHeaderExtensionCapability>
RTCRtpTransceiverInterface::HeaderExtensionsToOffer() const {
	return{};
}

webrtc::RTCError RTCRtpTransceiverInterface::SetOfferedRtpHeaderExtensions(
	rtc::ArrayView<const RtpHeaderExtensionCapability>
	header_extensions_to_offer) {
	return webrtc::RTCError(webrtc::RTCErrorType::UNSUPPORTED_OPERATION);
}

std::vector<RtpHeaderExtensionCapability>
RTCRtpTransceiverInterface::HeaderExtensionsNegotiated() const {
	return{};
}

// TODO(bugs.webrtc.org/11839) Remove default implementations when clients
// are updated.
void RTCRtpTransceiverInterface::SetDirection(
	RtpTransceiverDirection new_direction) {
	SetDirectionWithError(new_direction);
}

RTCError RTCRtpTransceiverInterface::SetDirectionWithError(
	RtpTransceiverDirection new_direction) {
	RTC_NOTREACHED() << "Default implementation called";
	return RTCError::OK();
}

}