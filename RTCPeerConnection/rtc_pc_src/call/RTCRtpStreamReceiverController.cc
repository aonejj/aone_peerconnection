//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#include "rtc_base/logging.h"
#include "modules/rtp_rtcp/source/rtp_packet_received.h"

#include "RTCRtpStreamReceiverController.h"

namespace webrtc {

RTCRtpStreamReceiverController::RTCReceiver::RTCReceiver(
	RTCRtpStreamReceiverController* controller,
	uint32_t ssrc,
	RtpPacketSinkInterface* sink)
	: controller_(controller), sink_(sink) {
	const bool sink_added = controller_->AddSink(ssrc, sink_);		
	if (!sink_added) {
		RTC_LOG(LS_ERROR)
			<< "RTCRtpStreamReceiverController::RTCReceiver::RTCReceiver: Sink "
			"could not be added for SSRC="
			<< ssrc << ".";
	}
}

RTCRtpStreamReceiverController::RTCReceiver::~RTCReceiver() {
	controller_->RemoveSink(sink_);
}

RTCRtpStreamReceiverController::RTCRtpStreamReceiverController() {}

RTCRtpStreamReceiverController::~RTCRtpStreamReceiverController() = default;

std::unique_ptr<RtpStreamReceiverInterface>
RTCRtpStreamReceiverController::CreateReceiver(uint32_t ssrc,
	RtpPacketSinkInterface* sink) {
	return std::make_unique<RTCReceiver>(this, ssrc, sink);
}

bool RTCRtpStreamReceiverController::OnRtpPacket(const RtpPacketReceived& packet) {
	MutexLock lock(&demuxer_mutex_);
	return demuxer_.OnRtpPacket(packet);
}

bool RTCRtpStreamReceiverController::AddSink(uint32_t ssrc,
		RtpPacketSinkInterface* sink) {
	MutexLock lock(&demuxer_mutex_);
	return demuxer_.AddSink(ssrc, sink);
}

size_t RTCRtpStreamReceiverController::RemoveSink(
		const RtpPacketSinkInterface* sink) {
	MutexLock lock(&demuxer_mutex_);
	return demuxer_.RemoveSink(sink);
}

}