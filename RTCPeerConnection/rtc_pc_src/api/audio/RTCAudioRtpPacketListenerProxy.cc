//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#include "modules/rtp_rtcp/source/rtp_packet_received.h"
#include "rtc_base/logging.h"

#include "../RTCReceivedPacketSinks.h"
#include "RTCAudioRtpPacketListenerProxy.h"


namespace webrtc {

RTCAudioRtpPacketListenerProxy::RTCAudioRtpPacketListenerProxy() 
	:sink_(nullptr) {

}

RTCAudioRtpPacketListenerProxy::~RTCAudioRtpPacketListenerProxy() {

}

void RTCAudioRtpPacketListenerProxy::SetSink(RTCMediaReceivedPacketSink* sink) {
	MutexLock lock(&sink_lock_);
	sink_ = sink;
	if (_rtp_extensions != absl::nullopt) {
		if (sink_ != nullptr) {
			sink_->SetRtpHeaderExtensionMap(_rtp_extensions);
		}
	}
}

void RTCAudioRtpPacketListenerProxy::OnStart() {
	RTC_LOG(LS_VERBOSE) << "RTCAudioRtpPacketListenerProxy::OnStart";
}

void RTCAudioRtpPacketListenerProxy::OnStop() {
	RTC_LOG(LS_VERBOSE) << "RTCAudioRtpPacketListenerProxy::OnStop";
}

void RTCAudioRtpPacketListenerProxy::OnRtpPacket(const RtpPacketReceived& packet) {
	MutexLock lock(&sink_lock_);
	if (sink_ != nullptr) {
		//sink_->OnAudioRtpPacket(packet);
		sink_->OnRtpPacket(packet);
	}
}

void RTCAudioRtpPacketListenerProxy::SetRtpHeaderExtensionMap(absl::optional<RtpHeaderExtensionMap> extMap) {
	MutexLock lock(&sink_lock_);
	_rtp_extensions = extMap;
	if (sink_ != nullptr) {
		sink_->SetRtpHeaderExtensionMap(_rtp_extensions);
	}
}

}