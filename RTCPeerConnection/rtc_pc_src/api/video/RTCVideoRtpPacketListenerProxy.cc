//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#include "modules/rtp_rtcp/source/rtp_packet_received.h"
#include "rtc_base/logging.h"

#include "../RTCReceivedPacketSinks.h"
#include "RTCVideoRtpPacketListenerProxy.h"

namespace webrtc {

RTCVideoRtpPacketListenerProxy::RTCVideoRtpPacketListenerProxy()
: sink_(nullptr) {
}

RTCVideoRtpPacketListenerProxy::~RTCVideoRtpPacketListenerProxy() {

}

void RTCVideoRtpPacketListenerProxy::SetSink(RTCMediaReceivedPacketSink* sink) {
	MutexLock lock(&sink_lock_);
	sink_ = sink;
	if (_rtp_extensions != absl::nullopt) {
		if (sink_ != nullptr) {
			sink_->SetRtpHeaderExtensionMap(_rtp_extensions);
		}
	}
}

void RTCVideoRtpPacketListenerProxy::SetRtpHeaderExtensionMap(absl::optional<RtpHeaderExtensionMap> extMap) {
	MutexLock lock(&sink_lock_);
	_rtp_extensions = extMap;
	if (sink_ != nullptr) {
		sink_->SetRtpHeaderExtensionMap(_rtp_extensions);
	}
}

void RTCVideoRtpPacketListenerProxy::OnStart() {

}

void RTCVideoRtpPacketListenerProxy::OnStop() {

}

void RTCVideoRtpPacketListenerProxy::OnRtpPacket(const RtpPacketReceived& packet) {
	MutexLock lock(&sink_lock_);
	if (sink_ != nullptr) {
		sink_->OnRtpPacket(packet);
	}
}

void RTCVideoRtpPacketListenerProxy::OnRtxRtpPacket(const RtpPacketReceived& packet) {
	MutexLock lock(&sink_lock_);
	if (sink_ != nullptr) {
		//sink_->OnVideoRtxRtpPacket(packet);
		sink_->OnRtxRtpPacket(packet);
	}
}

}	// namespace webrtc