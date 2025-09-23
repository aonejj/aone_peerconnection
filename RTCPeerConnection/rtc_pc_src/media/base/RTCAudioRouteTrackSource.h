//////////////////////////////////////////////////////////////////////////
//
//	author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_AUDIO_ROUTE_TRACK_SOURCE_H__
#define __RTC_AUDIO_ROUTE_TRACK_SOURCE_H__

#include <memory>

#include "api/notifier.h"
#include "rtc_base/synchronization/mutex.h"
#include "rtc_base/system/rtc_export.h"
#include "rtc_base/thread_annotations.h"
#include "modules/rtp_rtcp/source/rtp_packet_received.h"

#include "../../api/audio/RTCAudioPacketRouteSourceInterface.h"

namespace rtc {

class RTC_EXPORT RTCAudioRouteTrackSource :
	public RTCAudioPacketRouteSourceInterface<std::unique_ptr<webrtc::RtpPacketReceived>> {
public:
	RTCAudioRouteTrackSource();
	~RTCAudioRouteTrackSource();

public:
	MediaSourceInterface::SourceState state() const override { return MediaSourceInterface::SourceState::kLive; }
	bool remote() const override { return true; }

	void OnRtpPacket(std::unique_ptr<webrtc::RtpPacketReceived> packet);

private:
	void AddPacketRoutingSink(RTCPacketRoutePtrSinkInterface<std::unique_ptr<webrtc::RtpPacketReceived>> *sink) override;
	void RemovePacketRoutingSink(RTCPacketRoutePtrSinkInterface<std::unique_ptr<webrtc::RtpPacketReceived>> *sink) override;

	void _routing_packet(std::unique_ptr<webrtc::RtpPacketReceived> packet);

private:
	webrtc::Mutex sink_mutex_;
	RTCPacketRoutePtrSinkInterface<std::unique_ptr<webrtc::RtpPacketReceived>>  *sink_ RTC_GUARDED_BY(sink_mutex_);
};

}	// namespace rtc

#endif // __RTC_AUDIO_ROUTE_TRACK_SOURCE_H__