//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : video packet routing track source
//
//////////////////////////////////////////////////////////////////////////


#ifndef __RTC_VIDEO_ROUTE_TRACK_SOURCE_H__
#define __RTC_VIDEO_ROUTE_TRACK_SOURCE_H__

#include <memory>

#include <stdint.h>

#include "absl/types/optional.h"

#include "api/notifier.h"
#include "rtc_base/synchronization/mutex.h"
#include "rtc_base/system/rtc_export.h"
#include "rtc_base/thread_annotations.h"
#include "modules/rtp_rtcp/source/rtp_packet_received.h"

#include "../../../src_update/api/_media_stream_interface.h"
#include "../../api/video/RTCVideoPacketRouteSourceInterface.h"



namespace rtc {

class RTC_EXPORT RTCVideoRouteTrackSource
	:public RTCVideoPacketRouteSourceInterface<std::unique_ptr<webrtc::RtpPacketReceived>> {
public:
	RTCVideoRouteTrackSource();
	~RTCVideoRouteTrackSource();

public:
	bool GetStats(Stats* stats) override;
	bool SupportsEncodedOutput() const override { return true;  }
	void GenerateKeyFrame() override;
	MediaSourceInterface::SourceState state() const override { return MediaSourceInterface::SourceState::kLive; }
	bool remote() const  override { return true; }

	void OnRtpPacket(std::unique_ptr<webrtc::RtpPacketReceived> packet);

	void SetRouteFeedbackSource(RTCVideoPacketRouteSourceFeedbackInterface* source);

private:
	void AddPacketRoutingSink(RTCPacketRoutePtrSinkInterface<std::unique_ptr<webrtc::RtpPacketReceived>> *sink) override;
	void RemovePacketRoutingSink(RTCPacketRoutePtrSinkInterface<std::unique_ptr<webrtc::RtpPacketReceived>> *sink) override;

	void _routing_packet(std::unique_ptr<webrtc::RtpPacketReceived> packet);

private:
	webrtc::Mutex stats_mutex_;
	absl::optional<Stats> stats_ RTC_GUARDED_BY(stats_mutex_);

	webrtc::Mutex sink_mutex_;
	RTCPacketRoutePtrSinkInterface<std::unique_ptr<webrtc::RtpPacketReceived>>  *sink_ RTC_GUARDED_BY(sink_mutex_);
	RTCVideoPacketRouteSourceFeedbackInterface* feedback_source_;
};

}


#endif // __RTC_VIDEO_ROUTE_TRACK_SOURCE_H__