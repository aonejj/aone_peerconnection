//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : video packet routing track
//
//////////////////////////////////////////////////////////////////////////

#include <string>

#include "api/notifier.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "rtc_base/location.h"
#include "rtc_base/ref_counted_object.h"
#include "rtc_base/synchronization/sequence_checker.h"

#include "RTCVideoRouteTrack.h"

namespace webrtc {

rtc::scoped_refptr<RTCVideoRouteTrack> RTCVideoRouteTrack::Create(
				const std::string& id,
				VideoTrackSourceInterface* source,
				rtc::Thread* worker_thread) {
	rtc::RefCountedObject<RTCVideoRouteTrack>* track =
		new rtc::RefCountedObject<RTCVideoRouteTrack>(id, source, worker_thread);
	return track;
}

RTCVideoRouteTrack::RTCVideoRouteTrack(
		const std::string& label,
		VideoTrackSourceInterface* video_source,
		rtc::Thread* worker_thread)
	: MediaStreamTrack<VideoTrackInterface>(label),
	  worker_thread_(worker_thread),
	  video_source_(video_source) {
	video_source_->RegisterObserver(this);
}

RTCVideoRouteTrack::~RTCVideoRouteTrack() {
	video_source_->UnregisterObserver(this);
}

std::string RTCVideoRouteTrack::kind() const {
	return kVideoKind;
}

bool RTCVideoRouteTrack::set_enabled(bool enable) {
	RTC_DCHECK(signaling_thread_checker_.IsCurrent());
	return MediaStreamTrack<VideoTrackInterface>::set_enabled(enable);
}

void RTCVideoRouteTrack::OnChanged() {
	RTC_DCHECK(signaling_thread_checker_.IsCurrent());
	if (video_source_->state() == MediaSourceInterface::kEnded) {
		set_state(kEnded);
	}
	else {
		set_state(kLive);
	}
}

}