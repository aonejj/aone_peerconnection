//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#include <string>

#include "rtc_base/logging.h"

#include "api/notifier.h"
#include "rtc_base/checks.h"
#include "rtc_base/location.h"
#include "rtc_base/ref_counted_object.h"
#include "rtc_base/synchronization/sequence_checker.h"

#include "RTCAudioRouteTrack.h"

namespace webrtc {

rtc::scoped_refptr<RTCAudioRouteTrack> RTCAudioRouteTrack::Create(
	const std::string& id, 
	RTCAudioSourceInterface* source, 
	rtc::Thread* worker_thread) {
	rtc::RefCountedObject<RTCAudioRouteTrack>* track =
		new rtc::RefCountedObject<RTCAudioRouteTrack>(id, source, worker_thread);
	return track;
}

RTCAudioRouteTrack::RTCAudioRouteTrack(
	const std::string& label,
	RTCAudioSourceInterface* audio_source, 
	rtc::Thread* worker_thread) 
	: MediaStreamTrack<RTCAudioTrackInterface>(label),
	  worker_thread_(worker_thread),
	  audio_source_(audio_source) {
	audio_source_->RegisterObserver(this);
}

RTCAudioRouteTrack::~RTCAudioRouteTrack() {
	audio_source_->UnregisterObserver(this);
}

std::string RTCAudioRouteTrack::kind() const {
	return kAudioKind;
}

bool RTCAudioRouteTrack::set_enabled(bool enable) {
	RTC_DCHECK(signaling_thread_checker_.IsCurrent());

	return MediaStreamTrack<RTCAudioTrackInterface>::set_enabled(enable);
}

void RTCAudioRouteTrack::OnChanged() {
	RTC_DCHECK(signaling_thread_checker_.IsCurrent());
	if (audio_source_->state() == MediaSourceInterface::kEnded) {
		set_state(kEnded);
	}
	else {
		set_state(kLive);
	}
}

}	// namespace webrtc