//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /pc/video_track_source.cc
//
//////////////////////////////////////////////////////////////////////////

#include "rtc_base/checks.h"

#include "RTCVideoTrackSource.h"

namespace webrtc {

RTCVideoTrackSource::RTCVideoTrackSource(bool remote)
	: state_(kInitializing), remote_(remote) {
	worker_thread_checker_.Detach();
}

void RTCVideoTrackSource::SetState(SourceState new_state) {
	if (state_ != new_state) {
		state_ = new_state;
		FireOnChanged();
	}
}

}