//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /pc/video_rtp_track_source.cc
//
//////////////////////////////////////////////////////////////////////////

#include "RTCVideoRtpTrackSource.h"

namespace webrtc {

RTCVideoRtpTrackSource::RTCVideoRtpTrackSource(Callback* callback)
	: RTCVideoTrackSource(true /* remote */), callback_(callback) {
	worker_sequence_checker_.Detach();
}

void RTCVideoRtpTrackSource::ClearCallback() {
	RTC_DCHECK_RUN_ON(&worker_sequence_checker_);
	callback_ = nullptr;
}


bool RTCVideoRtpTrackSource::SupportsEncodedOutput() const {
	return true;
}

void RTCVideoRtpTrackSource::GenerateKeyFrame() {
	RTC_DCHECK_RUN_ON(&worker_sequence_checker_);
	if (callback_) {
		callback_->OnGenerateKeyFrame();
	}
}

}  // namespace webrtc

