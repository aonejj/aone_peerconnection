//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /pc/video_.h
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_PC_VIDEO_TRACK_SOURCE_H__
#define __RTC_PC_VIDEO_TRACK_SOURCE_H__

#include "api/notifier.h"
#include "rtc_base/system/rtc_export.h"
#include "rtc_base/thread_checker.h"

#include "../../src_update/api/_media_stream_interface.h"
#include "../../src_update/media/base/_media_channel.h"

namespace webrtc {

class RTC_EXPORT RTCVideoTrackSource : public Notifier<VideoTrackSourceInterface> {
public:
	explicit RTCVideoTrackSource(bool remote);
	void SetState(SourceState new_state);

	SourceState state() const override { return state_; }
	bool remote() const override { return remote_; }
	
	bool GetStats(Stats* stats) override { return false; }
	bool SupportsEncodedOutput() const override { return false; }
	void GenerateKeyFrame() override {}

private:
	rtc::ThreadChecker worker_thread_checker_;
	SourceState state_;
	const bool remote_;
};

}  // namespace webrtc


#endif	// __RTC_PC_VIDEO_TRACK_SOURCE_H__