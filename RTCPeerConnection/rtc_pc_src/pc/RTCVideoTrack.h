//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /pc/video_track.h
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_PC_VIDEO_TRACK_H__
#define __RTC_PC_VIDEO_TRACK_H__

#include <string>


#include "api/media_stream_track.h"
#include "api/scoped_refptr.h"

#include "rtc_base/thread.h"
#include "rtc_base/thread_annotations.h"
#include "rtc_base/thread_checker.h"

#include "../../src_update/api/_media_stream_interface.h"

namespace webrtc {

class RTCVideoTrack : public MediaStreamTrack<VideoTrackInterface>,
	public ObserverInterface {
public:
	static rtc::scoped_refptr<RTCVideoTrack> Create(
		const std::string& label,
		VideoTrackSourceInterface* source,
		rtc::Thread* worker_thread);

	VideoTrackSourceInterface* GetSource() const override {
		return video_source_.get();
	}
	bool set_enabled(bool enable) override;
	std::string kind() const override;

protected:
	RTCVideoTrack(const std::string& id,
		VideoTrackSourceInterface* video_source,
		rtc::Thread* worker_thread);
	~RTCVideoTrack();

private:
	void OnChanged() override;

	rtc::Thread* const worker_thread_;
	rtc::ThreadChecker signaling_thread_checker_;
	rtc::scoped_refptr<VideoTrackSourceInterface> video_source_;
};

}  // namespace webrtc

#endif // __RTC_PC_VIDEO_TRACK_H__