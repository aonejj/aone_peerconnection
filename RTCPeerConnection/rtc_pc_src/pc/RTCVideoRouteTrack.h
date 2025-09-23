//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : video packet routing track
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_PC_VIDEO_ROUTE_TRACK_H__
#define __RTC_PC_VIDEO_ROUTE_TRACK_H__

#include <string>

#include "../../src_update/api/_media_stream_interface.h"

#include "api/media_stream_track.h"
#include "api/scoped_refptr.h"
#include "rtc_base/thread.h"
#include "rtc_base/thread_annotations.h"
#include "rtc_base/thread_checker.h"



namespace webrtc {

class RTCVideoRouteTrack : public MediaStreamTrack<VideoTrackInterface>,
						   public ObserverInterface {
public:
	static rtc::scoped_refptr<RTCVideoRouteTrack> Create(
		const std::string& id,
		VideoTrackSourceInterface* source,
		rtc::Thread* worker_thread);

	VideoTrackSourceInterface* GetSource() const override {
		return video_source_.get();
	}

	bool set_enabled(bool enable) override;
	std::string kind() const override;

protected:
	RTCVideoRouteTrack(const std::string& id,
					   VideoTrackSourceInterface* video_source,
					   rtc::Thread* worker_thread);
	~RTCVideoRouteTrack();

private:
	void OnChanged() override;

private:
	rtc::Thread* const worker_thread_;
	rtc::ThreadChecker signaling_thread_checker_;
	rtc::scoped_refptr<VideoTrackSourceInterface> video_source_;
};

}



#endif	// __RTC_PC_VIDEO_ROUTE_TRACK_H__