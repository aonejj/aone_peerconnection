//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_PC_AUDIO_ROUTE_TRACK_H__
#define __RTC_PC_AUDIO_ROUTE_TRACK_H__

#include <string>

#include "api/media_stream_track.h"
#include "api/scoped_refptr.h"
#include "rtc_base/thread.h"
#include "rtc_base/thread_annotations.h"
#include "rtc_base/thread_checker.h"

#include "../api/_rtc_media_stream_interface.h"

namespace webrtc {

class RTCAudioRouteTrack : public MediaStreamTrack<RTCAudioTrackInterface>,
						   public ObserverInterface {
public:
	static rtc::scoped_refptr<RTCAudioRouteTrack> Create(
		const std::string& id,
		RTCAudioSourceInterface* source,
		rtc::Thread* worker_thread);

	RTCAudioSourceInterface* GetSource() const override {
		return audio_source_.get();
	}

	bool set_enabled(bool enable) override;
	std::string kind() const override;

protected:
	RTCAudioRouteTrack(const std::string& id,
		RTCAudioSourceInterface* audio_source,
		rtc::Thread* worker_thread);
	~RTCAudioRouteTrack();

private:
	void OnChanged() override;

private:
	rtc::Thread* const worker_thread_;
	rtc::ThreadChecker signaling_thread_checker_;
	rtc::scoped_refptr<RTCAudioSourceInterface> audio_source_;
};

} // namespace webrtc

#endif // __RTC_PC_AUDIO_ROUTE_TRACK_H__