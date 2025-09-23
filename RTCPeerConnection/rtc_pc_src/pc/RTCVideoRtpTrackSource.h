//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /pc/video_rtp_track_source.h
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_PC_VIDEO_RTC_TRACK_SOURCE_H__
#define __RTC_PC_VIDEO_RTC_TRACK_SOURCE_H__

#include <vector>

#include "rtc_base/callback.h"
#include "rtc_base/synchronization/mutex.h"
#include "rtc_base/system/no_unique_address.h"

#include "RTCVideoTrackSource.h"

namespace webrtc {

// Video track source in use by VideoRtpReceiver
class RTCVideoRtpTrackSource : public RTCVideoTrackSource {
public:
	class Callback {
	public:
		virtual ~Callback() = default;

		// Called when a keyframe should be generated
		virtual void OnGenerateKeyFrame() = 0;
	};

	explicit RTCVideoRtpTrackSource(Callback* callback);

	// Call before the object implementing Callback finishes it's destructor. No
	// more callbacks will be fired after completion. Must be called on the
	// worker thread
	void ClearCallback();

	// Returns true. This method can be called on any thread.
	bool SupportsEncodedOutput() const override;

	// Generates a key frame. Must be called on the worker thread.
	void GenerateKeyFrame() override;

private:
	RTC_NO_UNIQUE_ADDRESS SequenceChecker worker_sequence_checker_;
	Callback* callback_ RTC_GUARDED_BY(worker_sequence_checker_);

	RTC_DISALLOW_COPY_AND_ASSIGN(RTCVideoRtpTrackSource);
};

}  // namespace webrtc


#endif // __RTC_PC_VIDEO_RTC_TRACK_SOURCE_H__