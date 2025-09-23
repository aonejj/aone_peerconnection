//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /api/video_track_source_proxy.h
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_API_VIDEO_TRACK_SOURCE_PROXY_H__
#define __RTC_API_VIDEO_TRACK_SOURCE_PROXY_H__


#include "api/proxy.h"

#include "../../src_update/api/_media_stream_interface.h"


namespace webrtc {

	// Makes sure the real VideoTrackSourceInterface implementation is destroyed on
	// the signaling thread and marshals all method calls to the signaling thread.
	// TODO(deadbeef): Move this to .cc file and out of api/. What threads methods
	// are called on is an implementation detail.
BEGIN_PROXY_MAP(VideoTrackSource)
PROXY_PRIMARY_THREAD_DESTRUCTOR()
PROXY_CONSTMETHOD0(SourceState, state)
BYPASS_PROXY_CONSTMETHOD0(bool, remote)
PROXY_METHOD1(bool, GetStats, Stats*)
PROXY_METHOD1(void, RegisterObserver, ObserverInterface*)
PROXY_METHOD1(void, UnregisterObserver, ObserverInterface*)
PROXY_CONSTMETHOD0(bool, SupportsEncodedOutput)
PROXY_SECONDARY_METHOD0(void, GenerateKeyFrame)
END_PROXY_MAP(VideoTrackSource)

}  // namespace webrtc



#endif // __RTC_API_VIDEO_TRACK_SOURCE_PROXY_H__