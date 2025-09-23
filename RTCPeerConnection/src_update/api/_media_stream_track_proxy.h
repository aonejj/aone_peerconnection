//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /api/media_stream_track_proxy.h
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_UPDATE_API_MEDIA_STREAM_TRACK_PROXY_H__
#define __RTC_UPDATE_API_MEDIA_STREAM_TRACK_PROXY_H__

#include <string>

#include "api/proxy.h"
#include "api/make_ref_counted.h"
#include "_media_stream_interface.h"

namespace webrtc {

BEGIN_PRIMARY_PROXY_MAP(AudioTrack)
PROXY_PRIMARY_THREAD_DESTRUCTOR()
BYPASS_PROXY_CONSTMETHOD0(std::string, kind)
BYPASS_PROXY_CONSTMETHOD0(std::string, id)
PROXY_CONSTMETHOD0(TrackState, state)
PROXY_CONSTMETHOD0(bool, enabled)
PROXY_CONSTMETHOD0(AudioSourceInterface*, GetSource)
PROXY_METHOD1(void, AddSink, AudioTrackSinkInterface*)
PROXY_METHOD1(void, RemoveSink, AudioTrackSinkInterface*)
PROXY_METHOD1(bool, GetSignalLevel, int*)
PROXY_METHOD1(bool, set_enabled, bool)
PROXY_METHOD1(void, RegisterObserver, ObserverInterface*)
PROXY_METHOD1(void, UnregisterObserver, ObserverInterface*)
END_PROXY_MAP(AudioTrack)

BEGIN_PROXY_MAP(VideoTrack)
PROXY_PRIMARY_THREAD_DESTRUCTOR()
BYPASS_PROXY_CONSTMETHOD0(std::string, kind)
BYPASS_PROXY_CONSTMETHOD0(std::string, id)
PROXY_CONSTMETHOD0(TrackState, state)
PROXY_CONSTMETHOD0(bool, enabled)
PROXY_METHOD1(bool, set_enabled, bool)
PROXY_CONSTMETHOD0(VideoTrackSourceInterface*, GetSource)

PROXY_METHOD1(void, RegisterObserver, ObserverInterface*)
PROXY_METHOD1(void, UnregisterObserver, ObserverInterface*)
END_PROXY_MAP(VideoTrack)

}  // namespace webrtc

#endif  // __RTC_UPDATE_API_MEDIA_STREAM_TRACK_PROXY_H__
