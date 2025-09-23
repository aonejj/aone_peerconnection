//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /api/media_stream_interface.h
//
//////////////////////////////////////////////////////////////////////////

#include "api/media_types.h"

#include "_media_stream_interface.h"

namespace webrtc {

const char* const MediaStreamTrackInterface::kVideoKind =
    cricket::kMediaTypeVideo;
const char* const MediaStreamTrackInterface::kAudioKind =
    cricket::kMediaTypeAudio;


bool AudioTrackInterface::GetSignalLevel(int* level) {
  return false;
}

const cricket::AudioOptions AudioSourceInterface::options() const {
  return {};
}

}  // namespace webrtc
