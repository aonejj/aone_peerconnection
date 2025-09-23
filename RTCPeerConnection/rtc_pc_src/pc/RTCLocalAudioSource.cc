//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : audio local source ...
// TODD... unknown
//
//////////////////////////////////////////////////////////////////////////

#include "rtc_base/ref_counted_object.h"

#include "RTCLocalAudioSource.h"

namespace webrtc {

rtc::scoped_refptr<RTCLocalAudioSource> RTCLocalAudioSource::Create(
	const cricket::AudioOptions* audio_options) {
	rtc::scoped_refptr<RTCLocalAudioSource> source(
		new rtc::RefCountedObject<RTCLocalAudioSource>());
	source->Initialize(audio_options);
	return source;
}

void RTCLocalAudioSource::Initialize(const cricket::AudioOptions* audio_options) {
	if (!audio_options)
		return;

	options_ = *audio_options;
}

}