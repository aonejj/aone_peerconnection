//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : audio local source ...
// TODD... unknown
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_LOCAL_AUDIO_SOURCE_H__
#define __RTC_LOCAL_AUDIO_SOURCE_H__

#include "api/audio_options.h"
#include "api/notifier.h"
#include "api/scoped_refptr.h"

#include "../../src_update/api/_media_stream_interface.h"

namespace webrtc {

class RTCLocalAudioSource : public Notifier<AudioSourceInterface> {
public:
	static rtc::scoped_refptr<RTCLocalAudioSource> Create(
		const cricket::AudioOptions* audio_options);

	SourceState state() const override { return kLive; }
	bool remote() const override { return false; }

	const cricket::AudioOptions options() const override { return options_; }

	void AddSink(AudioTrackSinkInterface* sink) override {}
	void RemoveSink(AudioTrackSinkInterface* sink) override {}

protected:
	RTCLocalAudioSource() {}
	~RTCLocalAudioSource() override {}

private:
	void Initialize(const cricket::AudioOptions *audio_options);

private:
	cricket::AudioOptions options_;
};

}


#endif // __RTC_LOCAL_AUDIO_SOURCE_H__