//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : audio track ...
// TODD... unknown
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_AUDIO_TRACK_H__
#define __RTC_AUDIO_TRACK_H__

#include "api/media_stream_track.h"
#include "api/scoped_refptr.h"
#include "rtc_base/thread_checker.h"

#include "../../src_update/api/_media_stream_interface.h"

namespace webrtc {

class RTCAudioTrack : public MediaStreamTrack<AudioTrackInterface>,
	public ObserverInterface {
protected:
	// Protected ctor to force use of factory method.
	RTCAudioTrack(const std::string& label,
		const rtc::scoped_refptr<AudioSourceInterface>& source);

	RTCAudioTrack() = delete;
	RTCAudioTrack(const RTCAudioTrack&) = delete;
	RTCAudioTrack& operator=(const RTCAudioTrack&) = delete;

	~RTCAudioTrack() override;

public:
	static rtc::scoped_refptr<RTCAudioTrack> Create(
		const std::string& id,
		const rtc::scoped_refptr<AudioSourceInterface>& source);

	// MediaStreamTrack implementation.
	std::string kind() const override;

private:
	// AudioTrackInterface implementation.
	AudioSourceInterface* GetSource() const override;

	void AddSink(AudioTrackSinkInterface* sink) override;
	void RemoveSink(AudioTrackSinkInterface* sink) override;

	// ObserverInterface implementation.
	void OnChanged() override;

private:
	const rtc::scoped_refptr<AudioSourceInterface> audio_source_;
	rtc::ThreadChecker thread_checker_;
};

}


#endif //__RTC_AUDIO_TRACK_H__