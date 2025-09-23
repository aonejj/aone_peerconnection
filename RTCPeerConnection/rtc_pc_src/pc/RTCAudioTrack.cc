//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : audio track ...
// TODD... unknown
//
//////////////////////////////////////////////////////////////////////////

#include "rtc_base/checks.h"
#include "rtc_base/ref_counted_object.h"

#include "RTCAudioTrack.h"

namespace webrtc {

// static
rtc::scoped_refptr<RTCAudioTrack> RTCAudioTrack::Create(
	const std::string& id,
	const rtc::scoped_refptr<AudioSourceInterface>& source) {
	return new rtc::RefCountedObject<RTCAudioTrack>(id, source);
}

RTCAudioTrack::RTCAudioTrack(const std::string& label,
	const rtc::scoped_refptr<AudioSourceInterface>& source)
	: MediaStreamTrack<AudioTrackInterface>(label), audio_source_(source) {
	if (audio_source_) {
		audio_source_->RegisterObserver(this);
		OnChanged();
	}
}

RTCAudioTrack::~RTCAudioTrack() {
	RTC_DCHECK(thread_checker_.IsCurrent());
	set_state(MediaStreamTrackInterface::kEnded);
	if (audio_source_)
		audio_source_->UnregisterObserver(this);
}

std::string RTCAudioTrack::kind() const {
	return kAudioKind;
}

AudioSourceInterface* RTCAudioTrack::GetSource() const {
	RTC_DCHECK(thread_checker_.IsCurrent());
	return audio_source_.get();
}

void RTCAudioTrack::AddSink(AudioTrackSinkInterface* sink) {
	RTC_DCHECK(thread_checker_.IsCurrent());
	if (audio_source_)
		audio_source_->AddSink(sink);
}

void RTCAudioTrack::RemoveSink(AudioTrackSinkInterface* sink) {
	RTC_DCHECK(thread_checker_.IsCurrent());
	if (audio_source_)
		audio_source_->RemoveSink(sink);
}

void RTCAudioTrack::OnChanged() {
	RTC_DCHECK(thread_checker_.IsCurrent());
	if (audio_source_->state() == MediaSourceInterface::kEnded) {
		set_state(kEnded);
	}
	else {
		set_state(kLive);
	}
}

}
