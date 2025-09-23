//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#include "RTCVoiceActivityDetector.h"
#include "modules/audio_processing/vad/voice_activity_detector.h"

namespace webrtc {

class RTCVoiceActivityDetector::Impl {
public:
	Impl() = default;
	~Impl() = default;

public:
	void ProcessChunk(const int16_t* audio, size_t length, int sample_rate_hz,
		bool &isValid, float& lastVoiceProbability);

private:
	VoiceActivityDetector _detector;
};

void RTCVoiceActivityDetector::Impl::ProcessChunk(const int16_t* audio, size_t length, int sample_rate_hz,
	bool &isValid, float& lastVoiceProbability) {
	_detector.ProcessChunkExt(audio, length, sample_rate_hz, isValid, lastVoiceProbability);
}

std::unique_ptr<RTCVoiceActivityDetector> RTCVoiceActivityDetector::Create() {
	std::unique_ptr<RTCVoiceActivityDetector> ptr(new RTCVoiceActivityDetector);
	return std::move(ptr);
}

RTCVoiceActivityDetector::RTCVoiceActivityDetector() {
	impl_ = std::make_unique<RTCVoiceActivityDetector::Impl>();
}

RTCVoiceActivityDetector::~RTCVoiceActivityDetector() {
}

void RTCVoiceActivityDetector::ProcessChunk(const int16_t* audio, size_t length, int sample_rate_hz,
	bool &isValid, float& lastVoiceProbability) {
	impl_->ProcessChunk(audio, length, sample_rate_hz, isValid, lastVoiceProbability);
}

}