//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_VOICE_ACTIVITY_DETECTOR_H__
#define __RTC_VOICE_ACTIVITY_DETECTOR_H__

#include <memory>

namespace webrtc {

class RTCVoiceActivityDetector {
public:
	static std::unique_ptr<RTCVoiceActivityDetector> Create();
public:
	~RTCVoiceActivityDetector();

public:
	void ProcessChunk(const int16_t* audio, size_t length, int sample_rate_hz,
					  bool &isValid, float& lastVoiceProbability);

private:
	class Impl;
	std::unique_ptr<Impl> impl_;

private:
	RTCVoiceActivityDetector();

private:
	RTCVoiceActivityDetector(const RTCVoiceActivityDetector &) = delete;
	RTCVoiceActivityDetector &operator=(const RTCVoiceActivityDetector &) = delete;
};

}

#endif // __RTC_VOICE_ACTIVITY_DETECTOR_H__