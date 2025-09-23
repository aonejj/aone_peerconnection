//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __API_AUDIO_AUDIO_OUTPUT_PROXY_H__
#define __API_AUDIO_AUDIO_OUTPUT_PROXY_H__

#include <memory>

#include "api/audio/audio_frame.h"
#include "rtc_base/ref_count.h"

namespace webrtc {

class RTCAudioOutputProxy : public rtc::RefCountInterface {
public:
	class Source {
	public:
		enum class AudioFrameInfo {
			kNormal,  // The samples in audio_frame are valid and should be used.
			kMuted,   // The samples in audio_frame should not be used, but
			// should be implicitly interpreted as zero. Other
			// fields in audio_frame may be read and should
			// contain meaningful values.
			kError,   // The audio_frame will not be used.
		};

		virtual AudioFrameInfo GetAudioFrameWithInfo(int32_t sample_rate_hz,
													 AudioFrame *audio_frame) = 0;
		virtual int32_t Ssrc() const = 0;

		virtual ~Source() {}
	};

	virtual bool RegistSource(Source *audio_source) = 0;
	virtual void UnRegistSource() = 0;

	virtual int32_t GetAudioData(size_t number_of_channels,
								 AudioFrame *audio_frame) = 0;

protected:
	~RTCAudioOutputProxy() override {}
};

}

#endif // __API_AUDIO_AUDIO_OUTPUT_PROXY_H__