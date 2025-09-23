//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /api/media_stream_interface.h
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_UPDATE_API_MEDIA_STREAM_INTERFACE_H__
#define __RTC_UPDATE_API_MEDIA_STREAM_INTERFACE_H__

#include <stddef.h>

#include <string>
#include <vector>

#include "absl/types/optional.h"

#include "api/audio_options.h"
#include "api/scoped_refptr.h"
#include "rtc_base/ref_count.h"
#include "rtc_base/system/rtc_export.h"
#include "rtc_base/checks.h"


namespace webrtc {

// Generic observer interface.
class ObserverInterface {
public:
	virtual void OnChanged() = 0;

protected:
	virtual ~ObserverInterface() {}
};

class NotifierInterface {
public:
	virtual void RegisterObserver(ObserverInterface* observer) = 0;
	virtual void UnregisterObserver(ObserverInterface* observer) = 0;

	virtual ~NotifierInterface() {}
};


class RTC_EXPORT MediaSourceInterface : public rtc::RefCountInterface,
										public NotifierInterface {
public:
	enum SourceState { kInitializing, kLive, kEnded, kMuted };

	virtual SourceState state() const = 0;

	virtual bool remote() const = 0;

protected:
	~MediaSourceInterface() override = default;
};

class RTC_EXPORT MediaStreamTrackInterface : public rtc::RefCountInterface,
											 public NotifierInterface {
public:
	enum TrackState {
		kLive,
		kEnded,
	};

	static const char* const kAudioKind;
	static const char* const kVideoKind;

	// The kind() method must return kAudioKind only if the object is a
	// subclass of AudioTrackInterface, and kVideoKind only if the
	// object is a subclass of VideoTrackInterface. It is typically used
	// to protect a static_cast<> to the corresponding subclass.
	virtual std::string kind() const = 0;

	// Track identifier.
	virtual std::string id() const = 0;

	// A disabled track will produce silence (if audio) or black frames (if
	// video). Can be disabled and re-enabled.
	virtual bool enabled() const = 0;
	virtual bool set_enabled(bool enable) = 0;

	// Live or ended. A track will never be live again after becoming ended.
	virtual TrackState state() const = 0;

protected:
	~MediaStreamTrackInterface() override = default;
};

class VideoTrackSourceInterface : public MediaSourceInterface {
public:
	struct Stats {
		// Original size of captured frame, before video adaptation.
		int input_width;
		int input_height;
	};

	virtual bool GetStats(Stats* stats) = 0;

	virtual bool SupportsEncodedOutput() const = 0;

	virtual void GenerateKeyFrame() = 0;

protected:
	~VideoTrackSourceInterface() override = default;
};

class RTC_EXPORT VideoTrackInterface : public MediaStreamTrackInterface {
public:
	// Register a video sink for this track. Used to connect the track to the
	// underlying video engine.
	virtual VideoTrackSourceInterface* GetSource() const = 0;

protected:
	~VideoTrackInterface() override = default;
};

class AudioTrackSinkInterface {
public:
	virtual void OnData(const void* audio_data,
						int bits_per_sample,
						int sample_rate,
						size_t number_of_channels,
						size_t number_of_frames) {
		RTC_NOTREACHED() << "This method must be overridden, or not used.";
	}

	// In this method, |absolute_capture_timestamp_ms|, when available, is
	// supposed to deliver the timestamp when this audio frame was originally
	// captured. This timestamp MUST be based on the same clock as
	// rtc::TimeMillis().
	virtual void OnData(const void* audio_data,
						int bits_per_sample,
						int sample_rate,
						size_t number_of_channels,
						size_t number_of_frames,
						absl::optional<int64_t> absolute_capture_timestamp_ms) {
		// TODO(bugs.webrtc.org/10739): Deprecate the old OnData and make this one
		// pure virtual.
		return OnData(audio_data, bits_per_sample, sample_rate, number_of_channels,
			number_of_frames);
	}

	// Returns the number of channels encoded by the sink. This can be less than
	// the number_of_channels if down-mixing occur. A value of -1 means an unknown
	// number.
	virtual int NumPreferredChannels() const { return -1; }

protected:
	virtual ~AudioTrackSinkInterface() {}
};

class RTC_EXPORT AudioSourceInterface : public MediaSourceInterface {
public:
	class AudioObserver {
	public:
		virtual void OnSetVolume(double volume) = 0;

	protected:
		virtual ~AudioObserver() {}
	};

	// TODO(deadbeef): Makes all the interfaces pure virtual after they're
	// implemented in chromium.

	// Sets the volume of the source. |volume| is in  the range of [0, 10].
	// TODO(tommi): This method should be on the track and ideally volume should
	// be applied in the track in a way that does not affect clones of the track.
	virtual void SetVolume(double volume) {}

	// Registers/unregisters observers to the audio source.
	virtual void RegisterAudioObserver(AudioObserver* observer) {}
	virtual void UnregisterAudioObserver(AudioObserver* observer) {}

	// TODO(tommi): Make pure virtual.
	virtual void AddSink(AudioTrackSinkInterface* sink) {}		
	virtual void RemoveSink(AudioTrackSinkInterface* sink) {}	

	// Returns options for the AudioSource.
	// (for some of the settings this approach is broken, e.g. setting
	// audio network adaptation on the source is the wrong layer of abstraction).
	virtual const cricket::AudioOptions options() const;
};


class RTC_EXPORT AudioTrackInterface : public MediaStreamTrackInterface {
public:
	// TODO(deadbeef): Figure out if the following interface should be const or
	// not.
	virtual AudioSourceInterface* GetSource() const = 0;

	// Add/Remove a sink that will receive the audio data from the track.
	virtual void AddSink(AudioTrackSinkInterface* sink) = 0;
	virtual void RemoveSink(AudioTrackSinkInterface* sink) = 0;

	// Get the signal level from the audio track.
	// Return true on success, otherwise false.
	// TODO(deadbeef): Change the interface to int GetSignalLevel() and pure
	// virtual after it's implemented in chromium.
	virtual bool GetSignalLevel(int* level);

protected:
	~AudioTrackInterface() override = default;
};

typedef std::vector<rtc::scoped_refptr<AudioTrackInterface> > AudioTrackVector;
typedef std::vector<rtc::scoped_refptr<VideoTrackInterface> > VideoTrackVector;

class MediaStreamInterface : public rtc::RefCountInterface,
							public NotifierInterface {
public:
	virtual std::string id() const = 0;

	virtual AudioTrackVector GetAudioTracks() = 0;
	virtual VideoTrackVector GetVideoTracks() = 0;
	virtual rtc::scoped_refptr<AudioTrackInterface> FindAudioTrack(
		const std::string& track_id) = 0;
	virtual rtc::scoped_refptr<VideoTrackInterface> FindVideoTrack(
		const std::string& track_id) = 0;

	virtual bool AddTrack(AudioTrackInterface* track) = 0;
	virtual bool AddTrack(VideoTrackInterface* track) = 0;
	virtual bool RemoveTrack(AudioTrackInterface* track) = 0;
	virtual bool RemoveTrack(VideoTrackInterface* track) = 0;

protected:
	~MediaStreamInterface() override = default;
};

}	// namespace webrtc

#endif	// __RTC_UPDATE_API_MEDIA_STREAM_INTERFACE_H__