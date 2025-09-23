//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /pc/audio_rtp_receiver.cc
//
//////////////////////////////////////////////////////////////////////////

#include <stddef.h>

#include <utility>
#include <vector>

#include "api/media_stream_proxy.h"
#include "pc/audio_track.h"
#include "pc/media_stream.h"
#include "rtc_base/checks.h"
#include "rtc_base/location.h"
#include "rtc_base/logging.h"

#include "RTCAudioRtpReceiver.h"

namespace webrtc {

RTCAudioRtpReceiver::RTCAudioRtpReceiver(rtc::Thread* worker_thread,
										std::string receiver_id,
										std::vector<std::string> stream_ids,
										rtc::RTCThreadManagerInterface* rtc_thread_manager
										)
	: RTCAudioRtpReceiver(worker_thread,
						  receiver_id,
						  CreateStreamsFromIds(std::move(stream_ids),
						  rtc_thread_manager
						  ),
						  rtc_thread_manager
						  ) {}

RTCAudioRtpReceiver::RTCAudioRtpReceiver(
	rtc::Thread* worker_thread,
	const std::string& receiver_id,
	const std::vector<rtc::scoped_refptr<MediaStreamInterface>>& streams,
	rtc::RTCThreadManagerInterface* rtc_thread_manager
	)
	: worker_thread_(worker_thread),
	id_(receiver_id),
	source_(new rtc::RefCountedObject<RemoteAudioSource>(worker_thread, rtc_thread_manager)),
	track_(AudioTrackProxyWithInternal<AudioTrack>::Create(
	rtc::Thread::Current(rtc_thread_manager),
	AudioTrack::Create(receiver_id, source_))),
	cached_track_enabled_(track_->enabled()),
	attachment_id_(GenerateUniqueId()), 
	_rtc_thread_manager(rtc_thread_manager)
{
	RTC_DCHECK(worker_thread_);
	RTC_DCHECK(track_->GetSource()->remote());
	track_->RegisterObserver(this);
	track_->GetSource()->RegisterAudioObserver(this);
	SetStreams(streams);
}

RTCAudioRtpReceiver::~RTCAudioRtpReceiver() {
	track_->GetSource()->UnregisterAudioObserver(this);
	track_->UnregisterObserver(this);
	Stop();
}

void RTCAudioRtpReceiver::OnChanged() {
	if (cached_track_enabled_ != track_->enabled()) {
		cached_track_enabled_ = track_->enabled();
		Reconfigure();
	}
}

bool RTCAudioRtpReceiver::SetOutputVolume(double volume) {
	RTC_DCHECK_GE(volume, 0.0);
	RTC_DCHECK_LE(volume, 10.0);
	RTC_DCHECK(media_channel_);
	RTC_DCHECK(!stopped_);

	return true;
}

void RTCAudioRtpReceiver::OnSetVolume(double volume) {
	RTC_DCHECK_GE(volume, 0);
	RTC_DCHECK_LE(volume, 10);
	cached_volume_ = volume;
	if (!media_channel_ || stopped_) {
		RTC_LOG(LS_ERROR)
			<< "AudioRtpReceiver::OnSetVolume: No audio channel exists.";
		return;
	}
	// When the track is disabled, the volume of the source, which is the
	// corresponding WebRtc Voice Engine channel will be 0. So we do not allow
	// setting the volume to the source when the track is disabled.
	if (!stopped_ && track_->enabled()) {
		if (!SetOutputVolume(cached_volume_)) {
			RTC_NOTREACHED();
		}
	}
}

std::vector<std::string> RTCAudioRtpReceiver::stream_ids() const {
	std::vector<std::string> stream_ids(streams_.size());
	for (size_t i = 0; i < streams_.size(); ++i)
		stream_ids[i] = streams_[i]->id();
	return stream_ids;
}

RtpParameters RTCAudioRtpReceiver::GetParameters() const {
	RTC_LOG(LS_VERBOSE) << "RTCAudioRtpReceiver::GetParameters";
#if 0
	if (!media_channel_ || stopped_) {
		return RtpParameters();
	}
	return worker_thread_->Invoke<RtpParameters>(RTC_FROM_HERE, [&] {
		return ssrc_ ? media_channel_->GetRtpReceiveParameters(*ssrc_)
			: media_channel_->GetDefaultRtpReceiveParameters();
	});
#else 
	RTC_DCHECK_RUN_ON(worker_thread_);
	if (!media_channel_ || stopped_) {
		return RtpParameters();
	}

	return ssrc_ ? media_channel_->GetRtpReceiveParameters(*ssrc_)
		: media_channel_->GetDefaultRtpReceiveParameters();
#endif
}

void RTCAudioRtpReceiver::Stop() {
	// TODO(deadbeef): Need to do more here to fully stop receiving packets.
	if (stopped_) {
		return;
	}
	if (media_channel_) {
		// Allow that SetOutputVolume fail. This is the normal case when the
		// underlying media channel has already been deleted.
		SetOutputVolume(0.0);
	}
	stopped_ = true;
}

void RTCAudioRtpReceiver::StopAndEndTrack() {
	Stop();
	track_->internal()->set_ended();
}

void RTCAudioRtpReceiver::RestartMediaChannel(absl::optional<uint32_t> ssrc) {
	RTC_DCHECK(media_channel_);
	if (!stopped_ && ssrc_ == ssrc) {
		return;
	}

	if (!stopped_) {
		source_->Stop(media_channel_, ssrc_);

	}
	ssrc_ = ssrc;
	stopped_ = false;
	source_->Start(media_channel_, ssrc);

	Reconfigure();
}

void RTCAudioRtpReceiver::SetupMediaChannel(uint32_t ssrc) {
	if (!media_channel_) {
		RTC_LOG(LS_ERROR)
			<< "AudioRtpReceiver::SetupMediaChannel: No audio channel exists.";
		return;
	}
	RestartMediaChannel(ssrc);
}

void RTCAudioRtpReceiver::SetupUnsignaledMediaChannel() {
	if (!media_channel_) {
		RTC_LOG(LS_ERROR) << "AudioRtpReceiver::SetupUnsignaledMediaChannel: No "
			"audio channel exists.";
	}
	RestartMediaChannel(absl::nullopt);
}

void RTCAudioRtpReceiver::set_stream_ids(std::vector<std::string> stream_ids) {
	SetStreams(CreateStreamsFromIds(std::move(stream_ids), _rtc_thread_manager));
}

void RTCAudioRtpReceiver::SetStreams(
	const std::vector<rtc::scoped_refptr<MediaStreamInterface>>& streams) {
	// Remove remote track from any streams that are going away.
	for (const auto& existing_stream : streams_) {
		bool removed = true;
		for (const auto& stream : streams) {
			if (existing_stream->id() == stream->id()) {
				RTC_DCHECK_EQ(existing_stream.get(), stream.get());
				removed = false;
				break;
			}
		}
		if (removed) {
			existing_stream->RemoveTrack(track_);
		}
	}
	// Add remote track to any streams that are new.
	for (const auto& stream : streams) {
		bool added = true;
		for (const auto& existing_stream : streams_) {
			if (stream->id() == existing_stream->id()) {
				RTC_DCHECK_EQ(stream.get(), existing_stream.get());
				added = false;
				break;
			}
		}
		if (added) {
			stream->AddTrack(track_);
		}
	}
	streams_ = streams;
}

std::vector<RtpSource> RTCAudioRtpReceiver::GetSources() const {
#if 0
	if (!media_channel_ || !ssrc_ || stopped_) {
		return{};
	}
	return worker_thread_->Invoke<std::vector<RtpSource>>(
		RTC_FROM_HERE, [&] { return media_channel_->GetSources(*ssrc_); });
#else 
	RTC_DCHECK_RUN_ON(worker_thread_);
	if (!media_channel_ || !ssrc_ || stopped_) {
		return{};
	}

	return media_channel_->GetSources(*ssrc_);
#endif
}

void RTCAudioRtpReceiver::Reconfigure() {
	if (!media_channel_ || stopped_) {
		RTC_LOG(LS_ERROR)
			<< "AudioRtpReceiver::Reconfigure: No audio channel exists.";
		return;
	}
	if (!SetOutputVolume(track_->enabled() ? cached_volume_ : 0)) {
		RTC_NOTREACHED();
	}
}

void RTCAudioRtpReceiver::SetObserver(RTCRtpReceiverObserverInterface* observer) {
	observer_ = observer;
	// Deliver any notifications the observer may have missed by being set late.
	if (received_first_packet_ && observer_) {
		observer_->OnFirstPacketReceived(media_type());
	}
}

void RTCAudioRtpReceiver::SetJitterBufferMinimumDelay(
	absl::optional<double> delay_seconds) {
}

void RTCAudioRtpReceiver::SetMediaChannel(cricket::MediaChannel* media_channel) {
	RTC_DCHECK(media_channel == nullptr ||
		media_channel->media_type() == media_type());
	media_channel_ = static_cast<cricket::VoiceMediaChannel*>(media_channel);
}

void RTCAudioRtpReceiver::NotifyFirstPacketReceived() {
	if (observer_) {
		observer_->OnFirstPacketReceived(media_type());
	}
	received_first_packet_ = true;
}

}  // namespace webrtc
