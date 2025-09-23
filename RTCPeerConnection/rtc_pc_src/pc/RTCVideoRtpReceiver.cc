//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /pc/video_rtp_receiver.cc
//
//////////////////////////////////////////////////////////////////////////

#include <stddef.h>

#include <utility>
#include <vector>

#include "api/media_stream_proxy.h"
#include "pc/media_stream.h"
#include "rtc_base/checks.h"
#include "rtc_base/location.h"
#include "rtc_base/logging.h"

#include "../api/RTCVideoTrackSourceProxy.h"
#include "RTCVideoRtpReceiver.h"


namespace webrtc {

RTCVideoRtpReceiver::RTCVideoRtpReceiver(rtc::Thread* worker_thread,
	std::string receiver_id,
	std::vector<std::string> stream_ids,
	rtc::RTCThreadManagerInterface* rtc_thread_manager
	)
	: RTCVideoRtpReceiver(worker_thread,
						  receiver_id,
						  CreateStreamsFromIds(std::move(stream_ids),
						  rtc_thread_manager ) , rtc_thread_manager) {
}

RTCVideoRtpReceiver::RTCVideoRtpReceiver(
	rtc::Thread* worker_thread,
	const std::string& receiver_id,
	const std::vector<rtc::scoped_refptr<MediaStreamInterface>>& streams,
	 rtc::RTCThreadManagerInterface* rtc_thread_manager
	)
	: worker_thread_(worker_thread),
	  id_(receiver_id),
	  source_(rtc::make_ref_counted<RTCVideoRtpTrackSource>(this)),
	  track_(VideoTrackProxyWithInternal<RTCVideoTrack>::Create(
				rtc::Thread::Current(rtc_thread_manager),
				worker_thread,
				RTCVideoTrack::Create(
				receiver_id,
				VideoTrackSourceProxy::Create(rtc::Thread::Current(rtc_thread_manager),
				worker_thread,
				source_),
				worker_thread))),
				attachment_id_(GenerateUniqueId()),
				_rtc_thread_manager(rtc_thread_manager)
{
	RTC_DCHECK(worker_thread_);
	SetStreams(streams);
	source_->SetState(MediaSourceInterface::kLive);
}

RTCVideoRtpReceiver::~RTCVideoRtpReceiver() {
	// Since cricket::VideoRenderer is not reference counted,
	// we need to remove it from the channel before we are deleted.
	Stop();
	// Make sure we can't be called by the |source_| anymore.
	worker_thread_->Invoke<void>(RTC_FROM_HERE,
		[this] { source_->ClearCallback(); });
}

std::vector<std::string> RTCVideoRtpReceiver::stream_ids() const {
	std::vector<std::string> stream_ids(streams_.size());
	for (size_t i = 0; i < streams_.size(); ++i)
		stream_ids[i] = streams_[i]->id();
	return stream_ids;
}

RtpParameters RTCVideoRtpReceiver::GetParameters() const {
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

void RTCVideoRtpReceiver::Stop() {
	// TODO(deadbeef): Need to do more here to fully stop receiving packets.
	if (stopped_) {
		return;
	}
	source_->SetState(MediaSourceInterface::kEnded);
	if (!media_channel_) {
		RTC_LOG(LS_WARNING) << "VideoRtpReceiver::Stop: No video channel exists.";
	}

	stopped_ = true;
}

void RTCVideoRtpReceiver::StopAndEndTrack() {
	Stop();
	track_->internal()->set_ended();
}

void RTCVideoRtpReceiver::RestartMediaChannel(absl::optional<uint32_t> ssrc) {
	RTC_DCHECK(media_channel_);
	if (!stopped_ && ssrc_ == ssrc) {
		return;
	}

	ssrc_ = ssrc;	
	stopped_ = false;
}

void RTCVideoRtpReceiver::SetupMediaChannel(uint32_t ssrc) {
	if (!media_channel_) {
		RTC_LOG(LS_ERROR)
			<< "VideoRtpReceiver::SetupMediaChannel: No video channel exists.";
	}
	RestartMediaChannel(ssrc);
}

void RTCVideoRtpReceiver::SetupUnsignaledMediaChannel() {
	if (!media_channel_) {
		RTC_LOG(LS_ERROR) << "RTCVideoRtpReceiver::SetupUnsignaledMediaChannel: No "
			"video channel exists.";
	}
	RestartMediaChannel(absl::nullopt);
}

void RTCVideoRtpReceiver::set_stream_ids(std::vector<std::string> stream_ids) {
	SetStreams(CreateStreamsFromIds(std::move(stream_ids), _rtc_thread_manager));
}

void RTCVideoRtpReceiver::SetStreams(
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

void RTCVideoRtpReceiver::SetObserver(RTCRtpReceiverObserverInterface* observer) {
	observer_ = observer;
	// Deliver any notifications the observer may have missed by being set late.
	if (received_first_packet_ && observer_) {
		observer_->OnFirstPacketReceived(media_type());
	}
}

void RTCVideoRtpReceiver::SetJitterBufferMinimumDelay(
	absl::optional<double> delay_seconds) {

}

void RTCVideoRtpReceiver::SetMediaChannel(cricket::MediaChannel* media_channel) {
	RTC_DCHECK(media_channel == nullptr ||
		media_channel->media_type() == media_type());
	worker_thread_->Invoke<void>(RTC_FROM_HERE, [&] {
		RTC_DCHECK_RUN_ON(worker_thread_);
		media_channel_ = static_cast<cricket::VideoMediaChannel*>(media_channel);

		if (media_channel_) {
			if (saved_generate_keyframe_) {
				// TODO(bugs.webrtc.org/8694): Stop using 0 to mean unsignalled SSRC
				media_channel_->GenerateKeyFrame(ssrc_.value_or(0));
				saved_generate_keyframe_ = false;
			}
		}
	});
}

void RTCVideoRtpReceiver::NotifyFirstPacketReceived() {
	if (observer_) {
		observer_->OnFirstPacketReceived(media_type());
	}
	received_first_packet_ = true;
}

std::vector<RtpSource> RTCVideoRtpReceiver::GetSources() const {
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

void RTCVideoRtpReceiver::OnGenerateKeyFrame() {
	RTC_DCHECK_RUN_ON(worker_thread_);
	if (!media_channel_) {
		RTC_LOG(LS_ERROR)
			<< "VideoRtpReceiver::OnGenerateKeyFrame: No video channel exists.";
		return;
	}
	// TODO(bugs.webrtc.org/8694): Stop using 0 to mean unsignalled SSRC
	media_channel_->GenerateKeyFrame(ssrc_.value_or(0));
	// We need to remember to request generation of a new key frame if the media
	// channel changes, because there's no feedback whether the keyframe
	// generation has completed on the channel.
	saved_generate_keyframe_ = true;
}

}  // namespace webrtc
