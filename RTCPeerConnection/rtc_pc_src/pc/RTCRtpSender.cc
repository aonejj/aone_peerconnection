//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /pc/rtp_sender.cc
//
//////////////////////////////////////////////////////////////////////////

#include <atomic>
#include <utility>
#include <vector>
#include <memory>

#include "api/audio_options.h"
#include "rtc_base/checks.h"
#include "rtc_base/helpers.h"
#include "rtc_base/location.h"
#include "rtc_base/logging.h"
#include "modules/rtp_rtcp/source/rtp_packet_received.h"

#include "api/make_ref_counted.h"

#include "../../src_update/api/_media_stream_interface.h"
#include "../../src_update/media/base/_media_engine.h"
#include "../api/video/RTCVideoPacketRouteSourceInterface.h"
#include "../api/audio/RTCAudioPacketRouteSourceInterface.h"
#include "RTCRtpSender.h"

namespace webrtc {

// This function is only expected to be called on the signaling thread.
// On the other hand, some test or even production setups may use
// several signaling threads.
int GenerateUniqueId() {
	static std::atomic<int> g_unique_id{ 0 };

	return ++g_unique_id;
}

// Returns true if a "per-sender" encoding parameter contains a value that isn't
// its default. Currently max_bitrate_bps and bitrate_priority both are
// implemented "per-sender," meaning that these encoding parameters
// are used for the RtpSender as a whole, not for a specific encoding layer.
// This is done by setting these encoding parameters at index 0 of
// RtpParameters.encodings. This function can be used to check if these
// parameters are set at any index other than 0 of RtpParameters.encodings,
// because they are currently unimplemented to be used for a specific encoding
// layer.
bool PerSenderRtpEncodingParameterHasValue(
	const RtpEncodingParameters& encoding_params) {
	if (encoding_params.bitrate_priority != kDefaultBitratePriority ||
		encoding_params.network_priority != Priority::kLow) {
		return true;
	}
	return false;
}

void RemoveEncodingLayers(const std::vector<std::string>& rids,
	std::vector<RtpEncodingParameters>* encodings) {
	RTC_DCHECK(encodings);
	encodings->erase(
		std::remove_if(encodings->begin(), encodings->end(),
		[&rids](const RtpEncodingParameters& encoding) {
		return absl::c_linear_search(rids, encoding.rid);
	}),
		encodings->end());
}

RtpParameters RestoreEncodingLayers(
	const RtpParameters& parameters,
	const std::vector<std::string>& removed_rids,
	const std::vector<RtpEncodingParameters>& all_layers) {
	RTC_DCHECK_EQ(parameters.encodings.size() + removed_rids.size(),
		all_layers.size());
	RtpParameters result(parameters);
	result.encodings.clear();
	size_t index = 0;
	for (const RtpEncodingParameters& encoding : all_layers) {
		if (absl::c_linear_search(removed_rids, encoding.rid)) {
			result.encodings.push_back(encoding);
			continue;
		}
		result.encodings.push_back(parameters.encodings[index++]);
	}
	return result;
}



// Returns true if any RtpParameters member that isn't implemented contains a
// value.
bool UnimplementedRtpParameterHasValue(const RtpParameters& parameters) {
	if (!parameters.mid.empty()) {
		return true;
	}
	for (size_t i = 0; i < parameters.encodings.size(); ++i) {
		// Encoding parameters that are per-sender should only contain value at
		// index 0.
		if (i != 0 &&
			PerSenderRtpEncodingParameterHasValue(parameters.encodings[i])) {
			return true;
		}
	}
	return false;
}

RTCRtpSenderBase::RTCRtpSenderBase(rtc::Thread* worker_thread,
									const std::string& id,
									SetStreamsObserver* set_streams_observer)
	: worker_thread_(worker_thread),
	  id_(id),
	  set_streams_observer_(set_streams_observer) {
	RTC_DCHECK(worker_thread);
	init_parameters_.encodings.emplace_back();
}

void RTCRtpSenderBase::SetMediaChannel(cricket::MediaChannel* media_channel) {
	RTC_DCHECK(media_channel == nullptr ||
		media_channel->media_type() == media_type());
	media_channel_ = media_channel;
}

RtpParameters RTCRtpSenderBase::GetParametersInternal() const {
	if (stopped_) {
		return RtpParameters();
	}
	if (!media_channel_ || !ssrc_) {
		return init_parameters_;
	}
	return worker_thread_->Invoke<RtpParameters>(RTC_FROM_HERE, [&] {
		RtpParameters result = media_channel_->GetRtpSendParameters(ssrc_);
		RemoveEncodingLayers(disabled_rids_, &result.encodings);
		return result;
	});
}

RtpParameters RTCRtpSenderBase::GetParameters() const {
	RtpParameters result = GetParametersInternal();
	last_transaction_id_ = rtc::CreateRandomUuid();
	result.transaction_id = last_transaction_id_.value();
	return result;
}

RTCError RTCRtpSenderBase::SetParametersInternal(const RtpParameters& parameters) {
	RTC_DCHECK(!stopped_);

	if (UnimplementedRtpParameterHasValue(parameters)) {
		LOG_AND_RETURN_ERROR(
			RTCErrorType::UNSUPPORTED_PARAMETER,
			"Attempted to set an unimplemented parameter of RtpParameters.");
	}
	if (!media_channel_ || !ssrc_) {
		auto result = cricket::CheckRtpParametersInvalidModificationAndValues(
			init_parameters_, parameters);
		if (result.ok()) {
			init_parameters_ = parameters;
		}
		return result;
	}
	return worker_thread_->Invoke<RTCError>(RTC_FROM_HERE, [&] {
		RtpParameters rtp_parameters = parameters;
		if (!disabled_rids_.empty()) {
			// Need to add the inactive layers.
			RtpParameters old_parameters =
				media_channel_->GetRtpSendParameters(ssrc_);
			rtp_parameters = RestoreEncodingLayers(parameters, disabled_rids_,
				old_parameters.encodings);
		}
		return media_channel_->SetRtpSendParameters(ssrc_, rtp_parameters);
	});
}

RTCError RTCRtpSenderBase::SetParameters(const RtpParameters& parameters) {
	if (is_transceiver_stopped_) {
		LOG_AND_RETURN_ERROR(
			RTCErrorType::INVALID_STATE,
			"Cannot set parameters on sender of a stopped transceiver.");
	}
	if (stopped_) {
		LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_STATE,
			"Cannot set parameters on a stopped sender.");
	}
	if (stopped_) {
		LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_STATE,
			"Cannot set parameters on a stopped sender.");
	}
	if (!last_transaction_id_) {
		LOG_AND_RETURN_ERROR(
			RTCErrorType::INVALID_STATE,
			"Failed to set parameters since getParameters() has never been called"
			" on this sender");
	}
	if (last_transaction_id_ != parameters.transaction_id) {
		LOG_AND_RETURN_ERROR(
			RTCErrorType::INVALID_MODIFICATION,
			"Failed to set parameters since the transaction_id doesn't match"
			" the last value returned from getParameters()");
	}

	RTCError result = SetParametersInternal(parameters);
	last_transaction_id_.reset();
	return result;
}

void RTCRtpSenderBase::SetStreams(const std::vector<std::string>& stream_ids) {
	set_stream_ids(stream_ids);
	if (set_streams_observer_)
		set_streams_observer_->OnSetStreams();
}

bool RTCRtpSenderBase::SetTrack(MediaStreamTrackInterface* track) {
	if (stopped_) {
		RTC_LOG(LS_INFO) << "SetTrack can't be called on a stopped RtpSender.";
		return false;
	}
	if (track && track->kind() != track_kind()) {
		RTC_LOG(LS_ERROR) << "SetTrack with " << track->kind()
			<< " called on RtpSender with " << track_kind()
			<< " track.";
		return false;
	}

	// Detach from old track.
	if (track_) {
		DetachTrack();
		track_->UnregisterObserver(this);
		RemoveTrackFromStats();
	}

	// Attach to new track.
	bool prev_can_send_track = can_send_track();
	// Keep a reference to the old track to keep it alive until we call SetSend.
	rtc::scoped_refptr<MediaStreamTrackInterface> old_track = track_;
	track_ = track;
	if (track_) {
		track_->RegisterObserver(this);
		AttachTrack();
	}

	// Update channel.
	if (can_send_track()) {
		SetSend();
		AddTrackToStats();
	}
	else if (prev_can_send_track) {
		ClearSend();
	}
	attachment_id_ = (track_ ? GenerateUniqueId() : 0);
	return true;
}

void RTCRtpSenderBase::SetSsrc(uint32_t ssrc) {
	if (stopped_ || ssrc == ssrc_) {
		return;
	}
	// If we are already sending with a particular SSRC, stop sending.
	if (can_send_track()) {
		ClearSend();
		RemoveTrackFromStats();
	}
	ssrc_ = ssrc;
	if (can_send_track()) {
		SetSend();
		AddTrackToStats();
	}
	if (!init_parameters_.encodings.empty()) {
		worker_thread_->Invoke<void>(RTC_FROM_HERE, [&] {
			RTC_DCHECK(media_channel_);
			// Get the current parameters, which are constructed from the SDP.
			// The number of layers in the SDP is currently authoritative to support
			// SDP munging for Plan-B simulcast with "a=ssrc-group:SIM <ssrc-id>..."
			// lines as described in RFC 5576.
			// All fields should be default constructed and the SSRC field set, which
			// we need to copy.
			RtpParameters current_parameters =
				media_channel_->GetRtpSendParameters(ssrc_);
			RTC_DCHECK_GE(current_parameters.encodings.size(),
				init_parameters_.encodings.size());
			for (size_t i = 0; i < init_parameters_.encodings.size(); ++i) {
				init_parameters_.encodings[i].ssrc =
					current_parameters.encodings[i].ssrc;
				init_parameters_.encodings[i].rid = current_parameters.encodings[i].rid;
				current_parameters.encodings[i] = init_parameters_.encodings[i];
			}
			current_parameters.degradation_preference =
				init_parameters_.degradation_preference;
			media_channel_->SetRtpSendParameters(ssrc_, current_parameters);
			init_parameters_.encodings.clear();
		});
	}
}

void RTCRtpSenderBase::Stop() {
	// TODO(deadbeef): Need to do more here to fully stop sending packets.
	if (stopped_) {
		return;
	}
	if (track_) {
		DetachTrack();
		track_->UnregisterObserver(this);
	}
	if (can_send_track()) {
		ClearSend();
		RemoveTrackFromStats();
	}
	media_channel_ = nullptr;
	set_streams_observer_ = nullptr;
	stopped_ = true;
}

RTCError RTCRtpSenderBase::DisableEncodingLayers(
	const std::vector<std::string>& rids) {
	if (stopped_) {
		LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_STATE,
			"Cannot disable encodings on a stopped sender.");
	}

	if (rids.empty()) {
		return RTCError::OK();
	}

	// Check that all the specified layers exist and disable them in the channel.
	RtpParameters parameters = GetParametersInternal();
	for (const std::string& rid : rids) {
		if (absl::c_none_of(parameters.encodings,
			[&rid](const RtpEncodingParameters& encoding) {
			return encoding.rid == rid;
		})) {
			LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_PARAMETER,
				"RID: " + rid + " does not refer to a valid layer.");
		}
	}

	if (!media_channel_ || !ssrc_) {
		RemoveEncodingLayers(rids, &init_parameters_.encodings);
		// Invalidate any transaction upon success.
		last_transaction_id_.reset();
		return RTCError::OK();
	}

	for (RtpEncodingParameters& encoding : parameters.encodings) {
		// Remain active if not in the disable list.
		encoding.active &= absl::c_none_of(
			rids,
			[&encoding](const std::string& rid) { return encoding.rid == rid; });
	}

	RTCError result = SetParametersInternal(parameters);
	if (result.ok()) {
		disabled_rids_.insert(disabled_rids_.end(), rids.begin(), rids.end());
		// Invalidate any transaction upon success.
		last_transaction_id_.reset();
	}
	return result;
}


RTCLocalAudioSinkAdapter::RTCLocalAudioSinkAdapter() : sink_(nullptr) {}

RTCLocalAudioSinkAdapter::~RTCLocalAudioSinkAdapter() {
	MutexLock lock(&lock_);
	if (sink_)
		sink_->OnClose();
}

void RTCLocalAudioSinkAdapter::OnData(
	const void* audio_data,
	int bits_per_sample,
	int sample_rate,
	size_t number_of_channels,
	size_t number_of_frames,
	absl::optional<int64_t> absolute_capture_timestamp_ms) {
	MutexLock lock(&lock_);
	if (sink_) {
		sink_->OnData(audio_data, bits_per_sample, sample_rate, number_of_channels,
			number_of_frames, absolute_capture_timestamp_ms);
		num_preferred_channels_ = sink_->NumPreferredChannels();
	}
}

void RTCLocalAudioSinkAdapter::SetSink(cricket::AudioSource::Sink* sink) {
	MutexLock lock(&lock_);
	RTC_DCHECK(!sink || !sink_);
	sink_ = sink;
}

rtc::scoped_refptr<RTCAudioRtpSender> RTCAudioRtpSender::Create(
	rtc::Thread* worker_thread,
	const std::string& id,
	SetStreamsObserver* set_streams_observer) {
	return rtc::scoped_refptr<RTCAudioRtpSender>(
		rtc::make_ref_counted<RTCAudioRtpSender>(worker_thread, id,
		set_streams_observer));
}

RTCAudioRtpSender::RTCAudioRtpSender(rtc::Thread* worker_thread,
	const std::string& id,
	SetStreamsObserver* set_streams_observer)
	: RTCRtpSenderBase(worker_thread, id, set_streams_observer) {
}

RTCAudioRtpSender::~RTCAudioRtpSender() {
	Stop();
}

void RTCAudioRtpSender::OnChanged() {
	RTC_DCHECK(!stopped_);
	if (cached_track_enabled_ != track_->enabled()) {
		cached_track_enabled_ = track_->enabled();
		if (can_send_track()) {
			SetSend();
		}
	}
}

void RTCAudioRtpSender::DetachTrack() {
	RTC_DCHECK(track_);
}

void RTCAudioRtpSender::AttachTrack() {
	RTC_DCHECK(track_);
	cached_track_enabled_ = track_->enabled();
}

void RTCAudioRtpSender::SetSend() {
	RTC_DCHECK(!stopped_);
	RTC_DCHECK(can_send_track());

	if (!media_channel_) {
		RTC_LOG(LS_ERROR) << "SetAudioSend: No audio channel exists.";
		return;
	}

	RTCAudioSourceInterface* source = audio_track()->GetSource();
	rtc::RTCAudioPacketRouteSourceInterface<std::unique_ptr<webrtc::RtpPacketReceived>> *routing_source =
		reinterpret_cast<rtc::RTCAudioPacketRouteSourceInterface<std::unique_ptr<webrtc::RtpPacketReceived>>*>(source);

	bool success = worker_thread_->Invoke<bool>(RTC_FROM_HERE, [&] {
		return voice_media_channel()->SetAudioSend(ssrc_, routing_source);
	});
	if (!success) {
		RTC_LOG(LS_ERROR) << "SetAudioSend: ssrc is incorrect: " << ssrc_;
	}
}

void RTCAudioRtpSender::ClearSend() {
	RTC_DCHECK(ssrc_ != 0);
	RTC_DCHECK(!stopped_);
	if (!media_channel_) {
		RTC_LOG(LS_WARNING) << "ClearAudioSend: No audio channel exists.";
		return;
	}
	cricket::AudioOptions options;
	bool success = worker_thread_->Invoke<bool>(RTC_FROM_HERE, [&] {
		return voice_media_channel()->SetAudioSend(ssrc_, nullptr);
	});
	if (!success) {
		RTC_LOG(LS_WARNING) << "ClearAudioSend: ssrc is incorrect: " << ssrc_;
	}
}

rtc::scoped_refptr<RTCVideoRtpSender> RTCVideoRtpSender::Create(
	rtc::Thread* worker_thread,
	const std::string& id,
	SetStreamsObserver* set_streams_observer) {
	return rtc::scoped_refptr<RTCVideoRtpSender>(
		rtc::make_ref_counted<RTCVideoRtpSender>(worker_thread, id,
		set_streams_observer));
}

RTCVideoRtpSender::RTCVideoRtpSender(rtc::Thread* worker_thread,
	const std::string& id,
	SetStreamsObserver* set_streams_observer)
	: RTCRtpSenderBase(worker_thread, id, set_streams_observer) {}

RTCVideoRtpSender::~RTCVideoRtpSender() {
	RTC_LOG(LS_VERBOSE) << "~RTCVideoRtpSender";
	Stop();
}

void RTCVideoRtpSender::OnChanged() {
	RTC_DCHECK(!stopped_);
	if (can_send_track()) {
		SetSend();
	}
}

void RTCVideoRtpSender::AttachTrack() {
	RTC_DCHECK(track_);
}

void RTCVideoRtpSender::SetSend() {
	RTC_DCHECK(!stopped_);
	RTC_DCHECK(can_send_track());
	if (!media_channel_) {
		RTC_LOG(LS_ERROR) << "SetVideoSend: No video channel exists.";
		return;
	}

	VideoTrackSourceInterface* source = video_track()->GetSource();

	rtc::RTCVideoPacketRouteSourceInterface<std::unique_ptr<webrtc::RtpPacketReceived>> *routing_source =
		reinterpret_cast<rtc::RTCVideoPacketRouteSourceInterface<std::unique_ptr<webrtc::RtpPacketReceived>>*>(source);

	bool success = worker_thread_->Invoke<bool>(RTC_FROM_HERE, [&] {
		return video_media_channel()->SetVideoSend(ssrc_, routing_source);
	});
	RTC_DCHECK(success);
}

void RTCVideoRtpSender::ClearSend() {
	RTC_DCHECK(ssrc_ != 0);
	RTC_DCHECK(!stopped_);
	if (!media_channel_) {
		RTC_LOG(LS_WARNING) << "SetVideoSend: No video channel exists.";
		return;
	}
	// Allow SetVideoSend to fail since |enable| is false and |source| is null.
	// This the normal case when the underlying media channel has already been
	// deleted.
	worker_thread_->Invoke<bool>(RTC_FROM_HERE, [&] {
		return video_media_channel()->SetVideoSend(ssrc_, nullptr);
	});
}

}  // namespace webrtc
