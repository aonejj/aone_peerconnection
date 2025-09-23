//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : Replace RtpTransceiver to RTCRtpTransceiver class for rtc media server.
//
//////////////////////////////////////////////////////////////////////////

#include <iterator>
#include <string>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"

#include "api/rtp_parameters.h"
#include "media/base/codec.h"
#include "media/base/media_constants.h"
#include "pc/rtp_media_utils.h"
#include "pc/session_description.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "rtc_base/synchronization/sequence_checker.h"
#include "rtc_base/thread.h"

#include "RTCRtpTransceiver.h"

namespace webrtc {

template <class T>
RTCError VerifyCodecPreferences(const std::vector<RtpCodecCapability>& codecs,
	const std::vector<T>& send_codecs,
	const std::vector<T>& recv_codecs) {
	// If the intersection between codecs and
	// RTCRtpSender.getCapabilities(kind).codecs or the intersection between
	// codecs and RTCRtpReceiver.getCapabilities(kind).codecs only contains RTX,
	// RED or FEC codecs or is an empty set, throw InvalidModificationError.
	// This ensures that we always have something to offer, regardless of
	// transceiver.direction.

	if (!absl::c_any_of(codecs, [&recv_codecs](const RtpCodecCapability& codec) {
		return codec.name != cricket::kRtxCodecName &&
			codec.name != cricket::kRedCodecName &&
			codec.name != cricket::kFlexfecCodecName &&
			absl::c_any_of(recv_codecs, [&codec](const T& recv_codec) {
			return recv_codec.MatchesCapability(codec);
		});
	})) {
		return RTCError(RTCErrorType::INVALID_MODIFICATION,
			"Invalid codec preferences: Missing codec from recv "
			"codec capabilities.");
	}

	if (!absl::c_any_of(codecs, [&send_codecs](const RtpCodecCapability& codec) {
		return codec.name != cricket::kRtxCodecName &&
			codec.name != cricket::kRedCodecName &&
			codec.name != cricket::kFlexfecCodecName &&
			absl::c_any_of(send_codecs, [&codec](const T& send_codec) {
			return send_codec.MatchesCapability(codec);
		});
	})) {
		return RTCError(RTCErrorType::INVALID_MODIFICATION,
			"Invalid codec preferences: Missing codec from send "
			"codec capabilities.");
	}

	// Let codecCapabilities be the union of
	// RTCRtpSender.getCapabilities(kind).codecs and
	// RTCRtpReceiver.getCapabilities(kind).codecs. For each codec in codecs, If
	// codec is not in codecCapabilities, throw InvalidModificationError.
	for (const auto& codec_preference : codecs) {
		bool is_recv_codec =
			absl::c_any_of(recv_codecs, [&codec_preference](const T& codec) {
			return codec.MatchesCapability(codec_preference);
		});

		bool is_send_codec =
			absl::c_any_of(send_codecs, [&codec_preference](const T& codec) {
			return codec.MatchesCapability(codec_preference);
		});

		if (!is_recv_codec && !is_send_codec) {
			return RTCError(
				RTCErrorType::INVALID_MODIFICATION,
				std::string("Invalid codec preferences: invalid codec with name \"") +
				codec_preference.name + "\".");
		}
	}

	// Check we have a real codec (not just rtx, red or fec)
	if (absl::c_all_of(codecs, [](const RtpCodecCapability& codec) {
		return codec.name == cricket::kRtxCodecName ||
			codec.name == cricket::kRedCodecName ||
			codec.name == cricket::kUlpfecCodecName;
	})) {
		return RTCError(RTCErrorType::INVALID_MODIFICATION,
			"Invalid codec preferences: codec list must have a non "
			"RTX, RED or FEC entry.");
	}

	return RTCError::OK();
}

static TaskQueueBase* GetCurrentTaskQueueOrThread(
	rtc::RTCThreadManagerInterface* rtc_thread_manager
	) {
	TaskQueueBase* current = TaskQueueBase::Current();
	if (!current) {
		current = rtc_thread_manager->Instance()->CurrentThread();
	}
	return current;
}


RTCRtpTransceiver::RTCRtpTransceiver(cricket::MediaType media_type,
	rtc::RTCThreadManagerInterface* rtc_thread_manager
	) 
	: thread_(GetCurrentTaskQueueOrThread(
		rtc_thread_manager
	  )),
	  unified_plan_(false),
	  media_type_(media_type) {
	RTC_DCHECK(media_type == cricket::MEDIA_TYPE_AUDIO ||
			   media_type == cricket::MEDIA_TYPE_VIDEO);
}

RTCRtpTransceiver::RTCRtpTransceiver(
	rtc::scoped_refptr<RTCRtpSenderProxyWithInternal<RTCRtpSenderInternal>> sender,
	rtc::scoped_refptr<RTCRtpReceiverProxyWithInternal<RTCRtpReceiverInternal>>
	receiver,
	cricket::RTCChannelManager* channel_manager,
	std::vector<RtpHeaderExtensionCapability> HeaderExtensionsToOffer,
	std::function<void()> on_negotiation_needed,
	rtc::RTCThreadManagerInterface* rtc_thread_manager
	)
	: thread_(GetCurrentTaskQueueOrThread(
		rtc_thread_manager
	)),
	  unified_plan_(true),
	  media_type_(sender->media_type()),
	  channel_manager_(channel_manager),
	  header_extensions_to_offer_(std::move(HeaderExtensionsToOffer)),
	  on_negotiation_needed_(std::move(on_negotiation_needed)) {
	RTC_DCHECK(media_type_ == cricket::MEDIA_TYPE_AUDIO ||
		media_type_ == cricket::MEDIA_TYPE_VIDEO);
	RTC_DCHECK_EQ(sender->media_type(), receiver->media_type());
	senders_.push_back(sender);
	receivers_.push_back(receiver);
}

RTCRtpTransceiver::~RTCRtpTransceiver() {
	StopInternal();
}

void RTCRtpTransceiver::SetChannel(cricket::ChannelInterface* channel) {
	if (stopped_ && channel) {
		return;
	}

	if (channel) {
		RTC_DCHECK_EQ(media_type(), channel->media_type());
	}

	if (channel_) {
		channel_->SignalFirstPacketReceived().disconnect(this);
	}

	channel_ = channel;

	if (channel_) {
		channel_->SignalFirstPacketReceived().connect(
			this, &RTCRtpTransceiver::OnFirstPacketReceived);
	}

	for (const auto& sender : senders_) {
		sender->internal()->SetMediaChannel(channel_ ? channel_->media_channel()
			: nullptr);
	}

	for (const auto& receiver : receivers_) {
		if (!channel_) {
			receiver->internal()->Stop();
		}

		receiver->internal()->SetMediaChannel(channel_ ? channel_->media_channel()
			: nullptr);
	}
}

void RTCRtpTransceiver::AddSender(
	rtc::scoped_refptr<RTCRtpSenderProxyWithInternal<RTCRtpSenderInternal>> sender) {
	RTC_DCHECK(!stopped_);
	RTC_DCHECK(!unified_plan_);
	RTC_DCHECK(sender);
	RTC_DCHECK_EQ(media_type(), sender->media_type());
	RTC_DCHECK(!absl::c_linear_search(senders_, sender));
	senders_.push_back(sender);
}

bool RTCRtpTransceiver::RemoveSender(RTCRtpSenderInterface* sender) {
	RTC_DCHECK(!unified_plan_);
	if (sender) {
		RTC_DCHECK_EQ(media_type(), sender->media_type());
	}
	auto it = absl::c_find(senders_, sender);
	if (it == senders_.end()) {
		return false;
	}
	(*it)->internal()->Stop();
	senders_.erase(it);
	return true;
}

void RTCRtpTransceiver::AddReceiver(
	rtc::scoped_refptr<RTCRtpReceiverProxyWithInternal<RTCRtpReceiverInternal>>
	receiver) {
	RTC_DCHECK(!stopped_);
//	RTC_DCHECK(!unified_plan_);
	RTC_DCHECK(receiver);
	RTC_DCHECK_EQ(media_type(), receiver->media_type());
	RTC_DCHECK(!absl::c_linear_search(receivers_, receiver));
	receivers_.push_back(receiver);
}

bool RTCRtpTransceiver::RemoveReceiver(RTCRtpReceiverInterface* receiver) {
	RTC_DCHECK(!unified_plan_);
	if (receiver) {
		RTC_DCHECK_EQ(media_type(), receiver->media_type());
	}
	auto it = absl::c_find(receivers_, receiver);
	if (it == receivers_.end()) {
		return false;
	}
	(*it)->internal()->Stop();
	// After the receiver has been removed, there's no guarantee that the
	// contained media channel isn't deleted shortly after this. To make sure that
	// the receiver doesn't spontaneously try to use it's (potentially stale)
	// media channel reference, we clear it out.
	(*it)->internal()->SetMediaChannel(nullptr);
	receivers_.erase(it);
	return true;
}


rtc::scoped_refptr<RTCRtpSenderInternal> RTCRtpTransceiver::sender_internal() const {
	RTC_DCHECK(unified_plan_);
	RTC_CHECK_EQ(1u, senders_.size());
	return senders_[0]->internal();
}

rtc::scoped_refptr<RTCRtpReceiverInternal> RTCRtpTransceiver::receiver_internal()
const {
	RTC_DCHECK(unified_plan_);
//	RTC_CHECK_EQ(1u, receivers_.size());	
	return receivers_[0]->internal();
}


cricket::MediaType RTCRtpTransceiver::media_type() const {
	return media_type_;
}

absl::optional<std::string> RTCRtpTransceiver::mid() const {
	return mid_;
}

void RTCRtpTransceiver::OnFirstPacketReceived(cricket::ChannelInterface* channel) {
	for (const auto& receiver : receivers_) {
		receiver->internal()->NotifyFirstPacketReceived();
	}
}

rtc::scoped_refptr<RTCRtpSenderInterface> RTCRtpTransceiver::sender() const {
	RTC_DCHECK(unified_plan_);
	RTC_CHECK_EQ(1u, senders_.size());
	return senders_[0];
}

rtc::scoped_refptr<RTCRtpReceiverInterface> RTCRtpTransceiver::receiver() const {
	RTC_DCHECK(unified_plan_);
//	RTC_CHECK_EQ(1u, receivers_.size());	
	return receivers_[0];
}

uint32_t RTCRtpTransceiver::receiver_size() const {
	RTC_DCHECK(unified_plan_);
	return receivers_.size();
}

rtc::scoped_refptr<RTCRtpReceiverInterface> RTCRtpTransceiver::receiver(uint32_t idx) const {
	RTC_DCHECK(unified_plan_);
	return receivers_[idx];
}

void RTCRtpTransceiver::set_current_direction(RtpTransceiverDirection direction) {
	RTC_LOG(LS_INFO) << "Changing transceiver (MID=" << mid_.value_or("<not set>")
		<< ") current direction from "
		<< (current_direction_ ? RtpTransceiverDirectionToString(
		*current_direction_)
		: "<not set>")
		<< " to " << RtpTransceiverDirectionToString(direction)
		<< ".";
	current_direction_ = direction;
	if (RtpTransceiverDirectionHasSend(*current_direction_)) {
		has_ever_been_used_to_send_ = true;
	}
}

void RTCRtpTransceiver::set_fired_direction(RtpTransceiverDirection direction) {
	fired_direction_ = direction;
}

bool RTCRtpTransceiver::stopped() const {
	return stopped_;
}

bool RTCRtpTransceiver::stopping() const {
	RTC_DCHECK_RUN_ON(thread_);
	return stopping_;
}

RtpTransceiverDirection RTCRtpTransceiver::direction() const {
	if (unified_plan_ && stopping())
		return webrtc::RtpTransceiverDirection::kStopped;

	return direction_;
}

RTCError RTCRtpTransceiver::SetDirectionWithError(
	RtpTransceiverDirection new_direction) {
	if (unified_plan_ && stopping()) {
		LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_STATE,
			"Cannot set direction on a stopping transceiver.");
	}
	if (new_direction == direction_)
		return RTCError::OK();

	if (new_direction == RtpTransceiverDirection::kStopped) {
		LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_PARAMETER,
			"The set direction 'stopped' is invalid.");
	}

	direction_ = new_direction;
	on_negotiation_needed_();

	return RTCError::OK();
}

absl::optional<RtpTransceiverDirection> RTCRtpTransceiver::current_direction()
const {
	if (unified_plan_ && stopped())
		return webrtc::RtpTransceiverDirection::kStopped;

	return current_direction_;
}

absl::optional<RtpTransceiverDirection> RTCRtpTransceiver::fired_direction() const {
	return fired_direction_;
}

void RTCRtpTransceiver::StopSendingAndReceiving() {
	RTC_DCHECK_RUN_ON(thread_);

	for (const auto& sender : senders_)
		sender->internal()->Stop();

	for (const auto& receiver : receivers_)
		receiver->internal()->StopAndEndTrack();

	stopping_ = true;
	direction_ = webrtc::RtpTransceiverDirection::kInactive;
}

RTCError RTCRtpTransceiver::StopStandard() {
	RTC_DCHECK_RUN_ON(thread_);

	if (!unified_plan_) {
		StopInternal();
		return RTCError::OK();
	}

	if (is_pc_closed_) {
		LOG_AND_RETURN_ERROR(RTCErrorType::INVALID_STATE,
			"PeerConnection is closed.");
	}

	if (stopping_)
		return RTCError::OK();

	StopSendingAndReceiving();

	on_negotiation_needed_();

	return RTCError::OK();
}

void RTCRtpTransceiver::StopInternal() {
	StopTransceiverProcedure();
}

void RTCRtpTransceiver::StopTransceiverProcedure() {
	RTC_DCHECK_RUN_ON(thread_);
	if (!stopping_) {
		StopSendingAndReceiving();
	}
	
	stopped_ = true;
	
	for (const auto& sender : senders_) {
		sender->internal()->SetTransceiverAsStopped();
	}

	current_direction_ = absl::nullopt;
}

RTCError RTCRtpTransceiver::SetCodecPreferences(
	rtc::ArrayView<RtpCodecCapability> codec_capabilities) {
	RTC_DCHECK(unified_plan_);

	// 3. If codecs is an empty list, set transceiver's [[PreferredCodecs]] slot
	// to codecs and abort these steps.
	if (codec_capabilities.empty()) {
		codec_preferences_.clear();
		return RTCError::OK();
	}

	// 4. Remove any duplicate values in codecs.
	std::vector<RtpCodecCapability> codecs;
	absl::c_remove_copy_if(codec_capabilities, std::back_inserter(codecs),
		[&codecs](const RtpCodecCapability& codec) {
		return absl::c_linear_search(codecs, codec);
	});

	// 6. to 8.
	RTCError result;
	if (media_type_ == cricket::MEDIA_TYPE_AUDIO) {
		std::vector<cricket::AudioCodec> recv_codecs, send_codecs;
		channel_manager_->GetSupportedAudioReceiveCodecs(&recv_codecs);
		channel_manager_->GetSupportedAudioSendCodecs(&send_codecs);

		result = VerifyCodecPreferences(codecs, send_codecs, recv_codecs);
	}
	else if (media_type_ == cricket::MEDIA_TYPE_VIDEO) {
		std::vector<cricket::VideoCodec> recv_codecs, send_codecs;
		channel_manager_->GetSupportedVideoReceiveCodecs(&recv_codecs);
		channel_manager_->GetSupportedVideoSendCodecs(&send_codecs);

		result = VerifyCodecPreferences(codecs, send_codecs, recv_codecs);
	}

	if (result.ok()) {
		codec_preferences_ = codecs;
	}

	return result;
}


std::vector<RtpHeaderExtensionCapability>
RTCRtpTransceiver::HeaderExtensionsToOffer() const {
	return header_extensions_to_offer_;
}

std::vector<RtpHeaderExtensionCapability>
RTCRtpTransceiver::HeaderExtensionsNegotiated() const {
	if (!channel_)
		return{};
	std::vector<RtpHeaderExtensionCapability> result;
	for (const auto& ext : channel_->GetNegotiatedRtpHeaderExtensions()) {
		result.emplace_back(ext.uri, ext.id, RtpTransceiverDirection::kSendRecv);
	}
	return result;
}

RTCError RTCRtpTransceiver::SetOfferedRtpHeaderExtensions(
	rtc::ArrayView<const RtpHeaderExtensionCapability>
	header_extensions_to_offer) {
	for (const auto& entry : header_extensions_to_offer) {
		// Handle unsupported requests for mandatory extensions as per
		// https://w3c.github.io/webrtc-extensions/#rtcrtptransceiver-interface.
		// Note:
		// - We do not handle setOfferedRtpHeaderExtensions algorithm step 2.1,
		//   this has to be checked on a higher level. We naturally error out
		//   in the handling of Step 2.2 if an unset URI is encountered.

		// Step 2.2.
		// Handle unknown extensions.
		auto it = std::find_if(
			header_extensions_to_offer_.begin(), header_extensions_to_offer_.end(),
			[&entry](const auto& offered) { return entry.uri == offered.uri; });
		if (it == header_extensions_to_offer_.end()) {
			return RTCError(RTCErrorType::UNSUPPORTED_PARAMETER,
				"Attempted to modify an unoffered extension.");
		}

		// Step 2.4-2.5.
		// - Use of the transceiver interface indicates unified plan is in effect,
		//   hence the MID extension needs to be enabled.
		// - Also handle the mandatory video orientation extensions.
		if ((entry.uri == RtpExtension::kMidUri ||
			entry.uri == RtpExtension::kVideoRotationUri) &&
			entry.direction != RtpTransceiverDirection::kSendRecv) {
			return RTCError(RTCErrorType::INVALID_MODIFICATION,
				"Attempted to stop a mandatory extension.");
		}
	}

	// Apply mutation after error checking.
	for (const auto& entry : header_extensions_to_offer) {
		auto it = std::find_if(
			header_extensions_to_offer_.begin(), header_extensions_to_offer_.end(),
			[&entry](const auto& offered) { return entry.uri == offered.uri; });
		it->direction = entry.direction;
	}

	return RTCError::OK();
}

void RTCRtpTransceiver::SetRTCConnectionClosed() {
	is_pc_closed_ = true;
}

}
