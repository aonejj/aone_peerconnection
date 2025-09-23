//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /module/rtp_rtcp/source/rtp_sender.cc
//
//////////////////////////////////////////////////////////////////////////

#include "modules/rtp_rtcp/source/rtp_header_extensions.h"
#include "modules/rtp_rtcp/source/rtp_packet_to_send.h"
#include "modules/rtp_rtcp/source/byte_io.h"
#include "rtc_base/numerics/safe_minmax.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"

#include "RTCRtpSender.h"

namespace webrtc {
namespace {
constexpr size_t kMinAudioPaddingLength = 50;
constexpr size_t kRtpHeaderLength = 12;
constexpr int kMinPayloadPaddingBytes = 50;
constexpr double kMaxPaddingSizeFactor = 3.0;

bool HasBweExtension(const RtpHeaderExtensionMap& extensions_map) {
	return extensions_map.IsRegistered(kRtpExtensionTransportSequenceNumber) ||
		extensions_map.IsRegistered(kRtpExtensionTransportSequenceNumber02) ||
		extensions_map.IsRegistered(kRtpExtensionAbsoluteSendTime) ||
		extensions_map.IsRegistered(kRtpExtensionTransmissionTimeOffset);
}
}

RTCRTPSender::RTCRTPSender(const RTCRtpRtcpInterface::Configuration& config,
		RtpPacketHistory* packet_history,
		RtpPacketSender* packet_sender)
	: clock_(config.clock),  
	  random_(clock_->TimeInMicroseconds()), 
	  audio_configured_(config.audio), 
	  ssrc_(config.local_media_ssrc), 
	  rtx_ssrc_(config.rtx_send_ssrc), 
	  packet_history_(packet_history), 
	  paced_sender_(packet_sender), 
	  max_packet_size_(IP_PACKET_SIZE - 28),
	  rtp_header_extension_map_(true), 
	  supports_bwe_extension_(false) {
	RTC_DCHECK(paced_sender_);
	RTC_DCHECK(packet_history_);
	
	if (!audio_configured_) {
		rtx_ = kRtxRetransmitted | kRtxRedundantPayloads;
	}
	else {
		rtx_ = kRtxOff;
	}
}

RTCRTPSender::~RTCRTPSender() {

}

void RTCRTPSender::SetRTPHeaderExtensionMap(const std::vector<RtpExtension>& extensions) {
	for (auto& it : extensions) {
		rtp_header_extension_map_.RegisterByUri(it.id, it.uri);
	}
	supports_bwe_extension_ = HasBweExtension(rtp_header_extension_map_);
}

void RTCRTPSender::SetSendingMediaStatus(bool enabled) {
	MutexLock lock(&send_mutex_);
	sending_media_ = enabled;
}

bool RTCRTPSender::SendingMedia() const {
	MutexLock lock(&send_mutex_);
	return sending_media_;
}

bool RTCRTPSender::IsAudioConfigured() const {
	return audio_configured_;
}

bool RTCRTPSender::SupportsPadding() const {
	MutexLock lock(&send_mutex_);
	return sending_media_ && supports_bwe_extension_;
}

bool RTCRTPSender::SupportsRtxPayloadPadding() const {
	MutexLock lock(&send_mutex_);
	return sending_media_ && supports_bwe_extension_ &&
		(rtx_ & kRtxRedundantPayloads);
}

std::vector<std::unique_ptr<RtpPacketToSend>> RTCRTPSender::GeneratePadding(
	size_t target_size_bytes,
	bool media_has_been_sent,
	bool can_send_padding_on_media_ssrc) {
	std::vector<std::unique_ptr<RtpPacketToSend>> padding_packets;
	size_t bytes_left = target_size_bytes;

	if (SupportsRtxPayloadPadding()) {
		while (bytes_left >= kMinPayloadPaddingBytes) {
			std::unique_ptr<RtpPacketToSend> packet =
				packet_history_->GetPayloadPaddingPacket(
				[&](const RtpPacketToSend& packet)
				-> std::unique_ptr<RtpPacketToSend> {
				// Limit overshoot, generate <= `kMaxPaddingSizeFactor` *
				// `target_size_bytes`.
				const size_t max_overshoot_bytes = static_cast<size_t>(
					((kMaxPaddingSizeFactor - 1.0) * target_size_bytes) + 0.5);
				if (packet.payload_size() + kRtxHeaderSize >
					max_overshoot_bytes + bytes_left) {
					return nullptr;
				}
				return BuildRtxPacket(packet);
			});
			if (!packet) {
				break;
			}

			bytes_left -= std::min(bytes_left, packet->payload_size());
			packet->set_packet_type(RtpPacketMediaType::kPadding);
			padding_packets.push_back(std::move(packet));
		}
	}

	MutexLock lock(&send_mutex_);
	if (!sending_media_) {
		return{};
	}

	size_t padding_bytes_in_packet;
	if (audio_configured_) {
		padding_bytes_in_packet = rtc::SafeClamp<size_t>(
			bytes_left, kMinAudioPaddingLength,
			rtc::SafeMin(max_packet_size_, kMaxPaddingLength));
	}
	else {
		padding_bytes_in_packet = rtc::SafeMin(max_packet_size_, kMaxPaddingLength);
	}

	while (bytes_left > 0) {
		auto padding_packet =
			std::make_unique<RtpPacketToSend>(&rtp_header_extension_map_);
		padding_packet->set_packet_type(RtpPacketMediaType::kPadding);
		padding_packet->SetMarker(false);
		if (rtx_ == kRtxOff) {
			if (!can_send_padding_on_media_ssrc) {
				break;
			}
			padding_packet->SetSsrc(ssrc_);
		}
		else {
			if (!media_has_been_sent &&
				!(rtp_header_extension_map_.IsRegistered(AbsoluteSendTime::kId) ||
				rtp_header_extension_map_.IsRegistered(
				TransportSequenceNumber::kId))) {
				break;
			}

			RTC_DCHECK(rtx_ssrc_);
			RTC_DCHECK(!rtx_payload_type_map_.empty());
			padding_packet->SetSsrc(*rtx_ssrc_);
			padding_packet->SetPayloadType(rtx_payload_type_map_.begin()->second);
		}

		if (rtp_header_extension_map_.IsRegistered(TransportSequenceNumber::kId)) {
			padding_packet->ReserveExtension<TransportSequenceNumber>();
		}
		if (rtp_header_extension_map_.IsRegistered(TransmissionOffset::kId)) {
			padding_packet->ReserveExtension<TransmissionOffset>();
		}
		if (rtp_header_extension_map_.IsRegistered(AbsoluteSendTime::kId)) {
			padding_packet->ReserveExtension<AbsoluteSendTime>();
		}

		padding_packet->SetPadding(padding_bytes_in_packet);
		bytes_left -= std::min(bytes_left, padding_bytes_in_packet);
		padding_packets.push_back(std::move(padding_packet));
	}

	return padding_packets;
}

void RTCRTPSender::OnReceivedNack(const std::vector<uint16_t>& nack_sequence_numbers,
	int64_t avg_rtt) {
	packet_history_->SetRtt(5 + avg_rtt);
	for (uint16_t seq_no : nack_sequence_numbers) {
		const int32_t bytes_sent = ReSendPacket(seq_no);
		if (bytes_sent < 0) {
			// Failed to send one Sequence number. Give up the rest in this nack.
			RTC_LOG(LS_WARNING) << "Failed resending RTP packet " << seq_no
				<< ", Discard rest of packets.";
			break;
		}
	}
}

int32_t RTCRTPSender::ReSendPacket(uint16_t packet_id) {
	int32_t packet_size = 0;
	const bool rtx = (RtxStatus() & kRtxRetransmitted) > 0;

	std::unique_ptr<RtpPacketToSend> packet =
		packet_history_->GetPacketAndMarkAsPending(
		packet_id, [&](const RtpPacketToSend& stored_packet) {
		// Check if we're overusing retransmission bitrate.
		// TODO(sprang): Add histograms for nack success or failure
		// reasons.
		packet_size = stored_packet.size();
		std::unique_ptr<RtpPacketToSend> retransmit_packet;
		if (rtx) {
			retransmit_packet = BuildRtxPacket(stored_packet);
		}
		else {
			retransmit_packet =
				std::make_unique<RtpPacketToSend>(stored_packet);
		}
		if (retransmit_packet) {
			retransmit_packet->set_retransmitted_sequence_number(
				stored_packet.SequenceNumber());
		}
		return retransmit_packet;
	});

	if (packet_size == 0) {
		RTC_DCHECK(!packet);
		return 0;
	}
	
	if (!packet) {
		return -1;
	}

	packet->set_packet_type(RtpPacketMediaType::kRetransmission);
	packet->set_fec_protect_packet(false);

	std::vector<std::unique_ptr<RtpPacketToSend>> packets;
	packets.emplace_back(std::move(packet));
	paced_sender_->EnqueuePackets(std::move(packets));

	return packet_size;
}

void RTCRTPSender::SetRtxPayloadType(int payload_type, int associated_payload_type) {
	RTC_DCHECK_LE(payload_type, 127);
	RTC_DCHECK_LE(associated_payload_type, 127);
	if (payload_type < 0) {
		RTC_LOG(LS_ERROR) << "Invalid RTX payload type: " << payload_type << ".";
		return;
	}

	rtx_payload_type_map_[associated_payload_type] = payload_type;
}

void RTCRTPSender::EnqueuePackets(std::vector<std::unique_ptr<RtpPacketToSend>> packets) {
	RTC_DCHECK(!packets.empty());
	Timestamp now = clock_->CurrentTime();
	for (auto& packet : packets) {
		RTC_DCHECK(packet);
		RTC_CHECK(packet->packet_type().has_value())
			<< "Packet type must be set before sending.";
		if (packet->capture_time_ms() <= 0ll) {
			packet->set_capture_time_ms(now.ms());
		}
	}

	paced_sender_->EnqueuePackets(std::move(packets));
}

void RTCRTPSender::EnqueuePacket(std::unique_ptr<RtpPacketToSend> packet) {
	Timestamp now = clock_->CurrentTime();
	RTC_DCHECK(packet);
	RTC_CHECK(packet->packet_type().has_value())
		<< "Packet type must be set before sending.";
	if (packet->capture_time_ms() <= 0ll) {
		packet->set_capture_time_ms(now.ms());
	}

	paced_sender_->EnqueuePacket(std::move(packet));
}

static void CopyHeaderAndExtensionsToRtxPacket(const RtpPacketToSend& packet,
	RtpPacketToSend* rtx_packet) {
	rtx_packet->SetMarker(packet.Marker());
	rtx_packet->SetTimestamp(packet.Timestamp());

	const std::vector<uint32_t> csrcs = packet.Csrcs();
	rtx_packet->SetCsrcs(csrcs);

	for (int extension_num = kRtpExtensionNone + 1;
		extension_num < kRtpExtensionNumberOfExtensions; ++extension_num) {
		auto extension = static_cast<RTPExtensionType>(extension_num);

		if (extension == kRtpExtensionMid ||
			extension == kRtpExtensionRtpStreamId) {
			continue;
		}

		if (!packet.HasExtension(extension)) {
			continue;
		}

		rtc::ArrayView<const uint8_t> source = packet.FindExtension(extension);

		rtc::ArrayView<uint8_t> destination =
			rtx_packet->AllocateExtension(extension, source.size());

		if (destination.empty() || source.size() != destination.size()) {
			continue;
		}

		std::memcpy(destination.begin(), source.begin(), destination.size());
	}
}

std::unique_ptr<RtpPacketToSend> RTCRTPSender::BuildRtxPacket(
	const RtpPacketToSend& packet) {
	std::unique_ptr<RtpPacketToSend> rtx_packet;
	{
		MutexLock lock(&send_mutex_);
		if (!sending_media_)
			return nullptr;

		RTC_DCHECK(rtx_ssrc_);

		auto kv = rtx_payload_type_map_.find(packet.PayloadType());
		if (kv == rtx_payload_type_map_.end())
			return nullptr;

		rtx_packet = std::make_unique<RtpPacketToSend>(&rtp_header_extension_map_,
			max_packet_size_);
		rtx_packet->SetPayloadType(kv->second);
		rtx_packet->SetSsrc(*rtx_ssrc_);

		CopyHeaderAndExtensionsToRtxPacket(packet, rtx_packet.get());
	}
	RTC_DCHECK(rtx_packet);

	uint8_t* rtx_payload =
		rtx_packet->AllocatePayload(packet.payload_size() + kRtxHeaderSize);
	if (rtx_payload == nullptr)
		return nullptr;

	ByteWriter<uint16_t>::WriteBigEndian(rtx_payload, packet.SequenceNumber());

	auto payload = packet.payload();
	memcpy(rtx_payload + kRtxHeaderSize, payload.data(), payload.size());

	rtx_packet->set_application_data(packet.application_data());
	rtx_packet->set_additional_data(packet.additional_data());

	rtx_packet->set_capture_time_ms(packet.capture_time_ms());

	return rtx_packet;
}

}