//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /call/audio_send_stream.h
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_CALL_AUDIO_SEND_STREAM_H__
#define __RTC_CALL_AUDIO_SEND_STREAM_H__

#include <memory>
#include <string>
#include <vector>

#include "absl/types/optional.h"

#include "api/audio_codecs/audio_format.h"
#include "api/call/transport.h"
#include "api/crypto/crypto_options.h"

#include "api/rtp_parameters.h"
#include "api/scoped_refptr.h"
#include "call/rtp_config.h"

#include "modules/rtp_rtcp/include/report_block_data.h"
#include "modules/rtp_rtcp/source/rtp_packet_received.h"

#include "../api/call/RTCTransportSequenceNumberGenerator.h"
#include "../api/audio/RTCAudioPacketRouteSourceInterface.h"


namespace webrtc {

class RTCAudioSendStream {
public:
	struct Stats {
		Stats();
		~Stats();

		// TODO(solenberg): Harmonize naming and defaults with receive stream stats.
		uint32_t local_ssrc = 0;
		int64_t payload_bytes_sent = 0;
		int64_t header_and_padding_bytes_sent = 0;
		// https://w3c.github.io/webrtc-stats/#dom-rtcoutboundrtpstreamstats-retransmittedbytessent
		uint64_t retransmitted_bytes_sent = 0;
		int32_t packets_sent = 0;
		// https://w3c.github.io/webrtc-stats/#dom-rtcoutboundrtpstreamstats-retransmittedpacketssent
		uint64_t retransmitted_packets_sent = 0;
		int32_t packets_lost = -1;
		float fraction_lost = -1.0f;
		std::string codec_name;
		absl::optional<int> codec_payload_type;
		int32_t jitter_ms = -1;
		int64_t rtt_ms = -1;
		int16_t audio_level = 0;
		// See description of "totalAudioEnergy" in the WebRTC stats spec:
		// https://w3c.github.io/webrtc-stats/#dom-rtcmediastreamtrackstats-totalaudioenergy
		double total_input_energy = 0.0;
		double total_input_duration = 0.0;
		bool typing_noise_detected = false;

		int64_t target_bitrate_bps = 0;
		// A snapshot of Report Blocks with additional data of interest to
		// statistics. Within this list, the sender-source SSRC pair is unique and
		// per-pair the ReportBlockData represents the latest Report Block that was
		// received for that pair.
		std::vector<ReportBlockData> report_block_datas;
	};

	struct Config {
		Config() = delete;
		explicit Config(Transport* send_transport);
		~Config();
		std::string ToString() const;

		// Send-stream specific RTP settings.
		struct Rtp {
			Rtp();
			~Rtp();
			std::string ToString() const;

			// Sender SSRC.
			uint32_t ssrc = 0;

			// The value to send in the RID RTP header extension if the extension is
			// included in the list of extensions.
			std::string rid;

			// The value to send in the MID RTP header extension if the extension is
			// included in the list of extensions.
			std::string mid;

			// Corresponds to the SDP attribute extmap-allow-mixed.
			bool extmap_allow_mixed = false;

			// RTP header extensions used for the sent stream.
			std::vector<RtpExtension> extensions;

			// RTCP CNAME, see RFC 3550.
			std::string c_name;
		} rtp;

		// Time interval between RTCP report for audio
		int rtcp_report_interval_ms = 5000;

		// Transport for outgoing packets. The transport is expected to exist for
		// the entire life of the AudioSendStream and is owned by the API client.
		Transport* send_transport = nullptr;

		RTCTransportSequenceNumberGenerator* transport_sequence_number_generator = nullptr;

		// Bitrate limits used for variable audio bitrate streams. Set both to -1 to
		// disable audio bitrate adaptation.
		// Note: This is still an experimental feature and not ready for real usage.
		int min_bitrate_bps = -1;
		int max_bitrate_bps = -1;

		double bitrate_priority = 1.0;
		bool has_dscp = false;

		struct SendCodecSpec {
			SendCodecSpec(int payload_type, const SdpAudioFormat& format);
			~SendCodecSpec();
			std::string ToString() const;

			bool operator==(const SendCodecSpec& rhs) const;
			bool operator!=(const SendCodecSpec& rhs) const {
				return !(*this == rhs);
			}

			int payload_type;
			SdpAudioFormat format;
			bool nack_enabled = false;
			bool transport_cc_enabled = false;
			absl::optional<int> cng_payload_type;
			absl::optional<int> red_payload_type;
			// If unset, use the encoder's default target bitrate.
			absl::optional<int> target_bitrate_bps;
		};
 
 		absl::optional<SendCodecSpec> send_codec_spec;

		// Track ID as specified during track creation.
		std::string track_id;

		// Per PeerConnection crypto options.
		webrtc::CryptoOptions crypto_options;
	};

	virtual ~RTCAudioSendStream() = default;

	virtual const RTCAudioSendStream::Config& GetConfig() const = 0;

	// Starts stream activity.
	// When a stream is active, it can receive, process and deliver packets.
	virtual void Start() = 0;
	// Stops stream activity.
	// When a stream is stopped, it can't receive, process or deliver packets.
	virtual void Stop() = 0;

	virtual void DeliverRtcp(const uint8_t* packet, size_t length) = 0;

	virtual void SetSource(rtc::RTCAudioPacketRouteSourceInterface<std::unique_ptr<webrtc::RtpPacketReceived>>* source) = 0;	

	virtual Stats GetStats() const = 0;
};

}

#endif // __RTC_CALL_AUDIO_SEND_STREAM_H__