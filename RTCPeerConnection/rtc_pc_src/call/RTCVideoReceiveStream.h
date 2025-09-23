//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /call/video_receive_stream.h
//
//////////////////////////////////////////////////////////////////////////


#ifndef __RTC_CALL_VIDEO_RECEIVE_STREAM_H__
#define __RTC_CALL_VIDEO_RECEIVE_STREAM_H__

#include <limits>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "common_video/frame_counts.h"
#include "api/video/video_content_type.h"
#include "api/call/transport.h"
#include "api/crypto/crypto_options.h"
#include "api/transport/rtp/rtp_source.h"
#include "api/video_codecs/sdp_video_format.h"
#include "call/rtp_config.h"
#include "modules/rtp_rtcp/include/rtp_rtcp_defines.h"
#include "modules/rtp_rtcp/include/rtcp_statistics.h"

namespace webrtc {
class RtpPacketSinkInterface;

class RTCVideoReceiveStream {
public:
	struct Decoder {
		Decoder();
		Decoder(const Decoder&);
		~Decoder();
		std::string ToString() const;

		SdpVideoFormat video_format;

		// Received RTP packets with this payload type will be sent to this decoder
		// instance.
		int payload_type = 0;
	};

	struct Stats {
		Stats();
		~Stats();
		std::string ToString(int64_t time_ms) const;

		// Decoder stats.
		std::string decoder_implementation_name = "unknown";
		FrameCounts frame_counts;
		VideoContentType content_type = VideoContentType::UNSPECIFIED;

		// https://w3c.github.io/webrtc-stats/#dom-rtcinboundrtpstreamstats-estimatedplayouttimestamp
		absl::optional<int64_t> estimated_playout_ntp_timestamp_ms;
		int sync_offset_ms = std::numeric_limits<int>::max();

		int width = 0;
		int height = 0;

		uint32_t ssrc = 0;
		std::string c_name;
		RtpReceiveStats rtp_stats;
		RtcpPacketTypeCounter rtcp_packet_type_counts;
	};

	struct Config {
	private:
		// Access to the copy constructor is private to force use of the Copy()
		// method for those exceptional cases where we do use it.
		Config(const Config&);

	public:
		Config() = delete;
		Config(Config&&);
		explicit Config(Transport* rtcp_send_transport);
		Config& operator=(Config&&);
		Config& operator=(const Config&) = delete;
		~Config();

		// Mostly used by tests.  Avoid creating copies if you can.
		Config Copy() const { return Config(*this); }

		std::string ToString() const;

		std::vector<Decoder> decoders;

		// Receive-stream specific RTP settings.
		struct Rtp {
			Rtp();
			Rtp(const Rtp&);
			~Rtp();
			std::string ToString() const;

			// Synchronization source (stream identifier) to be received.
			uint32_t remote_ssrc = 0;

			// Sender SSRC used for sending RTCP (such as receiver reports).
			uint32_t local_ssrc = 0;

			// See RtcpMode for description.
			RtcpMode rtcp_mode = RtcpMode::kCompound;

			// Extended RTCP settings.
			struct RtcpXr {
				// True if RTCP Receiver Reference Time Report Block extension
				// (RFC 3611) should be enabled.
				bool receiver_reference_time_report = false;
			} rtcp_xr;

			// See draft-holmer-rmcat-transport-wide-cc-extensions for details.
			bool transport_cc = false;

			// See LntfConfig for description.
			LntfConfig lntf;

			// See NackConfig for description.
			NackConfig nack;

			// Payload types for ULPFEC and RED, respectively.
			int ulpfec_payload_type = -1;
			int red_payload_type = -1;

			// SSRC for retransmissions.
			uint32_t rtx_ssrc = 0;

			// Set if the stream is protected using FlexFEC.
			bool protected_by_flexfec = false;

			// Map from rtx payload type -> media payload type.
			// For RTX to be enabled, both an SSRC and this mapping are needed.
			std::map<int, int> rtx_associated_payload_types;

			// Payload types that should be depacketized using raw depacketizer
			// (payload header will not be parsed and must not be present, additional
			// meta data is expected to be present in generic frame descriptor
			// RTP header extension).
			std::set<int> raw_payload_types;

			// RTP header extensions used for the received stream.
			std::vector<RtpExtension> extensions;
		} rtp;

		// Transport for outgoing packets (RTCP).
		Transport* rtcp_send_transport = nullptr;


		std::string sync_group;
		// TODO(nisse): Used with VideoDecoderFactory::LegacyCreateVideoDecoder.
		// Delete when that method is retired.
		std::string stream_id;

		// Per PeerConnection cryptography options.
		CryptoOptions crypto_options;
	};

	// Starts stream activity.
	// When a stream is active, it can receive, process and deliver packets.
	virtual void Start() = 0;
	// Stops stream activity.
	// When a stream is stopped, it can't receive, process or deliver packets.
	virtual void Stop() = 0;

	// TODO(pbos): Add info on currently-received codec to Stats.
	virtual Stats GetStats() const = 0;

	// RtpDemuxer only forwards a given RTP packet to one sink. However, some
	// sinks, such as FlexFEC, might wish to be informed of all of the packets
	// a given sink receives (or any set of sinks). They may do so by registering
	// themselves as secondary sinks.
	virtual void AddSecondarySink(RtpPacketSinkInterface* sink) = 0;
	virtual void RemoveSecondarySink(const RtpPacketSinkInterface* sink) = 0;

	virtual std::vector<RtpSource> GetSources() const = 0;

	// Sets a base minimum for the playout delay. Base minimum delay sets lower
	// bound on minimum delay value determining lower bound on playout delay.
	//
	// Returns true if value was successfully set, false overwise.
//	virtual bool SetBaseMinimumPlayoutDelayMs(int delay_ms) = 0;

	// Returns current value of base minimum delay in milliseconds.
//	virtual int GetBaseMinimumPlayoutDelayMs() const = 0;

	// Cause eventual generation of a key frame from the sender.
	virtual void GenerateKeyFrame() = 0;
	virtual void GenerateKeyFramePli() = 0;

protected:
	virtual ~RTCVideoReceiveStream() {}
};
}

#endif	// __RTC_CALL_VIDEO_RECEIVE_STREAM_H__