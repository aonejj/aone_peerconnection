//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /call/video_send_stream.h
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_CALL_VIDEO_SEND_STREAM_H__
#define __RTC_CALL_VIDEO_SEND_STREAM_H__

#include <stdint.h>

#include <map>
#include <string>
#include <vector>
#include <memory>

#include "absl/types/optional.h"

#include "common_video/frame_counts.h"
#include "modules/rtp_rtcp/include/report_block_data.h"
#include "modules/rtp_rtcp/include/rtcp_statistics.h"
#include "modules/rtp_rtcp/source/rtp_packet_received.h"
#include "api/video/video_content_type.h"
#include "api/call/transport.h"
#include "api/crypto/crypto_options.h"
#include "call/rtp_config.h"

#include "../api/call/RTCTransportSequenceNumberGenerator.h"
#include "../api/video/RTCVideoPacketRouteSourceInterface.h"

namespace webrtc {

class RTCVideoSendStream {
public:
	// Multiple StreamStats objects are present if simulcast is used (multiple
	// kMedia streams) or if RTX or FlexFEC is negotiated. Multiple SVC layers, on
	// the other hand, does not cause additional StreamStats.
	struct StreamStats {
		enum class StreamType {
			// A media stream is an RTP stream for audio or video. Retransmissions and
			// FEC is either sent over the same SSRC or negotiated to be sent over
			// separate SSRCs, in which case separate StreamStats objects exist with
			// references to this media stream's SSRC.
			kMedia,
			// RTX streams are streams dedicated to retransmissions. They have a
			// dependency on a single kMedia stream: |referenced_media_ssrc|.
			kRtx,
			// FlexFEC streams are streams dedicated to FlexFEC. They have a
			// dependency on a single kMedia stream: |referenced_media_ssrc|.
			kFlexfec,
		};

		StreamStats();
		~StreamStats();

		std::string ToString() const;

		StreamType type = StreamType::kMedia;
		// If |type| is kRtx or kFlexfec this value is present. The referenced SSRC
		// is the kMedia stream that this stream is performing retransmissions or
		// FEC for. If |type| is kMedia, this value is null.
		absl::optional<uint32_t> referenced_media_ssrc;
		FrameCounts frame_counts;
		int width = 0;
		int height = 0;
		// TODO(holmer): Move bitrate_bps out to the webrtc::Call layer.
		int total_bitrate_bps = 0;
		int retransmit_bitrate_bps = 0;
		int avg_delay_ms = 0;
		int max_delay_ms = 0;
		uint64_t total_packet_send_delay_ms = 0;
		StreamDataCounters rtp_stats;
		RtcpPacketTypeCounter rtcp_packet_type_counts;
		RtcpStatistics rtcp_stats;
		// A snapshot of the most recent Report Block with additional data of
		// interest to statistics. Used to implement RTCRemoteInboundRtpStreamStats.
		absl::optional<ReportBlockData> report_block_data;
		double encode_frame_rate = 0.0;
		int frames_encoded = 0;
		absl::optional<uint64_t> qp_sum;
		uint64_t total_encode_time_ms = 0;
		uint64_t total_encoded_bytes_target = 0;
		uint32_t huge_frames_sent = 0;
	};

	struct Stats {
		Stats();
		~Stats();
		std::string ToString(int64_t time_ms) const;
		std::string encoder_implementation_name = "unknown";
		// Bitrate the encoder is actually producing.
		int media_bitrate_bps = 0;
		bool suspended = false;
		std::map<uint32_t, StreamStats> substreams;
		webrtc::VideoContentType content_type =
			webrtc::VideoContentType::UNSPECIFIED;
	};

	struct Config {
	public:
		Config() = delete;
		Config(Config&&);
		explicit Config(Transport* send_transport);

		Config& operator=(Config&&);
		Config& operator=(const Config&) = delete;

		~Config();

		// Mostly used by tests.  Avoid creating copies if you can.
		Config Copy() const { return Config(*this); }

		std::string ToString() const;

		RtpConfig rtp;


		// Time interval between RTCP report for video
		int rtcp_report_interval_ms = 1000;

		// Transport for outgoing packets.
		Transport* send_transport = nullptr;

		RTCTransportSequenceNumberGenerator* transport_sequence_number_generator = nullptr;	

		// Per PeerConnection cryptography options.
		CryptoOptions crypto_options;
	private:
		// Access to the copy constructor is private to force use of the Copy()
		// method for those exceptional cases where we do use it.
		Config(const Config&);
	};

	// Starts stream activity.
	// When a stream is active, it can receive, process and deliver packets.
	virtual void Start() = 0;
	// Stops stream activity.
	// When a stream is stopped, it can't receive, process or deliver packets.
	virtual void Stop() = 0;


	virtual void SetSource(rtc::RTCVideoPacketRouteSourceInterface<std::unique_ptr<webrtc::RtpPacketReceived>>* source) = 0;

	virtual Stats GetStats() = 0;

protected:
	virtual ~RTCVideoSendStream() {}
};

}  // namespace webrtc


#endif // __RTC_CALL_VIDEO_SEND_STREAM_H__