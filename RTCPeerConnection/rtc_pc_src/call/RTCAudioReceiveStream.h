//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /call/audio_receiver_stream.h
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_CALL_AUDIO_RECEIVE_STREAM_H__
#define __RTC_CALL_AUDIO_RECEIVE_STREAM_H__

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "absl/types/optional.h"

#include "api/audio_codecs/audio_format.h"
#include "api/call/transport.h"
#include "api/crypto/crypto_options.h"

#include "api/rtp_parameters.h"
#include "api/scoped_refptr.h"
#include "api/transport/rtp/rtp_source.h"
#include "call/rtp_config.h"

namespace webrtc {
class AudioSinkInterface;

class RTCAudioReceiveStream {
public:
	struct Config {
		Config();
		~Config();

		std::string ToString() const;

		// Receive-stream specific RTP settings.
		struct Rtp {
			Rtp();
			~Rtp();

			std::string ToString() const;

			// Synchronization source (stream identifier) to be received.
			uint32_t remote_ssrc = 0;

			// Sender SSRC used for sending RTCP (such as receiver reports).
			uint32_t local_ssrc = 0;

			// Enable feedback for send side bandwidth estimation.
			// See
			// https://tools.ietf.org/html/draft-holmer-rmcat-transport-wide-cc-extensions
			// for details.
			bool transport_cc = false;

			// See NackConfig for description.
			NackConfig nack;

			// RTP header extensions used for the received stream.
			std::vector<RtpExtension> extensions;
		} rtp;

		Transport* rtcp_send_transport = nullptr;


		// Decoder specifications for every payload type that we can receive.
		std::map<int, SdpAudioFormat> decoder_map;			
	};

	// Reconfigure the stream according to the Configuration.
	virtual void Reconfigure(const Config& config) = 0;

	// Starts stream activity.
	// When a stream is active, it can receive, process and deliver packets.
	virtual void Start() = 0;
	// Stops stream activity.
	// When a stream is stopped, it can't receive, process or deliver packets.
	virtual void Stop() = 0;

	// Returns true if the stream has been started.
	virtual bool IsRunning() const = 0;

	virtual std::vector<RtpSource> GetSources() const = 0;

protected:
	virtual ~RTCAudioReceiveStream() {}
};

}  // namespace webrtc


#endif // __RTC_CALL_AUDIO_RECEIVE_STREAM_H__