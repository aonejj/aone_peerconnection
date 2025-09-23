//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /pc/rtp_receiver.h
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_PC_RTP_RECEIVER_H__
#define __RTC_PC_RTP_RECEIVER_H__

#include <stdint.h>

#include <string>
#include <vector>

#include "../rtc_pc_src/_deprecate_defines.h"

#include "absl/types/optional.h"
#include "api/media_types.h"
#include "api/rtp_parameters.h"
#include "api/scoped_refptr.h"
#include "rtc_base/ref_counted_object.h"
#include "rtc_base/thread.h"

#include "../../src_update/api/_media_stream_interface.h"
#include "../../src_update/media/base/_media_channel.h"
#include "../api/RTCRtpReceiverInterface.h"
#include "RTCVideoTrackSource.h"

#include "../rtc_pc_src/RTCThreadManagerInterface.h"

namespace webrtc {

class RTCRtpReceiverInternal : public RTCRtpReceiverInterface {
public:
	// Stops receiving. The track may be reactivated.
	virtual void Stop() = 0;
	// Stops the receiver permanently.
	// Causes the associated track to enter kEnded state. Cannot be reversed.
	virtual void StopAndEndTrack() = 0;

	// Sets the underlying MediaEngine channel associated with this RtpSender.
	// A VoiceMediaChannel should be used for audio RtpSenders and
	// a VideoMediaChannel should be used for video RtpSenders.
	// Must call SetMediaChannel(nullptr) before the media channel is destroyed.
	virtual void SetMediaChannel(cricket::MediaChannel* media_channel) = 0;

	// Configures the RtpReceiver with the underlying media channel, with the
	// given SSRC as the stream identifier.
	virtual void SetupMediaChannel(uint32_t ssrc) = 0;

	// Configures the RtpReceiver with the underlying media channel to receive an
	// unsignaled receive stream.
	virtual void SetupUnsignaledMediaChannel() = 0;

	virtual void set_transport(
		rtc::scoped_refptr<DtlsTransportInterface> dtls_transport) = 0;
	// This SSRC is used as an identifier for the receiver between the API layer
	// and the WebRtcVideoEngine, WebRtcVoiceEngine layer.
	virtual uint32_t ssrc() const = 0;

	// Call this to notify the RtpReceiver when the first packet has been received
	// on the corresponding channel.
	virtual void NotifyFirstPacketReceived() = 0;

	// Set the associated remote media streams for this receiver. The remote track
	// will be removed from any streams that are no longer present and added to
	// any new streams.
	virtual void set_stream_ids(std::vector<std::string> stream_ids) = 0;
	// TODO(https://crbug.com/webrtc/9480): Remove SetStreams() in favor of
	// set_stream_ids() as soon as downstream projects are no longer dependent on
	// stream objects.
	virtual void SetStreams(
		const std::vector<rtc::scoped_refptr<MediaStreamInterface>>& streams) = 0;

	// Returns an ID that changes if the attached track changes, but
	// otherwise remains constant. Used to generate IDs for stats.
	// The special value zero means that no track is attached.
	virtual int AttachmentId() const = 0;

protected:
	static int GenerateUniqueId();

	static std::vector<rtc::scoped_refptr<MediaStreamInterface>>
		CreateStreamsFromIds(std::vector<std::string> stream_ids	, rtc::RTCThreadManagerInterface* rtc_thread_manager	);
};

}  // namespace webrtc


#endif	// __RTC_PC_RTP_RECEIVER_H__