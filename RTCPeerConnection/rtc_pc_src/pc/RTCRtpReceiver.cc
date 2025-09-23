//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /pc/rtp_receiver.cc
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

#include "../../src_update/api/_media_stream_track_proxy.h"
#include "RTCRtpReceiver.h"


namespace webrtc {

// This function is only expected to be called on the signalling thread.
int RTCRtpReceiverInternal::GenerateUniqueId() {
	static int g_unique_id = 0;

	return ++g_unique_id;
}

std::vector<rtc::scoped_refptr<MediaStreamInterface>>
RTCRtpReceiverInternal::CreateStreamsFromIds(std::vector<std::string> stream_ids,
	rtc::RTCThreadManagerInterface* rtc_thread_manager
) {
	std::vector<rtc::scoped_refptr<MediaStreamInterface>> streams(
		stream_ids.size());
	for (size_t i = 0; i < stream_ids.size(); ++i) {
		streams[i] = MediaStreamProxy::Create(
			rtc::Thread::Current(rtc_thread_manager),
			MediaStream::Create(std::move(stream_ids[i])));
	}
	return streams;
}

}	// namespace webrtc