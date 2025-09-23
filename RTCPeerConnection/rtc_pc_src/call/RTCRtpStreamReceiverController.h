//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_CALL_RTP_STREAM_RECEIVER_CONTROLLER_H__
#define __RTC_CALL_RTP_STREAM_RECEIVER_CONTROLLER_H__

#include <memory.h>
#include "call/rtp_demuxer.h"
#include "call/rtp_stream_receiver_controller_interface.h"
#include "rtc_base/synchronization/mutex.h"

namespace webrtc {

class RtpPacketReceived;

class RTCRtpStreamReceiverController
	: public RtpStreamReceiverControllerInterface {
public:
	RTCRtpStreamReceiverController();
	~RTCRtpStreamReceiverController() override;

	// Implements RtpStreamReceiverControllerInterface.
	std::unique_ptr<RtpStreamReceiverInterface> CreateReceiver(
		uint32_t ssrc,
		RtpPacketSinkInterface* sink) override;

	// Thread-safe wrappers for the corresponding RtpDemuxer methods.
	bool AddSink(uint32_t ssrc, RtpPacketSinkInterface* sink) override;
	size_t RemoveSink(const RtpPacketSinkInterface* sink) override;

	// TODO(nisse): Not yet responsible for parsing.
	bool OnRtpPacket(const RtpPacketReceived& packet);

private:
	class RTCReceiver : public RtpStreamReceiverInterface {
	public:
		RTCReceiver(RTCRtpStreamReceiverController* controller,
			uint32_t ssrc,
			RtpPacketSinkInterface* sink);

		~RTCReceiver() override;

	private:
		RTCRtpStreamReceiverController* const controller_;
		RtpPacketSinkInterface* const sink_;
	};

	// At this level the demuxer is only configured to demux by SSRC, so don't
	// worry about MIDs (MIDs are handled by upper layers).
	RtpDemuxer demuxer_ { false /*use_mid*/ };

	mutable Mutex demuxer_mutex_;
};

}

#endif //__RTC_CALL_RTP_STREAM_RECEIVER_CONTROLLER_H__