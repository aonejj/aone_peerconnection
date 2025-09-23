//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /call/rtc_transport_controller_send_interface.h
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_CALL_RTP_TRANSPORT_CONTROLLER_SEND_INTERFACE_H__
#define __RTC_CALL_RTP_TRANSPORT_CONTROLLER_SEND_INTERFACE_H__

#include <stddef.h>
#include <stdint.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "absl/types/optional.h"
#include "api/transport/bitrate_settings.h"
#include "api/units/timestamp.h"
#include "call/rtp_config.h"
#include "common_video/frame_counts.h"
#include "modules/rtp_rtcp/include/report_block_data.h"
#include "modules/rtp_rtcp/include/rtcp_statistics.h"
#include "modules/rtp_rtcp/include/rtp_packet_sender.h"
#include "modules/rtp_rtcp/include/rtp_rtcp_defines.h"
#include "modules/rtp_rtcp/source/rtp_packet_received.h"

#include "../rtc_pc_src/_deprecate_defines.h"

namespace rtc {
	struct SentPacket;
	struct NetworkRoute;
	class TaskQueue;
}  // namespace rtc

namespace webrtc {

class TargetTransferRateObserver;
class Transport;
class RTCPacketRouter;
class RtcpBandwidthObserver;

struct RtpSenderObservers {
	RtcpRttStats* rtcp_rtt_stats;
	RtcpIntraFrameObserver* intra_frame_callback;
	RtcpLossNotificationObserver* rtcp_loss_notification_observer;
	RtcpStatisticsCallback* rtcp_stats;
	ReportBlockDataObserver* report_block_data_observer;
	StreamDataCountersCallback* rtp_stats;
	BitrateStatisticsObserver* bitrate_observer;
	FrameCountObserver* frame_count_observer;
	RtcpPacketTypeCounterObserver* rtcp_type_observer;
	SendSideDelayObserver* send_delay_observer;
	SendPacketObserver* send_packet_observer;
};

// An RtpTransportController should own everything related to the RTP
// transport to/from a remote endpoint. We should have separate
// interfaces for send and receive side, even if they are implemented
// by the same class. This is an ongoing refactoring project. At some
// point, this class should be promoted to a public api under
// webrtc/api/rtp/.
//
// For a start, this object is just a collection of the objects needed
// by the VideoSendStream constructor. The plan is to move ownership
// of all RTP-related objects here, and add methods to create per-ssrc
// objects which would then be passed to VideoSendStream. Eventually,
// direct accessors like packet_router() should be removed.
//
// This should also have a reference to the underlying
// webrtc::Transport(s). Currently, webrtc::Transport is implemented by
// WebRtcVideoChannel and WebRtcVoiceMediaChannel, and owned by
// WebrtcSession. Video and audio always uses different transport
// objects, even in the common case where they are bundled over the
// same underlying transport.
//
// Extracting the logic of the webrtc::Transport from BaseChannel and
// subclasses into a separate class seems to be a prerequesite for
// moving the transport here.
class RTCRtpTransportControllerSendInterface {
public:
	virtual ~RTCRtpTransportControllerSendInterface() {}
	virtual RTCPacketRouter* packet_router() = 0;


	virtual NetworkStateEstimateObserver* network_state_estimate_observer() = 0;
	virtual TransportFeedbackObserver* transport_feedback_observer() = 0;

	virtual RtpPacketSender* packet_sender() = 0;
	// SetAllocatedSendBitrateLimits sets bitrates limits imposed by send codec
	// settings.

	virtual StreamFeedbackProvider* GetStreamFeedbackProvider() = 0;
	virtual void RegisterTargetTransferRateObserver(
		TargetTransferRateObserver* observer) = 0;
	virtual void OnNetworkRouteChanged(
		const std::string& transport_name,
		const rtc::NetworkRoute& network_route) = 0;
	virtual void OnNetworkAvailability(bool network_available) = 0;
	virtual RtcpBandwidthObserver* GetBandwidthObserver() = 0;

//	virtual int64_t GetPacerQueuingDelayMs() const = 0;
	
//	virtual absl::optional<Timestamp> GetFirstPacketTime() const = 0;

//	virtual void EnablePeriodicAlrProbing(bool enable) = 0;

	virtual void OnSentPacket(const rtc::SentPacket& sent_packet) = 0;
	virtual void OnReceivedPacket(const ReceivedPacket& received_packet) = 0;

	virtual void SetSdpBitrateParameters(
		const BitrateConstraints& constraints) = 0;
	virtual void SetClientBitratePreferences(
		const BitrateSettings& preferences) = 0;

	virtual void OnTransportOverheadChanged(
		size_t transport_overhead_per_packet) = 0;

//	virtual void AccountForAudioPacketsInPacedSender(bool account_for_audio) = 0;
//	virtual void IncludeOverheadInPacedSender() = 0;

	virtual void EnsureStarted() = 0;
};

}  // namespace webrtc


#endif // __RTC_CALL_RTP_TRANSPORT_CONTROLLER_SEND_INTERFACE_H__