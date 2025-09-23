//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_MODULE_FEEDBACK_PACKET_ROUTER_H__
#define __RTC_MODULE_FEEDBACK_PACKET_ROUTER_H__

#include <vector>

#include "modules/remote_bitrate_estimator/include/remote_bitrate_estimator.h"
#include "modules/rtp_rtcp/include/rtp_rtcp_defines.h"
#include "modules/rtp_rtcp/source/rtcp_packet.h"
#include "rtc_base/constructor_magic.h"
#include "rtc_base/synchronization/mutex.h"
#include "rtc_base/thread_annotations.h"

#include "../rtp_rtcp/source/RTCRtpRtcpInterface.h"

namespace webrtc {

class RTCFeedbackPacketRouter : public RemoteBitrateObserver,
								public TransportFeedbackSenderInterface {
public:
	RTCFeedbackPacketRouter();
	~RTCFeedbackPacketRouter() override;

public:
	void AddSendRtpModule(RTCRtpRtcpInterface* rtp_module, bool remb_candidate);
	void RemoveSendRtpModule(RTCRtpRtcpInterface* rtp_module);

	void AddReceiveRtpModule(RtcpFeedbackSenderInterface* rtcp_sender,
							bool remb_candidate);
	void RemoveReceiveRtpModule(RtcpFeedbackSenderInterface* rtcp_sender);

	void OnReceiveBitrateChanged(const std::vector<uint32_t>& ssrcs,
								 uint32_t bitrate_bps) override;

	void SetMaxDesiredReceiveBitrate(int64_t bitrate_bps);

	bool SendRemb(int64_t bitrate_bps, const std::vector<uint32_t>& ssrcs);

	bool SendCombinedRtcpPacket(
		std::vector<std::unique_ptr<rtcp::RtcpPacket>> packets) override;

private:
	void AddSendRtpModuleToMap(RTCRtpRtcpInterface* rtp_module, uint32_t ssrc)
		RTC_EXCLUSIVE_LOCKS_REQUIRED(modules_mutex_);
	void RemoveSendRtpModuleFromMap(uint32_t ssrc)
		RTC_EXCLUSIVE_LOCKS_REQUIRED(modules_mutex_);

	void AddRembModuleCandidate(RtcpFeedbackSenderInterface* candidate_module, bool media_sender)
		RTC_EXCLUSIVE_LOCKS_REQUIRED(modules_mutex_);
	void MaybeRemoveRembModuleCandidate(RtcpFeedbackSenderInterface* candidate_module,
		bool media_sender) RTC_EXCLUSIVE_LOCKS_REQUIRED(modules_mutex_);
	void UnsetActiveRembModule() RTC_EXCLUSIVE_LOCKS_REQUIRED(modules_mutex_);
	void DetermineActiveRembModule() RTC_EXCLUSIVE_LOCKS_REQUIRED(modules_mutex_);

private:
	mutable Mutex modules_mutex_;
	std::vector<RtcpFeedbackSenderInterface*> rtcp_feedback_senders_
		RTC_GUARDED_BY(modules_mutex_);

	std::unordered_map<uint32_t, RTCRtpRtcpInterface*> send_modules_map_
		RTC_GUARDED_BY(modules_mutex_);
	std::list<RTCRtpRtcpInterface*> send_modules_list_
		RTC_GUARDED_BY(modules_mutex_);

	Mutex remb_mutex_;
	int64_t last_remb_time_ms_ RTC_GUARDED_BY(remb_mutex_);
	int64_t last_send_bitrate_bps_ RTC_GUARDED_BY(remb_mutex_);
	int64_t bitrate_bps_ RTC_GUARDED_BY(remb_mutex_);
	int64_t max_bitrate_bps_ RTC_GUARDED_BY(remb_mutex_);

	std::vector<RtcpFeedbackSenderInterface*> sender_remb_candidates_
		RTC_GUARDED_BY(modules_mutex_);
	std::vector<RtcpFeedbackSenderInterface*> receiver_remb_candidates_
		RTC_GUARDED_BY(modules_mutex_);
	RtcpFeedbackSenderInterface* active_remb_module_
		RTC_GUARDED_BY(modules_mutex_);


private:
	RTC_DISALLOW_COPY_AND_ASSIGN(RTCFeedbackPacketRouter);
};

}	// namespace webrtc

#endif // __RTC_MODULE_FEEDBACK_PACKET_ROUTER_H__