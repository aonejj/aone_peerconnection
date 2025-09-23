//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : Replace RtpTransceiver to RTCRtpTransceiver class for rtc media server.
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_RTP_TRANSCEIVER_H__
#define __RTC_RTP_TRANSCEIVER_H__

#include <stddef.h>

#include <algorithm>
#include <string>
#include <vector>

#include "../rtc_pc_src/_deprecate_defines.h"

#include "absl/types/optional.h"

#include "api/array_view.h"
#include "api/media_types.h"
#include "api/proxy.h"
#include "api/rtc_error.h"
#include "api/make_ref_counted.h"

#include "api/rtp_transceiver_direction.h"
#include "api/scoped_refptr.h"

#include "pc/channel_interface.h"

#include "rtc_base/ref_counted_object.h"
#include "rtc_base/third_party/sigslot/sigslot.h"
#include "rtc_base/thread_annotations.h"

#include "../api/RTCRtpTransceiverInterface.h"
#include "RTCRtpSender.h"
#include "RTCRtpReceiver.h"
#include "RTCChannelManager.h"

#include "../rtc_pc_src/RTCThreadManagerInterface.h"


namespace webrtc {

class RTCRtpTransceiver 
//	: public rtc::RefCountedObject<RTCRtpTransceiverInterface>,
	: public RTCRtpTransceiverInterface,
	  public sigslot::has_slots<> {
public:
	explicit RTCRtpTransceiver(cricket::MediaType media_type,
		rtc::RTCThreadManagerInterface* rtc_thread_manager
		);

	RTCRtpTransceiver(
		rtc::scoped_refptr<RTCRtpSenderProxyWithInternal<RTCRtpSenderInternal>> sender,
		rtc::scoped_refptr<RTCRtpReceiverProxyWithInternal<RTCRtpReceiverInternal>>
		receiver,
		cricket::RTCChannelManager* channel_manager,
		std::vector<RtpHeaderExtensionCapability> HeaderExtensionsToOffer,
		std::function<void()> on_negotiation_needed,
		rtc::RTCThreadManagerInterface* rtc_thread_manager
		);

	~RTCRtpTransceiver() override;

	cricket::ChannelInterface* channel() const { return channel_; }
	void SetChannel(cricket::ChannelInterface* channel);

	void AddSender(
		rtc::scoped_refptr<RTCRtpSenderProxyWithInternal<RTCRtpSenderInternal>> sender);
	bool RemoveSender(RTCRtpSenderInterface* sender);

	std::vector<rtc::scoped_refptr<RTCRtpSenderProxyWithInternal<RTCRtpSenderInternal>>>
		senders() const {
		return senders_;
	}

	void AddReceiver(
		rtc::scoped_refptr<RTCRtpReceiverProxyWithInternal<RTCRtpReceiverInternal>>
		receiver);

	bool RemoveReceiver(RTCRtpReceiverInterface* receiver);

	std::vector<
		rtc::scoped_refptr<RTCRtpReceiverProxyWithInternal<RTCRtpReceiverInternal>>>
		receivers() const {
		return receivers_;
	}

	// Returns the backing object for the transceiver's Unified Plan sender.
	rtc::scoped_refptr<RTCRtpSenderInternal> sender_internal() const;
	// Returns the backing object for the transceiver's Unified Plan receiver.
	rtc::scoped_refptr<RTCRtpReceiverInternal> receiver_internal() const;

	absl::optional<size_t> mline_index() const { return mline_index_; }
	void set_mline_index(absl::optional<size_t> mline_index) {
		mline_index_ = mline_index;
	}

	void set_mid(const absl::optional<std::string>& mid) { mid_ = mid; }

	void set_direction(RtpTransceiverDirection direction) {
		direction_ = direction;
	}

	void set_current_direction(RtpTransceiverDirection direction);

	void set_fired_direction(RtpTransceiverDirection direction); 

	void set_created_by_addtrack(bool created_by_addtrack) {
		created_by_addtrack_ = created_by_addtrack;
	}

	void set_reused_for_addtrack(bool reused_for_addtrack) {
		reused_for_addtrack_ = reused_for_addtrack;
	}

	bool created_by_addtrack() const { return created_by_addtrack_; }

	bool reused_for_addtrack() const { return reused_for_addtrack_; }

	bool has_ever_been_used_to_send() const {
		return has_ever_been_used_to_send_;
	}

	void SetRTCConnectionClosed();

	void StopTransceiverProcedure();

	// RtpTransceiverInterface implementation.
	cricket::MediaType media_type() const override;
	absl::optional<std::string> mid() const override;
	rtc::scoped_refptr<RTCRtpSenderInterface> sender() const override;
	rtc::scoped_refptr<RTCRtpReceiverInterface> receiver() const override;

	uint32_t receiver_size() const override;
	rtc::scoped_refptr<RTCRtpReceiverInterface> receiver(uint32_t idx) const override;

	bool stopped() const override;
	bool stopping() const override;
	RtpTransceiverDirection direction() const override;
	RTCError SetDirectionWithError(
		RtpTransceiverDirection new_direction) override;
	absl::optional<RtpTransceiverDirection> current_direction() const override;
	absl::optional<RtpTransceiverDirection> fired_direction() const override;
	RTCError StopStandard() override;
	void StopInternal() override;

	RTCError SetCodecPreferences(
		rtc::ArrayView<RtpCodecCapability> codecs) override;
	std::vector<RtpCodecCapability> codec_preferences() const override {
		return codec_preferences_;
	}
	std::vector<RtpHeaderExtensionCapability> HeaderExtensionsToOffer()
		const override;
	std::vector<RtpHeaderExtensionCapability> HeaderExtensionsNegotiated()
		const override;
	RTCError SetOfferedRtpHeaderExtensions(
		rtc::ArrayView<const RtpHeaderExtensionCapability>
		header_extensions_to_offer) override;

private:
	void OnFirstPacketReceived(cricket::ChannelInterface* channel);
	void StopSendingAndReceiving();

private:
	const TaskQueueBase* thread_;
	const bool unified_plan_;
	const cricket::MediaType media_type_;
	std::vector<rtc::scoped_refptr<RTCRtpSenderProxyWithInternal<RTCRtpSenderInternal>>>
		senders_;
	std::vector<
		rtc::scoped_refptr<RTCRtpReceiverProxyWithInternal<RTCRtpReceiverInternal>>>
		receivers_;


	bool stopped_ = false;
	bool stopping_ RTC_GUARDED_BY(thread_) = false;
	bool is_pc_closed_ = false;
	RtpTransceiverDirection direction_ = RtpTransceiverDirection::kInactive;
	absl::optional<RtpTransceiverDirection> current_direction_;
	absl::optional<RtpTransceiverDirection> fired_direction_;
	absl::optional<std::string> mid_;
	absl::optional<size_t> mline_index_;
	bool created_by_addtrack_ = false;
	bool reused_for_addtrack_ = false;
	bool has_ever_been_used_to_send_ = false;

	cricket::ChannelInterface* channel_ = nullptr;
	cricket::RTCChannelManager* channel_manager_ = nullptr;
	std::vector<RtpCodecCapability> codec_preferences_;
	std::vector<RtpHeaderExtensionCapability> header_extensions_to_offer_;
	const std::function<void()> on_negotiation_needed_;
};

BEGIN_PRIMARY_PROXY_MAP(RTCRtpTransceiver)
PROXY_PRIMARY_THREAD_DESTRUCTOR()
BYPASS_PROXY_CONSTMETHOD0(cricket::MediaType, media_type)
PROXY_CONSTMETHOD0(absl::optional<std::string>, mid)
PROXY_CONSTMETHOD0(rtc::scoped_refptr<RTCRtpSenderInterface>, sender)
PROXY_CONSTMETHOD0(rtc::scoped_refptr<RTCRtpReceiverInterface>, receiver)
PROXY_CONSTMETHOD0(uint32_t, receiver_size)
PROXY_CONSTMETHOD1(rtc::scoped_refptr<RTCRtpReceiverInterface>, receiver, uint32_t)
PROXY_CONSTMETHOD0(bool, stopped)
PROXY_CONSTMETHOD0(bool, stopping)
PROXY_CONSTMETHOD0(RtpTransceiverDirection, direction)
PROXY_METHOD1(webrtc::RTCError, SetDirectionWithError, RtpTransceiverDirection)
PROXY_CONSTMETHOD0(absl::optional<RtpTransceiverDirection>, current_direction)
PROXY_CONSTMETHOD0(absl::optional<RtpTransceiverDirection>, fired_direction)
PROXY_METHOD0(webrtc::RTCError, StopStandard)
PROXY_METHOD0(void, StopInternal)
PROXY_METHOD1(webrtc::RTCError,
				SetCodecPreferences,
				rtc::ArrayView<RtpCodecCapability>)
PROXY_CONSTMETHOD0(std::vector<RtpCodecCapability>, codec_preferences)
PROXY_CONSTMETHOD0(std::vector<RtpHeaderExtensionCapability>,
					HeaderExtensionsToOffer)
PROXY_CONSTMETHOD0(std::vector<RtpHeaderExtensionCapability>,
					HeaderExtensionsNegotiated)
PROXY_METHOD1(webrtc::RTCError,
				SetOfferedRtpHeaderExtensions,
rtc::ArrayView<const RtpHeaderExtensionCapability>)
END_PROXY_MAP(RTCRtpTransceiver)

}

#endif // __RTC_RTP_TRANSCEIVER_H__