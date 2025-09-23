//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /pc/transceiver_list.cc
//
//////////////////////////////////////////////////////////////////////////

#include "rtc_base/checks.h"

#include "RTCTransceiverList.h"

namespace webrtc {

void RTCTransceiverStableState::set_newly_created() {
	RTC_DCHECK(!has_m_section_);
	newly_created_ = true;
}

void RTCTransceiverStableState::SetMSectionIfUnset(
						absl::optional<std::string> mid,
						absl::optional<size_t> mline_index) {
	if (!has_m_section_) {
		mid_ = mid;
		mline_index_ = mline_index;
		has_m_section_ = true;
	}
}

void RTCTransceiverStableState::SetRemoteStreamIdsIfUnset(
							const std::vector<std::string>& ids) {
	if (!remote_stream_ids_.has_value()) {
		remote_stream_ids_ = ids;
	}
}

RTCRtpTransceiverProxyRefPtr RTCTransceiverList::FindBySender(
				rtc::scoped_refptr<RTCRtpSenderInterface> sender) const {
	for (auto transceiver : transceivers_) {
		if (transceiver->sender() == sender) {
			return transceiver;
		}
	}
	return nullptr;
}

RTCRtpTransceiverProxyRefPtr RTCTransceiverList::FindByMid(
									const std::string& mid) const {
	for (auto transceiver : transceivers_) {
		if (transceiver->mid() == mid) {
			return transceiver;
		}
	}
	return nullptr;
}

RTCRtpTransceiverProxyRefPtr RTCTransceiverList::FindByMLineIndex(
												size_t mline_index) const {
	for (auto transceiver : transceivers_) {
		if (transceiver->internal()->mline_index() == mline_index) {
			return transceiver;
		}
	}
	return nullptr;
}

}  // namespace webrtc
