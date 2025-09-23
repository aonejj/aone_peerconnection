//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /pc/transceiver_list.h
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_PC_TRANSCEIVER_LIST_H__
#define __RTC_PC_TRANSCEIVER_LIST_H__

#include <algorithm>
#include <map>
#include <string>
#include <vector>

#include "RTCRtpTransceiver.h"

namespace webrtc {

typedef rtc::scoped_refptr<RTCRtpTransceiverProxyWithInternal<RTCRtpTransceiver>>
	RTCRtpTransceiverProxyRefPtr;

// Captures partial state to be used for rollback. Applicable only in
// Unified Plan.
class RTCTransceiverStableState {
public:
	RTCTransceiverStableState() {}
	void set_newly_created();
	void SetMSectionIfUnset(absl::optional<std::string> mid,
		absl::optional<size_t> mline_index);
	void SetRemoteStreamIdsIfUnset(const std::vector<std::string>& ids);
	absl::optional<std::string> mid() const { return mid_; }
	absl::optional<size_t> mline_index() const { return mline_index_; }
	absl::optional<std::vector<std::string>> remote_stream_ids() const {
		return remote_stream_ids_;
	}
	bool has_m_section() const { return has_m_section_; }
	bool newly_created() const { return newly_created_; }

private:
	absl::optional<std::string> mid_;
	absl::optional<size_t> mline_index_;
	absl::optional<std::vector<std::string>> remote_stream_ids_;
	// Indicates that mid value from stable state has been captured and
	// that rollback has to restore the transceiver. Also protects against
	// subsequent overwrites.
	bool has_m_section_ = false;
	// Indicates that the transceiver was created as part of applying a
	// description to track potential need for removing transceiver during
	// rollback.
	bool newly_created_ = false;
};

class RTCTransceiverList {
public:
	std::vector<RTCRtpTransceiverProxyRefPtr> List() const { return transceivers_; }

	void Add(RTCRtpTransceiverProxyRefPtr transceiver) {
		transceivers_.push_back(transceiver);
	}
	void Remove(RTCRtpTransceiverProxyRefPtr transceiver) {
		transceivers_.erase(
			std::remove(transceivers_.begin(), transceivers_.end(), transceiver),
			transceivers_.end());
	}
	RTCRtpTransceiverProxyRefPtr FindBySender(
		rtc::scoped_refptr<RTCRtpSenderInterface> sender) const;
	RTCRtpTransceiverProxyRefPtr FindByMid(const std::string& mid) const;
	RTCRtpTransceiverProxyRefPtr FindByMLineIndex(size_t mline_index) const;

	// Find or create the stable state for a transceiver.
	RTCTransceiverStableState* StableState(RTCRtpTransceiverProxyRefPtr transceiver) {
		return &(transceiver_stable_states_by_transceivers_[transceiver]);
	}

	void DiscardStableStates() {
		transceiver_stable_states_by_transceivers_.clear();
	}

	std::map<RTCRtpTransceiverProxyRefPtr, RTCTransceiverStableState>& StableStates() {
		return transceiver_stable_states_by_transceivers_;
	}

private:
	std::vector<RTCRtpTransceiverProxyRefPtr> transceivers_;
	// Holds changes made to transceivers during applying descriptors for
	// potential rollback. Gets cleared once signaling state goes to stable.
	std::map<RTCRtpTransceiverProxyRefPtr, RTCTransceiverStableState>
		transceiver_stable_states_by_transceivers_;
	// Holds remote stream ids for transceivers from stable state.
	std::map<RTCRtpTransceiverProxyRefPtr, std::vector<std::string>>
		remote_stream_ids_by_transceivers_;
};

}  // namespace webrtc



#endif // __RTC_PC_TRANSCEIVER_LIST_H__