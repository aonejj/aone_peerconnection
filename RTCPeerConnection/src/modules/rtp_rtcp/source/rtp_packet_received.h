/*
 *  Copyright (c) 2016 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */
#ifndef MODULES_RTP_RTCP_SOURCE_RTP_PACKET_RECEIVED_H_
#define MODULES_RTP_RTCP_SOURCE_RTP_PACKET_RECEIVED_H_

#include <stdint.h>

#include <vector>

#include "../rtc_pc_src/_deprecate_defines.h"

#include "api/array_view.h"
#include "api/rtp_headers.h"
#include "modules/rtp_rtcp/source/rtp_packet.h"
#include "system_wrappers/include/ntp_time.h"

namespace webrtc {
// Class to hold rtp packet with metadata for receiver side.
class RtpPacketReceived : public RtpPacket {
 public:
  RtpPacketReceived();
  explicit RtpPacketReceived(const ExtensionManager* extensions);
  RtpPacketReceived(const RtpPacketReceived& packet);
  RtpPacketReceived(RtpPacketReceived&& packet);

  RtpPacketReceived& operator=(const RtpPacketReceived& packet);
  RtpPacketReceived& operator=(RtpPacketReceived&& packet);

  ~RtpPacketReceived();

  // TODO(danilchap): Remove this function when all code update to use RtpPacket
  // directly. Function is there just for easier backward compatibilty.
  void GetHeader(RTPHeader* header) const;

  // Time in local time base as close as it can to packet arrived on the
  // network.
  int64_t arrival_time_ms() const { return arrival_time_ms_; }
  void set_arrival_time_ms(int64_t time) { arrival_time_ms_ = time; }

  // Estimated from Timestamp() using rtcp Sender Reports.
  NtpTime capture_ntp_time() const { return capture_time_; }
  void set_capture_ntp_time(NtpTime time) { capture_time_ = time; }

  // Flag if packet was recovered via RTX or FEC.
  bool recovered() const { return recovered_; }
  void set_recovered(bool value) { recovered_ = value; }

  int payload_type_frequency() const { return payload_type_frequency_; }
  void set_payload_type_frequency(int value) {
    payload_type_frequency_ = value;
  }

  // Additional data bound to the RTP packet for use in application code,
  // outside of WebRTC.
  rtc::ArrayView<const uint8_t> application_data() const {
    return application_data_;
  }
  void set_application_data(rtc::ArrayView<const uint8_t> data) {
    application_data_.assign(data.begin(), data.end());
  }

  void set_key_frame() {
	  is_key_frame_ = true;
  }

  const bool get_key_frame() const {
	  return is_key_frame_;
  }

  void set_can_discontinuous() {
	  can_be_discontinuous_ = true;
  }

  const bool get_can_discontinuous() const {
	  return can_be_discontinuous_;
  }

  void set_first_packet_in_frame() {
	  is_first_packet_in_frame_ = true;
  }

  const bool get_first_packet_in_frame() const {
	  return is_first_packet_in_frame_;
  }

  void set_frame_size(uint16_t w, uint16_t h) {
	  width_ = w;
	  height_ = h;
  }

  const void get_frame_size(uint16_t &w, uint16_t &h) const {
	  w = width_;
	  h = height_;
  }

  void set_mid(std::string mid) {
	  _mid = mid;
  }

  const std::string& get_mid() const {
	  return _mid;
  }

 private:
  NtpTime capture_time_;
  int64_t arrival_time_ms_ = 0;
  int payload_type_frequency_ = 0;
  bool recovered_ = false;
  std::vector<uint8_t> application_data_;

  bool is_first_packet_in_frame_ = false;
  bool is_key_frame_ = false;
  bool can_be_discontinuous_ = false;
  uint16_t width_ = 0;
  uint16_t height_ = 0;

  std::string _mid;	

 public:
  bool		_is_video = false;
  uint32_t	_incoming_rate = 0;
  uint16_t	_bw = 3; // BandwidthUsage::kLast
};

}  // namespace webrtc
#endif  // MODULES_RTP_RTCP_SOURCE_RTP_PACKET_RECEIVED_H_
