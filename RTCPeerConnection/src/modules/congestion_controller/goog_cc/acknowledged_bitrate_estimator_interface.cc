/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/congestion_controller/goog_cc/acknowledged_bitrate_estimator_interface.h"

#include <algorithm>

#include "modules/congestion_controller/goog_cc/acknowledged_bitrate_estimator.h"
#include "modules/congestion_controller/goog_cc/robust_throughput_estimator.h"
#include "rtc_base/logging.h"

namespace webrtc {

constexpr char RobustThroughputEstimatorSettings::kKey[];

RobustThroughputEstimatorSettings::RobustThroughputEstimatorSettings(
    const WebRtcKeyValueConfig* key_value_config) {

}

AcknowledgedBitrateEstimatorInterface::
    ~AcknowledgedBitrateEstimatorInterface() {}

std::unique_ptr<AcknowledgedBitrateEstimatorInterface>
AcknowledgedBitrateEstimatorInterface::Create(
    const WebRtcKeyValueConfig* key_value_config) {
  RobustThroughputEstimatorSettings simplified_estimator_settings(
      key_value_config);
  if (simplified_estimator_settings.enabled) {
    return std::make_unique<RobustThroughputEstimator>(
        simplified_estimator_settings);
  }
  return std::make_unique<AcknowledgedBitrateEstimator>(key_value_config);
}

}  // namespace webrtc
