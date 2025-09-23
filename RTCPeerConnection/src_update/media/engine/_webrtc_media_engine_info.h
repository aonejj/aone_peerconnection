//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_UPDATE_MEDIA_ENGINE_WEBRTC_MEDIA_ENGINE_INFO_H__
#define __RTC_UPDATE_MEDIA_ENGINE_WEBRTC_MEDIA_ENGINE_INFO_H__

#include <memory>
#include <string>
#include <vector>

#include "absl/strings/string_view.h"

#include "api/rtp_parameters.h"
#include "api/transport/bitrate_settings.h"
#include "media/base/codec.h"

namespace cricket {

bool ValidateRtpExtensions(const std::vector<webrtc::RtpExtension>& extensions);

std::vector<webrtc::RtpExtension> FilterRtpExtensions(
	const std::vector<webrtc::RtpExtension>& extensions,
	bool(*supported)(absl::string_view),
	bool filter_redundant_extensions);

webrtc::BitrateConstraints GetBitrateConfigForCodec(const Codec& codec);

}


#endif // __RTC_UPDATE_MEDIA_ENGINE_WEBRTC_MEDIA_ENGINE_INFO_H__