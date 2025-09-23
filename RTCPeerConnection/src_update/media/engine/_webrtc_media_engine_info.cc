//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#include <memory>
#include <utility>

#include "absl/algorithm/container.h"
#include "absl/strings/match.h"

#include "api/transport/bitrate_settings.h"
#include "api/array_view.h"

#include "rtc_base/logging.h"

#include "_webrtc_media_engine_info.h"

namespace cricket {

	// Remove mutually exclusive extensions with lower priority.
	void DiscardRedundantExtensions(
		std::vector<webrtc::RtpExtension>* extensions,
		rtc::ArrayView<const char* const> extensions_decreasing_prio) {
		RTC_DCHECK(extensions);
		bool found = false;
		for (const char* uri : extensions_decreasing_prio) {
			auto it = absl::c_find_if(
				*extensions,
				[uri](const webrtc::RtpExtension& rhs) { return rhs.uri == uri; });
			if (it != extensions->end()) {
				if (found) {
					extensions->erase(it);
				}
				found = true;
			}
		}
	}

	bool ValidateRtpExtensions(
		const std::vector<webrtc::RtpExtension>& extensions) {
		bool id_used[1 + webrtc::RtpExtension::kMaxId] = { false };
		for (const auto& extension : extensions) {
			if (extension.id < webrtc::RtpExtension::kMinId ||
				extension.id > webrtc::RtpExtension::kMaxId) {
				RTC_LOG(LS_ERROR) << "Bad RTP extension ID: " << extension.ToString();
				return false;
			}
			if (id_used[extension.id]) {
				RTC_LOG(LS_ERROR) << "Duplicate RTP extension ID: "
					<< extension.ToString();
				return false;
			}
			id_used[extension.id] = true;
		}
		return true;
	}

	std::vector<webrtc::RtpExtension> FilterRtpExtensions(
		const std::vector<webrtc::RtpExtension>& extensions,
		bool(*supported)(absl::string_view),
		bool filter_redundant_extensions) {
		RTC_DCHECK(ValidateRtpExtensions(extensions));
		RTC_DCHECK(supported);
		std::vector<webrtc::RtpExtension> result;

		// Ignore any extensions that we don't recognize.
		for (const auto& extension : extensions) {
			if (supported(extension.uri)) {
				result.push_back(extension);
			}
			else {
				RTC_LOG(LS_WARNING) << "Unsupported RTP extension: "
					<< extension.ToString();
			}
		}

		// Sort by name, ascending (prioritise encryption), so that we don't reset
		// extensions if they were specified in a different order (also allows us
		// to use std::unique below).
		absl::c_sort(result, [](const webrtc::RtpExtension& rhs,
			const webrtc::RtpExtension& lhs) {
			return rhs.encrypt == lhs.encrypt ? rhs.uri < lhs.uri
				: rhs.encrypt > lhs.encrypt;
		});

		// Remove unnecessary extensions (used on send side).
		if (filter_redundant_extensions) {
			auto it = std::unique(
				result.begin(), result.end(),
				[](const webrtc::RtpExtension& rhs, const webrtc::RtpExtension& lhs) {
				return rhs.uri == lhs.uri && rhs.encrypt == lhs.encrypt;
			});
			result.erase(it, result.end());

			static const char* const kBweExtensionPriorities[] = {
				webrtc::RtpExtension::kTransportSequenceNumberUri,
				webrtc::RtpExtension::kAbsSendTimeUri };
			DiscardRedundantExtensions(&result, kBweExtensionPriorities);
		}
		return result;
	}

	webrtc::BitrateConstraints GetBitrateConfigForCodec(const Codec& codec) {
		webrtc::BitrateConstraints config;
		int bitrate_kbps = 0;
		if (codec.GetParam(kCodecParamMinBitrate, &bitrate_kbps) &&
			bitrate_kbps > 0) {
			config.min_bitrate_bps = bitrate_kbps * 1000;
		}
		else {
			config.min_bitrate_bps = 0;
		}
		if (codec.GetParam(kCodecParamStartBitrate, &bitrate_kbps) &&
			bitrate_kbps > 0) {
			config.start_bitrate_bps = bitrate_kbps * 1000;
		}
		else {
			// Do not reconfigure start bitrate unless it's specified and positive.
			config.start_bitrate_bps = -1;
		}
		if (codec.GetParam(kCodecParamMaxBitrate, &bitrate_kbps) &&
			bitrate_kbps > 0) {
			config.max_bitrate_bps = bitrate_kbps * 1000;
		}
		else {
			config.max_bitrate_bps = -1;
		}
		return config;
	}

}