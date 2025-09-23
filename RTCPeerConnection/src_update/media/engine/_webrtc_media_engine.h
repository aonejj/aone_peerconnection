//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /media/engine/webrtc_media_engine.h
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_UPDATE_MEDIA_ENGINE_WEBRTC_MEDIA_ENGINE_H__
#define __RTC_UPDATE_MEDIA_ENGINE_WEBRTC_MEDIA_ENGINE_H__


#include "media/base/codec.h"
#include "rtc_base/system/rtc_export.h"
#include "../base/_media_engine.h"

namespace cricket {

RTC_EXPORT std::unique_ptr<MediaEngineInterface> CreateMediaEngine(
	webrtc::TaskQueueFactory *task_queue_factory,
	std::vector<AudioCodec> audio_decoder_codecs,
	std::vector<AudioCodec> audio_encoder_codecs,
	std::vector<VideoCodec> video_decoder_codecs,
	std::vector<VideoCodec> video_encoder_codecs);

}


#endif // __RTC_UPDATE_MEDIA_ENGINE_WEBRTC_MEDIA_ENGINE_H__