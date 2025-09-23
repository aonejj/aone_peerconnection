//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// 
//
//////////////////////////////////////////////////////////////////////////
#ifndef __RTC_API_MEDIA_STREAM_INTERFACE_H__
#define __RTC_API_MEDIA_STREAM_INTERFACE_H__

#include "../../src_update/api/_media_stream_interface.h"

namespace webrtc {

class RTC_EXPORT RTCAudioSourceInterface : public MediaSourceInterface {
public:
	~RTCAudioSourceInterface() = default;
};

class RTC_EXPORT RTCAudioTrackInterface : public MediaStreamTrackInterface {
public:
	virtual RTCAudioSourceInterface* GetSource() const = 0;

protected:
	~RTCAudioTrackInterface() override = default;
};

} // namespace webrtc

#endif	// __RTC_API_MEDIA_STREAM_INTERFACE_H__