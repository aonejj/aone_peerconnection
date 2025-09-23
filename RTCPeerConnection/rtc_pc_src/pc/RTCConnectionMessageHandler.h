//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : Replace PeerConnectionMessageHandler to RTCConnectionMessgaeHandler class for rtc media server.
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_CONNECTION_MESSAGE_HANDLER_H__
#define __RTC_CONNECTION_MESSAGE_HANDLER_H__

#include <functional>

#include "api/jsep.h"
#include "api/rtc_error.h"
#include "rtc_base/message_handler.h"
#include "rtc_base/thread.h"
#include "rtc_base/thread_message.h"

namespace webrtc {

class RTCConnectionMessageHandler : public rtc::MessageHandler {
public:
	explicit RTCConnectionMessageHandler(rtc::Thread* signaling_thread)
		: signaling_thread_(signaling_thread) {}
	~RTCConnectionMessageHandler();

	// Implements MessageHandler.
	void OnMessage(rtc::Message* msg) override;
	void PostSetSessionDescriptionSuccess(
		SetSessionDescriptionObserver* observer);
	void PostSetSessionDescriptionFailure(SetSessionDescriptionObserver* observer,
		RTCError&& error);
	void PostCreateSessionDescriptionFailure(
		CreateSessionDescriptionObserver* observer,
		RTCError error);

private:
	rtc::Thread* signaling_thread() const { return signaling_thread_; }

	rtc::Thread* const signaling_thread_;
};

}




#endif // __RTC_CONNECTION_MESSAGE_HANDLER_H__