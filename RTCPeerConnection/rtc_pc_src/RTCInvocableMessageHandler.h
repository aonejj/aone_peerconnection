//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_MESSAGE_HANDLER_INTERFACE_H__
#define __RTC_MESSAGE_HANDLER_INTERFACE_H__

#include "absl/functional/any_invocable.h"

namespace rtc {

class Message;
class MessageData;


struct RTCAnyInvocableMessage final : public MessageData {
	explicit RTCAnyInvocableMessage(absl::AnyInvocable<void() && > task)
		: task(std::move(task)) {}
	absl::AnyInvocable<void() && > task;
};

class RTCAnyInvocableMessageHandler final : public MessageHandler {
public:
	void OnMessage(Message* msg) override {
		std::move(static_cast<RTCAnyInvocableMessage*>(msg->pdata)->task)();
		delete msg->pdata;
	}
};

}


#endif // __RTC_MESSAGE_HANDLER_INTERFACE_H__