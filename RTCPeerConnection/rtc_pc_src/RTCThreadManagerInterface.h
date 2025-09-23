//////////////////////////////////////////////////////////////////////////
//
// author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_THREAD_MANAGER_INTERFACE_H__
#define __RTC_THREAD_MANAGER_INTERFACE_H__

namespace rtc {

class ThreadManager;
class Thread;
class MessageHandler;

class RTCThreadManagerInterface {
public:
	virtual rtc::ThreadManager* Instance() = 0;
	virtual void Add(rtc::Thread* message_queue) = 0;
	virtual void Remove(rtc::Thread* message_queue) = 0;
	virtual void Clear(rtc::MessageHandler* handler) = 0;
	virtual void* GetAnyInvocableMessageHandler() = 0;

};

}
#endif