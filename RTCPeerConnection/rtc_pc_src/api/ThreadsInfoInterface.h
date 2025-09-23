//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : thread info interface abstract class for rtc media server.
//
//////////////////////////////////////////////////////////////////////////

#ifndef __THREAD_INFO_INTERFACE_H__
#define __THREAD_INFO_INTERFACE_H__

#include "rtc_base/thread.h"

namespace webrtc {

class ThreadsInfoInterface {
public:
	virtual rtc::Thread* signaling_thread() = 0;
	virtual rtc::Thread* worker_thread() = 0;
	virtual rtc::Thread* network_thread() = 0;
};

}


#endif // __THREAD_INFO_INTERFACE_H__