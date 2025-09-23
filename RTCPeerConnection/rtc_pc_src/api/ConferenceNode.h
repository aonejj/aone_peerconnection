//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : Pure Abstract Node for rtc media server.
//			TODO... impl
//
//////////////////////////////////////////////////////////////////////////

#ifndef __CONFERENCE_NODE_H__
#define __CONFERENCE_NODE_H__

#include "rtc_base/ref_count.h"

namespace rtc_media_server {

class ConferenceNode : public rtc::RefCountInterface {
public:
	ConferenceNode() {}
	virtual ~ConferenceNode() {}

public:
	// TODO.... abstract method...

public:
	static void WrapCurrentThread();
};

}

#endif	// __CONFERENCE_NODE_H__