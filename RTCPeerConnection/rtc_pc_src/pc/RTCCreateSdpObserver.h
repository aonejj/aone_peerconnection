//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : CreateSessionDescriptionObserver Impl class
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_SDP_OBSERVER_H__
#define __RTC_SDP_OBSERVER_H__

#include <memory>

#include "api/rtc_error.h"
#include "api/jsep.h"


namespace webrtc {

class RTCSdpObserver {
public:
	RTCSdpObserver() = default;
	~RTCSdpObserver() = default;

public:
	virtual void OnCreateSuccess(SessionDescriptionInterface* desc) = 0;
	virtual void OnCreateFailure(RTCError error) = 0;
	virtual void OnSetSuccess(bool is_local, SessionDescriptionInterface* desc) = 0;
	virtual void OnSetFailure(RTCError error, bool is_local) = 0;
};

	 
class RTCCreateSdpObserver : public CreateSessionDescriptionObserver {
public:
	RTCCreateSdpObserver(RTCSdpObserver *observer);
	~RTCCreateSdpObserver() override;

public:
	void OnSuccess(SessionDescriptionInterface* desc) override;
	void OnFailure(RTCError error) override;

private:
	RTCSdpObserver* _observer;
};

class RTCSetSdpObserver : public SetSessionDescriptionObserver {
public:
	RTCSetSdpObserver(RTCSdpObserver *observer, bool is_local, std::unique_ptr<SessionDescriptionInterface> desc);
	~RTCSetSdpObserver();

public:
	void OnSuccess() override;
	void OnFailure(RTCError error) override;

private:
	RTCSdpObserver* _observer;
	bool			_is_local;
	std::unique_ptr<SessionDescriptionInterface> _desc;
};

}


#endif //__RTC_SDP_OBSERVER_H__