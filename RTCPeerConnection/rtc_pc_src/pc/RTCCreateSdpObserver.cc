//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : CreateSessionDescriptionObserver Impl class
//
//////////////////////////////////////////////////////////////////////////

#include "rtc_base/logging.h"

#include "RTCCreateSdpObserver.h"

namespace webrtc {

RTCCreateSdpObserver::RTCCreateSdpObserver(RTCSdpObserver *observer) 
	:_observer(observer) {

}

RTCCreateSdpObserver::~RTCCreateSdpObserver() = default;

void RTCCreateSdpObserver::OnSuccess(SessionDescriptionInterface* desc) {
	if (_observer) {
		_observer->OnCreateSuccess(desc);
	}
}

void RTCCreateSdpObserver::OnFailure(RTCError error) {
	if (_observer) {
		_observer->OnCreateFailure(error);
	}
}

RTCSetSdpObserver::RTCSetSdpObserver(RTCSdpObserver *observer, bool is_local, std::unique_ptr<SessionDescriptionInterface> desc)
	:_observer(observer),
	 _is_local(is_local),
	 _desc(std::move(desc)) {

}

RTCSetSdpObserver::~RTCSetSdpObserver() = default;

void RTCSetSdpObserver::OnSuccess() {
	if (_observer) {
		_observer->OnSetSuccess(_is_local, _desc.get());
	}
}

void RTCSetSdpObserver::OnFailure(RTCError error) {
	if (_observer) {
		_observer->OnSetFailure(error, _is_local);
	}
}

}
