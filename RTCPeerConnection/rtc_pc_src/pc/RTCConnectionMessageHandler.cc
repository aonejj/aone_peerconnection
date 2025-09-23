//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : Replace PeerConnectionMessageHandler to RTCConnectionMessgaeHandler class for rtc media server.
//
//////////////////////////////////////////////////////////////////////////

#include "api/scoped_refptr.h"
#include "rtc_base/checks.h"
#include "rtc_base/location.h"
#include "rtc_base/synchronization/sequence_checker.h"

#include "RTCConnectionMessageHandler.h"

namespace webrtc {

enum {
	MSG_SET_SESSIONDESCRIPTION_SUCCESS = 0,
	MSG_SET_SESSIONDESCRIPTION_FAILED,
	MSG_CREATE_SESSIONDESCRIPTION_FAILED,
	MSG_GETSTATS,
	MSG_REPORT_USAGE_PATTERN,
};

struct SetSessionDescriptionMsg : public rtc::MessageData {
	explicit SetSessionDescriptionMsg(
		webrtc::SetSessionDescriptionObserver* observer)
		: observer(observer) {}

	rtc::scoped_refptr<webrtc::SetSessionDescriptionObserver> observer;
	RTCError error;
};

struct CreateSessionDescriptionMsg : public rtc::MessageData {
	explicit CreateSessionDescriptionMsg(
		webrtc::CreateSessionDescriptionObserver* observer)
		: observer(observer) {}

	rtc::scoped_refptr<webrtc::CreateSessionDescriptionObserver> observer;
	RTCError error;
};

struct RequestUsagePatternMsg : public rtc::MessageData {
	explicit RequestUsagePatternMsg(std::function<void()> func)
		: function(func) {}
	std::function<void()> function;
};

RTCConnectionMessageHandler::~RTCConnectionMessageHandler() {
	// Process all pending notifications in the message queue. If we don't do
	// this, requests will linger and not know they succeeded or failed.
	rtc::MessageList list;
	signaling_thread()->Clear(this, rtc::MQID_ANY, &list);
	for (auto& msg : list) {
		if (msg.message_id == MSG_CREATE_SESSIONDESCRIPTION_FAILED) {
			// Processing CreateOffer() and CreateAnswer() messages ensures their
			// observers are invoked even if the PeerConnection is destroyed early.
			OnMessage(&msg);
		}
		else {
			// TODO(hbos): Consider processing all pending messages. This would mean
			// that SetLocalDescription() and SetRemoteDescription() observers are
			// informed of successes and failures; this is currently NOT the case.
			delete msg.pdata;
		}
	}
}

void RTCConnectionMessageHandler::OnMessage(rtc::Message* msg) {
	RTC_DCHECK_RUN_ON(signaling_thread());
	switch (msg->message_id) {
	case MSG_SET_SESSIONDESCRIPTION_SUCCESS: {
		SetSessionDescriptionMsg* param =
			static_cast<SetSessionDescriptionMsg*>(msg->pdata);
		param->observer->OnSuccess();
		delete param;
		break;
	}
	case MSG_SET_SESSIONDESCRIPTION_FAILED: {
		SetSessionDescriptionMsg* param =
			static_cast<SetSessionDescriptionMsg*>(msg->pdata);
		param->observer->OnFailure(std::move(param->error));
		delete param;
		break;
	}
	case MSG_CREATE_SESSIONDESCRIPTION_FAILED: {
		CreateSessionDescriptionMsg* param =
			static_cast<CreateSessionDescriptionMsg*>(msg->pdata);
		param->observer->OnFailure(std::move(param->error));
		delete param;
		break;
	}
	case MSG_GETSTATS: {
		break;
	}
	case MSG_REPORT_USAGE_PATTERN: {
		break;
	}
	default:
		RTC_NOTREACHED() << "Not implemented";
		break;
	}
}

void RTCConnectionMessageHandler::PostSetSessionDescriptionSuccess(
	SetSessionDescriptionObserver* observer) {
	SetSessionDescriptionMsg* msg = new SetSessionDescriptionMsg(observer);
	signaling_thread()->Post(RTC_FROM_HERE, this,
		MSG_SET_SESSIONDESCRIPTION_SUCCESS, msg);
}

void RTCConnectionMessageHandler::PostSetSessionDescriptionFailure(
	SetSessionDescriptionObserver* observer,
	RTCError&& error) {
	RTC_DCHECK(!error.ok());
	SetSessionDescriptionMsg* msg = new SetSessionDescriptionMsg(observer);
	msg->error = std::move(error);
	signaling_thread()->Post(RTC_FROM_HERE, this,
		MSG_SET_SESSIONDESCRIPTION_FAILED, msg);
}

void RTCConnectionMessageHandler::PostCreateSessionDescriptionFailure(
	CreateSessionDescriptionObserver* observer,
	RTCError error) {
	RTC_DCHECK(!error.ok());
	CreateSessionDescriptionMsg* msg = new CreateSessionDescriptionMsg(observer);
	msg->error = std::move(error);
	signaling_thread()->Post(RTC_FROM_HERE, this,
		MSG_CREATE_SESSIONDESCRIPTION_FAILED, msg);
}

}