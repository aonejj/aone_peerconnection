//////////////////////////////////////////////////////////////////////////
//
//	author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_API_CALL_GET_NETWORK_ESTIMATE_INTERFACE_H__
#define __RTC_API_CALL_GET_NETWORK_ESTIMATE_INTERFACE_H__

namespace webrtc {

// ... unused
class RTCGetNetworkEstimateInterface {
public:
	virtual uint32_t GetReceiveNetworkEstimate() = 0;
	virtual uint32_t GetSendNetworkEstimate() = 0;

protected:
	virtual ~RTCGetNetworkEstimateInterface() {}
};

}

#endif // __RTC_API_CALL_GET_NETWORK_ESTIMATE_INTERFACE_H__