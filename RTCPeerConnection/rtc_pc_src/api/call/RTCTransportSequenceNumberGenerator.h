//////////////////////////////////////////////////////////////////////////
//
//	author : kimi
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_API_CALL_TRANSPORT_SEQUENCE_NUMBER_GENERATOR_H__
#define __RTC_API_CALL_TRANSPORT_SEQUENCE_NUMBER_GENERATOR_H__

namespace webrtc {

class RTCTransportSequenceNumberGenerator {
public:
	virtual uint64_t GetTransportSequenceNumber() = 0;

protected:
	virtual ~RTCTransportSequenceNumberGenerator() {}
};

}


#endif	// __RTC_API_CALL_TRANSPORT_SEQUENCE_NUMBER_GENERATOR_H__