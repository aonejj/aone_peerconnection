//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /api/rtp_receiver_interface.h
//
//////////////////////////////////////////////////////////////////////////


#ifndef __RTC_API_RTP_RECEIVER_INTERFACE_H__
#define __RTC_API_RTP_RECEIVER_INTERFACE_H__

#include <string>
#include <vector>

#include "api/dtls_transport_interface.h"
#include "api/media_types.h"
#include "api/proxy.h"
#include "api/rtp_parameters.h"
#include "api/scoped_refptr.h"
#include "api/transport/rtp/rtp_source.h"
#include "api/make_ref_counted.h"
#include "rtc_base/deprecation.h"
#include "rtc_base/ref_count.h"
#include "rtc_base/system/rtc_export.h"

#include "../../src_update/api/_media_stream_interface.h"


namespace webrtc {

class RTCRtpReceiverObserverInterface {
public:
	// Note: Currently if there are multiple RtpReceivers of the same media type,
	// they will all call OnFirstPacketReceived at once.
	//
	// In the future, it's likely that an RtpReceiver will only call
	// OnFirstPacketReceived when a packet is received specifically for its
	// SSRC/mid.
	virtual void OnFirstPacketReceived(cricket::MediaType media_type) = 0;

protected:
	virtual ~RTCRtpReceiverObserverInterface() {}
};

class RTC_EXPORT RTCRtpReceiverInterface : public rtc::RefCountInterface {
public:
	virtual rtc::scoped_refptr<MediaStreamTrackInterface> track() const = 0;

	// The dtlsTransport attribute exposes the DTLS transport on which the
	// media is received. It may be null.
	// https://w3c.github.io/webrtc-pc/#dom-rtcrtpreceiver-transport
	// TODO(https://bugs.webrtc.org/907849) remove default implementation
	virtual rtc::scoped_refptr<DtlsTransportInterface> dtls_transport() const;

	// The list of streams that |track| is associated with. This is the same as
	// the [[AssociatedRemoteMediaStreams]] internal slot in the spec.
	// https://w3c.github.io/webrtc-pc/#dfn-associatedremotemediastreams
	// TODO(hbos): Make pure virtual as soon as Chromium's mock implements this.
	// TODO(https://crbug.com/webrtc/9480): Remove streams() in favor of
	// stream_ids() as soon as downstream projects are no longer dependent on
	// stream objects.
	virtual std::vector<std::string> stream_ids() const;
	virtual std::vector<rtc::scoped_refptr<MediaStreamInterface>> streams() const;

	// Audio or video receiver?
	virtual cricket::MediaType media_type() const = 0;

	// Not to be confused with "mid", this is a field we can temporarily use
	// to uniquely identify a receiver until we implement Unified Plan SDP.
	virtual std::string id() const = 0;

	// The WebRTC specification only defines RTCRtpParameters in terms of senders,
	// but this API also applies them to receivers, similar to ORTC:
	// http://ortc.org/wp-content/uploads/2016/03/ortc.html#rtcrtpparameters*.
	virtual RtpParameters GetParameters() const = 0;

	virtual uint32_t ssrc() const = 0;	
	// TODO(dinosaurav): Delete SetParameters entirely after rolling to Chromium.
	// Currently, doesn't support changing any parameters.
	virtual bool SetParameters(const RtpParameters& parameters) { return false; }

	// Does not take ownership of observer.
	// Must call SetObserver(nullptr) before the observer is destroyed.
	virtual void SetObserver(RTCRtpReceiverObserverInterface* observer) = 0;

	// Sets the jitter buffer minimum delay until media playout. Actual observed
	// delay may differ depending on the congestion control. |delay_seconds| is a
	// positive value including 0.0 measured in seconds. |nullopt| means default
	// value must be used.
	virtual void SetJitterBufferMinimumDelay(
		absl::optional<double> delay_seconds) = 0;

	// TODO(zhihuang): Remove the default implementation once the subclasses
	// implement this. Currently, the only relevant subclass is the
	// content::FakeRtpReceiver in Chromium.
	virtual std::vector<RtpSource> GetSources() const;

protected:
	~RTCRtpReceiverInterface() override = default;
};

// Define proxy for RtpReceiverInterface.
// TODO(deadbeef): Move this to .cc file and out of api/. What threads methods
// are called on is an implementation detail.
BEGIN_PROXY_MAP(RTCRtpReceiver)
PROXY_PRIMARY_THREAD_DESTRUCTOR()
PROXY_CONSTMETHOD0(rtc::scoped_refptr<MediaStreamTrackInterface>, track)
PROXY_CONSTMETHOD0(rtc::scoped_refptr<DtlsTransportInterface>, dtls_transport)
PROXY_CONSTMETHOD0(std::vector<std::string>, stream_ids)
PROXY_CONSTMETHOD0(std::vector<rtc::scoped_refptr<MediaStreamInterface>>,
				   streams)
BYPASS_PROXY_CONSTMETHOD0(cricket::MediaType, media_type)
BYPASS_PROXY_CONSTMETHOD0(std::string, id)
PROXY_SECONDARY_CONSTMETHOD0(RtpParameters, GetParameters)
BYPASS_PROXY_CONSTMETHOD0(uint32_t, ssrc)
PROXY_METHOD1(void, SetObserver, RTCRtpReceiverObserverInterface*)
PROXY_METHOD1(void, SetJitterBufferMinimumDelay, absl::optional<double>)
PROXY_SECONDARY_CONSTMETHOD0(std::vector<RtpSource>, GetSources)
END_PROXY_MAP(RTCRtpReceiver)

}

#endif // __RTC_API_RTP_RECEIVER_INTERFACE_H__