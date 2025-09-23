//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /api/rtp_sender_interface.h
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_API_RTP_SENDER_INTERFACE_H__
#define __RTC_API_RTP_SENDER_INTERFACE_H__

#include <string>
#include <vector>

#include "api/dtls_transport_interface.h"
#include "api/media_types.h"
#include "api/proxy.h"
#include "api/rtc_error.h"
#include "api/rtp_parameters.h"
#include "api/scoped_refptr.h"
#include "api/make_ref_counted.h"
#include "rtc_base/ref_count.h"
#include "rtc_base/system/rtc_export.h"

#include "../../src_update/api/_media_stream_interface.h"


namespace webrtc {

class RTC_EXPORT RTCRtpSenderInterface : public rtc::RefCountInterface {
public:
// Returns true if successful in setting the track.
// Fails if an audio track is set on a video RtpSender, or vice-versa.
virtual bool SetTrack(MediaStreamTrackInterface* track) = 0;
virtual rtc::scoped_refptr<MediaStreamTrackInterface> track() const = 0;

// The dtlsTransport attribute exposes the DTLS transport on which the
// media is sent. It may be null.
// https://w3c.github.io/webrtc-pc/#dom-rtcrtpsender-transport
// TODO(https://bugs.webrtc.org/907849) remove default implementation
virtual rtc::scoped_refptr<DtlsTransportInterface> dtls_transport() const;

// Returns primary SSRC used by this sender for sending media.
// Returns 0 if not yet determined.
// TODO(deadbeef): Change to absl::optional.
// TODO(deadbeef): Remove? With GetParameters this should be redundant.
virtual uint32_t ssrc() const = 0;

// Audio or video sender?
virtual cricket::MediaType media_type() const = 0;

// Not to be confused with "mid", this is a field we can temporarily use
// to uniquely identify a receiver until we implement Unified Plan SDP.
virtual std::string id() const = 0;

// Returns a list of media stream ids associated with this sender's track.
// These are signalled in the SDP so that the remote side can associate
// tracks.
virtual std::vector<std::string> stream_ids() const = 0;

// Sets the IDs of the media streams associated with this sender's track.
// These are signalled in the SDP so that the remote side can associate
// tracks.
virtual void SetStreams(const std::vector<std::string>& stream_ids) {}

// Returns the list of encoding parameters that will be applied when the SDP
// local description is set. These initial encoding parameters can be set by
// PeerConnection::AddTransceiver, and later updated with Get/SetParameters.
// TODO(orphis): Make it pure virtual once Chrome has updated
virtual std::vector<RtpEncodingParameters> init_send_encodings() const;

virtual RtpParameters GetParameters() const = 0;
// Note that only a subset of the parameters can currently be changed. See
// rtpparameters.h
// The encodings are in increasing quality order for simulcast.
virtual RTCError SetParameters(const RtpParameters& parameters) = 0;

virtual void SetOutSsrc(uint32_t ssrc) = 0;	
virtual void SetOutRtxSsrc(uint32_t rtx_ssrc) = 0;	
virtual absl::optional<uint32_t> GetOutSsrc() const = 0;
virtual absl::optional<uint32_t> GetOutRtxSsrc() const = 0;

protected:
	~RTCRtpSenderInterface() override = default;
};

// Define proxy for RtpSenderInterface.
// TODO(deadbeef): Move this to .cc file and out of api/. What threads methods
// are called on is an implementation detail.
BEGIN_PRIMARY_PROXY_MAP(RTCRtpSender)
PROXY_PRIMARY_THREAD_DESTRUCTOR()
PROXY_METHOD1(bool, SetTrack, MediaStreamTrackInterface*)
PROXY_CONSTMETHOD0(rtc::scoped_refptr<MediaStreamTrackInterface>, track)
PROXY_CONSTMETHOD0(rtc::scoped_refptr<DtlsTransportInterface>, dtls_transport)
PROXY_CONSTMETHOD0(uint32_t, ssrc)
BYPASS_PROXY_CONSTMETHOD0(cricket::MediaType, media_type)
BYPASS_PROXY_CONSTMETHOD0(std::string, id)
PROXY_CONSTMETHOD0(std::vector<std::string>, stream_ids)
PROXY_CONSTMETHOD0(std::vector<RtpEncodingParameters>, init_send_encodings)
PROXY_CONSTMETHOD0(RtpParameters, GetParameters)
PROXY_METHOD1(RTCError, SetParameters, const RtpParameters&)
PROXY_METHOD1(void, SetStreams, const std::vector<std::string>&)
PROXY_METHOD1(void, SetOutSsrc, uint32_t)						
PROXY_METHOD1(void, SetOutRtxSsrc, uint32_t)					
PROXY_CONSTMETHOD0(absl::optional<uint32_t>, GetOutSsrc)		
PROXY_CONSTMETHOD0(absl::optional<uint32_t>, GetOutRtxSsrc)		
END_PROXY_MAP(RTCRtpSender)

}  // namespace webrtc



#endif	// __RTC_API_RTP_SENDER_INTERFACE_H__