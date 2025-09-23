//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : peer connection internal
//
//////////////////////////////////////////////////////////////////////////


#ifndef __RTC_CONNECTION_INTERNAL_H__
#define __RTC_CONNECTION_INTERNAL_H__

#include <string>
#include <vector>

#include "pc/channel_interface.h"
#include "rtc_base/thread.h"
#include "rtc_base/ssl_stream_adapter.h"

#include "../../src_update/api/_peer_connection_interface_defs.h"
#include "../api/RTCConnectionInterface.h"
#include "../call/RTCCall.h"
#include "RTCConnectionMessageHandler.h"
#include "RTCRtpTransmissionManager.h"
#include "RTCChannel.h"


namespace webrtc {

class RTCDataChannelController;

// for sdp offer answer methods
class RTCConnectionInternal : public RTCConnectionInterface {
public:
	virtual ~RTCConnectionInternal() = default;

	virtual rtc::Thread* network_thread() const = 0;
	virtual rtc::Thread* signaling_thread() const = 0;
	virtual rtc::Thread* worker_thread() const = 0;

	virtual std::string session_id() const = 0;
	virtual RTCConnectionObserver* Observer() const = 0;
	virtual cricket::DataChannelType data_channel_type() const = 0;
	virtual bool GetSctpSslRole(rtc::SSLRole* role) = 0;
	virtual void SetIceConnectionState(PeerConnectionInterfaceDefs::IceConnectionState new_state) = 0;
	virtual bool IsClosed() const = 0;
	virtual const PeerConnectionInterfaceDefs::RTCConfiguration* configuration() const = 0;
	virtual bool IsUnifiedPlan() const = 0;
	virtual bool ValidateBundleSettings(const cricket::SessionDescription* desc) = 0;
	virtual absl::optional<std::string> GetDataMid() const = 0;
	virtual CryptoOptions GetCryptoOptions() = 0;
	virtual std::vector<RtpHeaderExtensionCapability> GetAudioRtpHeaderExtensions() const = 0;
	virtual std::vector<RtpHeaderExtensionCapability> GetVideoRtpHeaderExtensions() const = 0;
	virtual absl::optional<std::string> sctp_mid() = 0;
	virtual RtpTransportInternal* GetRtpTransport(const std::string& mid) = 0;
	virtual RTCCall* call_ptr() = 0;
	virtual bool SrtpRequired() const RTC_RUN_ON(signaling_thread()) = 0;
	virtual bool SetupDataChannelTransport_n(const std::string& mid) RTC_RUN_ON(network_thread()) = 0;
	virtual void SetSctpDataMid(const std::string& mid) = 0;
	virtual void TeardownDataChannelTransport_n() RTC_RUN_ON(network_thread()) = 0;
	virtual void ResetSctpDataMid() = 0;
	virtual cricket::ChannelInterface* GetChannel(const std::string& content_name) = 0;
	virtual RTCConnectionMessageHandler* message_handler() = 0;
	virtual bool dtls_enabled() const = 0;
	virtual bool IsDisableEncryption() const = 0;
	virtual const std::vector<cricket::AudioCodec>& GetSendAudioCodecs() const = 0;
	virtual const std::vector<cricket::AudioCodec>& GetRecvAudioCodecs() const = 0;
	virtual const std::vector<cricket::VideoCodec>& GetSendVideoCodecs() const = 0;
	virtual const std::vector<cricket::VideoCodec>& GetRecvVideoCodecs() const = 0;
	virtual cricket::RTCChannelManager* channel_manager() const = 0;
	virtual RTCRtpTransmissionManager* rtp_manager() = 0;
	virtual const RTCRtpTransmissionManager* rtp_manager() const = 0;
	virtual JsepTransportController* transport_controller() = 0;
	virtual RTCDataChannelController* data_channel_controller() = 0;
	virtual cricket::PortAllocator* port_allocator() = 0;
	virtual RTCErrorOr<rtc::scoped_refptr<RTCRtpTransceiverInterface>> AddTransceiver(
		cricket::MediaType media_type,
		rtc::scoped_refptr<MediaStreamTrackInterface> track,
		const RTCRtpTransceiverInit& init,
		bool fire_callback = true) = 0;
	virtual void BaseChannelConnectSentPacketSignal(cricket::RTCBaseChannel *channel) = 0;
	virtual void SctpDataChannelConnectClosedSignal(SctpDataChannel *channel) = 0;

};

}	// namespace webrtc



#endif // __RTC_CONNECTION_INTERNAL_H__