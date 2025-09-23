//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : peerconnection factory lite version for rtc media server.
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_NODE_H__
#define __RTC_NODE_H__

#include <stdint.h>
#include <memory>
#include <functional>
#include <string>
#include <vector>
#include <map>

#include "../_deprecate_defines.h"

#include "api/rtc_error.h"
#include "api/scoped_refptr.h"
#include "api/task_queue/task_queue_factory.h"
#include "rtc_base/thread.h"
#include "rtc_base/message_handler.h"
#include "rtc_base/thread_message.h"


#include "../../src_update/api/_media_stream_interface.h"
#include "../../src_update/api/_peer_connection_interface_defs.h"

#include "_rtc_media_stream_interface.h"
#include "ConferenceNode.h"
#include "RTCConnectionInterface.h"
#include "ThreadsInfoInterface.h"
#include "RTCRtpReceiverInterface.h"
#include "./call/RTCCallFactoryInterface.h"
#include "./audio/RTCAudioRtpPacketListenerProxy.h"
#include "./video/RTCVideoRtpPacketListenerProxy.h"
#include "RTCNodeObserver.h"
#include "../call/RTCCall.h"
#include "../media/base/RTCVideoRouteTrackSource.h"
#include "../media/base/RTCAudioRouteTrackSource.h"
#include "../pc/RTCVideoRouteTrack.h"
#include "../pc/RTCAudioRouteTrack.h"
#include "../pc/RTCCreateSdpObserver.h"

#include "../RTCThreadManagerInterface.h"

#include "../RTCInvocableMessageHandler.h"


namespace webrtc {
class RTCRtpSenderInterface;
//class RTCAudioReceivedPacketSink;
//class RTCVideoReceivedPacketSink;
class RTCMediaReceivedPacketSink;
class DataChannelInterface;
}

namespace rtc_media_server {

class RTCNode : public ConferenceNode, 
				public webrtc::ThreadsInfoInterface,
				public webrtc::RTCConnectionObserver,
				public webrtc::RTCSdpObserver,
				public webrtc::RTCVideeRtpPacketListenerProxyGetInterface,
				public rtc::RTCThreadManagerInterface {
public:
	static rtc::scoped_refptr<RTCNode> Create(RTCNodeObserver *observer);
	bool CreateRTCConnection(webrtc::PeerConnectionInterfaceDefs::RTCConfiguration &rtcConfigure);

	void CreateOffer();
	void CreateAnswer();
	void SetLocalDescription(webrtc::SessionDescriptionInterface *desc_ptr);
	void SetLocalDescription(
		std::unique_ptr<webrtc::SessionDescriptionInterface> desc);
	void SetRemoteDescription(webrtc::SessionDescriptionInterface *desc_ptr);
	void SetRemoteDescription(
		std::unique_ptr<webrtc::SessionDescriptionInterface> desc);
	void AddIceCandidate(
		const webrtc::IceCandidateInterface *candidate);
	void AddIceCandidate(
		std::unique_ptr<webrtc::IceCandidateInterface> candidate,
		std::function<void(webrtc::RTCError)> callback);

	rtc::scoped_refptr<rtc::RTCVideoRouteTrackSource> CreateVideoRouteTrackSource();
	rtc::scoped_refptr<webrtc::RTCVideoRouteTrack> CreateVideoRouteTrack(
						std::string &track_id, webrtc::VideoTrackSourceInterface* source);

	rtc::scoped_refptr<rtc::RTCAudioRouteTrackSource> CreateAudioRouteTrackSource();
	rtc::scoped_refptr<webrtc::RTCAudioRouteTrack> CreateAudioRouteTrack(
						std::string& track_id, webrtc::RTCAudioSourceInterface* source);

	rtc::scoped_refptr<webrtc::RTCRtpTransceiverInterface> AddTransceiver(
		rtc::scoped_refptr<webrtc::MediaStreamTrackInterface> track);
	rtc::scoped_refptr<webrtc::RTCRtpTransceiverInterface> AddTransceiver(
		rtc::scoped_refptr<webrtc::MediaStreamTrackInterface> track,
		const webrtc::RTCRtpTransceiverInit& init);
	rtc::scoped_refptr<webrtc::RTCRtpTransceiverInterface> AddTransceiver(
		cricket::MediaType media_type);
	rtc::scoped_refptr<webrtc::RTCRtpTransceiverInterface> AddTransceiver(
		cricket::MediaType media_type, 
		const webrtc::RTCRtpTransceiverInit& init);

	rtc::scoped_refptr<webrtc::RTCRtpSenderInterface> AddTrack(
		rtc::scoped_refptr<webrtc::MediaStreamTrackInterface> track,
		const std::vector<std::string>& stream_ids);
	bool RemoveTrack(rtc::scoped_refptr<webrtc::RTCRtpSenderInterface> sender);

	std::vector<rtc::scoped_refptr<webrtc::RTCRtpSenderInterface>> GetSenders() const;
	std::vector<rtc::scoped_refptr<webrtc::RTCRtpReceiverInterface>> GetReceivers() const;
	std::vector<rtc::scoped_refptr<webrtc::RTCRtpTransceiverInterface>> GetTransceivers() const;

	rtc::scoped_refptr<webrtc::DataChannelInterface> CreateDataChannel(const std::string& label);

	void SetAudioReceivedPacketSink(webrtc::RTCMediaReceivedPacketSink* sink);
	void SetVideoReceivedPacketSink(webrtc::RTCMediaReceivedPacketSink* sink);
	void ResetVideoReceivedPacketSink(webrtc::RTCMediaReceivedPacketSink* sink);
	bool CreateVideoReceivedPacketListenerProxyGroup(webrtc::RTCMediaReceivedPacketSink* sink,
													 std::vector<uint32_t>& ssrcs);

	void GenerateKeyFrame();
	void GenerateKeyFramePli(uint32_t ssrc);

	webrtc::RTCCall::Stats GetCallStats();

	void Close();

public:
	rtc::Thread* signaling_thread() override;
	rtc::Thread* worker_thread() override;
	rtc::Thread* network_thread() override;

	// RTCConnectionObserver
	void OnSignalingChange(webrtc::PeerConnectionInterfaceDefs::SignalingState new_state) override;
	void OnAddStream(rtc::scoped_refptr<webrtc::MediaStreamInterface> stream) override;
	void OnRemoveStream(rtc::scoped_refptr<webrtc::MediaStreamInterface> stream) override;
	void OnDataChannel(rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel) override;
	void OnRenegotiationNeeded() override;
	void OnNegotiationNeededEvent(uint32_t event_id) override;
	void OnIceConnectionChange(
		webrtc::PeerConnectionInterfaceDefs::IceConnectionState new_state) override;
	void OnStandardizedIceConnectionChange(
		webrtc::PeerConnectionInterfaceDefs::IceConnectionState new_state) override;
	void OnConnectionChange(
		webrtc::PeerConnectionInterfaceDefs::PeerConnectionState new_state) override;
	void OnIceGatheringChange(
		webrtc::PeerConnectionInterfaceDefs::IceGatheringState new_state) override;
	void OnIceCandidate(const webrtc::IceCandidateInterface* candidate) override;
	void OnIceCandidateError(const std::string& host_candidate,
		const std::string& url,
		int error_code,
		const std::string& error_text) override;
	void OnIceCandidateError(const std::string& address,
		int port,
		const std::string& url,
		int error_code,
		const std::string& error_text) override;
	void OnIceCandidatesRemoved(
		const std::vector<cricket::Candidate>& candidates) override;
	void OnIceConnectionReceivingChange(bool receiving) override;
	void OnIceSelectedCandidatePairChanged(
		const cricket::CandidatePairChangeEvent& event) override;

	void OnAddTrack(
		rtc::scoped_refptr<webrtc::RTCRtpReceiverInterface> receiver,
		const std::vector<rtc::scoped_refptr<webrtc::MediaStreamInterface>>& streams) override;

	void OnAddSimulcastTrack(std::vector<rtc::scoped_refptr<webrtc::RTCRtpReceiverInterface>>& receivers,
			const std::vector<rtc::scoped_refptr<webrtc::MediaStreamInterface>>& streams) override;

	void OnTrack(rtc::scoped_refptr<webrtc::RTCRtpTransceiverInterface> transceiver) override;

	// RTCSdpObserver 
	void OnCreateSuccess(webrtc::SessionDescriptionInterface* desc) override;
	void OnCreateFailure(webrtc::RTCError error) override;
	void OnSetSuccess(bool is_local, webrtc::SessionDescriptionInterface* desc) override;
	void OnSetFailure(webrtc::RTCError error, bool is_local) override;

	// RTCVideeRtpPacketListenerProxyGetInterface
	webrtc::RTCVideoRtpPacketListenSinkInterface* GetVideoRtpPacketProxyInterface(uint32_t ssrc) override;

	// RTCThreadManagerInterface
	rtc::ThreadManager* Instance() override;
	void Add(rtc::Thread* message_queue) override;
	void Remove(rtc::Thread* message_queue) override;
	void Clear(rtc::MessageHandler* handler) override;
	void* GetAnyInvocableMessageHandler() override;

protected:
	RTCNode(RTCNodeObserver *observer);
	virtual ~RTCNode() override;

private:
	class RTCVideoRtpPacketLisenterProxyGroup {
	public:
		RTCVideoRtpPacketLisenterProxyGroup();
		~RTCVideoRtpPacketLisenterProxyGroup();

	public:
		void SetSink(webrtc::RTCMediaReceivedPacketSink* sink);
		void PushVideoRtpPacketListenerProxy(uint32_t ssrc, std::unique_ptr<webrtc::RTCVideoRtpPacketListenerProxy> proxy);
		webrtc::RTCVideoRtpPacketListenerProxy* GetProxy(uint32_t ssrc);

	private:
		std::map<uint32_t, std::unique_ptr<webrtc::RTCVideoRtpPacketListenerProxy>> _proxies;
	};

private:
	webrtc::RTCError _initialize();
	void _create_threads();
	std::unique_ptr<webrtc::RTCCall> _create_call_w();
	std::unique_ptr<webrtc::RTCVideoRtpPacketListenerProxy> _get_video_rtp_packet_listener_proxy(uint32_t ssrc);
	void _close();

private:
	// threads
	std::unique_ptr<rtc::Thread> _network_thread;
	std::unique_ptr<rtc::Thread> _worker_thread;
	std::unique_ptr<rtc::Thread> _signaling_thread;
	std::unique_ptr<webrtc::TaskQueueFactory> _task_queue_factory;
	std::unique_ptr<webrtc::RTCCallFactoryInterface> _call_factory;

	std::unique_ptr<webrtc::RTCAudioRtpPacketListenerProxy> _audio_rtp_packet_listener_proxy;
	
	std::map<uint32_t, std::unique_ptr<webrtc::RTCVideoRtpPacketListenerProxy>> _video_rtp_packet_listener_proxies;

	std::map<webrtc::RTCMediaReceivedPacketSink*, std::unique_ptr<RTCVideoRtpPacketLisenterProxyGroup>> _proxy_groups;

	RTCNodeObserver	*_observer;

	rtc::scoped_refptr<webrtc::RTCConnectionInterface> _rtc_connection;

	bool _is_unified_plan;

	std::unique_ptr<rtc::ThreadManager> _thread_manager;
	std::unique_ptr<rtc::RTCAnyInvocableMessageHandler> _msg_handler;
};

}

#endif // __RTC_NODE_H__