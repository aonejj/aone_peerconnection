//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : peerconnection factory lite version for rtc media server.
//
//////////////////////////////////////////////////////////////////////////

#include <syscall.h>	// for __NR_gettid

#include "rtc_base/logging.h"
#include "rtc_base/checks.h"
#include "rtc_base/bind.h"

#include "api/rtc_error.h"
#include "api/task_queue/default_task_queue_factory.h"
#include "api/data_channel_interface.h"

#include "../pc/RTCConnection.h"
#include "RTCReceivedPacketSinks.h"
#include "RTCRtpSenderInterface.h"
#include "RTCConnectionProxy.h"
#include "RTCNode.h"

namespace rtc_media_server {

//////////////////////////////////////////////////////////////////////////
// RTCVideoRtpPacketLisenterProxyGroup
RTCNode::RTCVideoRtpPacketLisenterProxyGroup::RTCVideoRtpPacketLisenterProxyGroup() {
	RTC_LOG(LS_VERBOSE) << "proxy_trace RTCVideoRtpPacketLisenterProxyGroup";
}

RTCNode::RTCVideoRtpPacketLisenterProxyGroup::~RTCVideoRtpPacketLisenterProxyGroup() {
	RTC_LOG(LS_VERBOSE) << "proxy_trace ~RTCVideoRtpPacketLisenterProxyGroup";
	_proxies.clear();
}

void RTCNode::RTCVideoRtpPacketLisenterProxyGroup::SetSink(webrtc::RTCMediaReceivedPacketSink* sink) {
	for (auto it = _proxies.begin();
		it != _proxies.end(); ++it) {
		it->second->SetSink(sink);
	}
}

void RTCNode::RTCVideoRtpPacketLisenterProxyGroup::PushVideoRtpPacketListenerProxy(uint32_t ssrc, std::unique_ptr<webrtc::RTCVideoRtpPacketListenerProxy> proxy) {
//	_proxies.push_back(std::move(proxy));
	_proxies[ssrc] = std::move(proxy);
}

webrtc::RTCVideoRtpPacketListenerProxy* RTCNode::RTCVideoRtpPacketLisenterProxyGroup::GetProxy(uint32_t ssrc) {
	auto it = _proxies.find(ssrc);
	if (it == _proxies.end()) {
		return nullptr;
	}

	return it->second.get();
}


//////////////////////////////////////////////////////////////////////////
// RTCNode
rtc::scoped_refptr<RTCNode> RTCNode::Create(RTCNodeObserver *observer) {
	rtc::scoped_refptr<RTCNode> node(new rtc::RefCountedObject<RTCNode>(observer));

	webrtc::RTCError init_error = node->_initialize();
	if (!init_error.ok()) {
		RTC_LOG(LS_VERBOSE) << "RTCNode initialize failed";
		return nullptr;
	}

	return node;
}

RTCNode::RTCNode(RTCNodeObserver *observer) 
	:_observer(observer)
	, _thread_manager(nullptr)
	, _msg_handler(std::unique_ptr<rtc::RTCAnyInvocableMessageHandler>(new rtc::RTCAnyInvocableMessageHandler))
{
}

RTCNode::~RTCNode() {
	_close();
}

bool RTCNode::CreateRTCConnection(
	webrtc::PeerConnectionInterfaceDefs::RTCConfiguration &rtcConfigure) {

	if (_signaling_thread && !_signaling_thread->IsCurrent()) {
		return _signaling_thread->Invoke<bool>(
			RTC_FROM_HERE, [&] {
			return CreateRTCConnection(rtcConfigure);
		});
	}
	
	std::unique_ptr<webrtc::RTCCall> call = worker_thread()->Invoke<std::unique_ptr<webrtc::RTCCall>>(		// source
		RTC_FROM_HERE,
		rtc::Bind(&RTCNode::_create_call_w, this));

	auto result = webrtc::RTCConnection::Create(this, std::move(call),
												_task_queue_factory.get(), 
												rtcConfigure, this, this);
	if (result == nullptr)
		return false;

	_rtc_connection =
		webrtc::RTCConnectionProxy::Create(_signaling_thread.get(), std::move(result));

	_is_unified_plan = false;

	if (rtcConfigure.sdp_semantics == webrtc::SdpSemantics::kUnifiedPlan) {
		_is_unified_plan = true;
	}

	return true;
}

void RTCNode::CreateOffer() {
	if (_rtc_connection == nullptr) {
		RTC_LOG(LS_ERROR) << "["<<this<<"] RTCNode::CreateOffer rtc connection NULL";
		return;
	}

	rtc::scoped_refptr<webrtc::RTCCreateSdpObserver> 
		observer(new rtc::RefCountedObject<webrtc::RTCCreateSdpObserver>(this));
	webrtc::PeerConnectionInterfaceDefs::RTCOfferAnswerOptions options;
	options.offer_to_receive_audio = (_is_unified_plan ? webrtc::PeerConnectionInterfaceDefs::RTCOfferAnswerOptions::kUndefined :
		webrtc::PeerConnectionInterfaceDefs::RTCOfferAnswerOptions::kOfferToReceiveMediaTrue);
	options.offer_to_receive_video = (_is_unified_plan ? webrtc::PeerConnectionInterfaceDefs::RTCOfferAnswerOptions::kUndefined :
		webrtc::PeerConnectionInterfaceDefs::RTCOfferAnswerOptions::kOfferToReceiveMediaTrue);

	_rtc_connection->CreateOffer(observer, options);
}

void RTCNode::CreateAnswer() {
	if (_rtc_connection == nullptr) {
		RTC_LOG(LS_ERROR) << "["<<this<<"] RTCNode::CreateAnswer rtc connection NULL";
		return;
	}

	rtc::scoped_refptr<webrtc::RTCCreateSdpObserver> 
		observer(new rtc::RefCountedObject<webrtc::RTCCreateSdpObserver>(this));
	webrtc::PeerConnectionInterfaceDefs::RTCOfferAnswerOptions options;
	options.offer_to_receive_audio = (_is_unified_plan ? webrtc::PeerConnectionInterfaceDefs::RTCOfferAnswerOptions::kUndefined :
		webrtc::PeerConnectionInterfaceDefs::RTCOfferAnswerOptions::kOfferToReceiveMediaTrue);
	options.offer_to_receive_video = (_is_unified_plan ? webrtc::PeerConnectionInterfaceDefs::RTCOfferAnswerOptions::kUndefined :
		webrtc::PeerConnectionInterfaceDefs::RTCOfferAnswerOptions::kOfferToReceiveMediaTrue);

	_rtc_connection->CreateAnswer(observer, options);
}

void RTCNode::SetLocalDescription(webrtc::SessionDescriptionInterface *desc_ptr) {
	if (_rtc_connection == nullptr) {
		RTC_LOG(LS_ERROR) << "[" << this <<"] RTCNode::SetLocalDescription rtc connection NULL";
		return;
	}

	rtc::scoped_refptr<webrtc::RTCSetSdpObserver> 
		observer(new rtc::RefCountedObject<webrtc::RTCSetSdpObserver>(this, true, desc_ptr->Clone()));
	_rtc_connection->SetLocalDescription(observer, desc_ptr);
}

void RTCNode::SetLocalDescription(
	std::unique_ptr<webrtc::SessionDescriptionInterface> desc) {
	if (_rtc_connection == nullptr) {
		RTC_LOG(LS_ERROR) << "["<<this<<"] RTCNode::SetLocalDescription rtc connection NULL";
		return;
	}

	rtc::scoped_refptr<webrtc::RTCSetSdpObserver>
		observer(new rtc::RefCountedObject<webrtc::RTCSetSdpObserver>(this, true, desc->Clone()));

	_rtc_connection->SetLocalDescription(observer, std::move(desc));
}

void RTCNode::SetRemoteDescription(webrtc::SessionDescriptionInterface *desc_ptr) {
	if (_rtc_connection == nullptr) {
		RTC_LOG(LS_ERROR) << "["<<this<<"] RTCNode::SetRemoteDescription rtc connection NULL";
		return;
	}

	rtc::scoped_refptr<webrtc::RTCSetSdpObserver> 
		observer(new rtc::RefCountedObject<webrtc::RTCSetSdpObserver>(this, false, nullptr));
	_rtc_connection->SetRemoteDescription(observer, desc_ptr);
}

void RTCNode::SetRemoteDescription(
	std::unique_ptr<webrtc::SessionDescriptionInterface> desc) {
	if (_rtc_connection == nullptr) {
		RTC_LOG(LS_ERROR) << "["<<this<<"] RTCNode::SetRemoteDescription rtc connection NULL";
		return;
	}

	rtc::scoped_refptr<webrtc::RTCSetSdpObserver>
		observer(new rtc::RefCountedObject<webrtc::RTCSetSdpObserver>(this, false, nullptr));
	_rtc_connection->SetRemoteDescription(observer, std::move(desc));
}

void RTCNode::AddIceCandidate(
	const webrtc::IceCandidateInterface *candidate) {
	if (_rtc_connection == nullptr) {
		RTC_LOG(LS_ERROR) << "["<<this<<"] RTCNode::AddIceCandidate rtc connection NULL";
		return;
	}

	_rtc_connection->AddIceCandidate(candidate);
}

void RTCNode::AddIceCandidate(
	std::unique_ptr<webrtc::IceCandidateInterface> candidate,
	std::function<void(webrtc::RTCError)> callback) {
	// TODO....
}

rtc::scoped_refptr<rtc::RTCVideoRouteTrackSource> RTCNode::CreateVideoRouteTrackSource() {
	rtc::scoped_refptr<rtc::RTCVideoRouteTrackSource> source(new rtc::RefCountedObject<rtc::RTCVideoRouteTrackSource>());

	return source;
}

rtc::scoped_refptr<webrtc::RTCVideoRouteTrack> RTCNode::CreateVideoRouteTrack(
	std::string &track_id, webrtc::VideoTrackSourceInterface* source) {
	rtc::scoped_refptr<webrtc::RTCVideoRouteTrack> track = webrtc::RTCVideoRouteTrack::Create(track_id, source, worker_thread());

	return track;
}

rtc::scoped_refptr<rtc::RTCAudioRouteTrackSource> RTCNode::CreateAudioRouteTrackSource() {
	rtc::scoped_refptr<rtc::RTCAudioRouteTrackSource> source(new rtc::RefCountedObject<rtc::RTCAudioRouteTrackSource>());

	return source;
}

rtc::scoped_refptr<webrtc::RTCAudioRouteTrack> RTCNode::CreateAudioRouteTrack(
	std::string& track_id, webrtc::RTCAudioSourceInterface* source) {
	rtc::scoped_refptr<webrtc::RTCAudioRouteTrack> track = webrtc::RTCAudioRouteTrack::Create(track_id, source, worker_thread());

	return track;
}

rtc::scoped_refptr<webrtc::RTCRtpTransceiverInterface> RTCNode::AddTransceiver(
	rtc::scoped_refptr<webrtc::MediaStreamTrackInterface> track) {
	webrtc::RTCErrorOr<rtc::scoped_refptr<webrtc::RTCRtpTransceiverInterface>> result =
		_rtc_connection->AddTransceiver(track);

	if (!result.ok()) {
		return nullptr;
	}

	return result.MoveValue();
}

rtc::scoped_refptr<webrtc::RTCRtpTransceiverInterface> RTCNode::AddTransceiver(
	rtc::scoped_refptr<webrtc::MediaStreamTrackInterface> track,
	const webrtc::RTCRtpTransceiverInit& init) {
	webrtc::RTCErrorOr<rtc::scoped_refptr<webrtc::RTCRtpTransceiverInterface>> result =
		_rtc_connection->AddTransceiver(track, init);

	if (!result.ok()) {
		return nullptr;
	}

	return result.MoveValue();
}

rtc::scoped_refptr<webrtc::RTCRtpTransceiverInterface> RTCNode::AddTransceiver(
	cricket::MediaType media_type) {
	webrtc::RTCErrorOr<rtc::scoped_refptr<webrtc::RTCRtpTransceiverInterface>> result =
		_rtc_connection->AddTransceiver(media_type);

	if (!result.ok()) {
		return nullptr;
	}

	return result.MoveValue();
}

rtc::scoped_refptr<webrtc::RTCRtpTransceiverInterface> RTCNode::AddTransceiver(
	cricket::MediaType media_type,
	const webrtc::RTCRtpTransceiverInit& init) {
	webrtc::RTCErrorOr<rtc::scoped_refptr<webrtc::RTCRtpTransceiverInterface>> result =
		_rtc_connection->AddTransceiver(media_type, init);

	if (!result.ok()) {
		return nullptr;
	}

	return result.MoveValue();
}

rtc::scoped_refptr<webrtc::RTCRtpSenderInterface> RTCNode::AddTrack(
	rtc::scoped_refptr<webrtc::MediaStreamTrackInterface> track,
	const std::vector<std::string>& stream_ids) {
	webrtc::RTCErrorOr<rtc::scoped_refptr<webrtc::RTCRtpSenderInterface>> result = 
		_rtc_connection->AddTrack(track, stream_ids);

	if (!result.ok()) {
		RTC_LOG(LS_ERROR) << "["<<this<<"] RTCNode::AddTrack Failed to add track: " << result.error().message();
		return nullptr;
	}

	return result.MoveValue();
}

bool RTCNode::RemoveTrack(rtc::scoped_refptr<webrtc::RTCRtpSenderInterface> sender) {
	return _rtc_connection->RemoveTrack(sender).ok();
}

std::vector<rtc::scoped_refptr<webrtc::RTCRtpSenderInterface>> RTCNode::GetSenders() const {
	return _rtc_connection->GetSenders();
}

std::vector<rtc::scoped_refptr<webrtc::RTCRtpReceiverInterface>> RTCNode::GetReceivers() const {
	return _rtc_connection->GetReceivers();
}

std::vector<rtc::scoped_refptr<webrtc::RTCRtpTransceiverInterface>> RTCNode::GetTransceivers() const {
	return _rtc_connection->GetTransceivers();
}

rtc::scoped_refptr<webrtc::DataChannelInterface> RTCNode::CreateDataChannel(const std::string& label) {
	return _rtc_connection->CreateDataChannel(label, nullptr);
}

void RTCNode::SetAudioReceivedPacketSink(webrtc::RTCMediaReceivedPacketSink* sink) {
	_audio_rtp_packet_listener_proxy->SetSink(sink);
}

void RTCNode::SetVideoReceivedPacketSink(webrtc::RTCMediaReceivedPacketSink* sink) {
	auto it = _proxy_groups.find(sink);
	if (it != _proxy_groups.end()) {
		it->second->SetSink(sink);
	}
}

void RTCNode::ResetVideoReceivedPacketSink(webrtc::RTCMediaReceivedPacketSink* sink) {
	auto it = _proxy_groups.find(sink);
	if (it != _proxy_groups.end()) {
		it->second->SetSink(nullptr);
	}
}

bool RTCNode::CreateVideoReceivedPacketListenerProxyGroup(webrtc::RTCMediaReceivedPacketSink* sink,
	std::vector<uint32_t>& ssrcs) {

	_proxy_groups[sink] = std::unique_ptr<RTCVideoRtpPacketLisenterProxyGroup>(new RTCVideoRtpPacketLisenterProxyGroup());
	for (auto it : ssrcs) {
		_proxy_groups[sink]->PushVideoRtpPacketListenerProxy(it, _get_video_rtp_packet_listener_proxy(it));
	}

	return true;
}

void RTCNode::GenerateKeyFrame() {
	if (_rtc_connection != nullptr) {
		_rtc_connection->GenerateKeyFrame();
	}
}

void RTCNode::GenerateKeyFramePli(uint32_t ssrc) {
	if (_rtc_connection != nullptr) {
		_rtc_connection->GenerateKeyFramePli(ssrc);
	}
}

webrtc::RTCCall::Stats RTCNode::GetCallStats() {
	if (_rtc_connection != nullptr) {
		return _rtc_connection->GetCallStats();
	}

	return webrtc::RTCCall::Stats();
}

void RTCNode::Close() {
	_close();
}

rtc::Thread* RTCNode::signaling_thread() {
	if (_signaling_thread == nullptr) {
		return NULL;
	}

	return _signaling_thread.get();
}

rtc::Thread* RTCNode::worker_thread() {
	if (_worker_thread == nullptr) {
		return NULL;
	}

	return _worker_thread.get();
}

rtc::Thread* RTCNode::network_thread() {
	if (_network_thread == nullptr) {
		return NULL;
	}

	return _network_thread.get();
}

webrtc::RTCError RTCNode::_initialize() {
	_create_threads();

	_task_queue_factory = webrtc::CreateDefaultTaskQueueFactory();

	_call_factory = webrtc::CreateCallFactory();

	_audio_rtp_packet_listener_proxy = std::unique_ptr<webrtc::RTCAudioRtpPacketListenerProxy>(new webrtc::RTCAudioRtpPacketListenerProxy());

	return webrtc::RTCError::OK();
}

void RTCNode::_create_threads() {

	_network_thread = rtc::Thread::CreateWithSocketServer(
		(void*)((rtc::RTCThreadManagerInterface*)this)
		);
	_network_thread->SetName("network_thread", nullptr);
	RTC_CHECK(_network_thread->Start()) << "Failed to start thread";

	_worker_thread = rtc::Thread::Create(
		(void*)((rtc::RTCThreadManagerInterface*)this)
		);
	_worker_thread->SetName("worker_thread", nullptr);
	RTC_CHECK(_worker_thread->Start()) << "Failed to start thread";

	_signaling_thread = rtc::Thread::Create(
		(void*)((rtc::RTCThreadManagerInterface*)this)
		);
	_signaling_thread->SetName("signal_thread", nullptr);
	RTC_CHECK(_signaling_thread->Start()) << "Failed to start thread";
}

std::unique_ptr<webrtc::RTCCall> RTCNode::_create_call_w() {
	RTC_DCHECK_RUN_ON(worker_thread());
	return std::unique_ptr<webrtc::RTCCall>(_call_factory->CreateRTCCall(_task_queue_factory.get(), 
																		 _audio_rtp_packet_listener_proxy.get(),
																		 this
																		  , this
																		 ));
}


std::unique_ptr<webrtc::RTCVideoRtpPacketListenerProxy> 
RTCNode::_get_video_rtp_packet_listener_proxy(uint32_t ssrc) {
	std::unique_ptr<webrtc::RTCVideoRtpPacketListenerProxy> proxy;
	auto it = _video_rtp_packet_listener_proxies.find(ssrc);
	if (it == _video_rtp_packet_listener_proxies.end()) {
		return nullptr;
	}

	proxy = std::move(it->second);
	_video_rtp_packet_listener_proxies.erase(it);

	return std::move(proxy);
}

void RTCNode::_close() {
	if (_rtc_connection != nullptr) {
		_rtc_connection->Close();
		_rtc_connection = nullptr;
	}

	if (!_video_rtp_packet_listener_proxies.empty()) {
		_video_rtp_packet_listener_proxies.clear();
	}

	if (_network_thread != nullptr)
		_network_thread = nullptr;

	if (_worker_thread != nullptr)
		_worker_thread = nullptr;

	if (_signaling_thread != nullptr)
		_signaling_thread = nullptr;
}

//////////////////////////////////////////////////////////////////////////
// RTCConnectionObserver
void RTCNode::OnSignalingChange(webrtc::PeerConnectionInterfaceDefs::SignalingState new_state) {
}

void RTCNode::OnAddStream(rtc::scoped_refptr<webrtc::MediaStreamInterface> stream) {
	if (_observer) {
		_observer->OnAddStream(stream);
	}
}

void RTCNode::OnRemoveStream(rtc::scoped_refptr<webrtc::MediaStreamInterface> stream) {
	if (_observer) {
		_observer->OnRemoveStream(stream);
	}
}

void RTCNode::OnDataChannel(rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel) {
	RTC_LOG(LS_VERBOSE) << "RTCNode::OnDataChannel lable " << data_channel->label().c_str();
	if (_observer) {
		_observer->OnDataChannel(data_channel);
	}
}


void RTCNode::OnRenegotiationNeeded() {

}

void RTCNode::OnNegotiationNeededEvent(uint32_t event_id) {

}

void RTCNode::OnIceConnectionChange(
	webrtc::PeerConnectionInterfaceDefs::IceConnectionState new_state) {
	if (_observer) {
		_observer->OnIceConnectionChange(new_state);
	}
}

void RTCNode::OnStandardizedIceConnectionChange(
	webrtc::PeerConnectionInterfaceDefs::IceConnectionState new_state) {

}

void RTCNode::OnConnectionChange(
	webrtc::PeerConnectionInterfaceDefs::PeerConnectionState new_state) {

}

void RTCNode::OnIceGatheringChange(
	webrtc::PeerConnectionInterfaceDefs::IceGatheringState new_state) {

}

void RTCNode::OnIceCandidate(const webrtc::IceCandidateInterface* candidate) {
	if (candidate) {
		std::string out;
		candidate->ToString(&out);
		RTC_LOG(LS_VERBOSE) << "OnIceCandidate : " << out;
		if (_observer) {
			_observer->OnIceCandidate(candidate);
		}
	}
	else {
		RTC_LOG(LS_VERBOSE) << "OnIceCandidate : nullptr";
	}
}

void RTCNode::OnIceCandidateError(const std::string& host_candidate,
								  const std::string& url,
								  int error_code,
								  const std::string& error_text) {

}

void RTCNode::OnIceCandidateError(const std::string& address,
								  int port,
								  const std::string& url,
								  int error_code,
								  const std::string& error_text) {

}

void RTCNode::OnIceCandidatesRemoved(
	const std::vector<cricket::Candidate>& candidates) {

}

void RTCNode::OnIceConnectionReceivingChange(bool receiving) {

}

void RTCNode::OnIceSelectedCandidatePairChanged(
	const cricket::CandidatePairChangeEvent& event) {

}

void RTCNode::OnAddTrack(
	rtc::scoped_refptr<webrtc::RTCRtpReceiverInterface> receiver,
	const std::vector<rtc::scoped_refptr<webrtc::MediaStreamInterface>>& streams) {

	if (_observer) {
		_observer->OnAddTrack(receiver, streams);
	}
}

void RTCNode::OnAddSimulcastTrack(std::vector<rtc::scoped_refptr<webrtc::RTCRtpReceiverInterface>>& receivers,
	const std::vector<rtc::scoped_refptr<webrtc::MediaStreamInterface>>& streams) {

	if (_observer) {
		_observer->OnAddSimulcastTrack(receivers, streams);
	}
}

void RTCNode::OnTrack(rtc::scoped_refptr<webrtc::RTCRtpTransceiverInterface> transceiver) {
	if (_observer) {
		_observer->OnTrack(transceiver);
	}
}

//////////////////////////////////////////////////////////////////////////
// RTCSdpObserver interface
void RTCNode::OnCreateSuccess(webrtc::SessionDescriptionInterface* desc) {
	if (_observer) {
		_observer->OnSdpCreateSuccess(desc);
	}
}

void RTCNode::OnCreateFailure(webrtc::RTCError error) {
	if (_observer) {
		_observer->OnSdpCreateFailure(error);
	}
}

void RTCNode::OnSetSuccess(bool is_local, webrtc::SessionDescriptionInterface* desc) {
	if (_observer) {
		_observer->OnSdpSetSuccess(is_local, desc);
	}
}
void RTCNode::OnSetFailure(webrtc::RTCError error, bool is_local) {
	if (_observer) {
		_observer->OnSdpSetFailure(error, is_local);
	}
}

webrtc::RTCVideoRtpPacketListenSinkInterface* RTCNode::GetVideoRtpPacketProxyInterface(uint32_t ssrc) {
	for (auto it = _proxy_groups.begin();
		it != _proxy_groups.end(); ++it) {
		webrtc::RTCVideoRtpPacketListenerProxy* proxy = it->second->GetProxy(ssrc);
		if (proxy != nullptr) {
			RTC_LOG(LS_VERBOSE) << "proxy_trace RTCNode::GetVideoRtpPacketProxyInterface--- find proxy group  ssrc " << ssrc;
			return proxy;
		}
	}

	_video_rtp_packet_listener_proxies[ssrc] = 
		std::unique_ptr<webrtc::RTCVideoRtpPacketListenerProxy>(new webrtc::RTCVideoRtpPacketListenerProxy());
	
	return _video_rtp_packet_listener_proxies[ssrc].get();
}

rtc::ThreadManager* RTCNode::Instance() {
	if (_thread_manager == nullptr) {
		_thread_manager = std::unique_ptr<rtc::ThreadManager>(new rtc::ThreadManager((void*)this));
	}
	return _thread_manager.get();
}

void RTCNode::Add(rtc::Thread* message_queue) {
	_thread_manager->AddInternal(message_queue);
}

void RTCNode::Remove(rtc::Thread* message_queue) {
	_thread_manager->RemoveInternal(message_queue);
}

void RTCNode::Clear(rtc::MessageHandler* handler) {
	_thread_manager->ClearInternal(handler);
}

void*  RTCNode::GetAnyInvocableMessageHandler() {
	return (void*)_msg_handler.get();
}

}