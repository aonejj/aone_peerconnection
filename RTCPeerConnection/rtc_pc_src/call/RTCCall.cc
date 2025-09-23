//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /call/call.cc
//
//////////////////////////////////////////////////////////////////////////
#include <syscall.h>	// for __NR_gettid

#include "../rtc_pc_src/_deprecate_defines.h"

#include "absl/functional/bind_front.h"

#include "rtc_base/thread.h"
#include "rtc_base/strings/string_builder.h"
#include "rtc_base/logging.h"
#include "rtc_base/time_utils.h"
#include "modules/rtp_rtcp/source/rtp_utility.h"
#include "modules/rtp_rtcp/source/rtp_packet_received.h"
#include "modules/utility/include/process_thread.h"

#include "RTCCall.h"
#include "RTCFlexFecReceiveStreamImpl.h"
#include "../video/RTCVideoSendStreamImpl.h"
#include "../audio/RTCAudioSendStreamImpl.h"
#include "../api/video/RTCVideoRtpPacketListenerProxy.h"
#include "../api/audio/RTCAudioRtpPacketListenerProxy.h"


namespace webrtc {

bool SendPeriodicFeedback(const std::vector<RtpExtension>& extensions) {
	for (const auto& extension : extensions) {
		if (extension.uri == RtpExtension::kTransportSequenceNumberV2Uri)
			return false;
	}
	return true;
}

bool IsRtcp(const uint8_t* packet, size_t length) {
	RtpUtility::RtpHeaderParser rtp_parser(packet, length);
	return rtp_parser.RTCP();
}

static TaskQueueBase* GetCurrentTaskQueueOrThread(rtc::RTCThreadManagerInterface* rtc_thread_manager) {
	TaskQueueBase* current = TaskQueueBase::Current();
	if (!current) {
		current = rtc_thread_manager->Instance()->CurrentThread();
	}
	return current;
}

std::string RTCCall::Stats::ToString(int64_t time_ms) const {
	char buf[1024];
	rtc::SimpleStringBuilder ss(buf);

 	ss << "Call stats: " << time_ms << ", {";
 	ss << "send_bw_bps: " << send_bandwidth_bps << ", ";
 	ss << "recv_bw_bps: " << recv_bandwidth_bps << ", ";
 	ss << "pacer_delay_ms: " << pacer_delay_ms << ", ";
 	ss << " rtt_ms: " << rtt_ms;
 	ss << '}';
	return ss.str();
}

RTCCall* RTCCall::Create(
	TaskQueueFactory* task_queue_factory,
	RTCAudioRtpPacketListenSinkInterface* audio_rtp_packet_listener,
	RTCVideeRtpPacketListenerProxyGetInterface* video_rtp_packet_listener_proxy_get_interface
	, rtc::RTCThreadManagerInterface* rtc_thread_manager
	) {
	Clock* clock = Clock::GetRealTimeClock();

	BitrateConstraints bitrate_const;
	bitrate_const.max_bitrate_bps = RTC_kDefualMaxBitrateBps;
	bitrate_const.min_bitrate_bps = RTC_kDefaultMinBitrateBps;

	return new RTCCall(clock, 
					   task_queue_factory,
					   std::make_unique<RTCRtpTransportControllerSend>(
						clock,
						bitrate_const,
						task_queue_factory
					   ),
					   audio_rtp_packet_listener,
					   video_rtp_packet_listener_proxy_get_interface,
					   rtc_thread_manager
					   );
}

RTCCall::RTCCall(Clock* clock,
	TaskQueueFactory* task_queue_factory,
	std::unique_ptr<RTCRtpTransportControllerSendInterface> transport_send,
	RTCAudioRtpPacketListenSinkInterface *audio_rtp_packet_listener,
	RTCVideeRtpPacketListenerProxyGetInterface* video_rtp_packet_listener_proxy_get_interface,
     rtc::RTCThreadManagerInterface* rtc_thread_manager
	)
	: clock_(clock),
	task_queue_factory_(task_queue_factory),
	worker_thread_(GetCurrentTaskQueueOrThread(rtc_thread_manager)),
	network_thread_(worker_thread_),
	audio_network_state_(kNetworkDown),
	video_network_state_(kNetworkDown),
	aggregate_network_up_(false),
	received_bytes_per_second_counter_(clock_, nullptr, true),
	received_audio_bytes_per_second_counter_(clock_, nullptr, true),
	received_video_bytes_per_second_counter_(clock_, nullptr, true),
	received_rtcp_bytes_per_second_counter_(clock_, nullptr, true),
	last_bandwidth_bps_(0),
	receive_side_cc_(clock_, 
		absl::bind_front(&RTCPacketRouter::SendCombinedRtcpPacket,
						 transport_send->packet_router()),
		absl::bind_front(&RTCPacketRouter::SendRemb,
						 transport_send->packet_router())	),
	receive_time_calculator_(nullptr),
	call_stats_(new CallStats(clock_, worker_thread_)),
	start_ms_(clock_->TimeInMilliseconds()),
	audio_rtp_packet_listener_(audio_rtp_packet_listener),
	video_rtp_packet_listener_proxy_get_interface_(video_rtp_packet_listener_proxy_get_interface),
	transport_send_ptr_(transport_send.get()),
	transport_send_(std::move(transport_send)),
	transport_sequence_number_(0) 
	, _rtc_thread_manager(rtc_thread_manager)
{

	RTC_DCHECK(network_thread_);
	RTC_DCHECK(worker_thread_->IsCurrent());

	sent_packet_sequence_checker_.Detach();

	call_stats_->RegisterStatsObserver(&receive_side_cc_);

	RTCReceiveSideCongestionController* receive_side_cc = &receive_side_cc_;
	receive_side_cc_periodic_task_ = RepeatingTaskHandle::Start(
		worker_thread_,
		[receive_side_cc] { return receive_side_cc->MaybeProcess(); },
		TaskQueueBase::DelayPrecision::kLow, clock_);
}

RTCCall::~RTCCall() {
	RTC_DCHECK_RUN_ON(worker_thread_);

	RTC_CHECK(audio_send_ssrcs_.empty());
	RTC_CHECK(video_send_ssrcs_.empty());
	RTC_CHECK(video_send_streams_.empty());
	RTC_CHECK(audio_receive_streams_.empty());
	RTC_CHECK(video_receive_streams_.empty());

	receive_side_cc_periodic_task_.Stop();

	call_stats_->DeregisterStatsObserver(&receive_side_cc_);
}

void RTCCall::EnsureStarted() {
	if (is_started_) {
		return;
	}
	is_started_ = true;

	transport_send_ptr_->RegisterTargetTransferRateObserver(this);
	transport_send_ptr_->EnsureStarted();
}

PacketReceiver* RTCCall::Receiver() {
	return this;
}

RTCAudioSendStream* RTCCall::CreateAudioSendStream(
	const RTCAudioSendStream::Config& config)
{
	RTC_DCHECK_RUN_ON(worker_thread_);
	EnsureStarted();

	const_cast<RTCAudioSendStream::Config*>(&config)->transport_sequence_number_generator = this;


	RTCAudioSendStream* send_stream = new RTCAudioSendStreamImpl(
		clock_,
		config,
		transport_send_ptr_,
		call_stats_->AsRtcpRttStats()	);

	RTC_DCHECK(audio_send_ssrcs_.find(config.rtp.ssrc) ==
		audio_send_ssrcs_.end());
	audio_send_ssrcs_[config.rtp.ssrc] = send_stream;


	UpdateAggregateNetworkState();
	return send_stream;
}

void RTCCall::DestroyAudioSendStream(RTCAudioSendStream* send_stream) {
	RTC_DCHECK_RUN_ON(worker_thread_);
	RTC_DCHECK(send_stream != nullptr);

	send_stream->Stop();

	const uint32_t ssrc = send_stream->GetConfig().rtp.ssrc;

	audio_send_ssrcs_.erase(ssrc);
	RTCAudioSendStreamImpl* send_stream_impl = static_cast<RTCAudioSendStreamImpl*>(send_stream);
	send_stream_impl->StopPermanently();

	UpdateAggregateNetworkState();
	delete send_stream;
}

RTCAudioReceiveStream* RTCCall::CreateAudioReceiveStream(
	RTCAudioReceiveStream::Config& config) {
	RTC_DCHECK_RUN_ON(worker_thread_);

	EnsureStarted();

	RTCAudioReceiveStreamImpl* receive_stream = new RTCAudioReceiveStreamImpl(
		clock_,
		transport_send_ptr_,
		&audio_receiver_controller_, 
		config,
		audio_rtp_packet_listener_);

#ifdef __ROUTING_PACKET_PATH_USING_NETWORK_THREAD__
	{
		MutexLock lock(&recv_rtc_config_mutex_);
		receive_rtp_config_.emplace(config.rtp.remote_ssrc, ReceiveRtpConfig(config));
	}
#else
	receive_rtp_config_.emplace(config.rtp.remote_ssrc, ReceiveRtpConfig(config));
#endif

	((RTCAudioRtpPacketListenerProxy*)audio_rtp_packet_listener_)->SetRtpHeaderExtensionMap(RtpHeaderExtensionMap(config.rtp.extensions));
	audio_receive_streams_.insert(receive_stream);

	UpdateAggregateNetworkState();

	return receive_stream;
}

void RTCCall::DestroyAudioReceiveStream(RTCAudioReceiveStream* receive_stream) {
	RTC_DCHECK_RUN_ON(worker_thread_);
	RTC_DCHECK(receive_stream != nullptr);

	RTCAudioReceiveStreamImpl* audio_receive_stream =
		static_cast<RTCAudioReceiveStreamImpl*>(receive_stream);

	const RTCAudioReceiveStream::Config& config = audio_receive_stream->config();
	uint32_t ssrc = config.rtp.remote_ssrc;

	receive_side_cc_.RemoveStream(ssrc);

	audio_receive_streams_.erase(audio_receive_stream);

#ifdef __ROUTING_PACKET_PATH_USING_NETWORK_THREAD__
	{
		MutexLock lock(&recv_rtc_config_mutex_);
		receive_rtp_config_.erase(ssrc);
	}
#else
	receive_rtp_config_.erase(ssrc);
#endif

	UpdateAggregateNetworkState();
	
	delete audio_receive_stream;	
}

RTCVideoSendStream* RTCCall::CreateVideoSendStream(
	RTCVideoSendStream::Config config, VideoEncoderConfig encoder_config) {
	EnsureStarted();

	config.transport_sequence_number_generator = this;

	std::vector<uint32_t> ssrcs = config.rtp.ssrcs;

	RTCVideoSendStream *send_stream = new RTCVideoSendStreamImpl(clock_,
		std::move(config),
		transport_send_ptr_,
		call_stats_->AsRtcpRttStats());

	for (uint32_t ssrc : ssrcs) {
		RTC_DCHECK(video_send_ssrcs_.find(ssrc) == video_send_ssrcs_.end());
		video_send_ssrcs_[ssrc] = send_stream;
	}

	video_send_streams_.insert(send_stream);


	UpdateAggregateNetworkState();
	return send_stream;
}

void RTCCall::DestroyVideoSendStream(RTCVideoSendStream* send_stream) {
	RTC_DCHECK(send_stream != nullptr);
	RTC_DCHECK_RUN_ON(worker_thread_);

	send_stream->Stop();

	RTCVideoSendStreamImpl* send_stream_impl = nullptr;
	auto it = video_send_ssrcs_.begin();
	while (it != video_send_ssrcs_.end()) {
		if (it->second == send_stream) {
			send_stream_impl = static_cast<RTCVideoSendStreamImpl*>(it->second);
			video_send_ssrcs_.erase(it++);
			break;
		}
		else {
			++it;
		}
	}

	video_send_streams_.erase(send_stream_impl);
	RTC_CHECK(send_stream_impl != nullptr);
	send_stream_impl->StopPermanently();
	UpdateAggregateNetworkState();

	delete send_stream_impl;	
}

RTCVideoReceiveStream* RTCCall::CreateVideoReceiveStream(
	RTCVideoReceiveStream::Config configuration) {
	RTC_DCHECK_RUN_ON(worker_thread_);

 	receive_side_cc_.SetSendPeriodicFeedback(
 		SendPeriodicFeedback(configuration.rtp.extensions));

	EnsureStarted();
	TaskQueueBase* current = GetCurrentTaskQueueOrThread(_rtc_thread_manager);
	RTC_CHECK(current);

	RTCVideoRtpPacketListenSinkInterface* sink = 
		video_rtp_packet_listener_proxy_get_interface_->GetVideoRtpPacketProxyInterface(configuration.rtp.remote_ssrc);

	if (sink == nullptr) {
		return nullptr;
	}
	
	RTCVideoReceiveStream2Impl* receive_stream = new RTCVideoReceiveStream2Impl(
		current,
		transport_send_ptr_,
		&video_receiver_controller_,
		std::move(configuration),
		call_stats_.get(),
		clock_,
		sink);

	const RTCVideoReceiveStream::Config& config = receive_stream->config();

#ifdef __ROUTING_PACKET_PATH_USING_NETWORK_THREAD__
	{
		MutexLock lock(&recv_rtc_config_mutex_);
		if (config.rtp.rtx_ssrc) {
			receive_rtp_config_.emplace(config.rtp.rtx_ssrc, ReceiveRtpConfig(config));
		}
		receive_rtp_config_.emplace(config.rtp.remote_ssrc, ReceiveRtpConfig(config));
	}
#else
	if (config.rtp.rtx_ssrc) {
		receive_rtp_config_.emplace(config.rtp.rtx_ssrc, ReceiveRtpConfig(config));
	}
	receive_rtp_config_.emplace(config.rtp.remote_ssrc, ReceiveRtpConfig(config));
#endif

	((RTCVideoRtpPacketListenerProxy*)sink)->SetRtpHeaderExtensionMap(RtpHeaderExtensionMap(config.rtp.extensions));

	video_receive_streams_.insert(receive_stream);

// 	if (_rr_estimate_ssrc == 0) {
// 		_rr_estimate_ssrc = config.rtp.remote_ssrc;
// 	}

	receive_stream->SignalNetworkState(video_network_state_);
	UpdateAggregateNetworkState();

	return receive_stream;
}

void RTCCall::DestroyVideoReceiveStream(RTCVideoReceiveStream* receive_stream) {
	RTC_DCHECK_RUN_ON(worker_thread_);
	RTC_DCHECK(receive_stream != nullptr);

	RTCVideoReceiveStream2Impl* receive_stream_impl =
		static_cast<RTCVideoReceiveStream2Impl*>(receive_stream);
	const RTCVideoReceiveStream::Config& config = receive_stream_impl->config();

#ifdef __ROUTING_PACKET_PATH_USING_NETWORK_THREAD__
	{
		MutexLock lock(&recv_rtc_config_mutex_);
		receive_rtp_config_.erase(config.rtp.remote_ssrc);
		if (config.rtp.rtx_ssrc) {
			receive_rtp_config_.erase(config.rtp.rtx_ssrc);
		}
	}
#else
	receive_rtp_config_.erase(config.rtp.remote_ssrc);
	if (config.rtp.rtx_ssrc) {
		receive_rtp_config_.erase(config.rtp.rtx_ssrc);
	}
#endif
	video_receive_streams_.erase(receive_stream_impl);

	receive_side_cc_.RemoveStream(config.rtp.remote_ssrc);

	UpdateAggregateNetworkState();
	delete receive_stream_impl;
}

FlexfecReceiveStream* RTCCall::CreateFlexfecReceiveStream(
	const FlexfecReceiveStream::Config& config) {
	RTC_DCHECK_RUN_ON(worker_thread_);

	RecoveredPacketReceiver* recovered_packet_receiver = this;

	RTCFlexFecReceiveStreamImpl* receive_stream;

	// Unlike the video and audio receive streams, FlexfecReceiveStream implements
	// RtpPacketSinkInterface itself, and hence its constructor passes its |this|
	// pointer to video_receiver_controller_->CreateStream(). Calling the
	// constructor while on the worker thread ensures that we don't call
	// OnRtpPacket until the constructor is finished and the object is
	// in a valid state, since OnRtpPacket runs on the same thread.
	receive_stream = new RTCFlexFecReceiveStreamImpl(
		clock_, &video_receiver_controller_, config, recovered_packet_receiver,
		call_stats_->AsRtcpRttStats());

#ifdef __ROUTING_PACKET_PATH_USING_NETWORK_THREAD__
	{
		MutexLock lock(&recv_rtc_config_mutex_);
		RTC_DCHECK(receive_rtp_config_.find(config.remote_ssrc) ==
			receive_rtp_config_.end());

		receive_rtp_config_.emplace(config.remote_ssrc, ReceiveRtpConfig(config));
	}
#else
	RTC_DCHECK(receive_rtp_config_.find(config.remote_ssrc) ==
		receive_rtp_config_.end());

	receive_rtp_config_.emplace(config.remote_ssrc, ReceiveRtpConfig(config));
#endif

	return receive_stream;
}

void RTCCall::DestroyFlexfecReceiveStream(
		FlexfecReceiveStream* receive_stream) {
	RTC_DCHECK_RUN_ON(worker_thread_);

	RTC_DCHECK(receive_stream != nullptr);
	const FlexfecReceiveStream::Config& config = receive_stream->GetConfig();
	uint32_t ssrc = config.remote_ssrc;
#ifdef __ROUTING_PACKET_PATH_USING_NETWORK_THREAD__
	{
		MutexLock lock(&recv_rtc_config_mutex_);
		receive_rtp_config_.erase(ssrc);
	}
#else
	receive_rtp_config_.erase(ssrc);
#endif
	// Remove all SSRCs pointing to the FlexfecReceiveStreamImpl to be
	// destroyed.

	receive_side_cc_.RemoveStream(ssrc);

	delete receive_stream;
}

RTCRtpTransportControllerSendInterface* RTCCall::GetTransportControllerSend() {
	return transport_send_ptr_;
}

RTCCall::Stats RTCCall::GetStats() const {
	RTC_DCHECK_RUN_ON(worker_thread_);
	RTCCall::Stats stats;
	stats.rtt_ms = call_stats_->LastProcessedRtt();
	// Fetch available send/receive bitrates.
	std::vector<unsigned int> ssrcs;
	uint32_t recv_bandwidth = 0;

	receive_side_cc_.LatestReceiveSideEstimate(&ssrcs, &recv_bandwidth);

	stats.recv_bandwidth_bps = recv_bandwidth;
	stats.send_bandwidth_bps = last_bandwidth_bps_;

	_recv_network_estimate_bps = stats.recv_bandwidth_bps;
	_send_network_estimate_bps = stats.send_bandwidth_bps;

	return stats;
}

void RTCCall::GenerateKeyFrame() {
	worker_thread_->PostTask(
		SafeTask(task_safety_.flag(), [this](){
		std::set<RTCVideoReceiveStream2Impl*>::iterator it = video_receive_streams_.begin();
		(*it)->GenerateKeyFrame();
	}));
}

void RTCCall::GenerateKeyFramePli(uint32_t ssrc) {
	worker_thread_->PostTask(
		SafeTask(task_safety_.flag(), [this, ssrc](){
		for (auto it : video_receive_streams_) {
			if (it->RemoteSsrc() == ssrc) {
				it->GenerateKeyFramePli();
				break;
			}
		}
	}));
}

void RTCCall::SignalChannelNetworkState(MediaType media, NetworkState state) {
	RTC_DCHECK_RUN_ON(worker_thread_);
	switch (media) {
	case MediaType::AUDIO:
		audio_network_state_ = state;
		break;
	case MediaType::VIDEO:
		video_network_state_ = state;
		break;
	case MediaType::ANY:
	case MediaType::DATA:
		RTC_NOTREACHED();
		break;
	}

	UpdateAggregateNetworkState();
}

void RTCCall::OnAudioTransportOverheadChanged(int transport_overhead_per_packet) {
	RTC_DCHECK_RUN_ON(worker_thread_);
}

void RTCCall::UpdateAggregateNetworkState() {
	RTC_DCHECK_RUN_ON(worker_thread_);

	bool have_audio =
		!audio_send_ssrcs_.empty() || !audio_receive_streams_.empty();
	bool have_video =
		!video_send_ssrcs_.empty() || !video_receive_streams_.empty();

	bool aggregate_network_up =
		((have_video && video_network_state_ == kNetworkUp) ||
		(have_audio && audio_network_state_ == kNetworkUp));

	if (aggregate_network_up != aggregate_network_up_) {
		RTC_LOG(LS_INFO)
			<< "UpdateAggregateNetworkState: aggregate_state change to "
			<< (aggregate_network_up ? "up" : "down");
	}
	else {
		RTC_LOG(LS_VERBOSE)
			<< "UpdateAggregateNetworkState: aggregate_state remains at "
			<< (aggregate_network_up ? "up" : "down");
	}
	aggregate_network_up_ = aggregate_network_up;

	transport_send_ptr_->OnNetworkAvailability(aggregate_network_up);
}

void RTCCall::OnSentPacket(const rtc::SentPacket& sent_packet) {
	if (last_sent_packet_.has_value() && last_sent_packet_->packet_id != -1 &&
		last_sent_packet_->packet_id == sent_packet.packet_id &&
		last_sent_packet_->send_time_ms == sent_packet.send_time_ms) {
		return;
	}
	else if (sent_packet.packet_id == -1) {	
		return;
	}

	last_sent_packet_ = sent_packet;
	
 	transport_send_ptr_->OnSentPacket(sent_packet);
}

void RTCCall::OnStartRateUpdate(DataRate start_rate) {
}

void RTCCall::OnTargetTransferRate(TargetTransferRate msg) {
 	uint32_t target_bitrate_bps = msg.target_rate.bps();

 	// For controlling the rate of feedback messages.
 	receive_side_cc_.OnBitrateChanged(target_bitrate_bps);
 
	worker_thread_->PostTask(
		SafeTask(task_safety_.flag(), [this, target_bitrate_bps]() {
		RTC_DCHECK_RUN_ON(worker_thread_);
		last_bandwidth_bps_ = target_bitrate_bps;
	}));
}

uint64_t RTCCall::GetTransportSequenceNumber() {
	const uint64_t transport_seq = transport_sequence_number_.fetch_add(1, std::memory_order_relaxed);
	return transport_seq;
}

PacketReceiver::DeliveryStatus RTCCall::DeliverRtcp(MediaType media_type,
	const uint8_t* packet,
	size_t length) {
	
	if (received_bytes_per_second_counter_.HasSample()) {
		// First RTP packet has been received.
		received_bytes_per_second_counter_.Add(static_cast<int>(length));
		received_rtcp_bytes_per_second_counter_.Add(static_cast<int>(length));
	}

	bool rtcp_delivered = false;

	if (media_type == MediaType::ANY || media_type == MediaType::VIDEO) {
		for (RTCVideoReceiveStream2Impl* stream : video_receive_streams_) {
			if (stream->DeliverRtcp(packet, length))
				rtcp_delivered = true;
		}
	}
	if (media_type == MediaType::ANY || media_type == MediaType::AUDIO) {
		for (RTCAudioReceiveStreamImpl* stream : audio_receive_streams_) {
			stream->DeliverRtcp(packet, length);
			rtcp_delivered = true;
		}
	}

	if (media_type == MediaType::ANY || media_type == MediaType::VIDEO) {
		for (RTCVideoSendStream* stream : video_send_streams_) {
			static_cast<RTCVideoSendStreamImpl*>(stream)->DeliverRtcp(packet, length);
			rtcp_delivered = true;
		}
	}

	if (media_type == MediaType::ANY || media_type == MediaType::AUDIO) {
		for (auto& kv : audio_send_ssrcs_) {
			kv.second->DeliverRtcp(packet, length);
			rtcp_delivered = true;
		}
	}

	return rtcp_delivered ? DELIVERY_OK : DELIVERY_PACKET_ERROR;
}


static void _video_rtp_packet_dump(RtpPacketReceived &parsed_packet) {
	uint32_t abs_time;
	if (parsed_packet.GetExtension<AbsoluteSendTime>(&abs_time)) {
		RTC_LOG(LS_VERBOSE) << "video-recv-stream-trace extention hdr abs send time " << abs_time;
	}

	uint16_t ts_wide_cc;
	if (parsed_packet.GetExtension<TransportSequenceNumber>(&ts_wide_cc)) {
		RTC_LOG(LS_VERBOSE) << "video-recv-stream-trace extention hdr transport wide cc " << ts_wide_cc;
	}
}
//////////////////////////////////////////////////////////////////////////

PacketReceiver::DeliveryStatus RTCCall::DeliverRtp(MediaType media_type,
	rtc::CopyOnWriteBuffer packet,
	int64_t packet_time_us) {

	RtpPacketReceived parsed_packet;
	if (!parsed_packet.Parse(std::move(packet))) {
		return DELIVERY_PACKET_ERROR;
	}

	if (packet_time_us != -1) {

		if (receive_time_calculator_) {
			// Repair packet_time_us for clock resets by comparing a new read of
			// the same clock (TimeUTCMicros) to a monotonic clock reading.
			packet_time_us = receive_time_calculator_->ReconcileReceiveTimes(
				packet_time_us, rtc::TimeUTCMicros(), clock_->TimeInMicroseconds());
		}

		//		parsed_packet.set_arrival_time_ms((packet_time_us + 500) / 1000);	// kimi... 
		parsed_packet.set_arrival_time_ms(packet_time_us / 1000);	// kimi... replace
	}
	else {
		parsed_packet.set_arrival_time_ms(clock_->TimeInMilliseconds());
	}

	// We might get RTP keep-alive packets in accordance with RFC6263 section 4.6.
	// These are empty (zero length payload) RTP packets with an unsignaled
	// payload type.
	const bool is_keep_alive_packet = parsed_packet.payload_size() == 0;

	RTC_DCHECK(media_type == MediaType::AUDIO || media_type == MediaType::VIDEO ||
		is_keep_alive_packet);

#ifdef __ROUTING_PACKET_PATH_USING_NETWORK_THREAD__
	{
		MutexLock lock(&recv_rtc_config_mutex_);
		auto it = receive_rtp_config_.find(parsed_packet.Ssrc());
		if (it == receive_rtp_config_.end()) {
			RTC_LOG(LS_ERROR) << "receive_rtp_config_ lookup failed for ssrc "
				<< parsed_packet.Ssrc();
			// Destruction of the receive stream, including deregistering from the
			// RtpDemuxer, is not protected by the |worker_thread_|.
			// But deregistering in the |receive_rtp_config_| map is. So by not passing
			// the packet on to demuxing in this case, we prevent incoming packets to be
			// passed on via the demuxer to a receive stream which is being torned down.

			return DELIVERY_UNKNOWN_SSRC;
		}

		parsed_packet.IdentifyExtensions(it->second.extensions);
	}
#else
	auto it = receive_rtp_config_.find(parsed_packet.Ssrc());
	if (it == receive_rtp_config_.end()) {
		RTC_LOG(LS_ERROR) << "receive_rtp_config_ lookup failed for ssrc "
			<< parsed_packet.Ssrc();
		// Destruction of the receive stream, including deregistering from the
		// RtpDemuxer, is not protected by the |worker_thread_|.
		// But deregistering in the |receive_rtp_config_| map is. So by not passing
		// the packet on to demuxing in this case, we prevent incoming packets to be
		// passed on via the demuxer to a receive stream which is being torned down.

		return DELIVERY_UNKNOWN_SSRC;
	}

	parsed_packet.IdentifyExtensions(it->second.extensions);
#endif
	NotifyBweOfReceivedPacket(parsed_packet, media_type);

	// RateCounters expect input parameter as int, save it as int,
	// instead of converting each time it is passed to RateCounter::Add below.
	int length = static_cast<int>(parsed_packet.size());
	if (media_type == MediaType::AUDIO) {
		if (audio_receiver_controller_.OnRtpPacket(parsed_packet)) {
			received_bytes_per_second_counter_.Add(length);
			received_audio_bytes_per_second_counter_.Add(length);
			const int64_t arrival_time_ms = parsed_packet.arrival_time_ms();
			if (!first_received_rtp_audio_ms_) {
				first_received_rtp_audio_ms_.emplace(arrival_time_ms);
			}
			last_received_rtp_audio_ms_.emplace(arrival_time_ms);
			return DELIVERY_OK;
		}
	}
	else if (media_type == MediaType::VIDEO) {
		parsed_packet.set_payload_type_frequency(kVideoPayloadTypeFrequency);
		if (video_receiver_controller_.OnRtpPacket(parsed_packet)) {
			received_bytes_per_second_counter_.Add(length);
			received_video_bytes_per_second_counter_.Add(length);
			const int64_t arrival_time_ms = parsed_packet.arrival_time_ms();
			if (!first_received_rtp_video_ms_) {
				first_received_rtp_video_ms_.emplace(arrival_time_ms);
			}
			last_received_rtp_video_ms_.emplace(arrival_time_ms);

			return DELIVERY_OK;
		}
	}
	return DELIVERY_UNKNOWN_SSRC;
}

PacketReceiver::DeliveryStatus RTCCall::DeliverPacket(
	MediaType media_type,
	rtc::CopyOnWriteBuffer packet,
	int64_t packet_time_us) {

#ifndef __ROUTING_PACKET_PATH_USING_NETWORK_THREAD__
	RTC_DCHECK_RUN_ON(worker_thread_);
#endif

	if (IsRtcp(packet.cdata(), packet.size())) {
#ifdef __ROUTING_PACKET_PATH_USING_NETWORK_THREAD__
		TaskQueueBase *current_thread = rtc::Thread::Current(_rtc_thread_manager);
		if (current_thread == worker_thread_) {
			return DeliverRtcp(media_type, packet.cdata(), packet.size());
		}

		worker_thread_->PostTask(SafeTask(
			task_safety_.flag(), [this, media_type, p = std::move(packet)]{
			return DeliverRtcp(media_type, p.cdata(), p.size());
		}));
#else
		return DeliverRtcp(media_type, packet.cdata(), packet.size());
#endif
	}

	return DeliverRtp(media_type, std::move(packet), packet_time_us);
}

void RTCCall::DeliverPacketAsync(MediaType media_type,
	rtc::CopyOnWriteBuffer packet,
	int64_t packet_time_us,
	PacketCallback callback) {
	RTC_DCHECK_RUN_ON(network_thread_);

	TaskQueueBase* network_thread = rtc::Thread::Current(_rtc_thread_manager);

	RTC_DCHECK(network_thread);
	worker_thread_->PostTask(SafeTask(
		task_safety_.flag(), [this, network_thread, media_type, p = std::move(packet),
		packet_time_us, cb = std::move(callback)] {
		RTC_DCHECK_RUN_ON(worker_thread_);
		DeliveryStatus status = DeliverPacket(media_type, p, packet_time_us);
		if (cb) {
			network_thread->PostTask(
				SafeTask(task_safety_.flag(), [cb = std::move(cb), status, media_type,
				p = std::move(p), packet_time_us]() {
				cb(status, media_type, std::move(p), packet_time_us);
			}));
		}
	}));
}


void RTCCall::OnRecoveredPacket(const uint8_t* packet, size_t length) {
	RTC_DCHECK_RUN_ON(worker_thread_);

	RtpPacketReceived parsed_packet;
	if (!parsed_packet.Parse(packet, length))
		return;

	parsed_packet.set_recovered(true);

#ifdef __ROUTING_PACKET_PATH_USING_NETWORK_THREAD__
	{
		MutexLock lock(&recv_rtc_config_mutex_);
		auto it = receive_rtp_config_.find(parsed_packet.Ssrc());
		if (it == receive_rtp_config_.end()) {
			// Destruction of the receive stream, including deregistering from the
			// RtpDemuxer, is not protected by the |worker_thread_|.
			// But deregistering in the |receive_rtp_config_| map is.
			// So by not passing the packet on to demuxing in this case, we prevent
			// incoming packets to be passed on via the demuxer to a receive stream
			// which is being torn down.
			return;
		}
		parsed_packet.IdentifyExtensions(it->second.extensions);
	}
#else
	auto it = receive_rtp_config_.find(parsed_packet.Ssrc());
	if (it == receive_rtp_config_.end()) {
		// Destruction of the receive stream, including deregistering from the
		// RtpDemuxer, is not protected by the |worker_thread_|.
		// But deregistering in the |receive_rtp_config_| map is.
		// So by not passing the packet on to demuxing in this case, we prevent
		// incoming packets to be passed on via the demuxer to a receive stream
		// which is being torn down.
		return;
	}
	parsed_packet.IdentifyExtensions(it->second.extensions);
#endif

	// TODO(brandtr): Update here when we support protecting audio packets too.
	parsed_packet.set_payload_type_frequency(kVideoPayloadTypeFrequency);
	video_receiver_controller_.OnRtpPacket(parsed_packet);
}

void RTCCall::NotifyBweOfReceivedPacket(const RtpPacketReceived& packet,
										MediaType media_type) {
	RTPHeader header;
	packet.GetHeader(&header);

	ReceivedPacket packet_msg;
	packet_msg.size = DataSize::Bytes(packet.payload_size());				
	packet_msg.receive_time = Timestamp::Millis(packet.arrival_time_ms());	
	if (header.extension.hasAbsoluteSendTime) {
		packet_msg.send_time = header.extension.GetAbsoluteSendTimestamp();
	}

	transport_send_ptr_->OnReceivedPacket(packet_msg);	

	receive_side_cc_.OnReceivedPacket(packet, media_type);
}

}