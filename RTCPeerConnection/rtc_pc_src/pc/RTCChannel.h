//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : replace to  /pc/channel.h
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_CHANNEL_H__
#define __RTC_CHANNEL_H__

#include <stddef.h>
#include <stdint.h>

#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "../rtc_pc_src/_deprecate_defines.h"

#include "absl/types/optional.h"

#include "api/rtp_transceiver_direction.h"
#include "api/crypto/crypto_options.h"
#include "media/base/stream_params.h"
#include "call/rtp_packet_sink_interface.h"
#include "call/rtp_demuxer.h"
#include "pc/channel_interface.h"
#include "pc/rtp_transport_internal.h"
#include "pc/session_description.h"
#include "p2p/base/dtls_transport_internal.h"
#include "rtc_base/message_handler.h"
#include "rtc_base/thread.h"
#include "rtc_base/unique_id_generator.h"
#include "rtc_base/third_party/sigslot/sigslot.h"
#include "rtc_base/copy_on_write_buffer.h"
#include "rtc_base/async_packet_socket.h"
#include "rtc_base/task_utils/pending_task_safety_flag.h"

#include "../../src_update/media/base/_media_channel.h"

#include "../rtc_pc_src/RTCThreadManagerInterface.h"

namespace cricket {

struct CryptoParams;

class RTCBaseChannel : public ChannelInterface,
					   public rtc::MessageHandlerAutoCleanup,
					   public sigslot::has_slots<>,
					   public MediaChannel::NetworkInterface,
					   public webrtc::RtpPacketSinkInterface {
public:
	RTCBaseChannel(rtc::Thread* worker_thread,
				   rtc::Thread* network_thread,
				   rtc::Thread* signaling_thread,
				   std::unique_ptr<MediaChannel> media_channel,
				   const std::string& content_name,
				   bool srtp_required,
				   webrtc::CryptoOptions crypto_options,
				   rtc::UniqueRandomIdGenerator* ssrc_generator,
				   rtc::RTCThreadManagerInterface* rtc_thread_manager
				   );
	virtual ~RTCBaseChannel();
	virtual void Init_w(webrtc::RtpTransportInternal* rtp_transport);

	void Deinit();

	rtc::Thread* worker_thread() const { return worker_thread_; }
	rtc::Thread* network_thread() const { return network_thread_; }
	const std::string& content_name() const override { return content_name_; }


	const std::string& transport_name() const override { return transport_name_; }
	bool enabled() const override { return enabled_; }

	// This function returns true if using SRTP (DTLS-based keying or SDES).
	bool srtp_active() const {
		RTC_DCHECK_RUN_ON(network_thread());
		return rtp_transport_ && rtp_transport_->IsSrtpActive();
	}

	bool writable() const { return writable_; }

	bool SetRtpTransport(webrtc::RtpTransportInternal* rtp_transport) override;

	webrtc::RtpTransportInternal* rtp_transport() const {
		RTC_DCHECK_RUN_ON(network_thread());
		return rtp_transport_;
	}

	// Channel control
	bool SetLocalContent(const MediaContentDescription* content,
		webrtc::SdpType type,
		std::string* error_desc) override;
	bool SetRemoteContent(const MediaContentDescription* content,
		webrtc::SdpType type,
		std::string* error_desc) override;

	bool SetPayloadTypeDemuxingEnabled(bool enabled) override;

	bool Enable(bool enable) override;

	const std::vector<StreamParams>& local_streams() const override {
		return local_streams_;
	}
	const std::vector<StreamParams>& remote_streams() const override {
		return remote_streams_;
	}

	// Used for latency measurements.
	sigslot::signal1<ChannelInterface*>& SignalFirstPacketReceived() override;

	// Forward SignalSentPacket to worker thread.
	sigslot::signal1<const rtc::SentPacket&>& SignalSentPacket();

	// From RtpTransport - public for testing only
	void OnTransportReadyToSend(bool ready);

	// Only public for unit tests.  Otherwise, consider protected.
	int SetOption(SocketType type, rtc::Socket::Option o, int val) override;
	int SetOption_n(SocketType type, rtc::Socket::Option o, int val)
		RTC_RUN_ON(network_thread());

	// RtpPacketSinkInterface overrides.
	void OnRtpPacket(const webrtc::RtpPacketReceived& packet) override;

	MediaChannel* media_channel() const override {
		return media_channel_.get();
	}

protected:
	bool was_ever_writable() const {
		RTC_DCHECK_RUN_ON(network_thread());
		return was_ever_writable_;
	}
	void set_local_content_direction(webrtc::RtpTransceiverDirection direction) {
		RTC_DCHECK_RUN_ON(worker_thread());
		local_content_direction_ = direction;
	}
	void set_remote_content_direction(webrtc::RtpTransceiverDirection direction) {
		RTC_DCHECK_RUN_ON(worker_thread());
		remote_content_direction_ = direction;
	}

	bool IsReadyToReceiveMedia_w() const RTC_RUN_ON(worker_thread());
	bool IsReadyToSendMedia_w() const RTC_RUN_ON(worker_thread());
	rtc::Thread* signaling_thread() const { return signaling_thread_; }

	void FlushRtcpMessages_n() RTC_RUN_ON(network_thread());

	// NetworkInterface implementation, called by MediaEngine
	bool SendPacket(rtc::CopyOnWriteBuffer* packet,
		const rtc::PacketOptions& options) override;
	bool SendRtcp(rtc::CopyOnWriteBuffer* packet,
		const rtc::PacketOptions& options) override;

	// From RtpTransportInternal
	void OnWritableState(bool writable);

	void OnNetworkRouteChanged(absl::optional<rtc::NetworkRoute> network_route);

	bool PacketIsRtcp(const rtc::PacketTransportInternal* transport,
		const char* data,
		size_t len);
	bool SendPacket(bool rtcp,
		rtc::CopyOnWriteBuffer* packet,
		const rtc::PacketOptions& options);

	void EnableMedia_w() RTC_RUN_ON(worker_thread());
	void DisableMedia_w() RTC_RUN_ON(worker_thread());

	void UpdateWritableState_n() RTC_RUN_ON(network_thread());
	void ChannelWritable_n() RTC_RUN_ON(network_thread());
	void ChannelNotWritable_n() RTC_RUN_ON(network_thread());

	bool AddRecvStream_w(const StreamParams& sp) RTC_RUN_ON(worker_thread());
	bool RemoveRecvStream_w(uint32_t ssrc) RTC_RUN_ON(worker_thread());
	void ResetUnsignaledRecvStream_w() RTC_RUN_ON(worker_thread());
	bool SetPayloadTypeDemuxingEnabled_w(bool enabled)
		RTC_RUN_ON(worker_thread());
	bool AddSendStream_w(const StreamParams& sp) RTC_RUN_ON(worker_thread());
	bool RemoveSendStream_w(uint32_t ssrc) RTC_RUN_ON(worker_thread());

	// Should be called whenever the conditions for
	// IsReadyToReceiveMedia/IsReadyToSendMedia are satisfied (or unsatisfied).
	// Updates the send/recv state of the media channel.
	void UpdateMediaSendRecvState();
	virtual void UpdateMediaSendRecvState_w() = 0;

	bool UpdateLocalStreams_w(const std::vector<StreamParams>& streams,
		webrtc::SdpType type,
		std::string* error_desc)
		RTC_RUN_ON(worker_thread());
	bool UpdateRemoteStreams_w(const std::vector<StreamParams>& streams,
		webrtc::SdpType type,
		std::string* error_desc)
		RTC_RUN_ON(worker_thread());
	virtual bool SetLocalContent_w(const MediaContentDescription* content,
		webrtc::SdpType type,
		std::string* error_desc) = 0;
	virtual bool SetRemoteContent_w(const MediaContentDescription* content,
		webrtc::SdpType type,
		std::string* error_desc) = 0;
	// Return a list of RTP header extensions with the non-encrypted extensions
	// removed depending on the current crypto_options_ and only if both the
	// non-encrypted and encrypted extension is present for the same URI.
	RtpHeaderExtensions GetFilteredRtpHeaderExtensions(
		const RtpHeaderExtensions& extensions);

	// From MessageHandler
	void OnMessage(rtc::Message* pmsg) override;

	// Helper function template for invoking methods on the worker thread.
	template <class T>
	T InvokeOnWorker(const rtc::Location& posted_from,
		rtc::FunctionView<T()> functor) {
		return worker_thread_->Invoke<T>(posted_from, functor);
	}

	// Add |payload_type| to |demuxer_criteria_| if payload type demuxing is
	// enabled.
	void MaybeAddHandledPayloadType(int payload_type) RTC_RUN_ON(worker_thread());

	void ClearHandledPayloadTypes() RTC_RUN_ON(worker_thread());

	void UpdateRtpHeaderExtensionMap(
		const RtpHeaderExtensions& header_extensions);

	bool RegisterRtpDemuxerSink_w() RTC_RUN_ON(worker_thread());
	bool RegisterRtpDemuxerSink_n() RTC_RUN_ON(network_thread());

	// Return description of media channel to facilitate logging
	std::string ToString() const;

	void SetNegotiatedHeaderExtensions_w(const RtpHeaderExtensions& extensions);

	// ChannelInterface overrides
	RtpHeaderExtensions GetNegotiatedRtpHeaderExtensions() const override;

 private:
	 bool ConnectToRtpTransport();
	 void DisconnectFromRtpTransport();
	 void SignalSentPacket_n(const rtc::SentPacket& sent_packet)
		 RTC_RUN_ON(network_thread());
	 bool IsReadyToSendMedia_n() const RTC_RUN_ON(network_thread());

	 rtc::Thread* const worker_thread_;
	 rtc::Thread* const network_thread_;
	 rtc::Thread* const signaling_thread_;
	 rtc::scoped_refptr<webrtc::PendingTaskSafetyFlag> alive_;
	 sigslot::signal1<ChannelInterface*> SignalFirstPacketReceived_
		 RTC_GUARDED_BY(signaling_thread_);
	 sigslot::signal1<const rtc::SentPacket&> SignalSentPacket_
		 RTC_GUARDED_BY(worker_thread_);

	 const std::string content_name_;

	 bool has_received_packet_ = false;

	 std::string transport_name_;

	 webrtc::RtpTransportInternal* rtp_transport_
		 RTC_GUARDED_BY(network_thread()) = nullptr;

	 std::vector<std::pair<rtc::Socket::Option, int> > socket_options_
		 RTC_GUARDED_BY(network_thread());
	 std::vector<std::pair<rtc::Socket::Option, int> > rtcp_socket_options_
		 RTC_GUARDED_BY(network_thread());
	 // TODO(bugs.webrtc.org/12230): writable_ is accessed in tests
	 // outside of the network thread.
	 bool writable_ = false;
	 bool was_ever_writable_ RTC_GUARDED_BY(network_thread()) = false;
	 const bool srtp_required_ = true;
	 const webrtc::CryptoOptions crypto_options_;

	 // MediaChannel related members that should be accessed from the worker
	 // thread.
	 const std::unique_ptr<MediaChannel> media_channel_;
	 // Currently the |enabled_| flag is accessed from the signaling thread as
	 // well, but it can be changed only when signaling thread does a synchronous
	 // call to the worker thread, so it should be safe.
	 bool enabled_ = false;
	 bool payload_type_demuxing_enabled_ RTC_GUARDED_BY(worker_thread()) = true;
	 std::vector<StreamParams> local_streams_ RTC_GUARDED_BY(worker_thread());
	 std::vector<StreamParams> remote_streams_ RTC_GUARDED_BY(worker_thread());
	 // TODO(bugs.webrtc.org/12230): local_content_direction and
	 // remote_content_direction are set on the worker thread, but accessed on the
	 // network thread.
	 webrtc::RtpTransceiverDirection local_content_direction_ =
		 webrtc::RtpTransceiverDirection::kInactive;
	 webrtc::RtpTransceiverDirection remote_content_direction_ =
		 webrtc::RtpTransceiverDirection::kInactive;

	 // Cached list of payload types, used if payload type demuxing is re-enabled.
	 std::set<uint8_t> payload_types_ RTC_GUARDED_BY(worker_thread());
	 // TODO(bugs.webrtc.org/12239): Modified on worker thread, accessed
	 // on network thread in RegisterRtpDemuxerSink_n (called from Init_w)
	 webrtc::RtpDemuxerCriteria demuxer_criteria_;
	 // This generator is used to generate SSRCs for local streams.
	 // This is needed in cases where SSRCs are not negotiated or set explicitly
	 // like in Simulcast.
	 // This object is not owned by the channel so it must outlive it.
	 rtc::UniqueRandomIdGenerator* const ssrc_generator_;

	 // |negotiated_header_extensions_| is read on the signaling thread, but
	 // written on the worker thread while being sync-invoked from the signal
	 // thread in SdpOfferAnswerHandler::PushdownMediaDescription(). Hence the lock
	 // isn't strictly needed, but it's anyway placed here for future safeness.
	 mutable webrtc::Mutex negotiated_header_extensions_lock_;
	 RtpHeaderExtensions negotiated_header_extensions_
		 RTC_GUARDED_BY(negotiated_header_extensions_lock_);

	 rtc::RTCThreadManagerInterface* _rtc_thread_manager;
};

class RTCVoiceChannel : public RTCBaseChannel {
public:
	RTCVoiceChannel(rtc::Thread* worker_thread,
		rtc::Thread* network_thread,
		rtc::Thread* signaling_thread,
		std::unique_ptr<VoiceMediaChannel> channel,
		const std::string& content_name,
		bool srtp_required,
		webrtc::CryptoOptions crypto_options,
		rtc::UniqueRandomIdGenerator* ssrc_generator,
		rtc::RTCThreadManagerInterface* rtc_thread_manager
		);
	~RTCVoiceChannel();

	// downcasts a MediaChannel
	VoiceMediaChannel* media_channel() const override {
		return static_cast<VoiceMediaChannel*>(RTCBaseChannel::media_channel());
	}

	cricket::MediaType media_type() const override {
		return cricket::MEDIA_TYPE_AUDIO;
	}
	void Init_w(webrtc::RtpTransportInternal* rtp_transport) override;

private:
	// overrides from BaseChannel
	void UpdateMediaSendRecvState_w() override;
	bool SetLocalContent_w(const MediaContentDescription* content,
		webrtc::SdpType type,
		std::string* error_desc) override;
	bool SetRemoteContent_w(const MediaContentDescription* content,
		webrtc::SdpType type,
		std::string* error_desc) override;

	// Last AudioSendParameters sent down to the media_channel() via
	// SetSendParameters.
	AudioSendParameters last_send_params_;
	// Last AudioRecvParameters sent down to the media_channel() via
	// SetRecvParameters.
	AudioRecvParameters last_recv_params_;
};

class RTCVideoChannel : public RTCBaseChannel {
public:
	RTCVideoChannel(rtc::Thread* worker_thread,
		rtc::Thread* network_thread,
		rtc::Thread* signaling_thread,
		std::unique_ptr<VideoMediaChannel> media_channel,
		const std::string& content_name,
		bool srtp_required,
		webrtc::CryptoOptions crypto_options,
		rtc::UniqueRandomIdGenerator* ssrc_generator,
		rtc::RTCThreadManagerInterface* rtc_thread_manager
		);
	~RTCVideoChannel();

	// downcasts a MediaChannel
	VideoMediaChannel* media_channel() const override {
		return static_cast<VideoMediaChannel*>(RTCBaseChannel::media_channel());
	}

	cricket::MediaType media_type() const override {
		return cricket::MEDIA_TYPE_VIDEO;
	}

private:
	// overrides from BaseChannel
	void UpdateMediaSendRecvState_w() override;
	bool SetLocalContent_w(const MediaContentDescription* content,
		webrtc::SdpType type,
		std::string* error_desc) override;
	bool SetRemoteContent_w(const MediaContentDescription* content,
		webrtc::SdpType type,
		std::string* error_desc) override;

	// Last VideoSendParameters sent down to the media_channel() via
	// SetSendParameters.
	VideoSendParameters last_send_params_;
	// Last VideoRecvParameters sent down to the media_channel() via
	// SetRecvParameters.
	VideoRecvParameters last_recv_params_;
};

class RTCRtpDataChannel : public RTCBaseChannel {
public:
	RTCRtpDataChannel(rtc::Thread* worker_thread,
		rtc::Thread* network_thread,
		rtc::Thread* signaling_thread,
		std::unique_ptr<DataMediaChannel> channel,
		const std::string& content_name,
		bool srtp_required,
		webrtc::CryptoOptions crypto_options,
		rtc::UniqueRandomIdGenerator* ssrc_generator,
		rtc::RTCThreadManagerInterface* rtc_thread_manager
		);
	~RTCRtpDataChannel();
	// TODO(zhihuang): Remove this once the RtpTransport can be shared between
	// BaseChannels.
	void Init_w(DtlsTransportInternal* rtp_dtls_transport,
		DtlsTransportInternal* rtcp_dtls_transport,
		rtc::PacketTransportInternal* rtp_packet_transport,
		rtc::PacketTransportInternal* rtcp_packet_transport);
	void Init_w(webrtc::RtpTransportInternal* rtp_transport) override;

	virtual bool SendData(const SendDataParams& params,
		const rtc::CopyOnWriteBuffer& payload,
		SendDataResult* result);

	// Should be called on the signaling thread only.
	bool ready_to_send_data() const { return ready_to_send_data_; }

	sigslot::signal2<const ReceiveDataParams&, const rtc::CopyOnWriteBuffer&>
		SignalDataReceived;
	// Signal for notifying when the channel becomes ready to send data.
	// That occurs when the channel is enabled, the transport is writable,
	// both local and remote descriptions are set, and the channel is unblocked.
	sigslot::signal1<bool> SignalReadyToSendData;
	cricket::MediaType media_type() const override {
		return cricket::MEDIA_TYPE_DATA;
	}

protected:
	// downcasts a MediaChannel.
	DataMediaChannel* media_channel() const override {
		return static_cast<DataMediaChannel*>(RTCBaseChannel::media_channel());
	}

private:
	struct SendDataMessageData : public rtc::MessageData {
		SendDataMessageData(const SendDataParams& params,
			const rtc::CopyOnWriteBuffer* payload,
			SendDataResult* result)
			: params(params), payload(payload), result(result), succeeded(false) {}

		const SendDataParams& params;
		const rtc::CopyOnWriteBuffer* payload;
		SendDataResult* result;
		bool succeeded;
	};

	struct DataReceivedMessageData : public rtc::MessageData {
		// We copy the data because the data will become invalid after we
		// handle DataMediaChannel::SignalDataReceived but before we fire
		// SignalDataReceived.
		DataReceivedMessageData(const ReceiveDataParams& params,
			const char* data,
			size_t len)
			: params(params), payload(data, len) {}
		const ReceiveDataParams params;
		const rtc::CopyOnWriteBuffer payload;
	};

	typedef rtc::TypedMessageData<bool> DataChannelReadyToSendMessageData;

	// overrides from BaseChannel
	// Checks that data channel type is RTP.
	bool CheckDataChannelTypeFromContent(const MediaContentDescription* content,
		std::string* error_desc);
	bool SetLocalContent_w(const MediaContentDescription* content,
		webrtc::SdpType type,
		std::string* error_desc) override;
	bool SetRemoteContent_w(const MediaContentDescription* content,
		webrtc::SdpType type,
		std::string* error_desc) override;
	void UpdateMediaSendRecvState_w() override;

	void OnMessage(rtc::Message* pmsg) override;
	void OnDataReceived(const ReceiveDataParams& params,
		const char* data,
		size_t len);
	void OnDataChannelReadyToSend(bool writable);

	bool ready_to_send_data_ = false;

	// Last DataSendParameters sent down to the media_channel() via
	// SetSendParameters.
	DataSendParameters last_send_params_;
	// Last DataRecvParameters sent down to the media_channel() via
	// SetRecvParameters.
	DataRecvParameters last_recv_params_;
};

}


#endif //__RTC_CHANNEL_H__