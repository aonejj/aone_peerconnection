//////////////////////////////////////////////////////////////////////////
//
// author : kimi
// desc   : to move  /api/peer_connection_interface.h defs 
//
//////////////////////////////////////////////////////////////////////////

#ifndef __RTC_UPDATE_PEER_CONNECTION_INTERFACE_DEFS_H__
#define __RTC_UPDATE_PEER_CONNECTION_INTERFACE_DEFS_H__

#include <vector>
#include <string>

#include "rtc_base/system/rtc_export.h"
#include "rtc_base/rtc_certificate.h"
#include "api/transport/enums.h"
#include "api/turn_customizer.h"
#include "api/crypto/crypto_options.h"
#include "p2p/base/port_allocator.h"
#include "media/base/media_config.h"

namespace webrtc {

enum class SdpSemantics { kPlanB, kUnifiedPlan };

class PeerConnectionInterfaceDefs {
public:
	// See https://w3c.github.io/webrtc-pc/#dom-rtcsignalingstate
	enum SignalingState {
		kStable,
		kHaveLocalOffer,
		kHaveLocalPrAnswer,
		kHaveRemoteOffer,
		kHaveRemotePrAnswer,
		kClosed,
	};

	// See https://w3c.github.io/webrtc-pc/#dom-rtcicegatheringstate
	enum IceGatheringState {
		kIceGatheringNew,
		kIceGatheringGathering,
		kIceGatheringComplete
	};

	// See https://w3c.github.io/webrtc-pc/#dom-rtcpeerconnectionstate
	enum class PeerConnectionState {
		kNew,
		kConnecting,
		kConnected,
		kDisconnected,
		kFailed,
		kClosed,
	};


	// See https://w3c.github.io/webrtc-pc/#dom-rtciceconnectionstate
	enum IceConnectionState {
		kIceConnectionNew,
		kIceConnectionChecking,
		kIceConnectionConnected,
		kIceConnectionCompleted,
		kIceConnectionFailed,
		kIceConnectionDisconnected,
		kIceConnectionClosed,
		kIceConnectionMax,
	};

	// TLS certificate policy.
	enum TlsCertPolicy {
		// For TLS based protocols, ensure the connection is secure by not
		// circumventing certificate validation.
		kTlsCertPolicySecure,
		// For TLS based protocols, disregard security completely by skipping
		// certificate validation. This is insecure and should never be used unless
		// security is irrelevant in that particular context.
		kTlsCertPolicyInsecureNoCheck,
	};

	struct RTC_EXPORT IceServer {
		IceServer() = default;
		IceServer(const IceServer&) = default;
		~IceServer() = default;

		// TODO(jbauch): Remove uri when all code using it has switched to urls.
		// List of URIs associated with this server. Valid formats are described
		// in RFC7064 and RFC7065, and more may be added in the future. The "host"
		// part of the URI may contain either an IP address or a hostname.
		std::string uri;
		std::vector<std::string> urls;
		std::string username;
		std::string password;
		TlsCertPolicy tls_cert_policy = kTlsCertPolicySecure;
		// If the URIs in |urls| only contain IP addresses, this field can be used
		// to indicate the hostname, which may be necessary for TLS (using the SNI
		// extension). If |urls| itself contains the hostname, this isn't
		// necessary.
		std::string hostname;
		// List of protocols to be used in the TLS ALPN extension.
		std::vector<std::string> tls_alpn_protocols;
		// List of elliptic curves to be used in the TLS elliptic curves extension.
		std::vector<std::string> tls_elliptic_curves;

		bool operator==(const IceServer& o) const {
			return uri == o.uri && urls == o.urls && username == o.username &&
				password == o.password && tls_cert_policy == o.tls_cert_policy &&
				hostname == o.hostname &&
				tls_alpn_protocols == o.tls_alpn_protocols &&
				tls_elliptic_curves == o.tls_elliptic_curves;
		}
		bool operator!=(const IceServer& o) const { return !(*this == o); }
	};
	typedef std::vector<IceServer> IceServers;

	enum IceTransportsType {
		// TODO(pthatcher): Rename these kTransporTypeXXX, but update
		// Chromium at the same time.
		kNone,
		kRelay,
		kNoHost,
		kAll
	};

	// https://tools.ietf.org/html/draft-ietf-rtcweb-jsep-24#section-4.1.1
	enum BundlePolicy {
		kBundlePolicyBalanced,
		kBundlePolicyMaxBundle,
		kBundlePolicyMaxCompat
	};

	// https://tools.ietf.org/html/draft-ietf-rtcweb-jsep-24#section-4.1.1
	enum RtcpMuxPolicy {
		kRtcpMuxPolicyNegotiate,
		kRtcpMuxPolicyRequire,
	};

	enum TcpCandidatePolicy {
		kTcpCandidatePolicyEnabled,
		kTcpCandidatePolicyDisabled
	};

	enum CandidateNetworkPolicy {
		kCandidateNetworkPolicyAll,
		kCandidateNetworkPolicyLowCost
	};

	enum ContinualGatheringPolicy { GATHER_ONCE, GATHER_CONTINUALLY };

	enum class RTCConfigurationType {
		// A configuration that is safer to use, despite not having the best
		// performance. Currently this is the default configuration.
		kSafe,
		// An aggressive configuration that has better performance, although it
		// may be riskier and may need extra support in the application.
		kAggressive
	};

	struct RTC_EXPORT RTCConfiguration {
		RTCConfiguration() = default;
		RTCConfiguration(const RTCConfiguration&) = default;
		explicit RTCConfiguration(RTCConfigurationType type) {
			if (type == RTCConfigurationType::kAggressive) {
				// These parameters are also defined in Java and IOS configurations,
				// so their values may be overwritten by the Java or IOS configuration.
				bundle_policy = kBundlePolicyMaxBundle;
				rtcp_mux_policy = kRtcpMuxPolicyRequire;
				ice_connection_receiving_timeout = kAggressiveIceConnectionReceivingTimeout;

				// These parameters are not defined in Java or IOS configuration,
				// so their values will not be overwritten.
				enable_ice_renomination = true;
				redetermine_role_on_ice_restart = false;
			}
		}
		~RTCConfiguration() = default;

		bool operator==(const RTCConfiguration& o) const;
		bool operator!=(const RTCConfiguration& o) const;

		// disable media_configs

		static const int kUndefined = -1;
		// Default maximum number of packets in the audio jitter buffer.
		static const int kAudioJitterBufferMaxPackets = 200;
		// ICE connection receiving timeout for aggressive configuration.
		static const int kAggressiveIceConnectionReceivingTimeout = 1000;

		// TODO(pthatcher): Rename this ice_servers, but update Chromium
		// at the same time.
		IceServers servers;
		// TODO(pthatcher): Rename this ice_transport_type, but update
		// Chromium at the same time.
		IceTransportsType type = kAll;
		BundlePolicy bundle_policy = kBundlePolicyBalanced;
		RtcpMuxPolicy rtcp_mux_policy = kRtcpMuxPolicyRequire;
		std::vector<rtc::scoped_refptr<rtc::RTCCertificate>> certificates;
		int ice_candidate_pool_size = 0;

		// If set to true, don't gather IPv6 ICE candidates.
		// TODO(deadbeef): Remove this? IPv6 support has long stopped being
		// experimental
		bool disable_ipv6 = false;

		// If set to true, don't gather IPv6 ICE candidates on Wi-Fi.
		// Only intended to be used on specific devices. Certain phones disable IPv6
		// when the screen is turned off and it would be better to just disable the
		// IPv6 ICE candidates on Wi-Fi in those cases.
		bool disable_ipv6_on_wifi = false;

		// By default, the PeerConnection will use a limited number of IPv6 network
		// interfaces, in order to avoid too many ICE candidate pairs being created
		// and delaying ICE completion.
		//
		// Can be set to INT_MAX to effectively disable the limit.
		int max_ipv6_networks = cricket::kDefaultMaxIPv6Networks;

		// Exclude link-local network interfaces
		// from consideration for gathering ICE candidates.
		bool disable_link_local_networks = false;

		// If set to true, use RTP data channels instead of SCTP.
		// TODO(deadbeef): Remove this. We no longer commit to supporting RTP data
		// channels, though some applications are still working on moving off of
		// them.
		bool enable_rtp_data_channel = false;

		// Use new combined audio/video bandwidth estimation?
		absl::optional<bool> combined_audio_video_bwe;


		// TODO(bugs.webrtc.org/9891) - Move to crypto_options
		// Can be used to disable DTLS-SRTP. This should never be done, but can be
		// useful for testing purposes, for example in setting up a loopback call
		// with a single PeerConnection.
		absl::optional<bool> enable_dtls_srtp;

		/////////////////////////////////////////////////
		// The below fields are not part of the standard.
		/////////////////////////////////////////////////

		// Can be used to disable TCP candidate generation.
		TcpCandidatePolicy tcp_candidate_policy = kTcpCandidatePolicyEnabled;

		// Can be used to avoid gathering candidates for a "higher cost" network,
		// if a lower cost one exists. For example, if both Wi-Fi and cellular
		// interfaces are available, this could be used to avoid using the cellular
		// interface.
		CandidateNetworkPolicy candidate_network_policy =
			kCandidateNetworkPolicyAll;

		int audio_jitter_buffer_max_packets = kAudioJitterBufferMaxPackets;

		bool audio_jitter_buffer_fast_accelerate = false;

		int audio_jitter_buffer_min_delay_ms = 0;

		bool audio_jitter_buffer_enable_rtx_handling = false;

		// Timeout in milliseconds before an ICE candidate pair is considered to be
		// "not receiving", after which a lower priority candidate pair may be
		// selected.
		int ice_connection_receiving_timeout = kUndefined;

		// Interval in milliseconds at which an ICE "backup" candidate pair will be
		// pinged. This is a candidate pair which is not actively in use, but may
		// be switched to if the active candidate pair becomes unusable.
		//
		// This is relevant mainly to Wi-Fi/cell handoff; the application may not
		// want this backup cellular candidate pair pinged frequently, since it
		// consumes data/battery.
		int ice_backup_candidate_pair_ping_interval = kUndefined;

		// Can be used to enable continual gathering, which means new candidates
		// will be gathered as network interfaces change. Note that if continual
		// gathering is used, the candidate removal API should also be used, to
		// avoid an ever-growing list of candidates.
		ContinualGatheringPolicy continual_gathering_policy = GATHER_ONCE;

		// If set to true, candidate pairs will be pinged in order of most likely
		// to work (which means using a TURN server, generally), rather than in
		// standard priority order.
		bool prioritize_most_likely_ice_candidate_pairs = false;

		struct cricket::MediaConfig media_config;

		// If set to true, only one preferred TURN allocation will be used per
		// network interface. UDP is preferred over TCP and IPv6 over IPv4. This
		// can be used to cut down on the number of candidate pairings.
		// Deprecated. TODO(webrtc:11026) Remove this flag once the downstream
		// dependency is removed.
		bool prune_turn_ports = false;

		// The policy used to prune turn port.
		PortPrunePolicy turn_port_prune_policy = NO_PRUNE;

		PortPrunePolicy GetTurnPortPrunePolicy() const {
			return prune_turn_ports ? PRUNE_BASED_ON_PRIORITY
				: turn_port_prune_policy;
		}

		// If set to true, this means the ICE transport should presume TURN-to-TURN
		// candidate pairs will succeed, even before a binding response is received.
		// This can be used to optimize the initial connection time, since the DTLS
		// handshake can begin immediately.
		bool presume_writable_when_fully_relayed = false;

		// If true, "renomination" will be added to the ice options in the transport
		// description.
		// See: https://tools.ietf.org/html/draft-thatcher-ice-renomination-00
		bool enable_ice_renomination = false;

		// If true, the ICE role is re-determined when the PeerConnection sets a
		// local transport description that indicates an ICE restart.
		//
		// This is standard RFC5245 ICE behavior, but causes unnecessary role
		// thrashing, so an application may wish to avoid it. This role
		// re-determining was removed in ICEbis (ICE v2).
		bool redetermine_role_on_ice_restart = true;

		// This flag is only effective when |continual_gathering_policy| is
		// GATHER_CONTINUALLY.
		//
		// If true, after the ICE transport type is changed such that new types of
		// ICE candidates are allowed by the new transport type, e.g. from
		// IceTransportsType::kRelay to IceTransportsType::kAll, candidates that
		// have been gathered by the ICE transport but not matching the previous
		// transport type and as a result not observed by PeerConnectionObserver,
		// will be surfaced to the observer.
		bool surface_ice_candidates_on_ice_transport_type_changed = false;

		// The following fields define intervals in milliseconds at which ICE
		// connectivity checks are sent.
		//
		// We consider ICE is "strongly connected" for an agent when there is at
		// least one candidate pair that currently succeeds in connectivity check
		// from its direction i.e. sending a STUN ping and receives a STUN ping
		// response, AND all candidate pairs have sent a minimum number of pings for
		// connectivity (this number is implementation-specific). Otherwise, ICE is
		// considered in "weak connectivity".
		//
		// Note that the above notion of strong and weak connectivity is not defined
		// in RFC 5245, and they apply to our current ICE implementation only.
		//
		// 1) ice_check_interval_strong_connectivity defines the interval applied to
		// ALL candidate pairs when ICE is strongly connected, and it overrides the
		// default value of this interval in the ICE implementation;
		// 2) ice_check_interval_weak_connectivity defines the counterpart for ALL
		// pairs when ICE is weakly connected, and it overrides the default value of
		// this interval in the ICE implementation;
		// 3) ice_check_min_interval defines the minimal interval (equivalently the
		// maximum rate) that overrides the above two intervals when either of them
		// is less.
		absl::optional<int> ice_check_interval_strong_connectivity;
		absl::optional<int> ice_check_interval_weak_connectivity;
		absl::optional<int> ice_check_min_interval;

		// The min time period for which a candidate pair must wait for response to
		// connectivity checks before it becomes unwritable. This parameter
		// overrides the default value in the ICE implementation if set.
		absl::optional<int> ice_unwritable_timeout;

		// The min number of connectivity checks that a candidate pair must sent
		// without receiving response before it becomes unwritable. This parameter
		// overrides the default value in the ICE implementation if set.
		absl::optional<int> ice_unwritable_min_checks;

		// The min time period for which a candidate pair must wait for response to
		// connectivity checks it becomes inactive. This parameter overrides the
		// default value in the ICE implementation if set.
		absl::optional<int> ice_inactive_timeout;

		// The interval in milliseconds at which STUN candidates will resend STUN
		// binding requests to keep NAT bindings open.
		absl::optional<int> stun_candidate_keepalive_interval;

		// Optional TurnCustomizer.
		// With this class one can modify outgoing TURN messages.
		// The object passed in must remain valid until PeerConnection::Close() is
		// called.
		TurnCustomizer* turn_customizer = nullptr;

		// Preferred network interface.
		// A candidate pair on a preferred network has a higher precedence in ICE
		// than one on an un-preferred network, regardless of priority or network
		// cost.
		absl::optional<rtc::AdapterType> network_preference;

		// Configure the SDP semantics used by this PeerConnection. Note that the
		// WebRTC 1.0 specification requires kUnifiedPlan semantics. The
		// RtpTransceiver API is only available with kUnifiedPlan semantics.
		//
		// kPlanB will cause PeerConnection to create offers and answers with at
		// most one audio and one video m= section with multiple RtpSenders and
		// RtpReceivers specified as multiple a=ssrc lines within the section. This
		// will also cause PeerConnection to ignore all but the first m= section of
		// the same media type.
		//
		// kUnifiedPlan will cause PeerConnection to create offers and answers with
		// multiple m= sections where each m= section maps to one RtpSender and one
		// RtpReceiver (an RtpTransceiver), either both audio or both video. This
		// will also cause PeerConnection to ignore all but the first a=ssrc lines
		// that form a Plan B stream.
		//
		// For users who wish to send multiple audio/video streams and need to stay
		// interoperable with legacy WebRTC implementations or use legacy APIs,
		// specify kPlanB.
		//
		// For all other users, specify kUnifiedPlan.
		SdpSemantics sdp_semantics = SdpSemantics::kPlanB;

		// TODO(bugs.webrtc.org/9891) - Move to crypto_options or remove.
		// Actively reset the SRTP parameters whenever the DTLS transports
		// underneath are reset for every offer/answer negotiation.
		// This is only intended to be a workaround for crbug.com/835958
		// WARNING: This would cause RTP/RTCP packets decryption failure if not used
		// correctly. This flag will be deprecated soon. Do not rely on it.
		bool active_reset_srtp_params = false;

		// Defines advanced optional cryptographic settings related to SRTP and
		// frame encryption for native WebRTC. Setting this will overwrite any
		// settings set in PeerConnectionFactory (which is deprecated).
		absl::optional<CryptoOptions> crypto_options;

		// Configure if we should include the SDP attribute extmap-allow-mixed in
		// our offer. Although we currently do support this, it's not included in
		// our offer by default due to a previous bug that caused the SDP parser to
		// abort parsing if this attribute was present. This is fixed in Chrome 71.
		// TODO(webrtc:9985): Change default to true once sufficient time has
		// passed.
		bool offer_extmap_allow_mixed = false;

		// TURN logging identifier.
		// This identifier is added to a TURN allocation
		// and it intended to be used to be able to match client side
		// logs with TURN server logs. It will not be added if it's an empty string.
		std::string turn_logging_id;

		// Added to be able to control rollout of this feature.
		bool enable_implicit_rollback = false;
	};

	// See: https://www.w3.org/TR/webrtc/#idl-def-rtcofferansweroptions
	struct RTCOfferAnswerOptions {
		static const int kUndefined = -1;
		static const int kMaxOfferToReceiveMedia = 1;

		// The default value for constraint offerToReceiveX:true.
		static const int kOfferToReceiveMediaTrue = 1;

		// These options are left as backwards compatibility for clients who need
		// "Plan B" semantics. Clients who have switched to "Unified Plan" semantics
		// should use the RtpTransceiver API (AddTransceiver) instead.
		//
		// offer_to_receive_X set to 1 will cause a media description to be
		// generated in the offer, even if no tracks of that type have been added.
		// Values greater than 1 are treated the same.
		//
		// If set to 0, the generated directional attribute will not include the
		// "recv" direction (meaning it will be "sendonly" or "inactive".
		int offer_to_receive_video = kUndefined;
		int offer_to_receive_audio = kUndefined;

		bool voice_activity_detection = true;
		bool ice_restart = false;

		// If true, will offer to BUNDLE audio/video/data together. Not to be
		// confused with RTCP mux (multiplexing RTP and RTCP together).
		bool use_rtp_mux = true;

		// If true, "a=packetization:<payload_type> raw" attribute will be offered
		// in the SDP for all video payload and accepted in the answer if offered.
		bool raw_packetization_for_video = false;

		// This will apply to all video tracks with a Plan B SDP offer/answer.
		int num_simulcast_layers = 1;

		// If true: Use SDP format from draft-ietf-mmusic-scdp-sdp-03
		// If false: Use SDP format from draft-ietf-mmusic-sdp-sdp-26 or later
		bool use_obsolete_sctp_sdp = false;

		RTCOfferAnswerOptions() = default;

		RTCOfferAnswerOptions(int offer_to_receive_video,
			int offer_to_receive_audio,
			bool voice_activity_detection,
			bool ice_restart,
			bool use_rtp_mux)
			: offer_to_receive_video(offer_to_receive_video),
			offer_to_receive_audio(offer_to_receive_audio),
			voice_activity_detection(voice_activity_detection),
			ice_restart(ice_restart),
			use_rtp_mux(use_rtp_mux) {}
	};

	// Used by GetStats to decide which stats to include in the stats reports.
	// |kStatsOutputLevelStandard| includes the standard stats for Javascript API;
	// |kStatsOutputLevelDebug| includes both the standard stats and additional
	// stats for debugging purposes.
	enum StatsOutputLevel {
		kStatsOutputLevelStandard,
		kStatsOutputLevelDebug,
	};
};

}

#endif //__PEER_CONNECTION_INTERFACE_DEFS_H__