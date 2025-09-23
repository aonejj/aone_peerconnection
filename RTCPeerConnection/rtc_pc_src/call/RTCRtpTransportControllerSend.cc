/*
*  Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
*
*  Use of this source code is governed by a BSD-style license
*  that can be found in the LICENSE file in the root of the source
*  tree. An additional intellectual property rights grant can be found
*  in the file PATENTS.  All contributing project authors may
*  be found in the AUTHORS file in the root of the source tree.
*/
#include <memory>
#include <utility>
#include <vector>

#include "absl/strings/match.h"
#include "absl/types/optional.h"
#include "api/transport/goog_cc_factory.h"
#include "api/transport/network_types.h"
#include "api/units/data_rate.h"
#include "api/units/time_delta.h"
#include "api/units/timestamp.h"
#include "api/units/data_size.h"	
#include "modules/rtp_rtcp/source/rtcp_packet/transport_feedback.h"
#include "rtc_base/checks.h"
#include "rtc_base/logging.h"
#include "rtc_base/rate_limiter.h"

#include "RTCRtpTransportControllerSend.h"

const int32_t kDefaultStartSFUPacerBitrateBps = 1000000;

namespace webrtc {
namespace {
static const int64_t kRetransmitWindowSizeMs = 500;
static const size_t kMaxOverheadBytes = 500;

constexpr TimeDelta kPacerQueueUpdateInterval = TimeDelta::Millis(25);

TargetRateConstraints ConvertConstraints(int min_bitrate_bps,
	int max_bitrate_bps,
	int start_bitrate_bps,
	Clock* clock) {
	TargetRateConstraints msg;
	msg.at_time = Timestamp::Millis(clock->TimeInMilliseconds());
	msg.min_data_rate = min_bitrate_bps >= 0
		? DataRate::BitsPerSec(min_bitrate_bps)
		: DataRate::Zero();
	msg.max_data_rate = max_bitrate_bps > 0
		? DataRate::BitsPerSec(max_bitrate_bps)
		: DataRate::Infinity();
	if (start_bitrate_bps > 0)
		msg.starting_rate = DataRate::BitsPerSec(start_bitrate_bps);
	return msg;
}

TargetRateConstraints ConvertConstraints(const BitrateConstraints& contraints,
	Clock* clock) {
	return ConvertConstraints(contraints.min_bitrate_bps,
		contraints.max_bitrate_bps,
		contraints.start_bitrate_bps, clock);
}

bool IsRelayed(const rtc::NetworkRoute& route) {
	return route.local.uses_turn() || route.remote.uses_turn();
}

}  // namespace

RTCRtpTransportControllerSend::RTCRtpTransportControllerSend(
	Clock* clock,
	const BitrateConstraints& bitrate_config,
	TaskQueueFactory* task_queue_factory)
	: clock_(clock),
	bitrate_configurator_(bitrate_config),
	observer_(nullptr),
	controller_factory_fallback_(
		std::make_unique<GoogCcNetworkControllerFactory>()),
	process_interval_(controller_factory_fallback_->GetProcessInterval()),
	last_report_block_time_(Timestamp::Millis(clock_->TimeInMilliseconds())),
	reset_feedback_on_route_change_(false),
	send_side_bwe_with_overhead_(false),
	relay_bandwidth_cap_("relay_cap", DataRate::PlusInfinity()),
	network_available_(false),
	congestion_window_size_(DataSize::PlusInfinity()),
	is_congested_(false),
	task_queue_(TaskQueueBase::Current()),
	transport_overhead_bytes_per_packet_(0),
	pacer_started_(false), 
	pacer_(clock,
			&packet_router_, 
			TimeDelta::Millis(3),	
			3,
			TimeDelta::Millis(30)) 
{


#ifdef __USING_RTC_SFU_PACING__
	packet_router_.SetSFUPacingListener(&pacer_);
#endif

	initial_config_.constraints = ConvertConstraints(bitrate_config, clock_);
	streams_config_.requests_alr_probing = true;
	pacer_.SetPacingRates(
		DataRate::BitsPerSec(kDefaultStartSFUPacerBitrateBps),
		DataRate::Zero());
}

RTCRtpTransportControllerSend::~RTCRtpTransportControllerSend() {
	pacer_queue_update_task_.Stop();
	controller_task_.Stop();
}

void RTCRtpTransportControllerSend::UpdateControlState() {
	absl::optional<TargetTransferRate> update = control_handler_->GetUpdate();
	if (!update)
		return;
	// We won't create control_handler_ until we have an observers.
	RTC_DCHECK(observer_ != nullptr);
	observer_->OnTargetTransferRate(*update);
}

void RTCRtpTransportControllerSend::UpdateCongestedState() {
	if (auto update = GetCongestedStateUpdate()) {
		is_congested_ = update.value();
		pacer_.SetCongested(update.value());
	}
}

absl::optional<bool> RTCRtpTransportControllerSend::GetCongestedStateUpdate() const {
	bool congested = transport_feedback_adapter_.GetOutstandingData() >=
		congestion_window_size_;
	if (congested != is_congested_)
		return congested;
	return absl::nullopt;
}

RTCPacketRouter* RTCRtpTransportControllerSend::packet_router() {
	return &packet_router_;
}

NetworkStateEstimateObserver*
RTCRtpTransportControllerSend::network_state_estimate_observer() {
	return this;
}

TransportFeedbackObserver*
RTCRtpTransportControllerSend::transport_feedback_observer() {
	return this;
}

RtpPacketSender* RTCRtpTransportControllerSend::packet_sender() {
	return &pacer_;
}

StreamFeedbackProvider*
RTCRtpTransportControllerSend::GetStreamFeedbackProvider() {
	return &feedback_demuxer_;
}

void RTCRtpTransportControllerSend::RegisterTargetTransferRateObserver(
	TargetTransferRateObserver* observer) {
	RTC_DCHECK_RUN_ON(&sequence_checker_);
	RTC_DCHECK(observer_ == nullptr);
	observer_ = observer;
	observer_->OnStartRateUpdate(*initial_config_.constraints.starting_rate);
	MaybeCreateControllers();
}

bool RTCRtpTransportControllerSend::IsRelevantRouteChange(
	const rtc::NetworkRoute& old_route,
	const rtc::NetworkRoute& new_route) const {
	// TODO(bugs.webrtc.org/11438): Experiment with using more information/
	// other conditions.
	bool connected_changed = old_route.connected != new_route.connected;
	bool route_ids_changed =
		old_route.local.network_id() != new_route.local.network_id() ||
		old_route.remote.network_id() != new_route.remote.network_id();
	if (relay_bandwidth_cap_->IsFinite()) {
		bool relaying_changed = IsRelayed(old_route) != IsRelayed(new_route);
		return connected_changed || route_ids_changed || relaying_changed;
	}
	else {
		return connected_changed || route_ids_changed;
	}
}

void RTCRtpTransportControllerSend::OnNetworkRouteChanged(
	const std::string& transport_name,
	const rtc::NetworkRoute& network_route) {
	RTC_DCHECK_RUN_ON(&sequence_checker_);
	// Check if the network route is connected.
	if (!network_route.connected) {
		// TODO(honghaiz): Perhaps handle this in SignalChannelNetworkState and
		// consider merging these two methods.
		return;
	}

	absl::optional<BitrateConstraints> relay_constraint_update =
		ApplyOrLiftRelayCap(IsRelayed(network_route));

	// Check whether the network route has changed on each transport.
	auto result =
		network_routes_.insert(std::make_pair(transport_name, network_route));
	auto kv = result.first;
	bool inserted = result.second;
	if (inserted || !(kv->second == network_route)) {
		RTC_LOG(LS_INFO) << "Network route changed on transport " << transport_name
			<< ": new_route = " << network_route.DebugString();
		if (!inserted) {
			RTC_LOG(LS_INFO) << "old_route = " << kv->second.DebugString();
		}
	}

	if (inserted) {
		if (relay_constraint_update.has_value()) {
			UpdateBitrateConstraints(*relay_constraint_update);
		}
		transport_overhead_bytes_per_packet_ = network_route.packet_overhead;
		// No need to reset BWE if this is the first time the network connects.
		return;
	}

	const rtc::NetworkRoute old_route = kv->second;
	kv->second = network_route;

	// Check if enough conditions of the new/old route has changed
	// to trigger resetting of bitrates (and a probe).
	if (IsRelevantRouteChange(old_route, network_route)) {
		BitrateConstraints bitrate_config = bitrate_configurator_.GetConfig();
		RTC_LOG(LS_INFO) << "Reset bitrates to min: "
			<< bitrate_config.min_bitrate_bps
			<< " bps, start: " << bitrate_config.start_bitrate_bps
			<< " bps,  max: " << bitrate_config.max_bitrate_bps
			<< " bps.";
		RTC_DCHECK_GT(bitrate_config.start_bitrate_bps, 0);

		NetworkRouteChange msg;
		msg.at_time = Timestamp::Millis(clock_->TimeInMilliseconds());
		msg.constraints = ConvertConstraints(bitrate_config, clock_);
		transport_overhead_bytes_per_packet_ = network_route.packet_overhead;

		if (reset_feedback_on_route_change_) {
			transport_feedback_adapter_.SetNetworkRoute(network_route);
		}
		if (controller_) {
			PostUpdates(controller_->OnNetworkRouteChange(msg));
		}
		else {
			UpdateInitialConstraints(msg.constraints);
		}

		is_congested_ = false;
		pacer_.SetCongested(false);
	}
}
void RTCRtpTransportControllerSend::OnNetworkAvailability(bool network_available) {
	NetworkAvailability msg;
	msg.at_time = Timestamp::Millis(clock_->TimeInMilliseconds());
	msg.network_available = network_available;

	RTC_DCHECK_RUN_ON(&sequence_checker_);
	if (network_available_ == msg.network_available)
		return;
	network_available_ = msg.network_available;

	if (msg.network_available) {
		pacer_.Resume();
	}
	else {
		pacer_.Pause();
	}
	is_congested_ = false;
	pacer_.SetCongested(false);

	if (controller_) {
		control_handler_->SetNetworkAvailability(network_available_);
		PostUpdates(controller_->OnNetworkAvailability(msg));
		UpdateControlState();
	}
	else {
		MaybeCreateControllers();
	}
}

RtcpBandwidthObserver* RTCRtpTransportControllerSend::GetBandwidthObserver() {
	return this;
}

void RTCRtpTransportControllerSend::OnSentPacket(
	const rtc::SentPacket& sent_packet) {

	if (TaskQueueBase::Current() != task_queue_) {
		task_queue_->PostTask(SafeTask(safety_.flag(), [this, sent_packet]() {
			RTC_DCHECK_RUN_ON(&sequence_checker_);
			ProcessSentPacket(sent_packet);
		}));
		return;
	}

	RTC_DCHECK_RUN_ON(&sequence_checker_);
	ProcessSentPacket(sent_packet);
}

void RTCRtpTransportControllerSend::OnReceivedPacket(
	const ReceivedPacket& packet_msg) {

	if (controller_) {
		PostUpdates(controller_->OnReceivedPacket(packet_msg));
	}
}

void RTCRtpTransportControllerSend::UpdateBitrateConstraints(
	const BitrateConstraints& updated) {
	TargetRateConstraints msg = ConvertConstraints(updated, clock_);
	RTC_DCHECK_RUN_ON(&sequence_checker_);
	if (controller_) {
		PostUpdates(controller_->OnTargetRateConstraints(msg));
	}
	else {
		UpdateInitialConstraints(msg);
	}
}

void RTCRtpTransportControllerSend::SetSdpBitrateParameters(
	const BitrateConstraints& constraints) {
	RTC_DCHECK_RUN_ON(&sequence_checker_);
	absl::optional<BitrateConstraints> updated =
		bitrate_configurator_.UpdateWithSdpParameters(constraints);
	if (updated.has_value()) {
		UpdateBitrateConstraints(*updated);
	}
	else {
		RTC_LOG(LS_VERBOSE)
			<< "WebRTC.RtpTransportControllerSend.SetSdpBitrateParameters: "
			"nothing to update";
	}
}

void RTCRtpTransportControllerSend::SetClientBitratePreferences(
	const BitrateSettings& preferences) {
	RTC_DCHECK_RUN_ON(&sequence_checker_);
	absl::optional<BitrateConstraints> updated =
		bitrate_configurator_.UpdateWithClientPreferences(preferences);
	if (updated.has_value()) {
		UpdateBitrateConstraints(*updated);
	}
}

absl::optional<BitrateConstraints>
RTCRtpTransportControllerSend::ApplyOrLiftRelayCap(bool is_relayed) {
	DataRate cap = is_relayed ? relay_bandwidth_cap_ : DataRate::PlusInfinity();
	return bitrate_configurator_.UpdateWithRelayCap(cap);
}

void RTCRtpTransportControllerSend::OnTransportOverheadChanged(
	size_t transport_overhead_bytes_per_packet) {
	RTC_DCHECK_RUN_ON(&sequence_checker_);
	if (transport_overhead_bytes_per_packet >= kMaxOverheadBytes) {
		RTC_LOG(LS_ERROR) << "Transport overhead exceeds " << kMaxOverheadBytes;
		return;
	}

	pacer_.SetTransportOverhead(
		DataSize::Bytes(transport_overhead_bytes_per_packet));
}

void RTCRtpTransportControllerSend::EnsureStarted() {
	RTC_DCHECK_RUN_ON(&sequence_checker_);
	if (!pacer_started_) {
		pacer_started_ = true;
		pacer_.EnsureStarted();
	}
}

void RTCRtpTransportControllerSend::OnReceivedEstimatedBitrate(uint32_t bitrate) {
	RemoteBitrateReport msg;
	msg.receive_time = Timestamp::Millis(clock_->TimeInMilliseconds());
	msg.bandwidth = DataRate::BitsPerSec(bitrate);
	RTC_DCHECK_RUN_ON(&sequence_checker_);
	if (controller_) {
		PostUpdates(controller_->OnRemoteBitrateReport(msg));
	}
}

void RTCRtpTransportControllerSend::OnReceivedRtcpReceiverReport(
	const ReportBlockList& report_blocks,
	int64_t rtt_ms,
	int64_t now_ms) {

	RTC_DCHECK_RUN_ON(&sequence_checker_);
	OnReceivedRtcpReceiverReportBlocks(report_blocks, now_ms);

	RoundTripTimeUpdate report;
	report.receive_time = Timestamp::Millis(now_ms);
	report.round_trip_time = TimeDelta::Millis(rtt_ms);
	report.smoothed = false;
	if (controller_ && !report.round_trip_time.IsZero()) {
		PostUpdates(controller_->OnRoundTripTimeUpdate(report));
	}
}

void RTCRtpTransportControllerSend::OnAddPacket(
	const RtpPacketSendInfo& packet_info) {

	RTC_DCHECK_RUN_ON(&sequence_checker_);

	Timestamp creation_time = Timestamp::Millis(clock_->TimeInMilliseconds());
	feedback_demuxer_.AddPacket(packet_info);
	transport_feedback_adapter_.AddPacket(
		packet_info, transport_overhead_bytes_per_packet_, creation_time);
}

void RTCRtpTransportControllerSend::OnTransportFeedback(
	const rtcp::TransportFeedback& feedback) {
	RTC_DCHECK_RUN_ON(&sequence_checker_);
	feedback_demuxer_.OnTransportFeedback(feedback);
	auto feedback_time = Timestamp::Millis(clock_->TimeInMilliseconds());
	absl::optional<TransportPacketsFeedback> feedback_msg =
		transport_feedback_adapter_.ProcessTransportFeedback(feedback,
		feedback_time);
	if (feedback_msg) {
		if (controller_)
			PostUpdates(controller_->OnTransportPacketsFeedback(*feedback_msg));

		// Only update outstanding data if any packet is first time acked.
		UpdateCongestedState();
	}
}

void RTCRtpTransportControllerSend::OnRemoteNetworkEstimate(
	NetworkStateEstimate estimate) {
	estimate.update_time = Timestamp::Millis(clock_->TimeInMilliseconds());
	RTC_DCHECK_RUN_ON(&sequence_checker_);
	estimate.update_time = Timestamp::Millis(clock_->TimeInMilliseconds());
	if (controller_) {
		PostUpdates(controller_->OnNetworkStateEstimate(estimate));
	}
}

void RTCRtpTransportControllerSend::MaybeCreateControllers() {
	RTC_DCHECK(!controller_);	
	RTC_DCHECK(!control_handler_);

	if (!network_available_ || !observer_)
		return;
	control_handler_ = std::make_unique<CongestionControlHandler>();

	initial_config_.constraints.at_time =
		Timestamp::Millis(clock_->TimeInMilliseconds());
	initial_config_.stream_based_config = streams_config_;

	controller_ = controller_factory_fallback_->Create(initial_config_);
	process_interval_ = controller_factory_fallback_->GetProcessInterval();

	UpdateControllerWithTimeInterval();
	StartProcessPeriodicTasks();
}

void RTCRtpTransportControllerSend::UpdateInitialConstraints(
	TargetRateConstraints new_contraints) {
	if (!new_contraints.starting_rate)
		new_contraints.starting_rate = initial_config_.constraints.starting_rate;
	RTC_DCHECK(new_contraints.starting_rate);
	initial_config_.constraints = new_contraints;
}

void RTCRtpTransportControllerSend::StartProcessPeriodicTasks() {
	RTC_DCHECK_RUN_ON(&sequence_checker_);

	if (!pacer_queue_update_task_.Running()) {
		pacer_queue_update_task_ = RepeatingTaskHandle::DelayedStart(
			task_queue_, kPacerQueueUpdateInterval, [this]() {
			RTC_DCHECK_RUN_ON(&sequence_checker_);
			TimeDelta expected_queue_time = pacer_.ExpectedQueueTime();
			control_handler_->SetPacerQueue(expected_queue_time);
			UpdateControlState();
			return kPacerQueueUpdateInterval;
		});
	}

	controller_task_.Stop();			
	if (process_interval_.IsFinite()) {
		controller_task_ = RepeatingTaskHandle::DelayedStart(
			task_queue_,
			process_interval_, [this]() {
			RTC_DCHECK_RUN_ON(&sequence_checker_);
			UpdateControllerWithTimeInterval();
			return process_interval_;
		});
	}
}

void RTCRtpTransportControllerSend::UpdateControllerWithTimeInterval() {
	RTC_DCHECK(controller_);
	ProcessInterval msg;
	msg.at_time = Timestamp::Millis(clock_->TimeInMilliseconds());
	PostUpdates(controller_->OnProcessInterval(msg));
}

void RTCRtpTransportControllerSend::UpdateStreamsConfig() {
	streams_config_.at_time = Timestamp::Millis(clock_->TimeInMilliseconds());
	if (controller_)
		PostUpdates(controller_->OnStreamsConfig(streams_config_));
}

void RTCRtpTransportControllerSend::PostUpdates(NetworkControlUpdate update) {
	if (update.congestion_window) {
		congestion_window_size_ = *update.congestion_window;
		UpdateCongestedState();
	}

	if (update.pacer_config) {
		pacer_.SetPacingRates(update.pacer_config->data_rate(),
			update.pacer_config->pad_rate());
	}
	if (!update.probe_cluster_configs.empty()) {
		pacer_.CreateProbeClusters(std::move(update.probe_cluster_configs));
	}

	if (update.target_rate) {
		control_handler_->SetTargetRate(*update.target_rate);
		UpdateControlState();
	}
}

void RTCRtpTransportControllerSend::OnReceivedRtcpReceiverReportBlocks(
	const ReportBlockList& report_blocks,
	int64_t now_ms) {
	if (report_blocks.empty())
		return;

	int total_packets_lost_delta = 0;
	int total_packets_delta = 0;

	// Compute the packet loss from all report blocks.
	for (const RTCPReportBlock& report_block : report_blocks) {
		auto it = last_report_blocks_.find(report_block.source_ssrc);
		if (it != last_report_blocks_.end()) {
			auto number_of_packets = report_block.extended_highest_sequence_number -
				it->second.extended_highest_sequence_number;
			total_packets_delta += number_of_packets;
			auto lost_delta = report_block.packets_lost - it->second.packets_lost;
			total_packets_lost_delta += lost_delta;
		}
		last_report_blocks_[report_block.source_ssrc] = report_block;
	}
	// Can only compute delta if there has been previous blocks to compare to. If
	// not, total_packets_delta will be unchanged and there's nothing more to do.
	if (!total_packets_delta)
		return;
	int packets_received_delta = total_packets_delta - total_packets_lost_delta;
	// To detect lost packets, at least one packet has to be received. This check
	// is needed to avoid bandwith detection update in
	// VideoSendStreamTest.SuspendBelowMinBitrate

	if (packets_received_delta < 1)
		return;
	Timestamp now = Timestamp::Millis(now_ms);
	TransportLossReport msg;
	msg.packets_lost_delta = total_packets_lost_delta;
	msg.packets_received_delta = packets_received_delta;
	msg.receive_time = now;
	msg.start_time = last_report_block_time_;
	msg.end_time = now;
	if (controller_) {
		PostUpdates(controller_->OnTransportLossReport(msg));
	}
	last_report_block_time_ = now;
}

void RTCRtpTransportControllerSend::ProcessSentPacket(const rtc::SentPacket& sent_packet) {
	RTC_DCHECK_RUN_ON(&sequence_checker_);
	absl::optional<SentPacket> packet_msg =
		transport_feedback_adapter_.ProcessSentPacket(sent_packet);
	if (!packet_msg)
		return;

	auto congestion_update = GetCongestedStateUpdate();
	NetworkControlUpdate control_update;
	if (controller_)
		control_update = controller_->OnSentPacket(*packet_msg);
	if (!congestion_update && !control_update.has_updates())
		return;
	ProcessSentPacketUpdates(std::move(control_update));
}

void RTCRtpTransportControllerSend::ProcessSentPacketUpdates(NetworkControlUpdate updates) {
	RTC_DCHECK_RUN_ON(&sequence_checker_);
	UpdateCongestedState();
	if (controller_) {
		PostUpdates(std::move(updates));
	}
}

}  // namespace webrtc
