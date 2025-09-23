SO_PATH = ../out

LIBRARY_NAME := librtc_sfu_peerconnection.so


WEBRTC_CODE_PIECE_INC := ./src

ABSEIL_INC := ./third_party/abseil/abseil-cpp

BORING_SSL_INC := ../../boringssl/include

LIBSRTP_INC := ./src/third_party/libsrtp/include
LIBSRTP_CONFIG_INC := ./src/third_party/libsrtp/config
LIBSRTP_CIPHER_INC := ./src/third_party/libsrtp/crypto/include

SRTP_INC := -I$(LIBSRTP_INC)
SRTP_INC += -I$(LIBSRTP_CONFIG_INC)
SRTP_INC += -I$(LIBSRTP_CIPHER_INC)
SRTP_INC += -I$(BORING_SSL_INC)

INC = -I$(ABSEIL_INC)
INC += -I$(WEBRTC_CODE_PIECE_INC)
INC += -I$(BORING_SSL_INC)
INC += -I$(LIBSRTP_INC)
INC += -I$(LIBSRTP_CONFIG_INC)
INC += -I$(LIBSRTP_CIPHER_INC)



ABSEIL_LIBS_BASE_PATH := ./third_party/abseil/abseil-cpp/build/absl

ABSEIL_LIBS_PATH := -L$(ABSEIL_LIBS_BASE_PATH)/base \
					-L$(ABSEIL_LIBS_BASE_PATH)/strings \
					-L$(ABSEIL_LIBS_BASE_PATH)/types

BORING_SSL_LIBS_PATH := -L../../boringssl/build/crypto \
						-L../../boringssl/build/ssl


LIB_PATH := $(ABSEIL_LIBS_PATH)
LIB_PATH += $(BORING_SSL_LIBS_PATH)

ABSEIL_LIBS := -labsl_base -labsl_strings -labsl_strings_internal -labsl_bad_variant_access -labsl_bad_optional_access -labsl_throw_delegate

BORING_SSL_LIBS := -lcrypto -lssl

USR_SCTP_LIBS := -lusrsctp

#OPUS_LIBS := -lopus	# 사용하지 않음


CFLAGS := -DWEBRTC_LINUX -DWEBRTC_POSIX -DDCHECK_ALWAYS_ON
CFLAGS += -DWEBRTC_OPUS_VARIABLE_COMPLEXITY
CFLAGS += -DUNUSED_TRACE_EVENT -DUNUSED_PEER_CONNECTION -DUNUSED_GTEST

LIBSRTP_CFLAGS := -DHAVE_CONFIG_H -DOPENSSL -DGCM -DHAVE_STDLIB_H -DHAVE_STRING_H -DHAVE_STDINT_H -DHAVE_INTTYPES_H
LIBSRTP_CFLAGS += -DHAVE_INT16_T -DHAVE_INT32_T -DHAVE_INT8_T -DHAVE_UINT16_T -DHAVE_UINT32_T -DHAVE_UINT64_T -DHAVE_UINT8_T
LIBSRTP_CFLAGS += -DHAVE_ARPA_INET_H -DHAVE_NETINET_IN_H -DHAVE_SYS_TYPES_H -DHAVE_UNISTD_H
LIBSRTP_CFLAGS += -DPACKAGE_STRING=\"860492290f7d1f25e2bd45da6471bfd4cd4d7add\" -DPACKAGE_VERSION=\"860492290f7d1f25e2bd45da6471bfd4cd4d7add\"

#C = clang -O2	# clang 6
#CC = clang++ -O2	# clang 6
#CC = clang++ -O0 -v -da	#for gdb debug clang 6

C = clang-9 -O2
#CC = clang++-9 -O2	# clang 9
CC = clang++-9 -O0 -v -da	#for gdb debug clang 9

API_SOURCES := \
	./src/api/rtc_event_log/rtc_event.cc \
	./src/api/rtc_event_log/rtc_event_log.cc \
	./src/api/video/hdr_metadata.cc \
	./src/api/video/color_space.cc \
	./src/api/video/video_content_type.cc \
	./src/api/video/video_timing.cc \
	./src/api/video/encoded_image.cc \
	./src/api/video/video_bitrate_allocation.cc \
	./src/api/video/video_bitrate_allocator.cc \
	./src/api/video_codecs/sdp_video_format.cc \
	./src/api/video_codecs/video_encoder_config.cc \
	./src/api/video_codecs/spatial_layer.cc \
	./src/api/rtp_headers.cc \
	./src/api/units/timestamp.cc \
	./src/api/units/time_delta.cc \
	./src/api/units/data_rate.cc \
	./src/api/units/data_size.cc \
	./src/api/units/frequency.cc \
	./src/api/transport/network_types.cc \
	./src/api/transport/stun.cc \
	./src/api/transport/bitrate_settings.cc \
	./src/api/transport/rtp/dependency_descriptor.cc \
	./src/api/transport/goog_cc_factory.cc \
	./src/api/task_queue/default_task_queue_factory_stdlib.cc \
	./src/api/task_queue/task_queue_base.cc \
	./src/api/media_types.cc \
	./src/api/rtp_parameters.cc \
	./src/api/rtp_packet_info.cc \
	./src/api/rtc_error.cc \
	./src/api/candidate.cc \
	./src/api/jsep.cc \
	./src/api/jsep_ice_candidate.cc \
	./src/api/crypto/crypto_options.cc \
	./src/api/ice_transport_factory.cc \
	./src/api/dtls_transport_interface.cc \
	./src/api/sctp_transport_interface.cc \
	./src/api/data_channel_interface.cc \
	./src/api/proxy.cc \
	./src/api/call/transport.cc \
	./src/api/audio_options.cc \
	./src/api/audio/channel_layout.cc \
	./src/api/audio/audio_frame.cc \
	./src/api/audio_codecs/opus/audio_encoder_opus_config.cc \
	./src/api/audio_codecs/audio_format.cc \
	./src/api/audio_codecs/audio_codec_pair_id.cc

AUDIO_SOURCES := \
	./src/audio/audio_level.cc \
	./src/audio/utility/audio_frame_operations.cc

BASE_SOURCES := \
	./src/base/check.cc \
	./src/base/check_op.cc
	

CALL_SOURCES := \
	./src/call/rtp_demuxer.cc \
	./src/call/rtp_config.cc \
	./src/call/flexfec_receive_stream.cc \
	./src/call/rtp_stream_receiver_controller.cc \
	./src/call/receive_time_calculator.cc \
	./src/call/rtp_bitrate_configurator.cc \
	./src/call/syncable.cc \
	./src/call/rtx_receive_stream.cc
	
#	./src/call/flexfec_receive_stream_impl.cc \		#unused
		

RTC_BASE_SOURCES := \
	./src/rtc_base/async_resolver_interface.cc \
	./src/rtc_base/checks.cc \
	./src/rtc_base/logging.cc \
	./src/rtc_base/platform_thread_types.cc \
	./src/rtc_base/containers/flat_tree.cc \
	./src/rtc_base/strings/string_builder.cc \
	./src/rtc_base/strings/audio_format_to_string.cc \
	./src/rtc_base/strings/string_format.cc \
	./src/rtc_base/string_encode.cc \
	./src/rtc_base/string_to_number.cc \
	./src/rtc_base/string_utils.cc \
	./src/rtc_base/time_utils.cc \
	./src/rtc_base/zero_memory.cc \
	./src/rtc_base/synchronization/mutex.cc \
	./src/rtc_base/synchronization/yield.cc \
	./src/rtc_base/synchronization/yield_policy.cc \
	./src/rtc_base/synchronization/sequence_checker.cc \
	./src/rtc_base/task_utils/pending_task_safety_flag.cc \
	./src/rtc_base/task_utils/repeating_task.cc \
	./src/rtc_base/copy_on_write_buffer.cc \
	./src/rtc_base/bit_buffer.cc \
	./src/rtc_base/random.cc \
	./src/rtc_base/socket_address.cc \
	./src/rtc_base/ip_address.cc \
	./src/rtc_base/net_helper.cc \
	./src/rtc_base/net_helpers.cc \
	./src/rtc_base/network_constants.cc \
	./src/rtc_base/helpers.cc \
	./src/rtc_base/third_party/sigslot/sigslot.cc \
	./src/rtc_base/third_party/base64/base64.cc \
	./src/rtc_base/deprecated/recursive_critical_section.cc \
	./src/rtc_base/internal/default_socket_server.cc \
	./src/rtc_base/socket.cc \
	./src/rtc_base/async_resolver.cc \
	./src/rtc_base/async_socket.cc \
	./src/rtc_base/async_tcp_socket.cc \
	./src/rtc_base/async_udp_socket.cc \
	./src/rtc_base/null_socket_server.cc \
	./src/rtc_base/message_handler.cc \
	./src/rtc_base/thread.cc \
	./src/rtc_base/location.cc \
	./src/rtc_base/event.cc \
	./src/rtc_base/task_queue.cc \
	./src/rtc_base/platform_thread.cc \
	./src/rtc_base/task_queue_stdlib.cc \
	./src/rtc_base/unique_id_generator.cc \
	./src/rtc_base/openssl_digest.cc \
	./src/rtc_base/message_digest.cc \
	./src/rtc_base/ssl_certificate.cc \
	./src/rtc_base/openssl_key_pair.cc \
	./src/rtc_base/ssl_identity.cc \
	./src/rtc_base/boringssl_certificate.cc \
	./src/rtc_base/boringssl_identity.cc \
	./src/rtc_base/openssl_utility.cc \
	./src/rtc_base/ssl_fingerprint.cc \
	./src/rtc_base/rtc_certificate.cc \
	./src/rtc_base/rtc_certificate_generator.h \
	./src/rtc_base/crypt_string.cc \
	./src/rtc_base/network/sent_packet.cc \
	./src/rtc_base/async_packet_socket.cc \
	./src/rtc_base/proxy_info.cc \
	./src/rtc_base/crc32.cc \
	./src/rtc_base/byte_buffer.cc \
	./src/rtc_base/network_monitor_factory.cc \
	./src/rtc_base/ifaddrs_converter.cc \
	./src/rtc_base/network_monitor.cc \
	./src/rtc_base/network.cc \
	./src/rtc_base/data_rate_limiter.cc \
	./src/rtc_base/numerics/event_based_exponential_moving_average.cc \
	./src/rtc_base/numerics/exp_filter.cc \
	./src/rtc_base/rate_tracker.cc \
	./src/rtc_base/weak_ptr.cc \
	./src/rtc_base/stream.cc \
	./src/rtc_base/openssl_session_cache.cc \
	./src/rtc_base/ssl_adapter.cc \
	./src/rtc_base/openssl_adapter.cc \
	./src/rtc_base/openssl_stream_adapter.cc \
	./src/rtc_base/ssl_stream_adapter.cc \
	./src/rtc_base/network_route.cc \
	./src/rtc_base/buffer_queue.cc \
	./src/rtc_base/experiments/field_trial_parser.cc \
	./src/rtc_base/experiments/field_trial_units.cc \
	./src/rtc_base/experiments/struct_parameters_parser.cc \
	./src/rtc_base/experiments/rate_control_settings.cc \
	./src/rtc_base/async_invoker.cc \
	./src/rtc_base/http_common.cc \
	./src/rtc_base/socket_adapters.cc \
	./src/rtc_base/socket_address_pair.cc \
	./src/rtc_base/physical_socket_server.cc \
	./src/rtc_base/memory/aligned_malloc.cc \
	./src/rtc_base/server_socket_adapters.cc \
	./src/rtc_base/nat_types.cc \
	./src/rtc_base/callback_list.cc \
	./src/rtc_base/operations_chain.cc \
	./src/rtc_base/race_checker.cc \
	./src/rtc_base/rate_statistics.cc \
	./src/rtc_base/rate_limiter.cc \
	./src/rtc_base/system/thread_registry.cc



#	./src/rtc_base/memory/fifo_buffer.cc \
#	./src/rtc_base/nat_socket_factory.cc \
#	./src/rtc_base/proxy_server.cc \
#	./src/rtc_base/nat_server.cc \
#	./src/rtc_base/virtual_socket_server.cc \
#	./src/rtc_base/fake_clock.cc \


#	./src/rtc_base/firewall_socket_server.cc \			# unused
#	./src/rtc_base/fake_ssl_identity.cc \				# unused
#	./src/rtc_base/gunit.cc \							# unused
#	./src/rtc_base/test_utils.cc \						# unused
#	./src/rtc_base/test_client.cc \						# unused
#	./src/rtc_base/test_echo_server.cc \				# unused
#	./src/rtc_base/deprecated/signal_thread.cc \		# unused
#	./src/rtc_base/system/file_wrapper.cc \				# unused
		


MODULES_SOURCES := \
	./src/modules/rtp_rtcp/source/rtp_video_header.cc \
	./src/modules/rtp_rtcp/source/create_video_rtp_depacketizer.cc \
	./src/modules/rtp_rtcp/source/rtp_format.cc \
	./src/modules/rtp_rtcp/source/rtp_format_h264.cc \
	./src/modules/rtp_rtcp/source/rtp_format_vp8.cc \
	./src/modules/rtp_rtcp/source/rtp_format_vp9.cc \
	./src/modules/rtp_rtcp/source/rtp_packet_to_send.cc \
	./src/modules/rtp_rtcp/source/rtp_header_extensions.cc \
	./src/modules/rtp_rtcp/include/rtp_rtcp_defines.cc \
	./src/modules/rtp_rtcp/include/report_block_data.cc \
	./src/modules/rtp_rtcp/source/rtcp_packet/app.cc \
	./src/modules/rtp_rtcp/source/rtcp_packet/remote_estimate.cc \
	./src/modules/rtp_rtcp/source/rtcp_packet/common_header.cc \
	./src/modules/rtp_rtcp/source/rtcp_packet/tmmb_item.cc \
	./src/modules/rtp_rtcp/source/rtcp_packet/dlrr.cc \
	./src/modules/rtp_rtcp/source/rtcp_packet/report_block.cc \
	./src/modules/rtp_rtcp/source/rtcp_packet/compound_packet.cc \
	./src/modules/rtp_rtcp/source/rtcp_packet/loss_notification.cc \
	./src/modules/rtp_rtcp/source/rtcp_packet/psfb.cc \
	./src/modules/rtp_rtcp/source/rtcp_packet/bye.cc \
	./src/modules/rtp_rtcp/source/rtcp_packet/extended_reports.cc \
	./src/modules/rtp_rtcp/source/rtcp_packet/rrtr.cc \
	./src/modules/rtp_rtcp/source/rtcp_packet/target_bitrate.cc \
	./src/modules/rtp_rtcp/source/rtcp_packet/fir.cc \
	./src/modules/rtp_rtcp/source/rtcp_packet/nack.cc \
	./src/modules/rtp_rtcp/source/rtcp_packet/rtpfb.cc \
	./src/modules/rtp_rtcp/source/rtcp_packet/pli.cc \
	./src/modules/rtp_rtcp/source/rtcp_packet/rapid_resync_request.cc \
	./src/modules/rtp_rtcp/source/rtcp_packet/receiver_report.cc \
	./src/modules/rtp_rtcp/source/rtcp_packet/remb.cc \
	./src/modules/rtp_rtcp/source/rtcp_packet/sdes.cc \
	./src/modules/rtp_rtcp/source/rtcp_packet/sender_report.cc \
	./src/modules/rtp_rtcp/source/rtcp_packet/tmmbn.cc \
	./src/modules/rtp_rtcp/source/rtcp_packet/tmmbr.cc \
	./src/modules/rtp_rtcp/source/rtcp_packet/transport_feedback.cc \
	./src/modules/rtp_rtcp/source/rtcp_packet.cc \
	./src/modules/rtp_rtcp/source/rtp_packet.cc \
	./src/modules/rtp_rtcp/source/rtp_header_extension_map.cc \
	./src/modules/rtp_rtcp/source/rtp_format_video_generic.cc \
	./src/modules/rtp_rtcp/source/rtp_dependency_descriptor_extension.cc \
	./src/modules/rtp_rtcp/source/rtp_generic_frame_descriptor.cc \
	./src/modules/rtp_rtcp/source/rtp_generic_frame_descriptor_extension.cc \
	./src/modules/rtp_rtcp/source/rtp_dependency_descriptor_reader.cc \
	./src/modules/rtp_rtcp/source/rtp_dependency_descriptor_writer.cc \
	./src/modules/rtp_rtcp/source/video_rtp_depacketizer.cc \
	./src/modules/rtp_rtcp/source/video_rtp_depacketizer_generic.cc \
	./src/modules/rtp_rtcp/source/video_rtp_depacketizer_h264.cc \
	./src/modules/rtp_rtcp/source/video_rtp_depacketizer_raw.cc \
	./src/modules/rtp_rtcp/source/video_rtp_depacketizer_vp8.cc \
	./src/modules/rtp_rtcp/source/video_rtp_depacketizer_vp9.cc \
	./src/modules/rtp_rtcp/source/rtp_video_layers_allocation_extension.cc \
	./src/modules/rtp_rtcp/source/rtp_packet_received.cc \
	./src/modules/rtp_rtcp/source/rtcp_receiver.cc \
	./src/modules/rtp_rtcp/source/rtcp_nack_stats.cc \
	./src/modules/rtp_rtcp/source/rtp_sequence_number_map.cc \
	./src/modules/rtp_rtcp/source/rtcp_sender.cc \
	./src/modules/rtp_rtcp/source/rtp_packet_history.cc \
	./src/modules/rtp_rtcp/source/rtp_header_extension_size.cc \
	./src/modules/rtp_rtcp/source/ulpfec_generator.cc \
	./src/modules/rtp_rtcp/source/forward_error_correction.cc \
	./src/modules/rtp_rtcp/source/forward_error_correction_internal.cc \
	./src/modules/rtp_rtcp/source/time_util.cc \
	./src/modules/rtp_rtcp/source/tmmbr_help.cc \
	./src/modules/rtp_rtcp/source/rtp_utility.cc \
	./src/modules/rtp_rtcp/source/flexfec_header_reader_writer.cc \
	./src/modules/rtp_rtcp/source/ulpfec_header_reader_writer.cc \
	./src/modules/rtp_rtcp/source/fec_private_tables_bursty.cc \
	./src/modules/rtp_rtcp/source/fec_private_tables_random.cc \
	./src/modules/rtp_rtcp/source/receive_statistics_impl.cc \
	./src/modules/rtp_rtcp/source/flexfec_receiver.cc \
	./src/modules/rtp_rtcp/source/flexfec_sender.cc \
	./src/modules/rtp_rtcp/source/source_tracker.cc \
	./src/modules/rtp_rtcp/source/absolute_capture_time_receiver.cc \
	./src/modules/rtp_rtcp/source/remote_ntp_time_estimator.cc \
	./src/modules/video_coding/histogram.cc \
	./src/modules/remote_bitrate_estimator/remote_bitrate_estimator_abs_send_time.cc \
	./src/modules/remote_bitrate_estimator/aimd_rate_control.cc \
	./src/modules/remote_bitrate_estimator/inter_arrival.cc \
	./src/modules/remote_bitrate_estimator/overuse_detector.cc \
	./src/modules/remote_bitrate_estimator/overuse_estimator.cc \
	./src/modules/remote_bitrate_estimator/remote_bitrate_estimator_single_stream.cc \
	./src/modules/remote_bitrate_estimator/bwe_defines.cc \
	./src/modules/congestion_controller/goog_cc/robust_throughput_estimator.cc \
	./src/modules/congestion_controller/goog_cc/bitrate_estimator.cc \
	./src/modules/congestion_controller/goog_cc/acknowledged_bitrate_estimator.cc \
	./src/modules/congestion_controller/goog_cc/trendline_estimator.cc \
	./src/modules/congestion_controller/goog_cc/loss_based_bandwidth_estimation.cc \
	./src/modules/congestion_controller/goog_cc/send_side_bandwidth_estimation.cc \
	./src/modules/congestion_controller/goog_cc/probe_controller.cc \
	./src/modules/congestion_controller/goog_cc/delay_based_bwe.cc \
	./src/modules/congestion_controller/goog_cc/probe_bitrate_estimator.cc \
	./src/modules/congestion_controller/goog_cc/congestion_window_pushback_controller.cc \
	./src/modules/congestion_controller/goog_cc/inter_arrival_delta.cc \
	./src/modules/congestion_controller/goog_cc/link_capacity_estimator.cc \
	./src/modules/congestion_controller/goog_cc/acknowledged_bitrate_estimator_interface.cc \
	./src/modules/congestion_controller/goog_cc/alr_detector.cc \
	./src/modules/congestion_controller/goog_cc/goog_cc_network_control.cc \
	./src/modules/congestion_controller/goog_cc/loss_based_bwe_v2.cc \
	./src/modules/congestion_controller/rtp/control_handler.cc \
	./src/modules/congestion_controller/rtp/transport_feedback_demuxer.cc \
	./src/modules/congestion_controller/rtp/transport_feedback_adapter.cc \
	./src/modules/pacing/bitrate_prober.cc \
	./src/modules/pacing/interval_budget.cc \
	./src/modules/pacing/prioritized_packet_queue.cc \


#	./src/modules/rtp_rtcp/source/rtp_sender.cc \
#	./src/modules/remote_bitrate_estimator/test/bwe_test_logging.cc \
#	./src/modules/video_coding/nack_module2.cc \
#	./src/modules/rtp_rtcp/source/rtp_rtcp_impl2.cc \
#	./src/modules/rtp_rtcp/source/rtp_sender_egress.cc \
#	./src/modules/pacing/pacing_controller.cc \			# unused origin pacing
#	./src/modules/pacing/task_queue_paced_sender.cc		# unused origin pacing
#	./src/modules/pacing/round_robin_packet_queue.cc	# unused origin pacing

#	./src/modules/pacing/paced_sender.cc \		#unused
#	./src/modules/utility/source/process_thread_impl.cc \	#unused

LOGGING_SOURCES := \
	./src/logging/rtc_event_log/events/rtc_event_ice_candidate_pair.cc \
	./src/logging/rtc_event_log/events/rtc_event_ice_candidate_pair_config.cc \
	./src/logging/rtc_event_log/events/rtc_event_dtls_transport_state.cc \
	./src/logging/rtc_event_log/events/rtc_event_dtls_writable_state.cc \
	./src/logging/rtc_event_log/ice_logger.cc


SYSTEM_WRAPPERS_SOURES := \
	./src/system_wrappers/source/clock.cc \
	./src/system_wrappers/source/metrics.cc \
	./src/system_wrappers/source/field_trial.cc \
	./src/system_wrappers/source/cpu_features.cc \
	./src/system_wrappers/source/rtp_to_ntp_estimator.cc


COMMON_VIDEO_SOURCES := \
	./src/common_video/h264/h264_common.cc \
	./src/common_video/h264/pps_parser.cc \
	./src/common_video/h264/sps_parser.cc \
	./src/common_video/h264/sps_vui_rewriter.cc \
	./src/common_video/generic_frame_descriptor/generic_frame_info.cc

COMMON_AUDIO_SOURCES := \
	./src/common_audio/smoothing_filter.cc \
	./src/common_audio/audio_util.cc \
	./src/common_audio/third_party/ooura/fft_size_128/ooura_fft.cc \
	./src/common_audio/third_party/ooura/fft_size_128/ooura_fft_sse2.cc \
	./src/common_audio/third_party/ooura/fft_size_256/fft4g.cc



	
BASE_THIRD_PARTY_DOUBLE_CONVERSION_SOURCES := \
	./src/base/third_party/double_conversion/double_conversion/bignum.cc \
	./src/base/third_party/double_conversion/double_conversion/bignum-dtoa.cc \
	./src/base/third_party/double_conversion/double_conversion/cached-powers.cc \
	./src/base/third_party/double_conversion/double_conversion/double-to-string.cc \
	./src/base/third_party/double_conversion/double_conversion/fast-dtoa.cc \
	./src/base/third_party/double_conversion/double_conversion/fixed-dtoa.cc \
	./src/base/third_party/double_conversion/double_conversion/string-to-double.cc \
	./src/base/third_party/double_conversion/double_conversion/strtod.cc


MEDIA_SOURCES := \
	./src/media/base/rid_description.cc \
	./src/media/base/stream_params.cc \
	./src/media/base/media_constants.cc \
	./src/media/base/h264_profile_level_id.cc \
	./src/media/base/vp9_profile.cc \
	./src/media/base/codec.cc \
	./src/media/base/turn_utils.cc \
	./src/media/base/rtp_utils.cc \
	./src/media/base/fake_rtp.cc \
	./src/media/base/rtp_data_engine.cc \
	./src/media/sctp/sctp_transport.cc \
	./src/media/engine/payload_type_mapper.cc \
	./src/media/engine/unhandled_packets_buffer.cc \
	./src/media/engine/constants.cc


P2P_SOURCES := \
	./src/p2p/base/ice_credentials_iterator.cc \
	./src/p2p/base/p2p_constants.cc \
	./src/p2p/base/transport_description.cc \
	./src/p2p/base/transport_description_factory.cc \
	./src/p2p/base/stun_request.cc \
	./src/p2p/base/connection_info.cc \
	./src/p2p/base/connection.h \
	./src/p2p/base/port.cc \
	./src/p2p/base/port_interface.cc \
	./src/p2p/base/port_allocator.cc \
	./src/p2p/base/packet_transport_internal.cc \
	./src/p2p/base/dtls_transport_internal.cc \
	./src/p2p/base/dtls_transport.cc \
	./src/p2p/base/ice_transport_internal.cc \
	./src/p2p/base/basic_ice_controller.cc \
	./src/p2p/base/regathering_controller.cc \
	./src/p2p/base/ice_controller_interface.cc \
	./src/p2p/base/p2p_transport_channel.cc \
	./src/p2p/base/async_stun_tcp_socket.cc \
	./src/p2p/base/basic_packet_socket_factory.cc \
	./src/p2p/base/stun_port.cc \
	./src/p2p/base/tcp_port.cc \
	./src/p2p/base/turn_port.cc \
	./src/p2p/base/stun_server.cc \
	./src/p2p/base/turn_server.cc \
	./src/p2p/base/test_stun_server.cc \
	./src/p2p/base/basic_async_resolver_factory.cc \
	./src/p2p/base/default_ice_transport_factory.cc \
	./src/p2p/client/basic_port_allocator.cc \
	./src/p2p/client/turn_port_factory.cc



PC_SOURCES := \
	./src/pc/media_protocol_names.cc \
	./src/pc/session_description.cc \
	./src/pc/simulcast_description.cc \
	./src/pc/media_session.cc \
	./src/pc/sctp_utils.cc \
	./src/pc/sctp_data_channel_transport.cc \
	./src/pc/jsep_transport.cc \
	./src/pc/jsep_ice_candidate.cc \
	./src/pc/jsep_session_description.cc \
	./src/pc/jsep_transport_controller.cc \
	./src/pc/composite_rtp_transport.cc \
	./src/pc/srtp_session.cc \
	./src/pc/rtp_transport.cc \
	./src/pc/srtp_transport.cc \
	./src/pc/dtls_srtp_transport.cc \
	./src/pc/ice_transport.cc \
	./src/pc/dtls_transport.cc \
	./src/pc/rtcp_mux_filter.cc \
	./src/pc/sctp_transport.cc \
	./src/pc/srtp_filter.cc \
	./src/pc/transport_stats.cc \
	./src/pc/rtp_media_utils.cc \
	./src/pc/external_hmac.cc \
	./src/pc/sdp_serializer.cc \
	./src/pc/webrtc_sdp.cc \
	./src/pc/ice_server_parsing.cc \
	./src/pc/webrtc_session_description_factory.cc \
	./src/pc/local_audio_source.cc \
	./src/pc/audio_track.cc \
	./src/pc/media_stream.cc \
	./src/pc/remote_audio_source.cc \
	./src/pc/media_stream_observer.cc \
	./src/pc/data_channel_utils.cc \
	./src/pc/rtp_data_channel.cc \
	./src/pc/sctp_data_channel.cc

#	./src/pc/jitter_buffer_delay.cc \  	# unused


VIDEO_SOURCES := \
	./src/video/stats_counter.cc \
	./src/video/call_stats2.cc \
	./src/video/transport_adapter.cc



TEST_SOURCES := \
	./src/test/field_trial.cc
	

# not used
#RTC_UPDATE_DEP_SOURCES := \
#	./rtc_update_dep/call/_rtp_demuxer_def.cc

CC_SOURCES := $(API_SOURCES)
#CC_SOURCES += $(BASE_SOURCES)		# 추가 하지 않음...
#CC_SOURCES += $(AUDIO_SOURCES)
CC_SOURCES += $(CALL_SOURCES)
CC_SOURCES += $(LOGGING_SOURCES)
CC_SOURCES += $(SYSTEM_WRAPPERS_SOURES)
CC_SOURCES += $(MODULES_SOURCES)
CC_SOURCES += $(COMMON_VIDEO_SOURCES)
CC_SOURCES += $(COMMON_AUDIO_SOURCES)
CC_SOURCES += $(BASE_THIRD_PARTY_DOUBLE_CONVERSION_SOURCES)
CC_SOURCES += $(MEDIA_SOURCES)
CC_SOURCES += $(P2P_SOURCES)
CC_SOURCES += $(PC_SOURCES)
CC_SOURCES += $(RTC_BASE_SOURCES)
CC_SOURCES += $(VIDEO_SOURCES)
CC_SOURCES += $(TEST_SOURCES)


################################################
#  rtc media server sources

#	./rtc_pc_src/api/RTCConnectionInterface.cc \

RTC_MEDIA_SERVER_API := \
	./rtc_pc_src/api/ConferenceNode.cc \
	./rtc_pc_src/api/RTCNode.cc \
	./rtc_pc_src/api/RTCRtpTransceiverInterface.cc \
	./rtc_pc_src/api/RTCRtpReceiverInterface.cc \
	./rtc_pc_src/api/RTCRtpSenderInterface.cc \
	./rtc_pc_src/api/audio/RTCAudioRtpPacketListenerProxy.cc \
	./rtc_pc_src/api/video/RTCVideoRtpPacketListenerProxy.cc
	
RTC_MEDIA_SERVER_AUDIO := \
	./rtc_pc_src/audio/RTCAudioReceiveStreamImpl.cc \
	./rtc_pc_src/audio/RTCChannelRtpPacketReceiver.cc \
	./rtc_pc_src/audio/RTCAudioSendStreamImpl.cc \
	./rtc_pc_src/audio/RTCAudioPacketRouteFeeder.cc \
	./rtc_pc_src/audio/RTCAudioSendStreamRouter.cc


RTC_MEDIA_SERVER_PC := \
	./rtc_pc_src/pc/RTCConnection.cc \
	./rtc_pc_src/pc/RTCSdpOfferAnswerHandler.cc \
	./rtc_pc_src/pc/RTCCreateSdpObserver.cc \
	./rtc_pc_src/pc/RTCConnectionMessageHandler.cc \
	./rtc_pc_src/pc/RTCRtpTransceiver.cc \
	./rtc_pc_src/pc/RTCChannel.cc \
	./rtc_pc_src/pc/RTCChannelManager.cc \
	./rtc_pc_src/pc/RTCVideoRouteTrack.cc \
	./rtc_pc_src/pc/RTCVideoTrackSource.cc \
	./rtc_pc_src/pc/RTCAudioRouteTrack.cc \
	./rtc_pc_src/pc/RTCRtpReceiver.cc \
	./rtc_pc_src/pc/RTCRtpSender.cc \
	./rtc_pc_src/pc/RTCTransceiverList.cc \
	./rtc_pc_src/pc/RTCAudioRtpReceiver.cc \
	./rtc_pc_src/pc/RTCVideoRtpTrackSource.cc \
	./rtc_pc_src/pc/RTCVideoTrack.cc \
	./rtc_pc_src/pc/RTCVideoRtpReceiver.cc \
	./rtc_pc_src/pc/RTCRtpTransmissionManager.cc \
	./rtc_pc_src/pc/RTCDataChannelController.cc
	

RTC_MEDIA_SERVER_MEDIA := \
	./rtc_pc_src/media/base/RTCVideoRouteTrackSource.cc \
	./rtc_pc_src/media/base/RTCAudioRouteTrackSource.cc \
	./rtc_pc_src/media/engine/RTCWebRTCVoiceEngine.cc \
	./rtc_pc_src/media/engine/RTCWebRTCVideoEngine.cc

RTC_MEDIA_SERVER_CALL := \
	./rtc_pc_src/call/RTCAudioSendStream.cc \
	./rtc_pc_src/call/RTCAudioReceiveStream.cc \
	./rtc_pc_src/call/RTCVideoSendStream.cc \
	./rtc_pc_src/call/RTCVideoReceiveStream.cc \
	./rtc_pc_src/call/RTCCall.cc \
	./rtc_pc_src/call/RTCCallFactory.cc \
	./rtc_pc_src/call/RTCRtxReceiveStream.cc \
	./rtc_pc_src/call/RTCRtpTransportControllerSend.cc \
	./rtc_pc_src/call/RTCFlexFecReceiveStreamImpl.cc \
	./rtc_pc_src/call/RTCRtpStreamReceiverController.cc

	
RTC_MEDIA_SERVER_VIDEO := \
	./rtc_pc_src/video/RTCVideoReceiveStream2Impl.cc \
	./rtc_pc_src/video/RTCRtpVideoPacketStreamReceiver.cc \
	./rtc_pc_src/video/RTCVideoPacketRouteFeeder.cc \
	./rtc_pc_src/video/RTCVideoSendStreamRouter.cc \
	./rtc_pc_src/video/RTCVideoSendStreamImpl.cc

	
RTC_MEDIA_SERVER_MODULES := \
	./rtc_pc_src/modules/feedback/RTCFeedbackPacketRouter.cc \
	./rtc_pc_src/modules/remote_bitrate_estimator/RTCRemoteEstimatorProxy.cc \
	./rtc_pc_src/modules/congestion_controller/RTCReceiveSideCongestionController.cc \
	./rtc_pc_src/modules/rtp_rtcp/source/RTCRtxRtpPacketHistory.cc \
	./rtc_pc_src/modules/rtp_rtcp/source/RTCRtpRtcpSendImpl.cc \
	./rtc_pc_src/modules/rtp_rtcp/source/RTCRtcpReceiver.cc \
	./rtc_pc_src/modules/rtp_rtcp/source/RTCRtcpSender.cc \
	./rtc_pc_src/modules/rtp_rtcp/source/RTCRtpRtcpRecvImpl.cc \
	./rtc_pc_src/modules/rtp_rtcp/source/RTCRtpRtcpSendImpl.cc \
	./rtc_pc_src/modules/rtp_rtcp/source/RTCRtpPacketRedundancyChecker.cc \
	./rtc_pc_src/modules/video_coding/RTCNackModule2.cc \
	

#	./rtc_pc_src/modules/rtp_rtcp/source/RTCRtcpHandler.cc \		# unused
	


RTC_MEDIA_SERVER_SOURCES := $(RTC_MEDIA_SERVER_API)
RTC_MEDIA_SERVER_SOURCES += $(RTC_MEDIA_SERVER_AUDIO)
RTC_MEDIA_SERVER_SOURCES += $(RTC_MEDIA_SERVER_PC)
RTC_MEDIA_SERVER_SOURCES += $(RTC_MEDIA_SERVER_MEDIA)
RTC_MEDIA_SERVER_SOURCES += $(RTC_MEDIA_SERVER_CALL)
RTC_MEDIA_SERVER_SOURCES += $(RTC_MEDIA_SERVER_VIDEO)
RTC_MEDIA_SERVER_SOURCES += $(RTC_MEDIA_SERVER_MODULES)

CC_SOURCES += $(RTC_MEDIA_SERVER_SOURCES)
################################################


################################################
#  rtc update dep src
RTC_UPDATE_DEP_SRC_MEDIA := \
	./src_update/media/base/_media_channel.cc \
	./src_update/media/base/_media_engine.cc \
	./src_update/media/engine/_webrtc_media_engine.cc \
	./src_update/media/engine/_webrtc_media_engine_info.cc

	
RTC_UPDATE_DEP_SRC_API := \
	./src_update/api/_media_stream_interface.cc \
	./src_update/api/video_codecs/_video_codec.cc


RTC_UPDATE_DEP_SOURCES := $(RTC_UPDATE_DEP_SRC_API)
RTC_UPDATE_DEP_SOURCES += $(RTC_UPDATE_DEP_SRC_MEDIA)

CC_SOURCES += $(RTC_UPDATE_DEP_SOURCES)
################################################


COMMON_AUDIO_C_SOURCES := \
	./src/common_audio/third_party/spl_sqrt_floor/spl_sqrt_floor.c \
	./src/common_audio/signal_processing/spl_init.c \
	./src/common_audio/signal_processing/min_max_operations.c \
	./src/common_audio/signal_processing/cross_correlation.c \
	./src/common_audio/signal_processing/downsample_fast.c \
	./src/common_audio/signal_processing/vector_scaling_operations.c



LIBSRTP_SOURCES := \
	./src/third_party/libsrtp/crypto/cipher/aes_gcm_ossl.c \
	./src/third_party/libsrtp/crypto/cipher/aes_icm_ossl.c \
	./src/third_party/libsrtp/crypto/cipher/cipher.c \
	./src/third_party/libsrtp/crypto/cipher/null_cipher.c \
	./src/third_party/libsrtp/crypto/hash/auth.c \
	./src/third_party/libsrtp/crypto/hash/hmac_ossl.c \
	./src/third_party/libsrtp/crypto/hash/null_auth.c \
	./src/third_party/libsrtp/crypto/kernel/alloc.c \
	./src/third_party/libsrtp/crypto/kernel/crypto_kernel.c \
	./src/third_party/libsrtp/crypto/kernel/err.c \
	./src/third_party/libsrtp/crypto/kernel/key.c \
	./src/third_party/libsrtp/crypto/math/datatypes.c \
	./src/third_party/libsrtp/crypto/math/stat.c \
	./src/third_party/libsrtp/crypto/replay/rdb.c \
	./src/third_party/libsrtp/crypto/replay/rdbx.c \
	./src/third_party/libsrtp/crypto/replay/ut_sim.c \
	./src/third_party/libsrtp/srtp/ekt.c \
	./src/third_party/libsrtp/srtp/srtp.c


C_SOURCES := $(LIBSRTP_SOURCES)
C_SOURCES += $(COMMON_AUDIO_C_SOURCES)

TARGET := $(SO_PATH)/$(LIBRARY_NAME)

C_OBJECTS := $(addsuffix .o, $(basename $(C_SOURCES)))
CPP_OBJECTS := $(addsuffix .o, $(basename $(CC_SOURCES)))


.SUFFIXES : .o .c .cc
%.o : %.c
	@echo ""
	@echo $< "=>" $@
	@$(C) -fPIC $(LIBSRTP_CFLAGS) $(SRTP_INC) $(INC) -o $@ -c $<
	
%.o : %.cc
	@echo ""
	@echo $< "=>" $@
	@$(CC) -std=c++17 -fPIC $(CFLAGS) $(LIBSRTP_CFLAGS) -DNDEBUG -Werror=return-type -Wno-multichar -fno-exceptions -fno-rtti -fno-use-cxa-atexit $(INC) -c $< -o $@

$(TARGET) : $(C_OBJECTS) $(CPP_OBJECTS)
	@echo $@ Linking...
	@$(CC) $(LIB_PATH) -shared -Wl,-soname,$(LIBRARY_NAME) -Wl,-Bsymbolic $^ $(ABSEIL_LIBS) $(BORING_SSL_LIBS) $(USR_SCTP_LIBS) -lpthread -o $@ -Wl, --no-undefined	
#	@$(CC) $(LIB_PATH) -shared -Wl,-soname,$(LIBRARY_NAME) -Wl,-Bsymbolic $^ $(ABSEIL_LIBS) $(BORING_SSL_LIBS) $(USR_SCTP_LIBS) $(OPUS_LIBS) -lpthread -o $@ -Wl, --no-undefined
#	@$(CC) $(LIB_PATH) -shared -stdlib=libstdc++ -Wl,-soname,$(LIBRARY_NAME) -Wl,-Bsymbolic $^ $(ABSEIL_LIBS) -lpthread -o $@ -Wl, --no-undefined
	@echo $@ build ok!
#	cp $(TARGET) ../RTCMediaServer/$(LIBRARY_NAME)

clean:
	rm -f $(TARGET) $(C_OBJECTS) $(CPP_OBJECTS)

