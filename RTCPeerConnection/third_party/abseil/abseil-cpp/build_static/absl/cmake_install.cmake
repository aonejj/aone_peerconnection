# Install script for directory: /home/kimi/aone_rtc_sfu/RTCPeerConnection/third_party/abseil/abseil-cpp/absl

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/kimi/aone_rtc_sfu/RTCPeerConnection/third_party/abseil/abseil-cpp/build_static/absl/base/cmake_install.cmake")
  include("/home/kimi/aone_rtc_sfu/RTCPeerConnection/third_party/abseil/abseil-cpp/build_static/absl/algorithm/cmake_install.cmake")
  include("/home/kimi/aone_rtc_sfu/RTCPeerConnection/third_party/abseil/abseil-cpp/build_static/absl/cleanup/cmake_install.cmake")
  include("/home/kimi/aone_rtc_sfu/RTCPeerConnection/third_party/abseil/abseil-cpp/build_static/absl/container/cmake_install.cmake")
  include("/home/kimi/aone_rtc_sfu/RTCPeerConnection/third_party/abseil/abseil-cpp/build_static/absl/crc/cmake_install.cmake")
  include("/home/kimi/aone_rtc_sfu/RTCPeerConnection/third_party/abseil/abseil-cpp/build_static/absl/debugging/cmake_install.cmake")
  include("/home/kimi/aone_rtc_sfu/RTCPeerConnection/third_party/abseil/abseil-cpp/build_static/absl/flags/cmake_install.cmake")
  include("/home/kimi/aone_rtc_sfu/RTCPeerConnection/third_party/abseil/abseil-cpp/build_static/absl/functional/cmake_install.cmake")
  include("/home/kimi/aone_rtc_sfu/RTCPeerConnection/third_party/abseil/abseil-cpp/build_static/absl/hash/cmake_install.cmake")
  include("/home/kimi/aone_rtc_sfu/RTCPeerConnection/third_party/abseil/abseil-cpp/build_static/absl/log/cmake_install.cmake")
  include("/home/kimi/aone_rtc_sfu/RTCPeerConnection/third_party/abseil/abseil-cpp/build_static/absl/memory/cmake_install.cmake")
  include("/home/kimi/aone_rtc_sfu/RTCPeerConnection/third_party/abseil/abseil-cpp/build_static/absl/meta/cmake_install.cmake")
  include("/home/kimi/aone_rtc_sfu/RTCPeerConnection/third_party/abseil/abseil-cpp/build_static/absl/numeric/cmake_install.cmake")
  include("/home/kimi/aone_rtc_sfu/RTCPeerConnection/third_party/abseil/abseil-cpp/build_static/absl/profiling/cmake_install.cmake")
  include("/home/kimi/aone_rtc_sfu/RTCPeerConnection/third_party/abseil/abseil-cpp/build_static/absl/random/cmake_install.cmake")
  include("/home/kimi/aone_rtc_sfu/RTCPeerConnection/third_party/abseil/abseil-cpp/build_static/absl/status/cmake_install.cmake")
  include("/home/kimi/aone_rtc_sfu/RTCPeerConnection/third_party/abseil/abseil-cpp/build_static/absl/strings/cmake_install.cmake")
  include("/home/kimi/aone_rtc_sfu/RTCPeerConnection/third_party/abseil/abseil-cpp/build_static/absl/synchronization/cmake_install.cmake")
  include("/home/kimi/aone_rtc_sfu/RTCPeerConnection/third_party/abseil/abseil-cpp/build_static/absl/time/cmake_install.cmake")
  include("/home/kimi/aone_rtc_sfu/RTCPeerConnection/third_party/abseil/abseil-cpp/build_static/absl/types/cmake_install.cmake")
  include("/home/kimi/aone_rtc_sfu/RTCPeerConnection/third_party/abseil/abseil-cpp/build_static/absl/utility/cmake_install.cmake")

endif()

