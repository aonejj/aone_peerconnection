# RTCPeerConnection (Server-Side)

A customized server-oriented implementation of Google's WebRTC Native `PeerConnection` designed for SFU/MCU media servers and real-time communication platforms.

This project provides a lightweight server-side PeerConnection core by restructuring Google's WebRTC codebase while keeping compatibility with the original API and build system.

---

## Project Structure

The repository is organized as follows:

- `rtc_pc_src/`   : New and replacement classes for server-side PeerConnection
- `src/`          : Google WebRTC origin source with unnecessary components removed & minimal patches
- `src_update/`   : Additional updates/patches for specific server features

Layout follows the Google WebRTC source tree to simplify navigation and future updates.

---

## Features

- **Server-Optimized PeerConnection**: Minimal client-side dependencies, tuned for SFU/MCU servers.
- **Modular Design**: Clean separation of new/modified classes from original Google sources.
- **Native Build System**: Uses GN + Ninja build flow as in Google WebRTC.
- **Google-Compatible Licensing**: Distributed under BSD-3-Clause.

---

## Build Environment

- **OS**: Ubuntu 21.04 or later
- **Compiler**: Clang (required)
- **Build System**: GN + Ninja
- **Python 3** (for GN scripts)

### Build Steps

1. Clone repository

   ```bash
   git clone https://github.com/aonejj/aone_peerconnection.git
   cd aone_peerconnection

2. Generate build files

   ```bash
   gn gen out/Default --args='is_debug=false'2. Generate build files

2. Generate build files

   ```bash
   ninja -C out/Default

## License

  This project is licensed under the BSD-3-Clause license.
  See the LICENSE file for full details.
  This project includes code derived from Google WebRTC, which is also licensed under BSD-3-Clause.

---

## Contact

Email: jacques97jj@gmail.com

[LinkedIn](https://www.linkedin.com/in/%EC%A4%91%EC%A0%9C-%EC%9E%A5-71a6b010b/)

---

## Contribution

This project is currently in a read-only state for evaluation and reference. Issues and discussions are welcome.

---

## Acknowledgments

This work builds upon Google WebRTC and respects its original design and licensing.

---
