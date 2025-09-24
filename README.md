# aone_peerconnection
Server PeerConnection (for SFU / RTC Server)

This repository provides a server-oriented RTCPeerConnection implementation
based on the Google WebRTC Native source code.
It is designed for use in SFU (Selective Forwarding Unit) servers,
custom RTC servers, or as the core of a server-side SDK,
where a lightweight, headless PeerConnection is required.

📂 Project Structure
RTCPeerConnection/
├─ rtc_pc_src/   # Custom classes replacing or extending Google WebRTC classes
├─ src/          # Google origin code with unused modules removed and light modifications
└─ src_update/   # Additional updates / patches


Each directory follows the original Google WebRTC folder layout,
making it easier to track upstream changes and compare with the official source.

🔧 Build Environment
Operating System: Ubuntu 21.04 or later (tested on Ubuntu 21/22)
Compiler:  Clang (standard Ubuntu packages)
Build System: GN + Ninja

Dependency: Google WebRTC Native Source (BSD-3-Clause)

Example Build Steps
# Generate GN build files
gn gen out/Default --args='is_debug=false'

# Compile using Ninja
ninja -C out/Default


Adjust gn args according to your environment or specific features you need.

⚡ Key Features

Server-side PeerConnection
Enables direct use of WebRTC PeerConnection in a headless server context.

Lightweight Core
Removes unnecessary client-side modules to reduce binary size and dependencies.

Extensible for SFU/MCU
Serves as a foundation for custom SFU servers, SDKs, or real-time RTC applications.

📜 License
This project is released under the BSD-3-Clause license.
The Google WebRTC source also uses BSD-3-Clause.

You must retain the original copyright notice,
the BSD-3-Clause license text, and the disclaimer in any redistributions.

BSD-3-Clause Summary:
✅ Commercial use, modification, redistribution, and closed-source integration are allowed.
✅ You must keep copyright and license notices in all copies.

The LICENSE file in this repository includes both
the original Google WebRTC license and this project’s BSD-3-Clause license.

⚠️ Notice
This project is an independent modification of Google WebRTC Native.
Google does not maintain or guarantee the quality of this code.
Use it at your own risk and ensure compliance with all licensing requirements.

📬 Contact
Email: jacques97jj@gmail.com
LinkedIn: LinkedIn Profile

Suggestions for Contributors
When submitting pull requests, please clearly document
any changes to the build process or folder structure.

Keep modifications isolated to simplify upstream WebRTC updates.
