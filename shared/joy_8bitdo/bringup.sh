#!/bin/bash

# 1. Install dependencies
apt update 
apt install build-essential cmake libudev-dev

# 2. Clone repository
git clone https://github.com/errorcodecritical/MoltenGamepad

# 3. Build and install
cd MoltenGamepad
make eventlists
make CXXFLAGS="-fno-PIE" LDFLAGS="-no-pie"

# 4. Run the daemon
./moltengamepad --mimic-xpad --daemon