#!/bin/bash

# Get the CPU architecture
arch=$(dpkg --print-architecture)

if [[ $arch == "x86_64" || $arch == "amd64" ]]; then
    echo "x86_64 architecture detected."
    tar -xvzf arena_sdk/ArenaSDK_v0.1.68_Linux_x64.tar.gz --directory /opt
    cd /opt/ArenaSDK_Linux_x64
    /bin/bash Arena_SDK_Linux_x64.conf
elif [[ $arch == "arm64" ]]; then
    echo "arm64 architecture detected."
    tar -xvzf arena_sdk/ArenaSDK_v0.1.49_Linux_ARM64.tar.gz --directory /opt
    cd /opt/ArenaSDK_Linux_ARM64
    /bin/bash Arena_SDK_ARM64.conf
else
    echo "Unsupported architecture: $arch"
fi

