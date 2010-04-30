#!/bin/sh
cd build
cd Release
echo "file attribute adjusting for  AtherosL1cEthernet.kext"

sudo rm -rf /System/Library/Extensions.mkext
sudo rm -rf /System/Library/Extensions.kextcache

sudo cp -R AtherosL1cEthernet.kext /System/Library/Extensions
sudo chown -R root:wheel AtherosL1cEthernet.kext
sudo find /System/Library/Extensions/AtherosL1cEthernet.kext -type d -exec chmod 0755 {} \;
sudo find /System/Library/Extensions/AtherosL1cEthernet.kext -type f -exec chmod 0644 {} \; 

sudo kextload -t AtherosL1cEthernet.kext
sudo kextcache -system-mkext /System/Library/Extensions

