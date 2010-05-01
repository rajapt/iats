#!/bin/sh
cd build
cd Release
echo "file attribute adjusting for  AtherosL1eEthernet.kext"

sudo rm -rf /System/Library/Extensions.mkext
sudo rm -rf /System/Library/Extensions.kextcache

sudo cp -R AtherosL1eEthernet.kext /System/Library/Extensions
sudo chown -R root:wheel AtherosL1eEthernet.kext
sudo find /System/Library/Extensions/AtherosL1eEthernet.kext -type d -exec chmod 0755 {} \;
sudo find /System/Library/Extensions/AtherosL1eEthernet.kext -type f -exec chmod 0644 {} \; 

sudo kextload -t AtherosL1eEthernet.kext
sudo kextcache -system-mkext /System/Library/Extensions

