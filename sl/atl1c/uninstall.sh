#!/bin/sh
cd build
cd Debug
echo "file attribute adjusting for  AtherosL1cEthernet.kext"
sudo chown -R root:wheel  /System/Library/Extensions/AtherosL1cEthernet.kext
sudo find  /System/Library/Extensions/AtherosL1cEthernet.kext -type d -exec chmod 0755 {} \;
sudo find  /System/Library/Extensions/AtherosL1cEthernet.kext -type f -exec chmod 0644 {} \; 

sudo kextunload  /System/Library/Extensions/AtherosL1cEthernet.kext
sudo rm -R /System/Library/Extensions/AtherosL1cEthernet.kext

echo "restore file attributes..."
sudo kextcache -system-mkext /System/Library/Extensions
