#!/bin/sh
echo "file attribute adjusting for  AtherosL1Ethernet.kext"
sudo chown -R root:wheel  /System/Library/Extensions/AtherosL1Ethernet.kext
sudo find  /System/Library/Extensions/AtherosL1Ethernet.kext -type d -exec chmod 0755 {} \;
sudo find  /System/Library/Extensions/AtherosL1Ethernet.kext -type f -exec chmod 0644 {} \; 

sudo kextunload  /System/Library/Extensions/AtherosL1Ethernet.kext
sudo rm -R /System/Library/Extensions/AtherosL1Ethernet.kext

echo "restore file attributes..."
sudo kextcache -system-mkext /System/Library/Extensions