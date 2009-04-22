#!/bin/sh
cd build
cd Debug
echo "file attribute adjusting for  AttansicL2Ethernet.kext"

sudo rm -rf /System/Library/Extensions.mkext
sudo rm -rf /System/Library/Extensions.kextcache

sudo cp -R AttansicL2Ethernet.kext /System/Library/Extensions
sudo chown -R root:wheel AttansicL2Ethernet.kext
sudo find /System/Library/Extensions/AttansicL2Ethernet.kext -type d -exec chmod 0755 {} \;
sudo find /System/Library/Extensions/AttansicL2Ethernet.kext -type f -exec chmod 0644 {} \; 

sudo kextload -t AttansicL2Ethernet.kext
sudo kextcache -k /System/Library/Extensions

