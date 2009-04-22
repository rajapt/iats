#!/bin/sh
cd build
cd Release
echo "file attribute adjusting for  AttansicL2Ethernet.kext"

sudo rm -rf /System/Library/Extensions.mkext
sudo rm -rf /System/Library/Extensions.kextcache

sudo chown -R root:wheel AttansicL2Ethernet.kext
sudo find AttansicL2Ethernet.kext -type d -exec chmod 0755 {} \;
sudo find AttansicL2Ethernet.kext -type f -exec chmod 0644 {} \; 

sudo kextload -t AttansicL2Ethernet.kext
sudo kextcache -k /System/Library/Extensions

