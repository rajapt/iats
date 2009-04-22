#!/bin/sh
cd build
cd Debug
echo "file attribute adjusting for  AttansicL1Ethernet.kext"

sudo rm -rf /System/Library/Extensions.mkext
sudo rm -rf /System/Library/Extensions.kextcache

sudo chown -R root:wheel AttansicL1Ethernet.kext
sudo find AttansicL1Ethernet.kext -type d -exec chmod 0755 {} \;
sudo find AttansicL1Ethernet.kext -type f -exec chmod 0644 {} \; 

sudo kextload -t AttansicL1Ethernet.kext
sudo kextcache -k /System/Library/Extensions

