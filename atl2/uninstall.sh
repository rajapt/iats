#!/bin/sh
cd build
cd Debug
echo "file attribute adjusting for  AttansicL2Ethernet.kext"
sudo chown -R root:wheel  /System/Library/Extensions/AttansicL2Ethernet.kext
sudo find  /System/Library/Extensions/AttansicL2Ethernet.kext -type d -exec chmod 0755 {} \;
sudo find  /System/Library/Extensions/AttansicL2Ethernet.kext -type f -exec chmod 0644 {} \; 

sudo kextunload  /System/Library/Extensions/AttansicL2Ethernet.kext
sudo rm -R /System/Library/Extensions/AttansicL2Ethernet.kext

echo "restore file attributes..."
sudo chown -R `whoami` /System/Library/Extensions/AttansicL2Ethernet.kext
sudo kextcache -k /System/Library/Extensions