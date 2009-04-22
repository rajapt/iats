#!/bin/sh
USER=`whoami`
cd build
cd Release
echo "file attribute adjusting for  AttansicL1Ethernet.kext"
sudo chown -R root:wheel AttansicL1Ethernet.kext
sudo find AttansicL1Ethernet.kext -type d -exec chmod 0755 {} \;
sudo find AttansicL1Ethernet.kext -type f -exec chmod 0644 {} \; 

sudo kextunload AttansicL1Ethernet.kext

echo "restore file attributes..."
sudo chown -R $USER AttansicL1Ethernet.kext
sudo kextcache -k /System/Library/Extensions
