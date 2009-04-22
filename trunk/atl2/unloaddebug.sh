#!/bin/sh
USER=`whoami`
cd build
cd Debug
echo "file attribute adjusting for  AttansicL2Ethernet.kext"
sudo chown -R root:wheel AttansicL2Ethernet.kext
sudo find AttansicL2Ethernet.kext -type d -exec chmod 0755 {} \;
sudo find AttansicL2Ethernet.kext -type f -exec chmod 0644 {} \; 

sudo kextunload AttansicL2Ethernet.kext

echo "restore file attributes..."
sudo chown -R $USER AttansicL2Ethernet.kext
sudo kextcache -k /System/Library/Extensions
