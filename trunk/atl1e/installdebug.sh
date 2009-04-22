#!/bin/sh
cd build
cd Debug
echo "file attribute adjusting for  AttansicL1eEthernet.kext"

sudo rm -rf /System/Library/Extensions.mkext
sudo rm -rf /System/Library/Extensions.kextcache

sudo cp -R AttansicL1eEthernet.kext /System/Library/Extensions
sudo chown -R root:wheel AttansicL1eEthernet.kext
sudo find /System/Library/Extensions/AttansicL1eEthernet.kext -type d -exec chmod 0755 {} \;
sudo find /System/Library/Extensions/AttansicL1eEthernet.kext -type f -exec chmod 0644 {} \; 

sudo kextload -t AttansicL1eEthernet.kext
sudo kextcache -k /System/Library/Extensions

