#!/bin/sh
cd build
cd Debug
echo "file attribute adjusting for  AttansicL1eEthernet.kext"
sudo chown -R root:wheel  /System/Library/Extensions/AttansicL1eEthernet.kext
sudo find  /System/Library/Extensions/AttansicL1eEthernet.kext -type d -exec chmod 0755 {} \;
sudo find  /System/Library/Extensions/AttansicL1eEthernet.kext -type f -exec chmod 0644 {} \; 

sudo kextunload  /System/Library/Extensions/AttansicL1eEthernet.kext
sudo rm -R /System/Library/Extensions/AttansicL1eEthernet.kext

echo "restore file attributes..."
sudo chown -R `whoami` /System/Library/Extensions/AttansicL1eEthernet.kext
sudo kextcache -k /System/Library/Extensions