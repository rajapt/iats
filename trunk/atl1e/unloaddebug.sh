#!/bin/sh
USER=`whoami`
cd build
cd Debug
echo "file attribute adjusting for  AttansicL1eEthernet.kext"
sudo chown -R root:wheel AttansicL1eEthernet.kext
sudo find AttansicL1eEthernet.kext -type d -exec chmod 0755 {} \;
sudo find AttansicL1eEthernet.kext -type f -exec chmod 0644 {} \; 

sudo kextunload AttansicL1eEthernet.kext

echo "restore file attributes..."
sudo chown -R $USER AttansicL1eEthernet.kext
sudo kextcache -k /System/Library/Extensions
