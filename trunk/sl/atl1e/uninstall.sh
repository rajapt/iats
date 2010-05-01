#!/bin/sh
echo "file attribute adjusting for  AtherosL1eEthernet.kext"
sudo chown -R root:wheel  /System/Library/Extensions/AtherosL1eEthernet.kext
sudo find  /System/Library/Extensions/AtherosL1eEthernet.kext -type d -exec chmod 0755 {} \;
sudo find  /System/Library/Extensions/AtherosL1eEthernet.kext -type f -exec chmod 0644 {} \; 

sudo kextunload  /System/Library/Extensions/AtherosL1eEthernet.kext
sudo rm -R /System/Library/Extensions/AtherosL1eEthernet.kext

echo "restore file attributes..."
sudo kextcache -system-mkext /System/Library/Extensions