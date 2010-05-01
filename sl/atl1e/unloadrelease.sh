#!/bin/sh
USER=`whoami`
cd build
cd Release
echo "file attribute adjusting for  AtherosL1eEthernet.kext"
sudo chown -R root:wheel AtherosL1eEthernet.kext
sudo find AtherosL1eEthernet.kext -type d -exec chmod 0755 {} \;
sudo find AtherosL1eEthernet.kext -type f -exec chmod 0644 {} \; 

sudo kextunload AtherosL1eEthernet.kext

echo "restore file attributes..."
sudo chown -R $USER AtherosL1eEthernet.kext

