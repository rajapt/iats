#!/bin/sh
cd build
cd Debug
echo "file attribute adjusting for  AtherosL1eEthernet.kext"

sudo chown -R root:wheel AtherosL1eEthernet.kext
sudo find AtherosL1eEthernet.kext -type d -exec chmod 0755 {} \;
sudo find AtherosL1eEthernet.kext -type f -exec chmod 0644 {} \; 

sudo kextload -t AtherosL1eEthernet.kext


