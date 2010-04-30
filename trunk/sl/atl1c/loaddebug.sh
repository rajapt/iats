#!/bin/sh
cd build
cd Debug
echo "file attribute adjusting for  AtherosL1cEthernet.kext"

sudo chown -R root:wheel AtherosL1cEthernet.kext
sudo find AtherosL1cEthernet.kext -type d -exec chmod 0755 {} \;
sudo find AtherosL1cEthernet.kext -type f -exec chmod 0644 {} \; 

sudo kextload -t AtherosL1cEthernet.kext


