#!/bin/sh
cd build
cd Release
echo "file attribute adjusting for  AtherosL1Ethernet.kext"

sudo chown -R root:wheel AtherosL1Ethernet.kext
sudo find AtherosL1Ethernet.kext -type d -exec chmod 0755 {} \;
sudo find AtherosL1Ethernet.kext -type f -exec chmod 0644 {} \; 

sudo kextload -t AtherosL1Ethernet.kext

