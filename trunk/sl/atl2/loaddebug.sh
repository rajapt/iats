#!/bin/sh
cd build
cd Debug
echo "file attribute adjusting for  AtherosL2Ethernet.kext"

sudo chown -R root:wheel AtherosL2Ethernet.kext
sudo find AtherosL2Ethernet.kext -type d -exec chmod 0755 {} \;
sudo find AtherosL2Ethernet.kext -type f -exec chmod 0644 {} \; 

sudo kextload -t AtherosL2Ethernet.kext

