#!/bin/bash

# Setup file structure
mkdir -p /mnt/ssd/ml && mkdir -p /mnt/ssd/logging # directories for machine learning data and logging
mkdir -p /etc/systemd/system && cp -r ./custom/systemd/* /etc/systemd/system

# Increase UDP socket buffer
echo 'net.core.rmem_default=26214400' >> /etc/sysctl.conf
echo 'net.core.rmem_max=26214400' >> /etc/sysctl.conf
echo 'net.core.wmem_default=26214400' >> /etc/sysctl.conf
echo 'net.core.wmem_max=26214400' >> /etc/sysctl.conf
echo 'net.ipv4.ipfrag_time=3' >> /etc/sysctl.conf
echo 'net.ipv4.ipfrag_high_thresh=134217728' >> /etc/sysctl.conf
sysctl -p

# Install netplan & config mtu for eth0
apt update
apt install -y netplan.io
mkdir -p /etc/netplan
cp -r ./custom/netplan/* /etc/netplan
netplan apply

# Install & config ntp service
apt install -y chrony ntpdate
echo server 192.168.1.1 prefer iburst minpoll 0 maxpoll 5 maxdelay 0.005 >> /etc/chrony/chrony.conf
# use 'chronyc sources' and 'ntpdate -q 192.168.1.1' to check if chrony is working

# Install & config ptp service
apt install -y linuxptp
cp /etc/linuxptp/ptp4l.conf /etc/linuxptp/ptp4l.conf.backup
sed -i "s/clockClass.*/clockClass 128/g" /etc/linuxptp/ptp4l.conf
sed -i "s/tx_timestamp_timeout.*/tx_timestamp_timeout 1000/g" /etc/linuxptp/ptp4l.conf

# Enable system services
systemctl enable ptp4l
systemctl enable sensors
systemctl daemon-reload

# Pull Docker images while the network is up
apt install -y python3 python3-pip libffi-dev python-openssl libssl-dev fping
pip3 install docker-compose
pull_image.sh

# Setup crontab (requires AWS credentials)
# cp /home/its/autoinstall/custom/cron/laarma_cron /etc/cron.d