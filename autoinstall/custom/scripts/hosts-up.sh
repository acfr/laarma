#!/bin/bash

# Default ping command wasn't waiting for the specified 60 seconds, so fping is used
# Wait up to 60 seconds for a sensor if it hasn't come online
fping -c1 -t60000 192.168.1.1 # router
fping -c1 -t60000 cam-near.lan
fping -c1 -t60000 cam-far.lan
fping -c1 -t60000 cam-thermal.lan
fping -c1 -t60000 192.168.1.101 # neuvition lidar

# Return a success exit code (0) so systemd is happy
exit 0
