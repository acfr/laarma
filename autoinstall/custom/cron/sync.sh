#!/bin/bash

/usr/local/bin/aws s3 sync --exclude "*" --include "*.mp4" /mnt/ssd/logging/laarma_throttled/ s3://your-s3-bucket/laarma_throttled/
/usr/local/bin/aws s3 sync --exclude "*" --include "voltage.txt" /home/its/autoinstall/custom/cron s3://your-s3-bucket/voltage
/usr/local/bin/aws s3 sync --exclude "*" --include "log.txt" /mnt/ssd/logging/signal s3://your-s3-bucket/signal
