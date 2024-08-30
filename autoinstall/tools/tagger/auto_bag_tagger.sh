#!/bin/bash

echo $1
docker run --rm	\
	-v /mnt/ssd/logging:/mnt/ssd/logging \
	-v /home/its/autoinstall/tools:/mnt/tools \
	laarma:latest \
	/bin/bash -c "find /mnt/ssd/logging/laarma_throttled/ -name ${1}*.mcap | python3 /mnt/tools/tagger/auto_bag_tagger.py > /mnt/ssd/logging/tagged_bags_${1}.txt"
