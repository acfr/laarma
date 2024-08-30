#!/bin/bash

rm *.mcap
while IFS= read -r line; do
	echo processing ${line}
	full_path=${line} #$(find /mnt/ssd/logging/laarma_throttled -name ${line})
	if [[ -z "${full_path}" ]]; then
		echo file NOT found, skip
	else
		echo file FOUND with full path ${full_path}
		ln -sf ${full_path} ${file}
		aws s3 sync --exclude "*" --include "*.mcap" . s3://your-s3-bucket/laarma_throttled/
	fi
done
echo syncing files with aws
#aws s3 sync --exclude "*" --include "*.mcap" . s3://your-s3-bucket/laarma_throttled/
#echo $?
rm *.mcap
