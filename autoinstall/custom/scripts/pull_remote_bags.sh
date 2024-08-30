#!/bin/bash

# Function to parse file names
parse_filename() {
  full_path="$1"
  filename=$(basename "$full_path")

  # Use regular expressions to extract the fields
  if [[ $filename =~ ([0-9]{2})([0-9]{2})([0-9]{2})_([0-9]{2})([0-9]{2})([0-9]{2})_.*_([0-9]{1,2})\.mcap ]]; then
    YY="${BASH_REMATCH[1]}"
    MM="${BASH_REMATCH[2]}"
    DD="${BASH_REMATCH[3]}"
    hh="${BASH_REMATCH[4]}"
    mm="${BASH_REMATCH[5]}"
    ss="${BASH_REMATCH[6]}"
    num="${BASH_REMATCH[7]}"

    # Calculate new_mm by adding mm and num
    new_mm=$(printf "%02d" $((mm + num)))

    echo "$YY$MM${DD}_$hh${new_mm}00"
  else
    echo "Invalid file name format: $filename"
  fi
}

# Remote machine details
remote_user="its"
remote_host="laarma-01"
remote_dir='/mnt/ssd/logging/laarma_throttled'

# Change this local directory to where you want to save the local bag files and the list of file paths
local_dir="."

remote_list="$local_dir/remote_mcap_list.csv"
echo collecting remote dir $remote_dir
echo "RemoteFilePath, SizeKB" > "$remote_list"
ssh "$remote_user@$remote_host" "find $remote_dir -name *.mcap -printf '%p, %k\n' | sort -h" >> "$remote_list"

local_list="$local_dir/local_mcap_list.csv"
echo collecting local dir $local_dir
echo "LocalFilePath, SizeKB" > "$local_list"
find $local_dir -name *.mcap -printf '%p, %k\n' | sort -h >> "$local_list"

echo parsing remote
remote_parsed_list="$local_dir/remote_mcap_list_parsed.csv"
echo "RemoteFilePath, SizeKB, ParsedFileName" > "$remote_parsed_list"
tail -n +2 "$remote_list" > "$local_dir/remote_mcap_list.tmp"
while IFS=', ' read -ra path_size; do
  parsed_filename=$(parse_filename "${path_size[0]}")
  echo "${path_size[0]}, ${path_size[1]}, $parsed_filename" >> "$remote_parsed_list"
done < "$local_dir/remote_mcap_list.tmp"
rm "$local_dir/remote_mcap_list.tmp"

echo parsing local
local_parsed_list="$local_dir/local_mcap_list_parsed.csv"
echo "LocalFilePath, SizeKB, ParsedFileName" > "$local_parsed_list"
tail -n +2 "$local_list" > "$local_dir/local_mcap_list.tmp"
while IFS=', ' read -ra path_size; do
  parsed_filename=$(parse_filename "${path_size[0]}")
  echo "${path_size[0]}, ${path_size[1]}, $parsed_filename" >> "$local_parsed_list"
done < "$local_dir/local_mcap_list.tmp"
rm "$local_dir/local_mcap_list.tmp"

echo matching
stamp_list="$local_dir/stamp_list"
matched_list="$local_dir/stamp_list_matched.csv"
echo "Stamp, MatchedRemoteFilePath, RemoteSizeKB, LocalSizeKB, ToDownload" > "$matched_list"
tail -n +2 "$remote_parsed_list" > "$local_dir/remote_parsed_list.tmp"
tail -n +2 "$local_parsed_list" > "$local_dir/local_parsed_list.tmp"
while IFS= read -r stamp; do
  while IFS=', ' read -ra remote_path_size; do
    #echo $stamp vs ${remote_path_size[2]}
    if [[ $stamp == ${remote_path_size[2]} ]]; then
      local_size=0
      while IFS=', ' read -ra local_path_size; do
        if [[ $stamp == ${local_path_size[2]} ]]; then
          local_size=${local_path_size[1]}
          break
        fi
      done < "$local_dir/local_parsed_list.tmp"
      if [[ ${remote_path_size[1]} -gt $local_size ]]; then
        to_download="True"
      else
        to_download="False"
      fi
      echo "$stamp, ${remote_path_size[0]}, ${remote_path_size[1]}, $local_size, $to_download" >> "$matched_list"
      break
    fi
  done < "$local_dir/remote_parsed_list.tmp"
done < "$stamp_list"
rm "$local_dir/remote_parsed_list.tmp"
rm "$local_dir/local_parsed_list.tmp"

echo downloading
tail -n +2 "$matched_list" > "$local_dir/stamp_list_matched.tmp"
while IFS=', ' read -ra remote_path_size; do
    if [[ ${remote_path_size[4]} -eq "True" ]]; then
      echo scp "$remote_user@$remote_host:${remote_path_size[1]} $local_dir"
    fi
done < "$local_dir/stamp_list_matched.tmp"
rm "$local_dir/stamp_list_matched.tmp"
exit 0
