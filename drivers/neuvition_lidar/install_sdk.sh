#!/bin/bash

# Get the CPU architecture
arch=$(dpkg --print-architecture)

tmp_dir="/tmp/neuvition"
install_dir="/opt/neuvition"

mkdir -p ${tmp_dir}
if [[ $arch == "x86_64" || $arch == "amd64" ]]; then
    echo "x86_64 architecture detected."
    mkdir -p ${tmp_dir}/neusdk-aarch64-3.0.9
    cp neuvition_sdk/neusdk-x86_64-3.0.9.zip ${tmp_dir}/neusdk-aarch64-3.0.9/neusdk.zip
    cd ${tmp_dir}/neusdk-aarch64-3.0.9/
    jar -xvf neusdk.zip
elif [[ $arch == "arm64" ]]; then
    echo "arm64 architecture detected."
    tar -xvzf neuvition_sdk/neusdk-aarch64-3.1.0.tar.gz --directory ${tmp_dir}
    cp neuvition_sdk/libneusdk-arm20.04.so ${tmp_dir}/neusdk-aarch64-3.0.9/lib
else
    echo "Unsupported architecture: $arch"
    exit -1
fi

mkdir -p ${install_dir}
cp -r ${tmp_dir}/neusdk-aarch64-3.0.9/include ${install_dir}
cp -r ${tmp_dir}/neusdk-aarch64-3.0.9/lib ${install_dir}

rm -f ${install_dir}/lib/libneusdk.so
if [[ $arch == "x86_64" || $arch == "amd64" ]]; then
    ln -s ${install_dir}/lib/libneusdk.so.3.0.6 ${install_dir}/lib/libneusdk.so # the standard SDK requires boost system and thread components from the system
elif [[ $arch == "arm64" ]]; then
    ln -s ${install_dir}/lib/libneusdk-arm20.04.so ${install_dir}/lib/libneusdk.so # use the SDK version containing required boost lib
fi

echo "Neuvition SDK installed to ${install_dir}"

rm -rf ${tmp_dir}

exit 0
