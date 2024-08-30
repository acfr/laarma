#!/bin/bash

# Get os code name
code=$(lsb_release -sc)
# Get the CPU architecture
arch=$(dpkg --print-architecture)

echo "Host system Ubuntu $code on $arch architecture detected"

if [[ $arch == "x86_64" || $arch == "amd64" ]]; then
    linux_arch="x86"
elif [[ $arch == "arm64" ]]; then
    linux_arch="aarch64"
else
    echo "Unsupported architecture: $arch"
    exit -1
fi

tmp_dir="/tmp/GigeV_sdk"
mkdir -p ${tmp_dir}
sdk_path="GigeV_sdk/GigE-V-Framework_${linux_arch}_2.20.0.0182.tar.gz"
echo "Unpacking tarball ${sdk_path}"
tar -xvzf ${sdk_path} --directory ${tmp_dir}

genicam_path="${tmp_dir}/DALSA/GigeV/GenICam_v3_0_0_linux_pkg.tgz"
echo "Unpacking tarball ${genicam_path}"
tar -xvzf ${genicam_path} --directory ${tmp_dir}

# Check CPU architecture to install to.
ARCH=`uname -m | sed -e s/i.86/i386/ -e s/x86_64/x86_64/ -e s/arm.*/armhf/ -e s/aarch64/aarch64/ `

if [[  $ARCH == "x86_64" ]] ; then
    ARCHNAME=x86_64
    ARCH_GENICAM_BIN="Linux64_x64"
    ARCH_GENICAM_SDK_TGZ_FILE="GenICam_SDK_gcc421_Linux64_x64_v3_0_0.tgz"
    ARCH_GENICAM_TGZ_FILE="GenICam_Runtime_gcc421_Linux64_x64_v3_0_0.tgz"
elif [[ $ARCH == "i386" ]] ; then
    ARCHNAME=i386
    ARCH_GENICAM_BIN="Linux32_i86"
    ARCH_GENICAM_SDK_TGZ_FILE="GenICam_SDK_gcc421_Linux32_i86_v3_0_0.tgz"
    ARCH_GENICAM_TGZ_FILE="GenICam_Runtime_gcc421_Linux32_i86_v3_0_0.tgz"
elif [[ $ARCH == "aarch64" ]] ; then
    ARCH_GENICAM_BIN="Linux64_ARM"
    ARCH_GENICAM_SDK_TGZ_FILE="GenICam_SDK_gcc54_Linux64_ARM_v3_0_0.tgz"
    ARCH_GENICAM_TGZ_FILE="GenICam_Runtime_gcc54_Linux64_ARM_v3_0_0.tgz"
else
    echo "Architecture $ARCH is not supported by this package!!"
fi

genicam_rt_path=${tmp_dir}/GenICam_v3_0_0_linux_pkg/GenICam_v3_0_0/${ARCH_GENICAM_TGZ_FILE}
echo "Unpacking tarball ${genicam_rt_path}"
mkdir -p ${tmp_dir}/GenICam_Runtime
tar -xvzf ${genicam_rt_path} --directory ${tmp_dir}/GenICam_Runtime
genicam_sdk_path=${tmp_dir}/GenICam_v3_0_0_linux_pkg/GenICam_v3_0_0/${ARCH_GENICAM_SDK_TGZ_FILE}
echo "Unpacking tarball ${genicam_sdk_path}"
mkdir -p ${tmp_dir}/GenICam_SDK
tar -xvzf ${genicam_sdk_path} --directory ${tmp_dir}/GenICam_SDK

GENICAM_INSTALL_BASE_PATH="/opt/genicam_v3_0"
mkdir -p ${GENICAM_INSTALL_BASE_PATH}
cp -r ${tmp_dir}/GenICam_Runtime/bin ${GENICAM_INSTALL_BASE_PATH}
cp -r ${tmp_dir}/GenICam_SDK/library ${GENICAM_INSTALL_BASE_PATH}
echo "GigE-V SDK installed to ${GENICAM_INSTALL_BASE_PATH}"

cd ${tmp_dir}/DALSA/src/w32lib/corw32
make install

gigev_install_dir="/opt/dalsa/GigeV"
export GENICAM_ROOT_V3_0=${GENICAM_INSTALL_BASE_PATH}
cd ${tmp_dir}/DALSA/src/gevlib
make install
mkdir -p ${gigev_install_dir}
cp -r ${tmp_dir}/DALSA/GigeV/include ${gigev_install_dir}

GENICAM_LIBRARY_PATH="$GENICAM_INSTALL_BASE_PATH/bin/$ARCH_GENICAM_BIN"
# Generate the ldconf config file and put it in /etc/ld.so.conf.d directory
if [ -d /etc/ld.so.conf.d ] ; then
    echo "$GENICAM_LIBRARY_PATH" > td_genicam_v3_0.conf
    cp td_genicam_v3_0.conf /etc/ld.so.conf.d
    rm td_genicam_v3_0.conf
fi

# Update the loadable library cache.
if [ -x /sbin/ldconfig ] ; then
    /sbin/ldconfig
fi

# Clear up
rm -rf ${tmp_dir}

echo "Done"

exit 0
