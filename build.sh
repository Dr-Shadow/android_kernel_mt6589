#!/bin/bash
#Stop script if something is broken
set -e

#Export CROSS_COMPILE to point toolchain
export CROSS_COMPILE="ccache ../toolchain/arm-eabi-4.8/bin/arm-eabi-"

#Export target product
export TARGET_KERNEL_PRODUCT=`cat DEVICE_TREE`

#Echo actual vars
echo "We are actually building for $TARGET_KERNEL_PRODUCT with $CROSS_COMPILE"

#Workaround for + appended on kernelrelease
export LOCALVERSION=

#Create vars for OUT and RAMDISK directories
OUT_DIRECTORY=../out/$TARGET_KERNEL_PRODUCT
RAMDISK_DIRECTORY=../ramdisk/$TARGET_KERNEL_PRODUCT

#Create out directory for your device
mkdir -p $OUT_DIRECTORY

#Kernel part
make -j4
cp arch/arm/boot/zImage $OUT_DIRECTORY/zImage

#Modules part
make modules
make INSTALL_MOD_STRIP=--strip-unneeded INSTALL_MOD_PATH=$OUT_DIRECTORY/system INSTALL_MOD_DIR=$OUT_DIRECTORY/system android_modules_install

#Repack part
if [ -d "$RAMDISK_DIRECTORY" ]; then
../mtk-tools/repack-MT65xx.pl -boot $OUT_DIRECTORY/zImage $RAMDISK_DIRECTORY $OUT_DIRECTORY/boot.img
rm $OUT_DIRECTORY/zImage
fi
