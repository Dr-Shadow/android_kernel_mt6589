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

#Create vars for OUT, SCRIPTS and RAMDISK directories
OUT_DIRECTORY=../out/$TARGET_KERNEL_PRODUCT
RAMDISK_DIRECTORY=../ramdisk/$TARGET_KERNEL_PRODUCT
SCRIPTS_DIRECTORY=../scripts/$TARGET_KERNEL_PRODUCT
CERTIFICATES_DIRECTORY=../.certificates

#Create and clean out directory for your device
mkdir -p $OUT_DIRECTORY
rm $OUT_DIRECTORY/* -R

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

#Flashable zip build
if [ -d "$SCRIPTS_DIRECTORY" ]; then
cp $SCRIPTS_DIRECTORY/* $OUT_DIRECTORY -R
FLASHABLE_ZIP="$OUT_DIRECTORY/`cat DEVICE_NAME`-`make kernelversion`-`git rev-parse --short HEAD`"
FLASHABLE_ZIP_2="`cat DEVICE_NAME`-`make kernelversion`-`git rev-parse --short HEAD`"
echo "Creating flashable at '$FLASHABLE_ZIP'.zip"
pushd $OUT_DIRECTORY
zip -r -0 "$FLASHABLE_ZIP_2".zip .
popd
if [ ! -d "$CERTIFICATES_DIRECTORY" ]; then
echo "Warning ! We can't sign flashable.zip, you need to run ./certificates.sh"
else
java -jar $SCRIPTS_DIRECTORY/../signapk.jar $CERTIFICATES_DIRECTORY/certificate.pem $CERTIFICATES_DIRECTORY/key.pk8 "$FLASHABLE_ZIP".zip "$FLASHABLE_ZIP"-signed.zip
fi
fi
fi
