#Export CROSS_COMPILE to point toolchain
#export CROSS_COMPILE=/home/dr-shadow/faea/prebuilts/gcc/linux-x86/arm/arm-eabi-4.6/bin/arm-eabi-
export CROSS_COMPILE=/home/jenkins/jobs/F2S-Omnirom/workspace/prebuilts/gcc/linux-x86/arm/arm-eabi-4.7/bin/arm-eabi-

#Target Product
export TARGET_PRODUCT=e960

#Create directory for your device
mkdir -p out/e960/

#Kernel part
make -j16
mediatek/build/tools/mkimage arch/arm/boot/zImage KERNEL > out/$TARGET_PRODUCT/zImage

#Modules part
make modules
make INSTALL_MOD_STRIP=1 INSTALL_MOD_PATH=./out/$TARGET_PRODUCT/system INSTALL_MOD_DIR=./out/$TARGET_PRODUCT/system android_modules_install

#Repack part - You need to set PATH var correctly poiting to a directory with https://github.com/bgcngm/mtk-tools.git and chmod +x mkbootimg
#You need ramdisk directory too
repack-MT65xx.pl -boot out/$TARGET_PRODUCT/zImage ../ramdisk out/$TARGET_PRODUCT/boot.img
rm out/$TARGET_PRODUCT/zImage
