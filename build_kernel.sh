#!/bin/bash
##############################################################################
#
#                           Kernel Build Script
#
##############################################################################
# 2016-08-17 Shev_t       : created
##############################################################################

##############################################################################
# define path to android 6.X sources
# Note: Set his value!
##############################################################################
ANDROID_SOURCES=~/AndroidSources/OmniROM-5.1/master

##############################################################################
# set toolchain
##############################################################################
export ARCH=arm64
export SUBARCH=arm64
export CROSS_COMPILE=$ANDROID_SOURCES/prebuilts/gcc/linux-x86/aarch64/aarch64-linux-android-4.9/bin/aarch64-linux-android-
export MY_CONFIG=cm_m2note_defconfig

##############################################################################
# set variables
##############################################################################
export KERNELDIR=`pwd`
KERNEL_OUT=$KERNELDIR/obj/KERNEL_OBJ
STRIP=${CROSS_COMPILE}strip

##############################################################################
# make kernel
##############################################################################
mkdir -p $KERNEL_OUT
mkdir -p $KERNEL_OUT/tmp/kernel
mkdir -p $KERNEL_OUT/tmp/system/lib/modules

make O=$KERNEL_OUT $MY_CONFIG
make -j10 O=$KERNEL_OUT

if [ -f $KERNEL_OUT/arch/arm64/boot/Image.gz-dtb ]
then
    cp -f $KERNEL_OUT/arch/arm64/boot/Image.gz-dtb ./
fi
