#!/bin/bash
MYIMAGE="Image"
ARCH="arm64"
CROSS_COMPILE="aarch64-linux-gnu-"

BOOTMNT="/media/root/bootfs"
ROOTMNT="/media/root/rootfs"

if [ ! -d ${BOOTMNT} ]
then
	echo "SD卡未挂载"
	exit 1
fi
if [ ! -d ${ROOTMNT} ]
then
	echo "SD卡未挂载"
	exit 1
fi

env PATH=$PATH make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- INSTALL_MOD_PATH=$ROOTMNT modules_install
if [ $? -ne 0 ]
then
	echo "安装内核模块失败!"
	exit 1
fi

cp arch/arm64/boot/Image $BOOTMNT/$MYIMAGE

if [ $? -ne 0 ]
then
	echo "拷贝内核镜像失败!"
	exit 1
fi

cp arch/arm64/boot/dts/broadcom/*.dtb $BOOTMNT/
if [ $? -ne 0 ]
then
	echo "拷贝dtb失败!"
	exit 1
fi

cp arch/arm64/boot/dts/overlays/*.dtb*  $BOOTMNT/overlays/
if [ $? -ne 0 ]
then
	echo "拷贝overlays失败!"
	exit 1
fi

cp arch/arm64/boot/dts/overlays/README $BOOTMNT/overlays/
if [ $? -ne 0 ]
then
	echo "拷贝README失败!"
	exit 1
fi

echo "安装完成."

exit 0

