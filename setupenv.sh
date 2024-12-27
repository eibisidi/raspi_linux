#!/bin/bash
export KERNEL="kernel8"
export ARCH="arm64"
export CROSS_COMPILE="aarch64-linux-gnu-"

#generat absolute path for tags, see cscope.files
export KBUILD_ABS_SRCTREE=1

