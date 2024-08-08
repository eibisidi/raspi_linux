#!/bin/bash



make V=0 -j8 ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- Image modules dtbs


