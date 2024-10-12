#!/bin/bash
BEAR=

#bear will generate complete_commands.json. clangd complaints about unknown flag -lp64
#remove lp64 with following command
#vim complete_commands.json
#:%s/"mabi=lp64"/""/g
if [ "$1" == "BEAR" ];
then
	echo "Build with bear"
	BEAR=bear
else
	echo "Normal Build"
fi

${BEAR} make V=0 -j12 ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- Image modules dtbs


