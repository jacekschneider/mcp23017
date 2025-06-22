# mcp23017
Linux kernel driver for mcp23017

# building with sdk
source /opt/poky/4.0.16/environment-setup-cortexa72-poky-linux

sudo chown -R user:user /opt/poky/4.0.16/sysroots/cortexa72-poky-linux/usr/src/kernel

sudo --preserve-env=PATH,CROSS_COMPILE,ARCH make ARCH=arm64 CROSS_COMPILE=aarch64-poky-linux- prepare

make -C /opt/poky/4.0.16/sysroots/cortexa72-poky-linux/usr/src/kernel M=$(pwd) modules
