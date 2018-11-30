#!/bin/sh
cd
cd ../../etc/udev/rules.d/

cat > dynamixel.rules << EOF
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE:="0777", SYMLINK+="dynamixelBus%n"
EOF
