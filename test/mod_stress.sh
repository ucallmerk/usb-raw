#!/bin/bash
#stress test
#check if module is inserted
for i in {1..100000}
do

check=$(lsmod | head -10 | grep pic32)
if [ "x$check" != "x" ]; then
	echo "module is inserted"
else
	echo " "
	echo "inserting module.."
	insmod pic32-usb.ko
fi
./nt /dev/pic320
	echo " "
	echo "Removing module.."
	rmmod pic32-usb.ko
done
