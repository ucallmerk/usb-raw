#!/bin/bash
#stress test
#check if module is inserted
for i in "$*"
do
   echo $i
done
check=$(lsmod | head -10 | grep pic32)
if [ "x$check" != "x" ]; then
	echo "module is inserted"
else
	echo "please insmod .."
	echo "bind the driver.."
	exit -1
fi
date +%N
#./stress1 /dev/pic320 1000000	
#./stress2 /dev/pic320 1000000
./openclose /dev/pic320 1000000
date +%N
