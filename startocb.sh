#!/bin/bash
#starocb.sh 启动openocb脚本
killall openocd

while (( 1 ))
do
	netstat -tan | grep 3333 | grep LISTEN
	if [ $? -ne 0 ]
	then
		break
	fi
echo "waiting openocd exit..."
sleep 1
done

echo "starting openocd..."

openocd -f jlink/ftdi_z2232.cfg -f jlink/raspi4.cfg &

while (( 1 ))
do
	netstat -tan | grep 3333 | grep LISTEN
	if [ $? -eq 0 ]
	then
		break
	fi
sleep 1
echo "waiting port 3333"
done

echo "port 3333 is listening."

telnet localhost 4444