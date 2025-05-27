#! /bin/bash

echo dongguan | sudo -S chmod 777 /dev/ttyACM*


echo dongguan | sudo -S chmod 777 /sys/class/gpio/export

echo dongguan | sudo -S echo 5 > /sys/class/gpio/export
echo dongguan | sudo -S echo 23 > /sys/class/gpio/export
echo dongguan | sudo -S echo 24 > /sys/class/gpio/export

echo dongguan | sudo -S chmod 777 /sys/class/gpio/gpio5/direction
echo dongguan | sudo -S chmod 777 /sys/class/gpio/gpio23/direction
echo dongguan | sudo -S chmod 777 /sys/class/gpio/gpio24/direction
echo dongguan | sudo -S chmod 777 /sys/class/gpio/gpio5/value
echo dongguan | sudo -S chmod 777 /sys/class/gpio/gpio23/value
echo dongguan | sudo -S chmod 777 /sys/class/gpio/gpio24/value

echo dongguan | sudo -S echo out > /sys/class/gpio/gpio5/direction
echo dongguan | sudo -S echo out > /sys/class/gpio/gpio23/direction
echo dongguan | sudo -S echo out > /sys/class/gpio/gpio24/direction


echo dongguan | sudo -S echo 0 > /sys/class/gpio/gpio5/value
echo dongguan | sudo -S echo 0 > /sys/class/gpio/gpio23/value
echo dongguan | sudo -S echo 0 > /sys/class/gpio/gpio24/value
