#!/bin/bash

set -e

echo $1 | sudo -S ip link set down can0
echo $1 | sudo -S ip link set can0 type can bitrate 250000 restart-ms 100
echo $1 | sudo -S ifconfig can0 up
