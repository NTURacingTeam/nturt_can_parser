#!/bin/bash

COLOR_REST='\e[0m'
HIGHLIGHT='\e[0;1m'
REVERSE='\e[0;7m'
COLOR_GREEN='\e[0;32m'
COLOR_RED='\e[1;31m'

print_usage() {
    echo "Usage: ./configure_can.sh [OPTIONS]"
    echo "Options:"
    echo "    -b --bitrate    Initialize can bus to the bitrate, default to 100000(100k)"
    echo "    -h --help       Print this help message and exit"
    echo "    -p --password   Password of the sudo user in order to configure can bus"
}

# default argument
PASSWORD=""
BITRATE=100000

PARAM=$(getopt -o b:hp: -l bitrate:,help,password:,ros-args -n "$0" -- "$@")

if [ $? != 0 ]; then
    print_usage
    exit 1
fi

set -e

eval set -- "${PARAM}"

while true; do
    case "$1" in
        -h|--help)
            print_usage
            exit
            shift
            ;;

        -p|--password)
            PASSWORD=$2
            shift 2
            ;;

        -b|--bitrate)
            BITRATE=$2
            shift 2
            ;;

        # ros2 will automatically pass this option in when launch
        --ros-args)
            shift
            ;;
        
        --)
            break
            ;;
    esac
done

echo "setting can bitrate to ${BITRATE}"
echo ${PASSWORD} | sudo -S ip link set down can0
echo ${PASSWORD} | sudo -S ip link set can0 type can bitrate ${BITRATE} restart-ms 100
echo ${PASSWORD} | sudo -S ifconfig can0 txqueuelen 65536
echo ${PASSWORD} | sudo -S ifconfig can0 up
