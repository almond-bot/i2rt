#!/bin/bash

if [ "$EUID" -ne 0 ]; then
  echo "Please run as root"
  exit 1
fi


ip link set up can_leader_l type can bitrate 1000000
ip link set up can_leader_r type can bitrate 1000000
ip link set up can_follower_l type can bitrate 1000000
ip link set up can_follower_r type can bitrate 1000000