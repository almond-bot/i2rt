#!/bin/bash

if [ "$EUID" -ne 0 ]; then
  echo "Please run as root"
  exit 1
fi

# Bring down devices if they're already up
ip link set down can_leader_l 2>/dev/null
ip link set down can_leader_r 2>/dev/null
ip link set down can_follower_l 2>/dev/null
ip link set down can_follower_r 2>/dev/null

# Bring up devices with configuration
ip link set up can_leader_l type can bitrate 1000000
ip link set up can_leader_r type can bitrate 1000000
ip link set up can_follower_l type can bitrate 1000000
ip link set up can_follower_r type can bitrate 1000000