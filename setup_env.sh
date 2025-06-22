#!/bin/bash

# ROS2 環境設定腳本
# 用於設定 TurtleBot4 的 CycloneDDS 和網路配置

echo "設定 ROS2 和 CycloneDDS 環境變數..."

# 設定 CycloneDDS URI
export CYCLONEDX_URI=/home/recoomputer/turtlebot4/cyclonedds_jetson.xml

# 設定 ROS Domain ID
export ROS_DOMAIN_ID=76

# 設定 RMW 實現
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "設定網路介面..."

# 啟動網路介面 l4tbr0
sudo ip link set l4tbr0 up

# 設定 IP 位址
sudo ip addr add 192.168.186.3/24 dev l4tbr0

echo "環境設定完成！"
echo "CycloneDDS URI: $CYCLONEDX_URI"
echo "ROS Domain ID: $ROS_DOMAIN_ID"
echo "RMW Implementation: $RMW_IMPLEMENTATION"
echo "網路介面 l4tbr0 已設定為 192.168.186.3/24"