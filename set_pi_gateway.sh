#!/bin/bash
# 配置树莓派 Zero 的默认网关和 DNS（用于 USB 网络）

GATEWAY_IP="192.168.7.1"

echo "[*] 添加默认路由..."
sudo ip route add default via $GATEWAY_IP

echo "[*] 设置临时 DNS..."
echo "nameserver 8.8.8.8" | sudo tee /etc/resolv.conf >/dev/null

echo "[✔] 已设置网关为 $GATEWAY_IP，DNS 为 8.8.8.8"

echo "测试网络连接..."
ping -c 4 8.8.8.8
ping -c 4 google.com
