#!/bin/bash
# 用于将 Ubuntu 主机的网络通过 USB 接口共享给树莓派 Zero

# 主机连接外网的接口名（根据实际情况修改）
INTERNET_IF="ens33"

# 主机与 Zero 连接的 USB 网口名称
USB_IF="enx7aa417f740e8"

echo "[*] 启用 IP 转发..."
sudo sysctl -w net.ipv4.ip_forward=1
sudo sed -i '/^net.ipv4.ip_forward/d' /etc/sysctl.conf
echo "net.ipv4.ip_forward=1" | sudo tee -a /etc/sysctl.conf >/dev/null

echo "[*] 配置 NAT 规则..."
sudo iptables -t nat -A POSTROUTING -o $INTERNET_IF -j MASQUERADE
sudo iptables -A FORWARD -i $USB_IF -o $INTERNET_IF -j ACCEPT
sudo iptables -A FORWARD -i $INTERNET_IF -o $USB_IF -m state --state RELATED,ESTABLISHED -j ACCEPT

echo "[*] 可选：安装 iptables-persistent（用于保存规则）"
read -p "是否保存规则到系统？[y/N] " yn
if [[ "$yn" == "y" || "$yn" == "Y" ]]; then
    sudo apt update
    sudo apt install -y iptables-persistent
    sudo netfilter-persistent save
fi

echo "[✔] 主机已设置完成！你现在可以在树莓派 Zero 上配置网关和 DNS。"
