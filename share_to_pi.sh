#!/bin/bash
# 为树莓派 Zero 设置 USB 网络共享的主机脚本（自动化）

# 1. 主机连接外网的接口（根据你的实际情况修改）
INTERNET_IF="ens33"

# 2. 自动检测 gadget 接口名（通过 detect_usb_gadget.sh）
USB_IF=$(bash ./detect_usb_gadget.sh)

if [[ -z "$USB_IF" ]]; then
    echo "❌ 未检测到 USB Gadget 网卡接口，退出"
    exit 1
fi

echo "✅ 检测到 USB Gadget 接口: $USB_IF"

# 3. 禁用 NetworkManager 管理该接口（防止连接状态干扰）
echo "[*] 禁止 NetworkManager 管理 $USB_IF..."
sudo nmcli device set "$USB_IF" managed no

# 4. 设置主机 gadget 网卡静态 IP
echo "[*] 配置 $USB_IF 静态 IP 地址..."
sudo ip addr flush dev "$USB_IF"
sudo ip addr add 192.168.7.1/24 dev "$USB_IF"
sudo ip link set "$USB_IF" up

# 5. 启用 IP 转发
echo "[*] 启用 IP 转发..."
sudo sysctl -w net.ipv4.ip_forward=1
sudo sed -i '/^net.ipv4.ip_forward/d' /etc/sysctl.conf
echo "net.ipv4.ip_forward=1" | sudo tee -a /etc/sysctl.conf >/dev/null

# 6. 设置 NAT 规则
echo "[*] 配置 iptables NAT 转发..."
sudo iptables -t nat -A POSTROUTING -o "$INTERNET_IF" -j MASQUERADE
sudo iptables -A FORWARD -i "$USB_IF" -o "$INTERNET_IF" -j ACCEPT
sudo iptables -A FORWARD -i "$INTERNET_IF" -o "$USB_IF" -m state --state RELATED,ESTABLISHED -j ACCEPT

# 7. 可选：保存 iptables 规则
echo "[*] 是否保存 iptables 规则？（重启后自动恢复）"
read -p "保存规则？[y/N]: " yn
if [[ "$yn" == "y" || "$yn" == "Y" ]]; then
    sudo apt update
    sudo apt install -y iptables-persistent
    sudo netfilter-persistent save
    echo "✅ 已保存 iptables 规则"
fi

echo "🎉 USB 网络共享配置完成！"
echo "📡 主机 $USB_IF IP: 192.168.7.1   → 树莓派 Zero 应使用网关 192.168.7.1"

