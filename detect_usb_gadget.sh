#!/bin/bash
# 自动检测 USB Gadget 网卡接口名（只输出名称，不带提示）

nmcli -t -f DEVICE,TYPE dev | grep ethernet | cut -d: -f1 | while read dev; do
    if udevadm info /sys/class/net/"$dev" | grep -q "ID_VENDOR_ID=0525"; then
        echo "$dev"
        break
    fi
done

