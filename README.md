# 开启wlan0
```bash
$ sudo ifconfig wlan0 up
```

# 搜索wifi ssid
```bash
$ sudo iwlist wlan0 scan | grep ESSID
```

# 配置interfaces文件
```bash
$ sudo nano /etc/network/interfaces
```
```
allow-hotplug wlan0
auto wlan0
iface wlan0 inet dhcp
wpa-conf /boot/wpa.conf
```

# 配置ssid和psk
```bash
$ sudo nano /boot/wpa.conf
```
```
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=CN

network={
  ssid="YWX3_2.4"
  key_mgmt=WPA-PSK
  psk="yuewuxian3"
  priority=5
}

network={
  ssid="Honor V10"
  key_mgmt=WPA-PSK
  psk="88888888"
  priority=5
}
```

# 重启系统
```bash
sudo reboot
```

# 挂载U盘
```bash
sudo mount /dev/sda1 /mnt
df -h
```

# codesys plc运行时下载
```
# DOWNLOAD CODESYS Control for Raspberry PI 3.5.16.20
https://store.codesys.com/en/codesys-control-for-raspberry-pi-sl.html
```

# 配置8152B USB以太网卡
```bash
# 编辑网络配置文件
sudo nano /etc/network/interfaces
auto eth0
iface eth0 inet dhcp

# 配置eth0使用DHCP
sudo nano /etc/dhcpcd.conf
interface eth0

# 重新启动网络服务
sudo systemctl restart networking

# 检查网络接口状态
ifconfig eth0
```
