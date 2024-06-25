# 系统信息
```bash
$ uname -a
Linux raspberrypi 5.10.103+ #1529 Tue Mar 8 12:19:18 GMT 2022 armv6l GNU/Linux
```
```bash
raspi-config
```

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
sudo iwlist wlan0 scanning | grep ESSID
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

# 抓拍摄像头
```bash
raspistill -t 2000 -o capture.jpg -w 640 -h 480 -q 10 -n
# raspistill：调用摄像头拍照的命令。
# -t 2000：设置拍摄前的延迟时间（以毫秒为单位）。这里是 2000 毫秒（2 秒）。摄像头在拍摄前等待 2 秒，给你时间准备或稳定摄像头。
# -o capture.jpg：指定输出文件的名称和格式。这将把拍摄的图像保存为 capture.jpg 文件。
# -w 640：设置输出图像的宽度为 640 像素。
# -h 480：设置输出图像的高度为 480 像素。
# -q 10：设置 JPEG 图像的质量，范围是 0（最低质量）到 100（最高质量）。这里设置为 10，是较低的图像质量，有助于减小文件大小。
# -n：禁用预览窗口。在一些使用场景中，尤其是无头操作（没有连接显示器的情况下），禁用预览窗口是必要的。

# 传图到电脑
scp capture* zx@192.168.43.19:~/Desktop
```
