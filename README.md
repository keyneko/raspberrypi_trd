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
sudo apt update
sudo apt install wireless-tools
sudo iwlist wlan0 scan | grep "ESSID"
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

# 重新连接网络
```bash
sudo systemctl restart dhcpcd
# or
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
sudo apt install raspi-config
sudo raspi-config

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

# 使用Docker环境交叉编译
```bash
cat /etc/os-release

sudo docker pull balenalib/rpi-raspbian:bullseye

# Dockerfile
FROM balenalib/rpi-raspbian:bullseye

# 更新包管理器并安装必要的工具
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    cmake \
    gcc-arm-linux-gnueabihf \
    g++-arm-linux-gnueabihf

# 安装其他需要的工具和库
# RUN apt-get install -y <其他依赖>

# 设置工作目录
WORKDIR /workspace

# 将项目文件复制到容器中
# COPY . .

# 默认命令
CMD ["/bin/bash"]

sudo docker build --network=host -t rpi-cross-compile .
sudo docker run --network=host -it --rm -v $(pwd):/workspace rpi-cross-compile
sudo docker run --platform linux/arm/v6 --network=host -it --rm -v $(pwd):/workspace rpi-cross-compile

```

# 通过nfs服务共享目录
```bash
# 安装 NFS 服务端
sudo apt update
sudo apt install nfs-kernel-server

# 创建共享目录
mkdir -p /home/zx/Documents/raspberrypi/zero2w
chmod 777 /home/zx/Documents/raspberrypi/zero2w

# 添加共享规则
sudo nano /etc/exports
/home/zx/Documents/raspberrypi/zero2w 192.168.1.0/24(rw,sync,no_subtree_check,no_root_squash)

# 启动并重新导出配置
sudo exportfs -ra
sudo systemctl restart nfs-kernel-server

```
```bash
# 安装 NFS 客户端, zero
sudo apt update
sudo apt install nfs-common

sudo mkdir -p /mnt/shared
sudo mount 192.168.1.145:/home/zx/Documents/raspberrypi/zero2w /mnt/shared
ls /mnt/shared

```

# 使用venv虚拟环境
```bash
sudo apt install python3-venv
python3 -m venv .venv
python3 -m venv .venv --system-site-packages # 创建包含系统包的虚拟环境

source .venv/bin/activate
sudo chown -R $USER:$USER .venv  # 确保用户有权访问

pip install opencv-python numpy RPi.GPIO
pip freeze > requirements.txt
pip install -r requirements.txt
```

# 为树莓派Zero设置USB网络通讯(Ethernet Gadget)  
```bash
sudo nano /boot/firmware/config.txt
在最后添加
dtoverlay=dwc2

sudo nano /boot/firmware/cmdline.txt
在 rootwait 后添加
modules-load=dwc2,g_ether

# 重启后会创建 usb0 虚拟网卡

# 创建 usb0 网络配置
# sudo nano /etc/network/interfaces.d/usb0
# 添加
# allow-hotplug usb0
# iface usb0 inet dhcp

# 新版树莓派系统是基于 Raspbian Bookworm（Debian 12），
# 默认采用了 systemd-networkd 网络管理方案，因此不再使用 /etc/network/interfaces 或 dhcpcd.conf

sudo nano /etc/systemd/network/usb0.network
添加以下内容
[Match]
Name=usb0

[Network]
Address=192.168.7.2/24

sudo systemctl enable systemd-networkd
sudo systemctl start systemd-networkd

# 确认配置
networkctl status usb0
ip addr show usb0

# 重启并验证
sudo reboot
ifconfig usb0

```

# 让树莓派 Zero 通过主机共享上网
# Ubuntu 主机侧
```bash
chmod +x detect_usb_gadget.sh
chmod +x share_to_pi.sh
./share_to_pi.sh
```

# zero侧
```bash
chmod +x set_pi_gateway.sh
./set_pi_gateway.sh
```
