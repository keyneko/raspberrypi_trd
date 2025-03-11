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

# 安装NodeJs（zero只支持到v11）
```bash
sudo apt-get update
sudo apt-get upgrade

wget https://nodejs.org/dist/v11.9.0/node-v11.9.0-linux-armv6l.tar.xz
tar -xvJf node-v11.9.0-linux-armv6l.tar.xz
# sudo mv node-v11.9.0-linux-armv6l /usr/local/nodejs
# echo 'export PATH=/usr/local/nodejs/bin:$PATH' >> ~/.bashrc
# source ~/.bashrc
cd node-v11.9.0-linux-armv6l/
sudo cp -R * /usr/local/

# or
# sudo ln -s /usr/local/bin/node /usr/bin/node
# sudo ln -s /usr/local/bin/npm /usr/bin/npm
# sudo unlink /usr/bin/node
# sudo unlink /usr/bin/npm

# sudo npm install -g n
# export N_NODE_MIRROR=https://unofficial-builds.nodejs.org/download/release
# sudo n stable
# sudo n lts
# sudo n 14.19.1
# sudo n rm 14.19.1
# n ls

# 安装nvm
wget -qO- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.3/install.sh | bash

npm config get registry
npm config set registry https://registry.npm.taobao.org
npm config set strict-ssl false

# nvm换源
nvm root
node_mirror: http://npmmirror.com/mirrors/node/ 
npm_mirror: http://npmmirror.com/mirrors/npm/

npm i create-nuxt-app -g
npx create-nuxt-app robot-web
sudo npm remove create-nuxt-app -g
```

# 开发micro-ROS
```bash
sudo apt update
sudo apt install python3-pip
pip3 install platformio
pip3 install --upgrade platformio

# 如果报错找不到虚拟环境，手动创建
python3 -m venv ~/.platformio/penv
source ~/.platformio/penv/bin/activate
pip install platformio
pip install catkin_pkg
pip install empy==3.3.4
pip install lark

pio project init --board raspberrypi_zero --project-dir zero_ROS-test

# platformio.ini添加依赖
[env:raspberrypi_zero]
platform = linux_arm
board = raspberrypi_zero
framework = wiringpi
lib_deps = https://github.com/micro-ROS/micro_ros_platformio
board_microros_distro = humble
board_microros_transport = wifi

# 安装更新依赖
pio pkg install
pio pkg list
pio run

# 清除缓存
pio run --target clean
rm -rf .pio
rm -rf .pio/libdeps/raspberrypi_zero/micro_ros_platformio
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


# zero 2w配置网络
```bash
sudo nano /etc/netplan/50-cloud-init.yaml
network:
    version: 2
    renderer: networkd
    wifis:
        wlan0:
            dhcp4: true
            optional: true
            access-points:
                "WiFi_网络_1":
                    password: "密码1"
                "WiFi_网络_2":
                    password: "密码2"
sudo netplan apply
ip a

```

# initramfs 压缩方式更改为 lz4，提高系统启动速度
```bash
sudo apt install lz4
sudo sed -i -e 's/COMPRESS=zstd/COMPRESS=lz4/' /etc/initramfs-tools/initramfs.conf
sudo update-initramfs -u
sudo reboot
```

# 安装ROS2
```bash
locale  # check for UTF-8

# 设置系统源
sudo apt install software-properties-common
sudo add-apt-repository universe

# 设置密钥
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade

# 安装无GUI版本
sudo apt install ros-humble-ros-base
sudo apt install ros-dev-tools

# 安装demos
sudo apt install ros-humble-demo-nodes-cpp ros-humble-demo-nodes-py

source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker

source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

# 启用摄像头
```bash
sudo apt install v4l-utils
v4l2-ctl --list-devices
sudo apt install ffmpeg
v4l2-ctl -d /dev/video0 --list-formats-ext

# 有读写权限
ls -l /dev/video0
sudo chmod 666 /dev/video0
ffmpeg -f v4l2 -input_format mjpeg -video_size 640x480 -i /dev/video0 -vframes 1 output.jpg

# ROS2 包
sudo apt install ros-humble-v4l2-camera

v4l2-ctl --device=/dev/video0 --set-fmt-video=width=320,height=240,pixelformat=MJPG
v4l2-ctl --device=/dev/video0 --set-parm=15
ros2 run v4l2_camera v4l2_camera_node
```

# python发布客户端
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class CompressedImagePublisher(Node):
    def __init__(self):
        super().__init__('compressed_image_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, 'camera/image/compressed', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)
        self.cap.set(cv2.CAP_PROP_FPS, 15)
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            _, buffer = cv2.imencode('.jpg', frame)
            image_message = CompressedImage()
            image_message.header.stamp = self.get_clock().now().to_msg()
            image_message.format = 'jpeg'
            image_message.data = buffer.tobytes()
            self.publisher_.publish(image_message)

def main(args=None):
    rclpy.init(args=args)
    image_publisher = CompressedImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
