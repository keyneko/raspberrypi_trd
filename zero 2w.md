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


# 安装 micro-ROS 到树莓派 Zero 2 W（Linux）
```bash
# # 设置 ROS 2 的 sources.list
# sudo apt update
# sudo apt install curl gnupg lsb-release
# sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
# echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
#   sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
# sudo apt update


mkdir -p ~/microros_ws/src
cd ~/microros_ws
git clone -b humble https://github.com/micro-ROS/micro_ros_setup src/micro_ros_setup
rosdep update
rosdep install --from-path src --ignore-src -y
colcon build
source install/local_setup.bash

# 克隆示例节点
cd src
git clone -b humble https://github.com/micro-ROS/micro-ROS-demos.git

# 主要组件
git clone -b humble https://github.com/ros2/rclc.git
git clone -b humble https://github.com/micro-ROS/rmw-microxrcedds.git
git clone https://github.com/eProsima/Micro-XRCE-DDS-Client.git microxrcedds_client
git clone https://github.com/eProsima/Micro-CDR.git microcdr
git clone -b humble https://github.com/micro-ROS/micro_ros_demos.git
git clone -b humble https://github.com/micro-ROS/rosidl_typesupport_microxrcedds.git
git clone -b humble https://github.com/micro-ROS/micro_ros_utilities.git

# 安装缺失依赖包
sudo apt install ros-humble-test-msgs
sudo apt install ros-humble-osrf-testing-tools-cpp
sudo apt install ros-humble-rosidl-default-generators \
                 ros-humble-rosidl-default-runtime \
                 ros-humble-example-interfaces \
                 ros-humble-test-msgs

# 编译
cd ~/microros_ws
rosdep install --from-paths src --ignore-src -y
# 清理
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash


# 运行 ping_pong demo（micro-ROS 发布/订阅对）
ros2 run micro_ros_demos_rclc ping_pong
```


# 上位机从源码构建 micro-ROS Agent
```bash
sudo apt install python3-colcon-common-extensions
mkdir -p ~/microros_agent_ws/src
cd ~/microros_agent_ws/src
git clone -b humble https://github.com/micro-ROS/micro-ROS-Agent.git
cd ..
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash

echo "source ~/microros_agent_ws/install/setup.bash" >> ~/.bashrc

# 运行 micro-ROS Agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

```

