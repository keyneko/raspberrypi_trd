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
