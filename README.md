|                         | Pi                    | Zero                              | Remote
| ---                     | ---                   | ---                               | ---
| **Role**                | Master                | Slave                             | HMI (Human-Machine Interface)
| **Primary Function**    | Mapping and Navigation| Movement                          | User Control
| **Physical Location**   | BB-8 Head             | BB-8 Sphere                       | User Side
| **Hardware**            | Raspberry Pi 4B       | Raspberry Pi Zero W               | User-Provided Device
| **Sensor Inputs**       | Lidar / IMU           | IMU / Magnetometer                | Keyboard / Mouse
| **Outputs**             | Eye LED / Speaker     | Wheels Motors / Head Pitch Stepper| Screen Display
| &nbsp;                  |                       |                                   | 
| **OS**                  | Ubuntu 20.04          | Bookworm                          | -
| **ROS Version**         | ROS 1 and ROS 2       | ROS 1                             | ROS 1 or ROS 2
| **ROS Distro**          | *noetic* and *foxy*   | *noetic*                          | -
| **Other Notes**         | Runs `roscore`        | -                                 | RViz
| **Programming Language**| C++                   | C++                               | Python
| **Package**             | [`bb8`](src/bb8/bb8)  | [`bb8_zero`](src/bb8/bb8)         | [`bb8_remote`](src/bb8/bb8)


# Data flow
`odom`      [xyz, rpy] where the robot thinks it is based on integrating the imu.
`map`       [xyz, rpy] where the robot thinks it is based on the lidar scan.
`base_link` [xyz, rpy] where the robot thinks it is based on the wheel odometry.

Head IMU + RPlidar feed Cartographer and publishes map and `map -> odom` transfrom (May jump).
Sphere IMU + Wheel odom (Optional) -> UKF -> `odom -> base_link` (Smooth, local driving)

For navigation planning use `map -> odom` to generate /cmd_vel
For setting motor speeds use `odom -> base_link` + `/cmd_vel` to generate /cmd_motor


# Packages structure
- bb8 => Main code, raspberry pi 4b
- zero => Sphere
- remote => remote controll

# BB8

This project is a BB8-inspired robot system built using ROS 1.

- **Master**: A Raspberry Pi 4 B running Ubuntu 20.04 with ROS 1 installed. It acts as the ROS master and runs the main control nodes.

- **Inner Ball**: A Raspberry Pi Pico W running a lightweight ROS 1-compatible node. It commands a box with wheels inside the ball. By shifting the center of mass, the inner mechanism causes the ball to roll.

- **Head**: Attached to the ball is the "head" of the robot, mounted in a way that allows it to tilt forward and backward. It houses the Raspberry Pi 4, an IMU, and a LiDAR sensor. The head uses Cartographer for mapping and localization.

- **Networking**: A ROS Bridge is set up to allow communication between devices. All components are on the same Wi-Fi network.

- **Remote Visualization**: A remote machine (e.g., a laptop or desktop) on the same network runs RViz/RViz2 to visualize data (mapping, robot state, etc.) in real-time.



## Setup

## RPI ZERO + [ROS 1](https://robokinesis.mihr.io/rospi)
```
rpi-imager
```
Raspberry Pi Devices => Raspberry Pi Zero
Operating System => Raspberry Pi OS (Other) => Raspberry Pi OS Lite (32-bit)
Storage => SD Card

Use OS Customization => Edit Settings
a. General
    1. Set hostname => zero
    2. Set username ans password => ros
    3. Configure wireless LAN => SSID and password
b. Services
    1. Enable SSH => Use password authentication

Save
Yes
Yes, continue

ROS 1

```
sudo apt-get update && sudo apt-get upgrade
sudo apt install -y python3-rosdep2 python3-rosinstall-generator python3-wstool python3-rosinstall build-essential cmake

```
#!&%#
```
sudo apt-get install liblog4cxx-dev


export ROS_OS_OVERRIDE=debian:wheezy
sudo rosdep init
rosdep update

mkdir ~/ros_ws && cd ~/ros_ws

rosinstall_generator ros_comm --rosdistro noetic --deps --wet-only --tar > noetic.rosinstall
wstool init src noetic.rosinstall

rosdep install -y --from-paths src --ignore-src --rosdistro noetic -r

sudo ./src/catkin/bin/catkin_make_isolated --install \
    --install-space /opt/ros/noetic -j1 \
    -DROSCONSOLE_BACKEND=print -DCATKIN_ENABLE_TESTING=0 \
    -DCMAKE_BUILD_TYPE=Release
```
Import by default, only ROS 1 on this pi
```
echo "export ROS_MASTER_URI=http://raspberrypi.local:11311" >> ~/.bashrc
echo "export ROS_IP=$(hostname -I | awk '{print $1}')" >> ~/.bashrc
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

### catkin build
# make sure pip itself is present
sudo apt update
sudo apt install -y python3-pip          # tiny, pure-Python

# install catkin-tools into your user site-packages
python3 -m pip install --user --break-system-packages \
        catkin-tools osrf-pycommon
# add ~/.local/bin to PATH once
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc

# verify
catkin --version                         # should print 0.9.x

```

### RPI PI [Ubuntu 20.04](https://cdimage.ubuntu.com/releases/focal/release/ubuntu-20.04.5-preinstalled-server-arm64+raspi.img.xz)

On the remote machine:
```
wget https://cdimage.ubuntu.com/releases/focal/release/ubuntu-20.04.5-preinstalled-server-arm64+raspi.img.xz
sudo apt install rpi-imager
rpi-imager
```
Raspberry Pi Devices => Raspberry Pi 4
Operating System => Use custom => <path_to_ubuntu_20_04_5>
Storage => SD Card


### [Keyboard command](https://github.com/ethz-asl/better_teleop/tree/main)
```
sudo pip install pynput
```

### Update
```
sudo apt update
sudo apt upgrade
```

### Setup raspberrypi.local
```
sudo apt install avahi-daemon
sudo systemctl enable --now avahi-daemon
```
This enables the device to be accessed via raspberrypi.local

### Install [ROS 1](https://wiki.ros.org/noetic/Installation/Ubuntu)
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update 
sudo apt install ros-noetic-ros-base
echo "alias sros1='source /opt/ros/noetic/setup.bash'" >> ~/.bashrc
source ~/.bashrc
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential ninja-build stow
sudo rosdep init
rosdep update
```

### Install [ROS 2](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) + [Bridge](https://roboticsknowledgebase.com/wiki/interfacing/ros1_ros2_bridge/)
```
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-foxy-ros-base python3-argcomplete
sudo apt install ros-dev-tools
echo "alias sros2='source /opt/ros/foxy/setup.bash'" >> ~/.bashrc
source ~/.bashrc
```
and the bridge
```
sudo apt install ros-foxy-ros1-bridge
```

### Install [catkin](https://catkin-tools.readthedocs.io/en/latest/installing.html)
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install python3-catkin-tools
```

### Install [catkin](https://catkin-tools.readthedocs.io/en/latest/installing.html) On Linux Mint
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install python3-catkin-tools
```

### Install [RPLIDAR](https://github.com/Slamtec/rplidar_ros) + [IMU](https://github.com/Brazilian-Institute-of-Robotics/mpu6050_driver)
```
mkdir -p bb8_ws/src/cartographer

cd bb8_ws/src
git submodule add https://github.com/Slamtec/rplidar_ros.git rplidar_ros
rosdep install --from-paths . --ignore-src -r -y

git submodule add https://github.com/Brazilian-Institute-of-Robotics/i2c_device_ros.git
git submodule add https://github.com/Brazilian-Institute-of-Robotics/mpu6050_driver.git

sudo apt install i2c-tools
GPIOs 2&3 (pins 3&5 on the header) are /dev/i2c-1
i2cdetect -y 1
Should se 68 as active address, Almost nice!

catkin build mpu6050_driver

To run the calibration process, put the sensor on its final place on your robot (this is strongly important!)
roslaunch mpu6050_driver mpu6050_calibration.launch

Wait for it to finish and copy and paste into mpu6090_driver/config/mpu_settings.yaml

roslaunch mpu6050_driver mpu6050_driver.launch
```


### Create [pacakge](http://wiki.ros.org/ROS/Tutorials/CreatingPackage) with catkin
```
cd src
catkin_create_pkg <package_name> std_msgs rospy roscpp
```

### Create [pacakge](http://wiki.ros.org/ROS/Tutorials/CreatingPackage) with colcon
```
ros2 pkg create --build-type ament_python <package_name>
ros2 pkg create --build-type ament_cmake <package_name> 
```


### Install [cartographer](https://google-cartographer-ros.readthedocs.io/en/latest/index.html)
```
cd cartographer
```
```
git submodule add https://github.com/cartographer-project/cartographer.git
```
```
git submodule add https://github.com/cartographer-project/cartographer_ros.git
```
```
rosdep install --from-paths . --ignore-src -r -y
```
```
./cartographer/scripts/install_abseil.sh
```
```
catkin config -a "-DPYTHON_EXECUTABLE=$(which python3)"
```
```
sudo apt-get install ros-noetic-pcl-conversions
```
```
sudo apt-get install ros-noetic-pcl-ros
```
```
cd ../..
```
```

sros1
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
source devel/setup.bash
```

### URDF
```
sudo apt update
sudo apt install ros-noetic-urdf ros-noetic-xacro ros-noetic-robot-state-publisher ros-noetic-joint-state-publisher
```

### Github (Optional)
```
ssh-keygen -t ed25519 -C "deploy-key" -f ~/.ssh/id_bb8_deploy
cat .ssh/id_bb8_deploy.pub
```
Paste the output on github: **Project** -> **Settings** -> **Deploy key** -> **Add deploy key**
```
nano ~/.ssh/config
Host github.com
    User raspberry
    IdentityFile ~/.ssh/id_bb8_deploy
    IdentitiesOnly yes

git config --global user.email "you@example.com"
git config --global user.name "Your Name"
```

## Ðevelop
### Build with catkin
Build only `bb8` without dependencies
```
catkin build bb8 --no-deps
```

### Build with catkin
```
colcon build --packages-up-to bb8_remote
colcon build --packages-select <name-of-pkg>
```

### [Calibrate](https://mjwhite8119.github.io/Robots/mpu6050)
```
roslaunch mpu6050_driver mpu6050_calibration.launch
```

### XARCO to URDF
```
cd ~/bb8_ws/src/bb8
rosrun xacro xacro urdf/bb8.urdf.xacro -o urdf/bb8.urdf


ros2 run xacro xacro src/bb8/urdf/bb8.urdf.xacro -o src/bb8/urdf/bb8.urdf

```

## RUN

### Run RPLIDAR
```
sros1
roslaunch rplidar_ros rplidar_a1.launch
```

### ROS bridge
```
sros1
sros2
sros2
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

### rviz with .rviz
```
ros2 run rviz2 rviz2 -d ./laser.rviz
ros2 run rviz2 rviz2 -d ./rviz/map.rviz
```

# Joy
ros2 run package_name executable_name --ros-args -p param_name:=param_value
ros2 run ps_ros2_common joy_test
ros2 run teleop_twist_joy teleop_node 

ros2 launch teleop_twist_joy teleop-launch.py --ros-args -p enable_button:=69 -p scale_linear:=2


ros2 launch teleop_twist_joy teleop-launch.py

### Pi zero
```
catkin_make -j1 -DCMAKE_BUILD_TYPE=Release --pkg bb8_zero 
```
or
```
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build bb8_zero -j1 -p1 --mem-limit 400M
rosrun bb8_zero bb8_zero_node
<!-- catkin build bb8_zero  -->
```



# TO ADD
# Pi 4 (master)
export ROS_MASTER_URI=http://raspberrypi.local:11311
export ROS_HOSTNAME=raspberrypi.local

# Pi Zero
export ROS_MASTER_URI=http://raspberrypi.local:11311
export ROS_HOSTNAME=zero.local

# Very nice!
sshfs zero:/home/ros/bb8_ws ~/pi_zero

sudo apt install pigpio
  
sudo systemctl enable pigpiod   # autostart every boot
sudo systemctl start  pigpiod   # start it right now

Check status is ok
systemctl status pigpiod

# for vscode
catkin build -DCMAKE_EXPORT_COMPILE_COMMANDS=1

move compile_commands from catkin_ws/build under your vscode work directory, i usually have it under catkin_ws/src, personal preference


# Fix cartographer dependency
# Build once (you already did):
cd /usr/src/googletest
sudo cmake .
sudo make            # creates googlemock/libgmock.a  googletest/libgtest.a

# Install to a standard path so every new build can find them:
sudo cp googlemock/libgmock*.a googletest/libgtest*.a /usr/lib/
sudo ldconfig        # refresh linker cache


# zero default noetic is missing common_msgs
git clone https://github.com/ros/common_msgs.git
catkin build common_msgs
rospack find geometry_msgs




# PIGPIO

```
wget https://github.com/joan2937/pigpio/archive/master.zip
unzip master.zip
cd pigpio-master
make
sudo make install
```

copy in /usr/local/lib/pkgconfig/pigpio.pc
```
prefix=/usr/local
exec_prefix=${prefix}
libdir=${exec_prefix}/lib
includedir=${prefix}/include

Name: pigpio
Description: pigpio C library for Raspberry Pi GPIO control
Version: 1.79
Libs: -L${libdir} -lpigpio -pthread -lrt
Cflags: -I${includedir}
```
test
pkg-config --cflags --libs pigpio
and get 
-I/usr/local/include -L/usr/local/lib -lpigpio -pthread -lrt

and to run:
sudo pigpiod


# Run as root
sudo su root
export ROS_MASTER_URI=http://raspberrypi.local:11311
export ROS_IP=zero.local
source /opt/ros/noetic/setup.bash
cd bb8_ws
source devel/setup.bash
sudo killall pigpiod
rosrun bb8_zero bb8_zero_node

![Pi zero W Pinout](imgs/pi_zero_w_pinout.png)
![Pi 4b Pinout](imgs/pi_4b_pinout.png)