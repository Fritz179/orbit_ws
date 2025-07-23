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
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics --bridge-all-services
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
or
ros2 run teleop_twist_keyboard teleop_twist_keyboard

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
rosrun bb8_zero bb8_zero

![Pi zero W Pinout](imgs/pi_zero_w_pinout.png)
![Pi 4b Pinout](imgs/pi_4b_pinout.png)


# tf pdf
rosrun tf view_frames



The BB8 Projects has a raspberry pi 4b board mounted on the head, conencted with an mpu6050 and a rplidar a1.
The secondary board is pi Zero W in the inside a sphere controlling a car and also has an mpu6050. it aslo controlls a stepper which can rotate the head along one axis, the head is attached magnetically to the sphere.
The head has a speaker and a led which can be controlled by the pi 4b.

The pi 4b runs a ROS noetic and runs the roscore on ubutnu 20.04 and uses cartographer for mapping and localization.
The zero also has nouetic but on a custom image, it has a custom kernel and uses pigpio to control the motors.
The center of mass is below the center of the sphere, but not by much, it is stable when unpowered

The wheels don't have an encoder, the position of the stepper is known an can therrfore theoretically transform from one imu to the other.

The code is located at https://github.com/Fritz179/bb8_ws



How do I best balance the robot? The wheels don't have an encoder, i was thinking of using robot_localization but unsure if I want to use the imu in the sphere or also the data from the top one, the position of the stepper is known an can therrfore theoretically transform from one imu to the other

# Remote

| /joy or keyboard
+-bb8_remote
| /cmd_vel
| /cmd_head
| /cmd_head_calibrate

# Pi 4b

| head/imu_raw
+-ekf_localization
| head/imu_filtered

| head/imu_raw
| head/rplidar
+-cartographer
| /map
| tf map->odom

+-ros_bridge

# Pi Zero W

| spere/imu_raw
+-ekf_localization
| spere/imu_filtered

| sphere/imu_filtered
| sphere/odometry (low weight)
+-ukf_localization_node
| /robot_state_sphere

| /robot_state_sphere
| /cmd_vel
| /cmd_head
| /cmd_head_calibrate
+-bb8_zero
| DC Motor
| Stepper Motor


// TODO:
rviz_imu_plugin for imu visualizzation  
sudo apt install ros-noetic-navigation


rosdep install -y --from-paths src --ignore-src --rosdistro humble -r --os=ubuntu:jammy
colcon build --packages-up-to moveit_setup_assistant --cmake-args -DCMAKE_BUILD_TYPE=Release


ros@raspberrypi:~$ cat /boot/firmware/usercfg.txt 
# Place "config.txt" changes (dtparam, dtoverlay, disable_overscan, etc.) in
# this file. Please refer to the README file for a description of the various
# configuration files on the boot partition.

dtparam=audio=on                     # keep analogue/PWM audio enabled
dtoverlay=audremap,pins_12_13        # move L/R to GPIO12 & GPIO13

cat /sys/class/thermal/thermal_zone0/temp


S2E05

# Bridge with cartographer
sros1
source devel/setup.bash # Sees cartographer messages

sros2
cd src
git clone https://github.com/ros2/ros1_bridge.git
cd ..
# colcon build --packages-select ros1_bridge
rosdep install -y --from-paths src/ros1_bridge --ignore-src --rosdistro foxy
colcon build --build-base colcon_build --packages-select ros1_bridge
source install/setup.bash

run the bride and the check with:
ros2 run ros1_bridge dynamic_bridge --print-pairs | grep SubmapList


ffmpeg -i sw.wav -af "volume=10dB" sws.wav
ffmpeg -i lo.wav -af "volume=-1dB" lob.wav
speaker-test -t wav -c 2

# God forgive me for what I am about to do
touch src/cartographer/cartographer_ros/COLCON_IGNORE
colcon build --packages-select cartographer_ros_msgs --build-base colcon_build

colcon build --packages-select ros1_bridge --cmake-force-configure --build-base colcon_build    


sudo systemd-nspawn -D /media/struct/a57fe4be-f8f5-47ba-a9cb-198aa9a5c682/ --boot --machine ubu20


https://www.handsontec.com/dataspecs/17HS4401S.pdf
https://de.aliexpress.com/item/1005004614528741.html

https://forum.arduino.cc/t/how-to-measure-and-set-the-correct-current-limit-on-stepper-motor-driver/588955  

" Be VERY CAREFUL never to disconnect the wires between the motor and the stepper driver 
    while the driver is powered up. The driver will be instantly destroyed. "

                                                                Robin2, Apr 2019
https://drive.google.com/drive/folders/1gnNBD1z_hAVE9SsqzKxrgG_n-57FNf3M




sudo nano /etc/resolv.conf
Remove any existing nameserver lines and add these:

nameserver 8.8.8.8
nameserver 1.1.1.1

colcon build --packages-select ros1_bridge --cmake-force-configure --build-base colcon_build

qt.qpa.xcb: XKeyboard extension not present on the X server

process has died [pid 2621, exit code -11, cmd /opt/ros/noetic/lib/rviz/rviz -d /home/ros/bb8_ws/src/cartographer/cartographer_ros/cartographer_ros/configuration_files/demo_3d.rviz __name:=rviz __log:=/home/ros/.ros/log/0343591a-301a-11f0-9d08-ed3397efa0af/rviz-5.log].
log file: /home/ros/.ros/log/0343591a-301a-11f0-9d08-ed3397efa0af/rviz-5*.log
Initiating shutdown!

[playbag-6] killing on exit

roslaunch cartographer_ros demo_backpack_3d.launch bag_filename:=${HOME}/Downloads/b3-2016-02-02-13-32-01.bag

x11vnc -display :0 \
       -auth /run/user/130/gdm/Xauthority \
       -rfbauth ~/.vnc/passwd \
       -forever -shared -noxdamage -xkb -clipboard

       x11vnc -display :0 -auth /run/user/130/gdm/Xauthority  -rfbauth ~/.vnc/passwd -forever -shared -noxdamage -xkb

respawn-pane -k

rosservice call /head/finish_trajectory "trajectory_id: 0"
rosservice call /head/write_state "filename: '/home/ros/bb8_ws/src/bb8/maps/last.pbstream'

rosservice call /head/get_trajectory_states "{}" 
rosservice call /head/finish_trajectory "trajectory_id: 1"

rosservice call /head/start_trajectory "configuration_directory: '$(rospack find bb8)/config'
configuration_basename: 'ros_2d_navigation.lua'
use_initial_pose: false
initial_pose:
  position: {x: 5.0, y: 0.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1}
relative_to_trajectory_id: 0" 


rosrun cartographer_ros start_trajectory_main  \
--configuration_directory $(rospack find bb8)/config  \
--configuration_basename ros_2d_navigation.lua \
--initial_pose="0,0,0,0" --relative_to_trajectory_id=0

rostopic echo /initalPose


rosservice call /head/start_trajectory "configuration_directory: '$(rospack find bb8)/config'
configuration_basename: 'ros_2d_navigation.lua'
use_initial_pose: true 
initial_pose:
  position: {x: -5.0, y: 0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1}
relative_to_trajectory_id: 0" 




# Time NTP
```
sudo timedatectl set-ntp true
sudo nano /etc/systemd/timesyncd.conf

[Time]
NTP=time.google.com time.cloudflare.com

sudo systemctl restart systemd-timesyncd
timedatectl status
```
you should see `System clock synchronized: yes` and the correct time.


