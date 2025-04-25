# Software

## Setup

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
sudo apt install ros-foxy-ros1-bridge
```

### Install [catkin](https://catkin-tools.readthedocs.io/en/latest/installing.html)
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install python3-catkin-tools
```

### Install [RPLIDAR](https://github.com/Slamtec/rplidar_ros) + [IMU](https://github.com/Brazilian-Institute-of-Robotics/mpu6050_driver)
```
mkdir -p BB8/src/cartographer

cd BB8/src
git submodule add https://github.com/Slamtec/rplidar_ros.git rplidar_ros
rosdep install --from-paths . --ignore-src -r -y

git submodule add https://github.com/Brazilian-Institute-of-Robotics/i2c_device_ros.git
git submodule add https://github.com/Brazilian-Institute-of-Robotics/mpu6050_driver.git
```

### Install [cartographer](https://google-cartographer-ros.readthedocs.io/en/latest/index.html)
```
cd cartographer
git submodule add https://github.com/cartographer-project/cartographer.git
git submodule add https://github.com/cartographer-project/cartographer_ros.git
rosdep install --from-paths . --ignore-src -r -y
./cartographer/scripts/install_abseil.sh
catkin config -a "-DPYTHON_EXECUTABLE=$(which python3)"
sudo apt-get install ros-noetic-pcl-conversions
sudo apt-get install ros-noetic-pcl-ros
cd ../..

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
Host github-bb8
    HostName github.com
    User raspberry
    IdentityFile ~/.ssh/id_bb8_deploy
    IdentitiesOnly yes

git config --global user.email "you@example.com"
git config --global user.name "Your Name"
```

## Ðevelop
### Build
Build only `bb8` without dependencies
```
catkin build bb8 --no-deps
```

### [Calibrate](https://mjwhite8119.github.io/Robots/mpu6050)
```
roslaunch mpu6050_driver mpu6050_calibration.launch
```

### XARCO to URDF
```
cd ~/BB8/src/bb8
rosrun xacro xacro urdf/bb8.urdf.xacro -o urdf/bb8.urdf
```

## RUN

### Run RPLIDAR
```
sros1
roslaunch rplidar_ros rplidar_a1.launch
```

### ROS bridge
```
sros2
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

### rviz with .rviz
```
ros2 run rviz2 rviz2 -d ./laser.rviz
```


