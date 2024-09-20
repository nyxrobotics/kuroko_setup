#!/bin/bash

### Use jetpack 5.1.1: https://developer.nvidia.com/embedded/jetpack-sdk-511
### Use "SD Card Image Method" and balenaEtcher
echo "----- Setting up ROS1 Noetic -----"
echo "reference: https://github.com/dnovischi/jetson-tutorials/blob/main/jetson-nano-ros-noetic-install.md"
echo "1. First make sure you have gcc-10 and g++-10 activated:"
echo "GCC:"
sudo update-alternatives --config gcc
echo "G++:"
sudo update-alternatives --config g++
echo "2. Adding the ROS repository"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential cmake
echo "3. Initialize ROS dependency manager tool"
sudo rosdep init
rosdep update
echo "4. Create a catkin workspace"
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
echo "5. Install ROS Noetic"
rosinstall_generator robot perception --rosdistro noetic --deps --tar > noetic-robot-perception.rosinstall
vcs import --input noetic-robot-perception.rosinstall ./src
echo "6. Resolve dependencies with rosdep"
rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y -r
echo "7. Fix OpenCV dependency"
sed -i 's/find_package(Boost REQUIRED python37)/find_package(Boost REQUIRED python3)/' ./src/vision_opencv/cv_bridge/CMakeLists.txt
echo "8. Build"
catkin build --install -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
echo "9. Setup ROS environment"

if ! grep -Fxq "## For ROS setup" ~/.bashrc
then
    echo -e "\n## For ROS setup"  >> ~/.bashrc
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
    echo "source `catkin locate --shell-verbs`" >> ~/.bashrc
    echo "export ROSCONSOLE_FORMAT='[${severity}] [${node}]: ${message}'" >> ~/.bashrc
    source ~/.bashrc
fi

if ! grep -Fxq "## CUDA and cuDNN paths" ~/.bashrc
then
    echo -e "\n## CUDA and cuDNN paths"  >> ~/.bashrc
    echo 'export PATH=/usr/local/cuda-11.4/bin:${PATH}' >> ~/.bashrc
    echo 'export LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64:${LD_LIBRARY_PATH}' >> ~/.bashrc
    source ~/.bashrc # reload .bashrc with cuda path
fi

echo "----- Setting up KUROKO ROS Package -----"
sudo apt install -y ros-noetic-catkin ros-noetic-rosbash python3-pip python3-catkin-tools python3-osrf-pycommon python3-vcstool python3-testresources
sudo apt install -y ros-noetic-robot-upstart
sudo pip3 install vcstool
sudo pip3 install pipenv
source ~/.bashrc
cd ~/catkin_ws/src
git clone -b feature/roboone https://github.com/nyxrobotics/kuroko_ros.git
cd ..
bash src/kuroko_ros/install_dependency.sh
# bash src/kuroko_ros/kuroko_image_recognition/trained/download_roboone_model.sh
sudo cp src/kuroko_ros/kuroko_bringup/udev/rules.d/99-kuroko-usb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
touch ~/catkin_ws/src/robotis/robotis_op3_tools/op3_action_editor/CATKIN_IGNORE

echo "----- SetUp Finished -----"
echo "----- 1. Please Pair your Bluetooth Gamepad with bluez -----"
echo "----- 2. Run commands below to enable startup -----"
echo "\nTERMINAL 1:"
echo "roscore"
echo "\nTERMINAL 2:"
echo "cd ~/catkin_ws/src && catkin source"
echo "rosrun robot_upstart install kuroko_bringup/launch/roboone_startup.launch --job roboone_startup --symlink"
### Restart:  sudo systemctl daemon-reload && sudo systemctl start roboone_startup
### Disable: rosrun robot_upstart uninstall roboone_startup
