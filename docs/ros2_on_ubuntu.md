# ROS 2 Iron on Raspberry Pi 5 Running Ubuntu 23.10

## Jan 7, 2024

## Introduction

While attempting to try out ROS 2 on the recently released Raspberry Pi 5, I quickly ran into trouble. Firstly, there will be no support for Ubuntu 22.04 on Raspberry Pi 5. However, downgrading the OS is not possible as there is no ROS 2 support for Ubuntu 23.10. This means that the alternatives are to either use Docker or build from source.

If one tries to boot a Raspberry Pi 5 using Ubuntu 22.04, startup will be blocked, and it will say that the OS is not supported, suggesting modifying the `config.txt` with `check_os=0`. This approach is not recommended as it will involve disabling other options as well, and devices like GPU and GPIO may not work properly.

While it is recommended to use Docker, in this post I will outline how I managed to build ROS 2 Iron on Raspberry Pi 5.

---

## Setup

First, you need to have Ubuntu 23.10 installed. One can follow the instructions [here](https://ubuntu.com/download/raspberry-pi) to install Ubuntu 23.10 onto the Raspberry Pi 5.

---

## Build

After installing Ubuntu 23.10, boot up the Raspberry Pi 5 device. The steps in this section are adapted from [Iron Ubuntu (Source)](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Binary.html).

### Check Locale

As long as the first command, `locale`, outputs something with UTF-8, you should be good to go and can skip to the next step.

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

### Add the ROS 2 apt Repository

```bash
# Enable Ubuntu universe repository
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo jammy) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

**Note:** Instead of `$(. /etc/os-release && echo $UBUNTU_CODENAME)`, `$(. /etc/os-release && echo jammy)` is used because there is no ROS 2 for Mantic (Ubuntu 23.10).

At this stage, one can attempt to install `ros-iron-desktop` from apt. However, Ubuntu 23.10 ships with Python 3.11, while iron uses Python 3.10. Downgrading is not recommended.

### Install Development Tools and ROS Tools

```bash
sudo apt update && sudo apt install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools

sudo apt install -y \
   python3-flake8-blind-except \
   python3-flake8-builtins \
   python3-flake8-class-newline \
   python3-flake8-comprehensions \
   python3-flake8-deprecated \
   python3-flake8-import-order \
   python3-flake8-quotes \
   python3-pytest-repeat \
   python3-pytest-rerunfailures
```

**Note:** If unable to install `ros-dev-tools`, check that `/etc/apt/sources.list.d/ros2.list` is properly populated.

### Get ROS 2 Code

```bash
mkdir -p ~/ros2_iron/src
cd ~/ros2_iron
vcs import --input https://raw.githubusercontent.com/ros2/ros2/iron/ros2.repos src
```

### Install Dependencies Using rosdep

This may take about 0.5 hours to complete.

```bash
sudo apt upgrade
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers" --os=ubuntu:jammy --rosdistro=iron
```

**Note:** Because we are on an unsupported OS, we have to "force" it to use Ubuntu 22.04 (`--os=ubuntu:jammy`). Further, we must also specify ROS 2 iron (`--rosdistro=iron`).

### Build

This step may take more than 1.5 hours to complete. Ensure that you do not have `source /opt/ros/${ROS_DISTRO}/setup.bash` in your `.bashrc`.

```bash
cd ~/ros2_iron/
colcon build --symlink-install
```

**Fix for Build Errors:**

If you encounter errors with `uint32_t` and `uint64_t`, add `#include <cstdint>` to the following files:

- `~/ros2_iron/src/ros2/rcpputils/include/rcpputils/filesystem_helper.hpp`
- `~/ros2_iron/src/ros-tooling/libstatistics_collector/include/libstatistics_collector/moving_average_statistics/types.hpp`
- `~/ros2_iron/src/ros2/rosbag2/rosbag2_compression/include/rosbag2_compression/compression_options.hpp`

---

## Test

### Pub Sub

#### First Window (Talker)

```bash
. ~/ros2_iron/install/local_setup.bash  # source if needed
ros2 run demo_nodes_cpp talker
```

#### Second Window (Listener)

```bash
. ~/ros2_iron/install/local_setup.bash  # source if needed
ros2 run demo_nodes_py listener
```

You should see the talker publishing messages and the listener saying “I heard” messages.

### RViz2

RViz2 does not run out of the box and segfaults on launch (related to rendering window creation errors). If using Wayland, OGRE does not support it. We must force X11:

```bash
. ~/ros2_iron/install/local_setup.bash  # source if needed
QT_QPA_PLATFORM=xcb rviz2
```

---

## Conclusion

In this post, I demonstrated how to build ROS 2 iron on Ubuntu 23.10 on Raspberry Pi 5. Though it compiles, I have yet to thoroughly test the stability and viability of this setup. Perhaps using the Docker approach is not that bad after all…

