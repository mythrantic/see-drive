just follow the manual in the docs folder

Ros2 - iron irwino version supports ubuntu 22.04 https://docs.ros.org/en/foxy/Releases/Release-Iron-Irwini.html
but rasberry pi 5 does not support ubuntu 22.04

so intead i think im gonna use the latest ubuntu and just manually add ros2: https://lkseng.github.io/posts/2024/01/07/ros-2-humble-on-raspberrypi-5.html
on ubuntu it works fine following the link but its for installing the full ros2. like what you would do for the master.

I used iron irwino version. this is the docs:
https://docs.ros.org/en/iron/Installation/Windows-Install-Binary.html#id7

for colcon on windows:
check this out after youve installed ros2: https://www.youtube.com/watch?v=HHM_cjfiy3c

unfortunately, i cant install ros2 on windows 11. i was stuck at trying to run the colcon build. it failed with code errors. nothing to do with envaronment variables etc. if i follow th error. they ere fixable. but i cant be bothered

https://docs.ros.org/en/humble/p/rplidar_ros/
https://www.armbian.com/jetson-nano/




after cloning this project run:
colcon build --symlink-install ## build the workspace
source install/setup.bash ## source the overlay workspace
ros2 launch my_package talker.launch.py
ros2 launch my_package listener.launch.py
