#! /bin/bash
# 设置ROS环境变量
gnome-terminal -- bash /home/wheeltec/ready/all_ready.sh
sleep 3

# roscore
gnome-terminal -- bash -c 'source /opt/ros/melodic/setup.bash &&
roscore'
sleep 5


# 启动GUI脚本
gnome-terminal -- bash -c 'source /opt/ros/melodic/setup.bash &&
python /home/wheeltec/iteration.py'




