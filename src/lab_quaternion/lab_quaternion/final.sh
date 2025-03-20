#!/bin/bash

# 运行 TurtleBot 使其到达起始点
ros2 run lab_quaternion car_forward

# 运行机械手抓取任务
ros2 run lab_quaternion final_pick

# 等待3秒，确保机械手完成任务
sleep 3

# 运行 TurtleBot 返回原始位置
ros2 run lab_quaternion car_back

# 保持进程运行
wait

