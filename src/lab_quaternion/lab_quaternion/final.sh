#!/bin/bash

# 运行机械手抓取任务
ros2 run lab_quaternion final_pick

# 等待3秒，确保机械手完成任务
sleep 2

# 运行 TurtleBot 返回原始位置
ros2 run lab_quaternion car_forward

# 保持进程运行
wait

