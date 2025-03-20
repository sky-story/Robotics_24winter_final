#!/bin/bash

ros2 run lab_quaternion car_basic_1

sleep 1

ros2 run lab_quaternion pick_basic

sleep 1

ros2 run lab_quaternion car_basic_2

wait

