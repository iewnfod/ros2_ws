#!/bin/bash

ros2 service call /vesc_11/detect_zero std_srvs/srv/Trigger "{}" &
ros2 service call /vesc_22/detect_zero std_srvs/srv/Trigger "{}" &
ros2 service call /vesc_33/detect_zero std_srvs/srv/Trigger "{}" &
ros2 service call /vesc_44/detect_zero std_srvs/srv/Trigger "{}"
