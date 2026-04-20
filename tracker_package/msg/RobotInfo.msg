

#Robot Status

uint8 RANDOMWALK = 0
uint8 WORKING = 1


#Type Work
uint8 NON_SPECIALIST = 0
uint8 WORK_YELLOW = 1
uint8 WORK_BLUE = 2

# RobotInfo.msg
nav_msgs/Odometry odom
uint8 robotstate
uint8 typework
float32 timework