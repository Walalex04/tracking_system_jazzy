

#Robot Status
uint8 SEARCHING = 0
uint8 WORKING = 1
uint8 REJECTED = 2

uint8 NON_SPECIALIST = 5

#Type Work
uint8 UNDEFINED = 0
uint8 WORK_GREEN = 1
uint8 WORK_BLUE = 2

# RobotInfo.msg
nav_msgs/Odometry odom
uint8 robotstate
uint8 typework
float32 timework