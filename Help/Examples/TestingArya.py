ros2 daemon stop

printenv | grep 'ROS'
printenv | grep 'RMW'
printenv | grep 'FAST'

source ~/.bashrc
export ROS_DOMAIN_ID=4
#export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE='disable.xml'

printenv | grep 'ROS'
printenv | grep 'RMW'
printenv | grep 'FAST'

ros2 topic list
#ros2 topic echo /AryaStark/battery_state
