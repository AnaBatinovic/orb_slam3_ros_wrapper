export UAV_NAMESPACE=red
export PIX_SYM=/dev/ttyUSB_px4:921600

export SENSOR=optitrack
export OBJECT_NAME=duckpink
export ODOM_TOPIC=/$OBJECT_NAME/vrpn_client/estimated_odometry
export OPTITRACK_IP=192.168.1.50

export CONTROL_TYPE=pid_cascade_node_yawrate
export CONTROL_PARAMS=custom_config/optitrack_position_control_duck.params.yaml
export RC_MAPPING=
