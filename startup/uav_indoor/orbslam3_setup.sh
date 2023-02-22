export UAV_NAMESPACE=red
export PIX_SYM=/dev/ttyUSB_px4:921600

#export SENSOR=orb_slam3
export ODOM_TOPIC=/$UAV_NAMESPACE/es_ekf/odom
export IMU_TOPIC=/$UAV_NAMESPACE/mavros/imu/data
export OBJECT_NAME=duckpink
export OPTITRACK_IP=192.168.1.50

export CONTROL_TYPE=pid_cascade_node_yawrate
export CONTROL_PARAMS=custom_config/slam_position_control_duck.params.yaml
export RC_MAPPING=

# Sensor fusion config
export SF_CONFIG=custom_config/sensor_orbslam_config.yaml
