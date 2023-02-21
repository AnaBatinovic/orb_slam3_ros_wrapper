export UAV_NAMESPACE=red
export PIX_SYM=/dev/ttyUSB_px4:921600

#export SENSOR=orb_slam3
export ODOM_TOPIC=/$UAV_NAMESPACE/es_ekf/odom
export IMU_TOPIC=/$UAV_NAMESPACE/lpms/imu/data

export CONTROL_TYPE=pid_cascade_node_yawrate
export CONTROL_PARAMS=custom_config/optitrack_position_control_duck.params.yaml
export RC_MAPPING=

# Sensor fusion config
export SF_CONFIG=custom_config/sensor_orbslam_config.yaml
