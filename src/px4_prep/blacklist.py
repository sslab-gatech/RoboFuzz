# irrelevant or tested parameters
# manage statically for now
blacklist = [
    "CAL_ACC0_XOFF", # calibrates the sensor offset
    "CAL_ACC0_YOFF",
    "CAL_ACC0_ZOFF",
    "CAL_ACC1_XOFF",
    "CAL_ACC1_YOFF",
    "CAL_ACC1_ZOFF",
    "CAL_ACC2_XOFF",
    "CAL_ACC2_YOFF",
    "CAL_ACC2_ZOFF",
    "CAL_GYRO0_XOFF",
    "CAL_GYRO0_YOFF",
    "CAL_GYRO0_ZOFF",
    "CAL_GYRO1_XOFF",
    "CAL_GYRO1_YOFF",
    "CAL_GYRO1_ZOFF",
    "CAL_GYRO2_XOFF",
    "CAL_GYRO2_YOFF",
    "CAL_GYRO2_ZOFF",
    "CAL_MAG0_XOFF",
    "CAL_MAG0_YOFF",
    "CAL_MAG0_ZOFF",
    "CAL_MAG1_XOFF",
    "CAL_MAG1_YOFF",
    "CAL_MAG1_ZOFF",
    "EKF2_GPS_POS_X", # doesn't make sense to restrict these (hw-dependent)
    "EKF2_GPS_POS_Y",
    "EKF2_GPS_POS_Z",
    "EKF2_IMU_POS_X",
    "EKF2_IMU_POS_Y",
    "EKF2_IMU_POS_Z",
]

# already confirmed as buggy
tested = [
    "MC_PITCHRATE_D",
    "MC_PITCHRATE_FF",
    "MC_PITCHRATE_I",

]
