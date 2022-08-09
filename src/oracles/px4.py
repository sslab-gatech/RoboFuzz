import math
import statistics


def check(config, msg_list, state_dict, feedback_list):
    errs = list()

    try:
        sensor_accel_list = state_dict["/SensorAccel_PubSubTopic"]
    except KeyError:
        print("[checker] no SensorAccel data available")
        sensor_accel_list = list()

    try:
        sensor_baro_list = state_dict["/SensorBaro_PubSubTopic"]
    except KeyError:
        print("[checker] no SensorBaro data available")
        sensor_baro_list = list()

    try:
        sensor_gps_list = state_dict["/SensorGps_PubSubTopic"]
    except KeyError:
        print("[checker] no SensorGps data available")
        sensor_gps_list = list()

    try:
        sensor_gyro_list = state_dict["/SensorGyro_PubSubTopic"]
    except KeyError:
        print("[checker] no SensorGyro data available")
        sensor_gyro_list = list()

    try:
        sensor_imu_list = state_dict["/SensorsStatusImu_PubSubTopic"]
    except KeyError:
        print("[checker] no SensorsStatusImu data available")
        sensor_imu_list = list()

    try:
        sensor_combined_list = state_dict["/SensorCombined_PubSubTopic"]
    except KeyError:
        print("[checker] no SensorCombined data available")
        sensor_combined_list = list()

    try:
        vehicle_odometry_list = state_dict["/VehicleOdometry_PubSubTopic"]
    except KeyError:
        print("[checker] no SensorCombined data available")
        vehicle_odometry_list = list()

    try:
        vehicle_imu_list = state_dict["/VehicleImu_PubSubTopic"]
    except KeyError:
        print("[checker] no VehicleImu data available")
        vehicle_imu_list = list()

    try:
        vehicle_gps_list = state_dict["/VehicleGpsPosition_PubSubTopic"]
    except KeyError:
        print("[checker] no VehicleGpsPosition data available")
        vehicle_gps_list = list()

    try:
        vehicle_global_position_list = state_dict["/VehicleGlobalPosition_PubSubTopic"]
    except KeyError:
        print("[checker] no VehicleGlobalPosition data available")
        vehicle_global_position_list = list()

    try:
        vehicle_local_position_list = state_dict["/VehicleLocalPosition_PubSubTopic"]
    except KeyError:
        print("[checker] no VehicleLocalPosition data available")
        vehicle_local_position_list = list()

    try:
        vehicle_attitude_list = state_dict["/VehicleAttitude_PubSubTopic"]
    except KeyError:
        print("[checker] no VehicleAttitude data available")
        vehicle_attitude_list = list()

    try:
        vehicle_acceleration_list = state_dict["/VehicleAcceleration_PubSubTopic"]
    except KeyError:
        print("[checker] no VehicleAcceleration data available")
        vehicle_acceleration_list = list()

    try:
        vehicle_angular_velocity = state_dict["/VehicleAngularVelocity_PubSubTopic"]
    except KeyError:
        print("[checker] no VehicleAngularVelocity data available")
        vehicle_angular_velocity = list()

    try:
        vehicle_angular_acceleration = state_dict["/VehicleAngularAcceleration_PubSubTopic"]
    except KeyError:
        print("[checker] no VehicleAngularAcceleration data available")
        vehicle_angular_acceleration = list()

    # 0. Filter timestamps when the drone is on the ground or hit the
    # ground. Timestamps are in nanoseconds.
    filter_ts_list = list()
    local_x_list = list()
    local_y_list = list()
    local_z_list = list()
    for (ts, pos) in vehicle_local_position_list:
        dist_bottom = pos.dist_bottom

        if dist_bottom < 0.15: # unit: meters
            filter_ts_list.append(ts)

        # to use in step 3
        local_x_list.append(pos.x)
        local_y_list.append(pos.y)
        local_z_list.append(pos.z)

    # 1. check for parameter violation
    # http://docs.px4.io/master/en/advanced_config/parameter_reference.html

    # 1.1. Acceleration
    for (ts, acc) in vehicle_acceleration_list:
        acc_x = acc.xyz[0]
        acc_y = acc.xyz[1]
        acc_z = acc.xyz[2] + 9.8 # 9.8 offsets gravity # +: down, -: up
        hor_acc = math.sqrt(pow(acc_x, 2) + pow(acc_y, 2))

        local_errs = list()

        if config.flight_mode == "POSCTL":
            if hor_acc > 5.0:
                local_errs.append(f"{ts} MPC_ACC_HOR_MAX violated: {hor_acc} > 3.0")

            if acc_z < -4.0:
                local_errs.append(f"{ts} MPC_ACC_UP_MAX violated: {acc_z} < -4.0")

            if acc_z > 3.0:
                local_errs.append(f"{ts} MPC_ACC_DOWN_MAX violated: {acc_z} > 3.0")

        if len(local_errs) > 0:
            # check if ground
            ts_diff = [abs(ts - f_ts) for f_ts in filter_ts_list]
            if len(ts_diff) == 0: # if drone was always above the ground
                errs.extend(local_errs)
            elif min(ts_diff) > 0.25 * 1000 * 1000 * 1000: # diff > 0.25 sec
                errs.extend(local_errs)
            else:
                pass
                # print("[checker] TS filtered:", local_errs)

    # 1.2. Velocity
    for (ts, pos) in vehicle_local_position_list:
        vx = pos.vx
        vy = pos.vy
        vz = pos.vz

        local_errs = list()
        hor_vel = math.sqrt(pow(vx, 2) + pow(vy, 2))

        if config.flight_mode == "MANUAL":
            if hor_vel > 10.0:
                local_errs.append(f"{ts} MPC_VEL_MANUAL violated: {hor_vel} > 10.0")

        elif config.flight_mode == "POSCTL":
            if vz < 0 and vz < -1.0: # going down
                local_errs.append(f"{ts} MPC_Z_VEL_MAX_DN violated: {vz} < -1.0")
            elif vz > 0 and vz > 3.0: # going up
                local_errs.append(f"{ts} MPC_Z_VEL_MAX_UP violated: {vz} > 3.0")

        elif config.flight_mode == "ALTCTL":
            if vz < 0 and vz < -1.0: # going down
                local_errs.append(f"{ts} MPC_Z_VEL_MAX_DN violated: {vz} < -1.0")
            elif vz > 0 and vz > 3.0: # going up
                local_errs.append(f"{ts} MPC_Z_VEL_MAX_UP violated: {vz} > 3.0")

        if len(local_errs) > 0:
            # check if ground
            ts_diff = [abs(ts - f_ts) for f_ts in filter_ts_list]
            if len(ts_diff) == 0: # if drone was always above the ground
                errs.extend(local_errs)
            elif min(ts_diff) > 0.25 * 1000 * 1000 * 1000: # diff > 0.25 sec
                errs.extend(local_errs)
            else:
                pass
                # print("[checker] TS filtered:", local_errs)

    # 2. check for sensor inconsistency
    # 2.1. IMU discrepancy
    acc_inconsistency_list = list()
    gyro_inconsistency_list = list()
    for (ts, imu_state) in sensor_imu_list:
        acc_inconsistency_list.append(imu_state.accel_inconsistency_m_s_s[0])
        gyro_inconsistency_list.append(imu_state.gyro_inconsistency_rad_s[0])

    for feedback in feedback_list:
        if feedback.name == "imu_accel_inconsistency":
            feedback.update_value(max(acc_inconsistency_list))
        elif feedback.name == "imu_gyro_inconsistency":
            feedback.update_value(max(gyro_inconsistency_list))

    # 2.2. GPS discrepancy
    # Measure discrepancy between vehicle_gps_position (copy of raw sensor
    # readings) VS vehicle_global_position (EKF2 estimation)

    # raw gps messages are far less frequently published than the
    # gps estimates, thus need timestamp matching
    gps_matched = list()
    lat_diff = list()
    lon_diff = list()
    skip = 0
    for (ts_gps_raw, gps_raw) in vehicle_gps_list:
        ts_diff = 9999999999999999999
        last_updated = 0
        updated = 0

        for i in range(skip, len(vehicle_global_position_list)):
            ts_gps_estim = vehicle_global_position_list[i][0]
            gps_estim = vehicle_global_position_list[i][1]

            ts_diff_last = abs(ts_gps_raw - ts_gps_estim)
            if ts_diff_last <= ts_diff:
                ts_diff = ts_diff_last
                last_updated = updated
                updated = 1
            else:
                last_updated = updated
                updated = 0

            if last_updated == 1 and updated == 0:
                skip = i
                break

        gps_matched.append(
            (
                ts_gps_raw,
                ts_gps_estim,
                gps_raw.lat,
                int(gps_estim.lat * 10000000),
                gps_raw.lon,
                int(gps_estim.lon * 10000000)
            )
        )
        lat_diff.append(gps_raw.lat - int(gps_estim.lat * 10000000))
        lon_diff.append(gps_raw.lon - int(gps_estim.lon * 10000000))

    updated = 0
    for feedback in feedback_list:
        if feedback.name == "gps_lat_inconsistency":
            feedback.update_value(max(lat_diff))
            updated += 1
        elif feedback.name == "gps_lon_inconsistency":
            feedback.update_value(max(lon_diff))
            updated += 1

        if updated == 2:
            break

    # 3. During parameter testing (in hold mode), drone should not move
    #    (stay within a reasonably small area)
    if config.exp_pgfuzz:
        mean_x = statistics.mean(local_x_list)
        mean_y = statistics.mean(local_y_list)
        mean_z = statistics.mean(local_z_list)

        # std_x = statistics.pstdev(local_x_list)
        # std_y = statistics.pstdev(local_y_list)
        # std_z = statistics.pstdev(local_z_list)

        # print("mean", mean_x, mean_y, mean_z)
        # print("stdev", std_x, std_y, std_z)

        diff_x = [abs(x - mean_x) for x in local_x_list]
        diff_y = [abs(y - mean_y) for y in local_y_list]
        diff_z = [abs(z - mean_z) for z in local_z_list]

        # deviation over thr (m) should be considered an error
        # empirically set thr to 0.3 m (1 foot)
        thr = 0.3
        if max(diff_x) > thr or max(diff_y) > thr or max(diff_z) > thr:
            diff_str = f"x {max(diff_x):.2f} y {max(diff_y):.2f} z {max(diff_z):.2f}"
            errs.append(f"{ts} Position changed in hold mode: {diff_str}")

    return errs

