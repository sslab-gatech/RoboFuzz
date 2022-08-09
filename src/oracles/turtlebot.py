import math
import statistics
import numpy as np


def check(config, msg_list, state_dict, feedback_list):
    errs = list()

    try:
        imu_list = state_dict["/imu"]
    except KeyError:
        print("[checker] no imu data available")
        imu_list = list()

    try:
        odom_list = state_dict["/odom"]
    except KeyError:
        print("[checker] no odom data available")
        odom_list = list()

    try:
        scan_list = state_dict["/scan"]
    except KeyError:
        print("[checker] no lidar data available")
        scan_list = list()

    imu_angles = list() # for deviation

    ts0 = imu_list[0][0]
    for (ts, imu) in imu_list:
        # Check NaN
        if math.isnan(imu.linear_acceleration.x):
            errs.append("imu.linear_acceleration.x is NaN")

        if math.isnan(imu.linear_acceleration.y):
            errs.append("imu.linear_acceleration.y is NaN")

        if math.isnan(imu.linear_acceleration.z):
            errs.append("imu.linear_acceleration.z is NaN")

        if math.isnan(imu.angular_velocity.x):
            errs.append("imu.angular_velocity.x is NaN")

        if math.isnan(imu.angular_velocity.y):
            errs.append("imu.angular_velocity.y is NaN")

        if math.isnan(imu.angular_velocity.z):
            errs.append("imu.angular_velocity.z is NaN")

        # Check INF
        if math.isinf(imu.linear_acceleration.x):
            errs.append("imu.linear_acceleration.x is INF")

        if math.isinf(imu.linear_acceleration.y):
            errs.append("imu.linear_acceleration.y is INF")

        if math.isinf(imu.linear_acceleration.z):
            errs.append("imu.linear_acceleration.z is INF")

        if math.isinf(imu.angular_velocity.x):
            errs.append("imu.angular_velocity.x is INF")

        if math.isinf(imu.angular_velocity.y):
            errs.append("imu.angular_velocity.y is INF")

        if math.isinf(imu.angular_velocity.z):
            errs.append("imu.angular_velocity.z is INF")

        # Check max
        if imu.angular_velocity.z > 2.84:
            errs.append(f"imu.angular_velocity.z ({imu.angular_velocity.z}) exceeded max (2.84)")

        # robot_pose_2 = np.arctan2(siny_cosp, cosy_cosp)
        imu_angle_ = np.arctan2(
            imu.orientation.x * imu.orientation.y + imu.orientation.w * imu.orientation.z,
            0.5 - imu.orientation.y * imu.orientation.y - imu.orientation.z * imu.orientation.z);
        imu_angles.append((ts, imu_angle_))

    robot_poses_odom = list()

    ts0 = odom_list[0][0]
    for (ts, odom) in odom_list:
        # Check NaN
        if math.isnan(odom.pose.pose.position.x):
            errs.append("odom.pose.pose.position.x is NaN")

        if math.isnan(odom.pose.pose.position.y):
            errs.append("odom.pose.pose.position.y is NaN")

        if math.isnan(odom.pose.pose.position.z):
            errs.append("odom.pose.pose.position.z is NaN")

        if math.isnan(odom.pose.pose.orientation.x):
            errs.append("odom.pose.pose.orientation.x is NaN")

        if math.isnan(odom.pose.pose.orientation.y):
            errs.append("odom.pose.pose.orientation.y is NaN")

        if math.isnan(odom.pose.pose.orientation.z):
            errs.append("odom.pose.pose.orientation.z is NaN")

        if math.isnan(odom.pose.pose.orientation.w):
            errs.append("odom.pose.pose.orientation.w is NaN")

        if math.isnan(odom.twist.twist.linear.x):
            errs.append("odom.twist.twist.linear.x is NaN")

        if math.isnan(odom.twist.twist.linear.y):
            errs.append("odom.twist.twist.linear.y is NaN")

        if math.isnan(odom.twist.twist.linear.z):
            errs.append("odom.twist.twist.linear.z is NaN")

        if math.isnan(odom.twist.twist.angular.x):
            errs.append("odom.twist.twist.angular.x is NaN")

        if math.isnan(odom.twist.twist.angular.y):
            errs.append("odom.twist.twist.angular.y is NaN")

        if math.isnan(odom.twist.twist.angular.z):
            errs.append("odom.twist.twist.angular.z is NaN")

        # Check INF
        if math.isinf(odom.pose.pose.position.x):
            errs.append("odom.pose.pose.position.x is INF")

        if math.isinf(odom.pose.pose.position.y):
            errs.append("odom.pose.pose.position.y is INF")

        if math.isinf(odom.pose.pose.position.z):
            errs.append("odom.pose.pose.position.z is INF")

        if math.isinf(odom.pose.pose.orientation.x):
            errs.append("odom.pose.pose.orientation.x is INF")

        if math.isinf(odom.pose.pose.orientation.y):
            errs.append("odom.pose.pose.orientation.y is INF")

        if math.isinf(odom.pose.pose.orientation.z):
            errs.append("odom.pose.pose.orientation.z is INF")

        if math.isinf(odom.pose.pose.orientation.w):
            errs.append("odom.pose.pose.orientation.w is INF")

        if math.isinf(odom.twist.twist.linear.x):
            errs.append("odom.twist.twist.linear.x is INF")

        if math.isinf(odom.twist.twist.linear.y):
            errs.append("odom.twist.twist.linear.y is INF")

        if math.isinf(odom.twist.twist.linear.z):
            errs.append("odom.twist.twist.linear.z is INF")

        if math.isinf(odom.twist.twist.angular.x):
            errs.append("odom.twist.twist.angular.x is INF")

        if math.isinf(odom.twist.twist.angular.y):
            errs.append("odom.twist.twist.angular.y is INF")

        if math.isinf(odom.twist.twist.angular.z):
            errs.append("odom.twist.twist.angular.z is INF")

        # check max
        lin_vel = odom.twist.twist.linear.x
        if lin_vel > 0.22:
            errs.append(f"linear velocity ({lin_vel}) exceeded max (0.22)")

        ang_vel = odom.twist.twist.angular.z
        if ang_vel > 2.84:
            errs.append(f"angular velocity ({ang_vel}) exceeded max (2.84)")

        # rev-compute theta (check Odometry::publish())
        robot_pose_0 = odom.pose.pose.position.x
        robot_pose_1 = odom.pose.pose.position.y

        x = odom.pose.pose.orientation.x
        y = odom.pose.pose.orientation.y
        z = odom.pose.pose.orientation.z
        w = odom.pose.pose.orientation.w
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        robot_pose_2 = np.arctan2(siny_cosp, cosy_cosp)

        robot_pose_ = [robot_pose_0, robot_pose_1, robot_pose_2]
        robot_poses_odom.append((ts, robot_pose_2))

    last_theta = 0
    robot_poses_imu = []
    delta_theta = 0.0
    for (ts, theta) in imu_angles:
        delta_theta += theta - last_theta
        last_theta = theta
        robot_poses_imu.append((ts, delta_theta))

    # imu data is published approx. 4x faster than odom data
    # need to match the granularity
    theta_matched = list() # ts, odom_theta, imu_theta
    skip = 0
    for (ts_odom, odom) in robot_poses_odom:
        ts_diff = 9999999999999999999
        last_updated = 0
        updated = 0

        for i in range(skip, len(robot_poses_imu)):
            ts_imu = robot_poses_imu[i][0]
            imu = robot_poses_imu[i][1]

            ts_diff_last = abs(ts_odom - ts_imu)
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

        theta_matched.append((ts_odom, ts_imu, odom, imu))

    theta_diff = list()
    for tup in theta_matched:
        # print(tup[0], tup[1], tup[2], tup[3], abs(tup[2] - tup[3]))
        theta_diff.append(abs(tup[2] - tup[3]))

    # todo: normalize diff, get anomaly and translate to feedback!!
    min_diff = min(theta_diff)
    max_diff = max(theta_diff)
    mean = statistics.mean(theta_diff)
    median = statistics.median(theta_diff)

    for feedback in feedback_list:
        if feedback.name == "theta_diff":
            feedback.update_value(max_diff)
            break

    if max_diff > 5.0:
      errs.append(f"theta estimation error is too huge: {max_diff}")

    for (ts, scan) in scan_list:
        found_nan = False
        range_error = False

        for scan_range in scan.ranges:
            if math.isnan(scan_range):
                found_nan = True
                break

            elif scan_range < 0 or scan_range > 65.535:
                range_error = True
                break

        if found_nan:
            errs.append("scan.range contains NaN")

        if range_error:
            errs.append(f"scan.range {scan_range} is out of range")

    return errs

