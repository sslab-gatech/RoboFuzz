import math


def check(config, msg_list, pose_list, feedback_list):
    errs = list()

    for (ts, pose) in pose_list:
        if pose.x < 0:
            errs.append("pose.x is below minimum")

        if pose.x >= 11.09:
            errs.append("pose.x is above maximum")

        if pose.y < 0:
            errs.append("pose.y is below minimum")

        if pose.y >= 11.09:
            errs.append("pose.y is above maximum")

        if pose.theta >= 3.141593:
            errs.append("pose.theta is above maximum")

        if pose.theta <= -3.141593:
            errs.append("pose.theta is below minimum")

        if math.isnan(pose.x):
            errs.append("pose.x is NaN")

        if math.isnan(pose.y):
            errs.append("pose.y is NaN")

        if math.isnan(pose.theta):
            errs.append("pose.theta is NaN")

        if math.isnan(pose.linear_velocity):
            errs.append("pose.linear_velocity is NaN")

        if math.isnan(pose.angular_velocity):
            errs.append("pose.anglar_velocity is NaN")

        if math.isinf(pose.x):
            errs.append("pose.x is INF")

        if math.isinf(pose.y):
            errs.append("pose.y is INF")

        if math.isinf(pose.theta):
            errs.append("pose.theta is INF")

        if math.isinf(pose.linear_velocity):
            errs.append("pose.linear_velocity is INF")

        if math.isinf(pose.angular_velocity):
            errs.append("pose.angular_velocity is INF")

    return errs

