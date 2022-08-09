import math
import statistics

import kinpy

# A margin of MARGIN degrees is set to suppress an existing bug (joint limits
# not matching the specification) from shadowing other bugs. To reproduce this
# bug, please set MARGIN = 0.0 instead.
MARGIN = 0.1

def check(config, msg_list, state_dict, feedback_list):
    errs = list()

    # Framka Emika's Panda robot
    # https://wiredworkers.io/wp-content/uploads/2019/12/Panda_FrankaEmika_ENG.pdf

    panda_urdf = "/opt/ros/foxy/share/moveit_resources_panda_description/urdf/panda.urdf"
    chain = kinpy.build_chain_from_urdf(open(panda_urdf).read())
    serial_chain = kinpy.build_serial_chain_from_urdf(
        open(panda_urdf).read(),
        "panda_hand"
    )

    try:
        joint_states_list = state_dict["/joint_states"]
    except KeyError:
        print("[checker] no /joint_state data available")
        joint_states_list = list()

    try:
        controller_states_list = state_dict["/panda_arm_controller/state"]
    except KeyError:
        print("[checker] no /panda_arm_controller/state data available")
        controller_states_list = list()

    try:
        move_action_status_list = state_dict["/move_action/_action/status"]
    except KeyError:
        print("[checker] no /move_action/_action/status data available")
        move_action_status_list = list()

    try:
        motion_plan_request_list = state_dict["/motion_plan_request"]
    except KeyError:
        print("[checker] no /motion_plan_request data available")
        motion_plan_request_list = list()

    # 1. joint position and velocity
    # Seven revolute joints have different position limits (deg):
    #   A1, A3, A5, A7: -166/166
    #   A2: -101/101
    #   A4: -176/-4
    #   A6: -1/215
    # note: messages are in rad

    final_joint_state = None
    for (ts, joint_state) in joint_states_list:
        joint_name_list = joint_state.name
        joint_position_list = joint_state.position
        joint_velocity_list = joint_state.velocity

        for pi, pos in enumerate(joint_position_list):

            # let's not trust the list order. use names for dereferencing.
            joint_name = joint_name_list[pi]

            # existential constraints
            if math.isnan(pos):
                errs.append(f"{ts} {joint_name}'s position is NaN")

            elif math.isinf(pos):
                errs.append(f"{ts} {joint_name}'s position is INF")

            else:
                pos_deg = math.degrees(pos)

                check_pos = False
                # specification constraints
                if (joint_name == "panda_joint1" or
                    joint_name == "panda_joint3" or
                    joint_name == "panda_joint5" or
                    joint_name == "panda_joint7"):
                    min_deg = -166 - MARGIN
                    max_deg = 166 + MARGIN
                    check_pos = True

                elif joint_name == "panda_joint2":
                    min_deg = -101 - MARGIN
                    max_deg = 101 + MARGIN
                    check_pos = True

                elif joint_name == "panda_joint4":
                    min_deg = -176 - MARGIN
                    max_deg = -4 + MARGIN
                    check_pos = True

                elif joint_name == "panda_joint6":
                    min_deg = -1 - MARGIN
                    max_deg = 215 + MARGIN
                    check_pos = True

                if check_pos:
                    if pos_deg < min_deg or pos_deg > max_deg:
                        errs.append(f"{ts} {joint_name}'s position {pos_deg} is not within {min_deg} ~ {max_deg}")

        final_joint_state = joint_state

    # 2. joint position and velocity (torque controller-level)
    #    Also collect the error data in the loop to handle feedback later
    error_pos_values = list()
    error_vel_values = list()

    for (ts, cont_state) in controller_states_list:
        num_joints = len(cont_state.joint_names)
        actual_pos_list = cont_state.actual.positions
        actual_vel_list = cont_state.actual.velocities

        # aggregate abs() of pos and vel errors for feedback
        error_pos_list = cont_state.error.positions
        error_vel_list = cont_state.error.velocities
        error_pos_values.extend([abs(pos) for pos in (error_pos_list)])
        error_vel_values.extend([abs(vel) for vel in (error_vel_list)])

        # check joint consistency
        if len(actual_pos_list) != num_joints:
            errs.append(f"{ts} num_joints mismatch: {num_joints} vs {len(actual_pos_list)}")

        if len(actual_vel_list) != num_joints:
            errs.append(f"{ts} num_joints mismatch: {num_joints} vs {len(actual_vel_list)}")

        # the same checks as above should be done on this topic too
        for pi, pos in enumerate(actual_pos_list):
            joint_name = cont_state.joint_names[pi]

            # existential constraints
            if math.isnan(pos):
                errs.append(f"{ts} {joint_name}'s position is NaN")

            elif math.isinf(pos):
                errs.append(f"{ts} {joint_name}'s position is INF")

            else:
                pos_deg = math.degrees(pos)

                check_pos = False
                # specification constraints
                if (joint_name == "panda_joint1" or
                    joint_name == "panda_joint3" or
                    joint_name == "panda_joint5" or
                    joint_name == "panda_joint7"):
                    min_deg = -166 - MARGIN
                    max_deg = 166 + MARGIN
                    check_pos = True

                elif joint_name == "panda_joint2":
                    min_deg = -101 - MARGIN
                    max_deg = 101 + MARGIN
                    check_pos = True

                elif joint_name == "panda_joint4":
                    min_deg = -176 - MARGIN
                    max_deg = -4 + MARGIN
                    check_pos = True

                elif joint_name == "panda_joint6":
                    min_deg = -1 - MARGIN
                    max_deg = 215 + MARGIN
                    check_pos = True

                if check_pos:
                    if pos_deg < min_deg or pos_deg > max_deg:
                        errs.append(f"{ts} {joint_name}'s position {pos_deg} is not within {min_deg} ~ {max_deg}")

        for vi, vel in enumerate(actual_vel_list):
            joint_name = cont_state.joint_names[vi]

            # existential constraints
            if math.isnan(vel):
                errs.append(f"{ts} {joint_name}'s velocity is NaN")

            elif math.isinf(vel):
                errs.append(f"{ts} {joint_name}'s velocity is INF")

            else:
                vel_deg = math.degrees(vel) # rad/s to deg/s

                check_vel = False
                if (joint_name == "panda_joint1" or
                    joint_name == "panda_joint2" or
                    joint_name == "panda_joint3" or
                    joint_name == "panda_joint4"):
                    min_vel = -150
                    max_vel = 150
                    check_vel = True

                elif (joint_name == "panda_joint5" or
                    joint_name == "panda_joint6" or
                    joint_name == "panda_joint7"):
                    min_vel = -180
                    max_vel = 180
                    check_vel = True

                if check_vel:
                    if vel_deg < min_vel or vel_deg > max_vel:
                        errs.append(f"{ts} {joint_name}'s velocity {vel_deg} is not within {min_vel} ~ {max_vel}")

    # 3. executed action states
    # status code: 2 executing 4 succeeded 6 aborted
    action_status = list()
    for (ts, action) in move_action_status_list:
        action_status.append(action.status_list[0].status)

    print(action_status)
    if len(action_status) == 2:
        if action_status[0] != 2:
            errs.append(f"action doesn't start with 2: {action_status[0]}")

        if action_status[-1] != 4 and action_status[-1] != 6:
            errs.append(f"action doesn't end with 4 or 6: {action_status[-1]}")

    else:
        errs.append(f"invalid goal action status: {str(action_status)}")

    # 4. check if only one motion_plan_request was received
    num_motion_plan_request = len(motion_plan_request_list)
    if num_motion_plan_request != 1:
        errs.append(f"# Motion plan request != 1: ({num_motion_plan_request})")

    else:
        # retrieve the requested goal
        (ts, mpr) = motion_plan_request_list[0]
        goal_constraints = mpr.goal_constraints[0]
        goal_position = goal_constraints.position_constraints[0].constraint_region.primitive_poses[0].position
        goal_orientation = goal_constraints.orientation_constraints[0].orientation

        goal_x = goal_position.x
        goal_y = goal_position.y
        goal_z = goal_position.z
        goal_w = goal_orientation.w

        # 5. check endpoint w.r.t. action status
        joint_angle_map = dict()
        for i, name in enumerate(final_joint_state.name):
            joint_angle_map[name] = final_joint_state.position[i]

        fwd_kinematics_sol = chain.forward_kinematics(joint_angle_map)
        final_end_effector_pos = fwd_kinematics_sol["panda_hand"].pos
        final_end_effector_rot = fwd_kinematics_sol["panda_hand"].rot

        final_pos_x = final_end_effector_pos[0]
        final_pos_y = final_end_effector_pos[1]
        final_pos_z = final_end_effector_pos[2]

        if len(action_status) == 2 and action_status[-1] == 4:
            # action succeeded - endpoint should be at the goal
            # 1) run forward kinematics with joint positions to find
            #    the end-effector position & orientation
            # 2) compare with goal position & orientation

            # pose repeatability < 0.1 mm (ISO 9283)

            dist_goal_to_final_pos = math.sqrt(
                pow(goal_x - final_pos_x, 2)
                + pow(goal_y - final_pos_y, 2)
                + pow(goal_z - final_pos_z, 2)
            )

            for feedback in feedback_list:
                if feedback.name == "end_point_deviation":
                    feedback.update_value(dist_goal_to_final_pos)

                elif feedback.name == "mean_joint_pos_error":
                    if len(error_pos_values) > 0:
                        feedback.update_value(
                            statistics.mean(error_pos_values)
                        )

                elif feedback.name == "max_joint_pos_error":
                    if len(error_pos_values) > 0:
                        feedback.update_value(
                            max(error_pos_values)
                        )

                elif feedback.name == "mean_joint_vel_error":
                    if len(error_vel_values) > 0:
                        feedback.update_value(
                            statistics.mean(error_vel_values)
                        )

                elif feedback.name == "max_joint_vel_error":
                    if len(error_vel_values) > 0:
                        feedback.update_value(
                            max(error_vel_values)
                        )


            print(f"D: {dist_goal_to_final_pos:.6f}")
            if dist_goal_to_final_pos > 0.001: # unit: m
                errs.append(f"goal and actual pos deviation too high: {dist_goal_to_final_pos}")

        elif len(action_status) == 2 and action_status[-1] == 6:
            # action aborted - endpoint should be at the initial pos
            # assure no joint has moved (thereby no endpoint movement)

            # [extra] check if an inverse kinematics solution exists.
            # If so, (and if the joint angles are within the constraints),
            # the controller should've found the solution and moved the
            # manipulator.

            """
            0.30689056659294117 -5.2219761264512675e-12 0.5902820523028391
            """
            ready_pos_x = 0.306890566
            ready_pos_y = 0
            ready_pos_z = 0.590282052

            dist_ready_to_final_pos = math.sqrt(
                pow(ready_pos_x - final_pos_x, 2)
                + pow(ready_pos_y - final_pos_y, 2)
                + pow(ready_pos_z - final_pos_z, 2)
            )

            print(f"D: {dist_ready_to_final_pos:.6f}")
            if dist_ready_to_final_pos > 0.001: # unit: m
                errs.append(f"robot shouldn't have moved: {dist_ready_to_final_pos}")

            tf_goal = kinpy.Transform()
            tf_goal.rot[0] = 0.0
            tf_goal.rot[1] = 0.0
            tf_goal.rot[2] = 0.0
            tf_goal.rot[3] = goal_w
            tf_goal.pos[0] = goal_x
            tf_goal.pos[1] = goal_y
            tf_goal.pos[2] = goal_z

            try:
                ik = serial_chain.inverse_kinematics(tf_goal)
            except:
                return errs

            joints = serial_chain.get_joint_parameter_names()
            valid_cnt = 0

            for i, joint_angle in enumerate(ik):
                joint_name = joints[i]
                angle = math.degrees(joint_angle)

                if (joint_name == "panda_joint1" or
                    joint_name == "panda_joint3" or
                    joint_name == "panda_joint5" or
                    joint_name == "panda_joint7"):

                    if (-166.0 <= angle) and (angle <= 166.0):
                        valid_cnt += 1

                elif joint_name == "panda_joint2":
                    if (-101.0 <= angle) and (angle <= 101.0):
                        valid_cnt += 1

                elif joint_name == "panda_joint4":
                    if (-176.0 <= angle) and (angle <= -4.0):
                        valid_cnt += 1

                elif joint_name == "panda_joint6":
                    if (-1.0 <= angle) and (angle <= 215.0):
                        valid_cnt += 1

            if valid_cnt == len(joints):
                errs.append(f"controller failed to find inverse kinematics solution: {ik}")

    return errs

