def inspect_target(fuzzer):
    fuzz_targets = []

    built_in_msg_types = ros_utils.get_all_message_types()
    subscriptions = ros_utils.get_subscriptions(fuzzer.node_ptr)

    if fuzzer.config.px4_sitl:
        if fuzzer.config.use_mavlink:
            topic_name = "/dummy_mavlink_topic"
            msg_type_class = ros_utils.get_msg_class_from_name(
                "px4_msgs", "ManualControlSetpoint"
            )
        else:
            topic_name = "/TrajectorySetpoint_PubSubTopic"
            msg_type_class = ros_utils.get_msg_class_from_name(
                "px4_msgs", "TrajectorySetpoint"
            )

        fuzz_targets.append([topic_name, msg_type_class, "drone"])
        return fuzz_targets

    elif fuzzer.config.tb3_sitl:
        topic_name = "/cmd_vel"
        msg_type_class = ros_utils.get_msg_class_from_name(
            "geometry_msgs", "Twist"
        )
        fuzz_targets.append(
            [topic_name, msg_type_class, "/turtlebot3_diff_drive"]
        )

        return fuzz_targets

    elif fuzzer.config.tb3_hitl:
        topic_name = "/cmd_vel"
        msg_type_class = ros_utils.get_msg_class_from_name(
            "geometry_msgs", "Twist"
        )
        fuzz_targets.append([topic_name, msg_type_class, "/turtlebot3_node"])

        return fuzz_targets

    elif fuzzer.config.test_rosidl:
        topic_name = "/metatopic" # fake topic (topic tbd by mutator)
        msg_type_class = ros_utils.get_msg_class_from_name(
            "std_msgs", "Empty"
        )
        fuzz_targets.append([topic_name, msg_type_class, "/rosidl_node"])

        return fuzz_targets

    elif fuzzer.config.test_moveit:
        # use below if testing without commander harness
        topic_name = "/motion_plan_request"
        msg_type_class = ros_utils.get_msg_class_from_name(
            "moveit_msgs", "MotionPlanRequest"
        )
        # for now, use commander harness for convenience
        topic_name = "/metatopic"
        msg_type_class = ros_utils.get_msg_class_from_name(
            "geometry_msgs", "Pose"
        )

        fuzz_targets.append([topic_name, msg_type_class, "/moveit_node"])

        return fuzz_targets

    for subscriber_node in subscriptions:
        print()
        print("[+] processing", subscriber_node)

        if subscriber_node.replace("/", "").startswith("_"):
            print("[-] skip internal node")
            continue

        for ti, topic in enumerate(subscriptions[subscriber_node]):
            topic_name = topic[0]
            msg_type_full = topic[1]

            print("[{}] {} {}".format(ti, topic_name, msg_type_full))

            if len(msg_type_full) > 1:
                print("[check] MULTIPLE MESSAGE TYPES!")

            msg_type = msg_type_full[0]
            msg_pkg = msg_type.split("/")[0]
            msg_name = msg_type.split("/")[-1]

            if msg_name == "ParameterEvent":
                print("[-] skip ParameterEvents")
                continue

            if msg_pkg in built_in_msg_types.keys():
                msg_type_class = ros_utils.get_msg_class_from_name(
                    msg_pkg, msg_name
                )
            else:
                msg_type_class = ros_utils.find_custom_msg(msg_type)

            if msg_type_class is None:
                print("[-] couldn't find msg class")
                continue

            print()
            print("Found fuzzing topic", topic_name, "of type", msg_type_class)
            print("- target node:", subscriber_node)
            fuzz_targets.append([topic_name, msg_type_class, subscriber_node])

    return fuzz_targets


def inspect_secure_target(fuzzer):
    fuzz_targets = []

    built_in_msg_types = ros_utils.get_all_message_types()
    subscriptions = ros_utils.get_secure_subscriptions(fuzzer.node_ptr)

    for subscriber_node in subscriptions:
        print()
        print("[+] processing", subscriber_node)

        if subscriber_node.replace("/", "").startswith("_"):
            print("[-] skip internal node")
            continue

        for ti, topic in enumerate(subscriptions[subscriber_node]):
            topic_name = topic[0]
            msg_type_full = topic[1]

            print("[{}] {} {}".format(ti, topic_name, msg_type_full))

            if len(msg_type_full) > 1:
                print("[check] MULTIPLE MESSAGE TYPES!")

            msg_type = msg_type_full[0]
            msg_pkg = msg_type.split("/")[0]
            msg_name = msg_type.split("/")[-1]

            if msg_name == "ParameterEvent":
                print("[-] skip ParameterEvents")
                continue

            if msg_pkg in built_in_msg_types.keys():
                msg_type_class = ros_utils.get_msg_class_from_name(
                    msg_pkg, msg_name
                )
            else:
                msg_type_class = ros_utils.find_custom_msg(msg_type)

            if msg_type_class is None:
                print("[-] couldn't find msg class")
                continue

            print()
            print("Found fuzzing topic", topic_name, "of type", msg_type_class)
            print("- target node:", subscriber_node)
            fuzz_targets.append([topic_name, msg_type_class, subscriber_node])

    fuzzer.kill_target()
    return fuzz_targets

def inspect_service(fuzzer):
    services = ros_utils.get_services(fuzzer.node_ptr)
    built_in_svc_types = ros_utils.get_all_service_types()

    target_services = []
    for tested_node in services:
        print()
        print("[+] processing {}'s services".format(tested_node))
        for si, srv in enumerate(services[tested_node]):
            srv_name = srv[0]
            srv_type_full = srv[1]
            print("[{}] {} {}".format(si, srv_name, srv_type_full))

            if len(srv_type_full) > 1:
                print("[check] MULTIPLE SERVICE TYPES!")

            if "parameter" in srv_name:
                print("[-] skip param servers")
                continue

            srv_type_full = srv_type_full[0]
            ss = srv_type_full.split("/")
            if len(ss) == 2:
                ss = [ss[0], "srv", ss[1]]
            srv_pkg = ss[0]
            srv_type = ss[-1]
            if srv_pkg == "action_msgs":
                print("[-] skip action messages")
                continue

            if ss[1] != "srv":
                print("[-] skip non-svc services")
                continue

            if srv_pkg not in built_in_svc_types.keys():
                print("service package not found")
                continue

            srv_module_name = ".".join(ss[:-1])
            try:
                module = importlib.import_module(srv_module_name)
                srv_module = getattr(module, srv_type)
                if not srv_pkg or not srv_module:
                    raise ValueError()
            except ValueError:
                raise RuntimeError("service type is invalid")

            target_services.append([srv_name, srv_module, srv_type_full])

    return target_services
