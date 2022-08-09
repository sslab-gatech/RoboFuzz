def check(config, msg_list, state_dict, feedback_list):
    errs = list()

    msg = msg_list[0]
    msg_name = type(msg).__name__
    topic_name = f"/idltest_{msg_name}_out"

    if topic_name not in state_dict:
        errs.append(f"Topic {topic_name} is lost")

    else:
        # replayed by rclcpp target
        msg_out_list = state_dict[topic_name]

        if len(msg_out_list) != 1:
            print("[-] multiple messages replayed by idl target")
            exit(-1)

        (ts, msg_out) = msg_out_list[0]

        # print(msg)
        # print(msg_out)

        if "Array" not in msg_name:
            # == opertor exists for built-in types
            if msg != msg_out:
                errs.append("Sent and replayed messages do not match")
        else:
            # not for the Array types
            if len(msg.data) != len(msg_out.data):
                errs.append("Sent and replayed array lengths do not match")

            else:
                for i in range(len(msg.data)):
                    if msg.data[i] != msg_out.data[i]:
                        errs.append("Sent and replayed messages do not match")
                        break

    return errs

