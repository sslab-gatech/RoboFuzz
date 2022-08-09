"""
ROS2 Fuzzer CLI interface main module.

:authors: Alias Robotics S.L. Borja Erice, Odei Olalde, Xabi Perez, Gorka Olalde
"""
import logging
from argparse import ArgumentParser
from hypothesis import given, settings, Verbosity, HealthCheck
from ros2_fuzzer.ros_commons import ros_interface_loader_str, map_ros_types, ROS2NodeFuzzer


def fuzz(interface_type, name, ros2_type):
    if interface_type == 'message':
        fuzz_message_wrapper(ros2_type, name)
    elif interface_type == 'service':
        fuzz_service_wrapper(ros2_type, name)
    else:
        raise ValueError(f'Interface type: {interface_type} not supported.')


def fuzz_message_wrapper(msg_type, topic_name):
    """
    Wrapper for the :func:`fuzz_message` method
    :param msg_type: The ROS2 message type
    :param topic_name: The name of the topic to be fuzzed
    """
    @settings(max_examples=100, verbosity=Verbosity.verbose)
    @given(msg=map_ros_types(msg_type))
    def fuzz_message(msg):
        fuzzer.publish(msg_type, msg, topic_name)

    with ROS2NodeFuzzer() as fuzzer:
        fuzz_message()


def fuzz_service_wrapper(srv_type, srv_name):
    """
    Wrapper for the :func:`fuzz_service` method
    :param srv_type: The ROS2 service type
    :param srv_name: The name of the service to be fuzzed
    """
    @settings(max_examples=100, verbosity=Verbosity.verbose, suppress_health_check=[HealthCheck.too_slow])
    @given(srv_request=map_ros_types(srv_type.Request))
    def fuzz_service(srv_request):
        fuzzer.send_request(srv_type, srv_name, srv_request)

    with ROS2NodeFuzzer() as fuzzer:
        fuzz_service()


def main():
    """
    Main method. Takes the ros2_interface, ros2_interface_type and name of the topic or service to be fuzzed
    as command line arguments and launches the function :func:`fuzz`
    """
    logging.basicConfig()
    logger = logging.getLogger(__name__)
    parser = ArgumentParser(description='ROS2 Fuzzer')
    subparsers = parser.add_subparsers(dest='interface_type', help='ROS2 interface type')
    subparsers.required = True

    message_parser = subparsers.add_parser('message')
    message_parser.add_argument('ros2_type', help='ROS2 message type')
    message_parser.add_argument('name', help='The name of the topic to be fuzzed')

    service_parser = subparsers.add_parser('service')
    service_parser.add_argument('ros2_type', help='ROS2 service type')
    service_parser.add_argument('name', help='The name of the service to be fuzzed')

    args = parser.parse_args()

    try:
        ros2_type = ros_interface_loader_str(args.ros2_type, args.interface_type)
        fuzz(args.interface_type, args.name, ros2_type)
    except Exception as e:
        logger.critical('Exception occurred during execution --> ' + str(e))


if __name__ == '__main__':
    main()








