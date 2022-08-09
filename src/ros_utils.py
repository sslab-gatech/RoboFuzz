from collections import defaultdict
import importlib

from ament_index_python import get_resource, get_resources, has_resource


def get_all_message_types():
    all_message_types = {}
    for package_name in get_resources("rosidl_interfaces"):
        message_types = get_message_types(package_name)
        if message_types:
            all_message_types[package_name] = message_types
    return all_message_types


def get_message_types(package_name):
    if not has_resource("packages", package_name):
        raise LookupError("Unknown package name")
    try:
        content, _ = get_resource("rosidl_interfaces", package_name)
    except LookupError:
        return []
    interface_names = content.splitlines()
    # TODO(dirk-thomas) this logic should come from a rosidl related package
    # Only return messages in msg folder
    return list(
        sorted(
            {
                n[4:-4]
                for n in interface_names
                if n.startswith("msg/") and n[-4:] in (".idl", ".msg")
            }
        )
    )


def get_all_service_types():
    all_service_types = {}
    for package_name in get_resources("rosidl_interfaces"):
        service_types = get_service_types(package_name)
        if service_types:
            all_service_types[package_name] = service_types
    return all_service_types


def get_service_types(package_name):
    if not has_resource("packages", package_name):
        raise LookupError("Unknown package name")
    try:
        content, _ = get_resource("rosidl_interfaces", package_name)
    except LookupError:
        return []
    interface_names = content.splitlines()
    # TODO(dirk-thomas) this logic should come from a rosidl related package
    # Only return services in srv folder
    return list(
        sorted(
            {
                n[4:-4]
                for n in interface_names
                if n.startswith("srv/") and n[-4:] in (".idl", ".srv")
            }
        )
    )


def get_publishers(node):
    publishers = defaultdict(list)

    for name, namespace in node.get_node_names_and_namespaces():
        if name == "_fuzzer":
            continue

        if namespace.endswith("/"):
            node_name = namespace + name
        else:
            node_name = namespace + "/" + name

        for (
            topic_name,
            topic_type,
        ) in node.get_publisher_names_and_types_by_node(name, namespace):
            publishers[topic_name].append(node_name)

    return publishers


def get_subscriptions(node):
    subscriptions = defaultdict(list)
    # {node_name: [(topic_name, topic_type), (topic_name, topic_type)]}

    for name, namespace in node.get_node_names_and_namespaces():
        if name == "_fuzzer":
            continue

        if namespace.endswith("/"):
            node_name = namespace + name
        else:
            node_name = namespace + "/" + name

        for (
            topic_name,
            topic_type,
        ) in node.get_subscriber_names_and_types_by_node(name, namespace):
            # print(node_name, "subs to", topic_name, topic_type)
            subscriptions[node_name].append((topic_name, topic_type))

    return subscriptions


def get_secure_subscriptions(node):
    subscriptions = defaultdict(list)
    # {node_name: [(topic_name, topic_type), (topic_name, topic_type)]}

    for (
        name,
        namespace,
        enclave,
    ) in node.get_node_names_and_namespaces_with_enclaves():
        if name == "_fuzzer":
            continue

        if namespace.endswith("/"):
            node_name = namespace + name
        else:
            node_name = namespace + "/" + name

        for (
            topic_name,
            topic_type,
        ) in node.get_subscriber_names_and_types_by_node(name, namespace):
            # print(node_name, "subs to", topic_name, topic_type)
            subscriptions[node_name].append((topic_name, topic_type))

    return subscriptions


def get_services(node):
    services = defaultdict(list)

    for name, namespace in node.get_node_names_and_namespaces():
        if name == "_fuzzer":
            continue

        if namespace.endswith("/"):
            node_name = namespace + name
        else:
            node_name = namespace + "/" + name

        for (
            service_name,
            service_type,
        ) in node.get_service_names_and_types_by_node(name, namespace):
            services[node_name].append((service_name, service_type))

    return services


def find_custom_msg(msg_type):
    return None


def get_msg_class_from_name(msg_pkg, msg_name):
    # e.g., get_msg_class_from_name("geometry_msgs", "Vector3")
    msg_type_class = None
    try:
        module = importlib.import_module(msg_pkg + ".msg")
        msg_type_class = module.__dict__[msg_name]
    except KeyError:
        print("[-] {} not in pkg {}".format(msg_name, msg_pkg))
    except ImportError:
        print("[-] pkg {} doesn't exist.".format(msg_pkg))
    except TypeError:
        print("[-] {} doesn't exist".format(msg_name))

    return msg_type_class


def get_msg_typename_from_class(msg_type_class):
    module = msg_type_class.__dict__["__module__"].split(".")[0]

    # mtc is a metaclass, and its instance is class
    class_name = msg_type_class().__class__.__name__

    typename = f"{module}/{class_name}"

    return typename


def flatten_nested_dict(indict, pre=None):
    pre = pre[:] if pre else []
    if isinstance(indict, dict):
        for key, value in indict.items():
            if isinstance(value, dict):
                for d in flatten_nested_dict(value, pre + [key]):
                    yield d
            elif isinstance(value, list) or isinstance(value, tuple):
                for v in value:
                    for d in flatten_nested_dict(v, pre + [key]):
                        yield d
            else:
                yield pre + [key, value]
    else:
        yield pre + [indict]
