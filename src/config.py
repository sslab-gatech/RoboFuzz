import enum
import subprocess as sp
import os
from enum import Enum, auto

class TestMode(Enum):
    GENERIC = auto()
    SROS2 = auto()
    PX4_SITL = auto()
    PX4_MISSION = auto()


class RuntimeConfig:

    def __init__(self):
        config_path = os.path.abspath(__file__)
        self.src_dir = os.path.dirname(config_path)
        self.proj_root = os.path.dirname(self.src_dir)
        self.gcov_dir = os.path.join(self.src_dir, "coverage")

        self.replay = False

        self.persistent = False
        self.rospkg = None
        self.rosnode = None
        self.exec_cmd = None

        self.px4_sitl = False
        self.tb3_sitl = False
        self.tb3_hitl = False
        self.sros2 = False
        self.test_rcl = False

    def find_package_metadata(self):
        # read from provided meta file
        # or find from ros installation

        ros_pkg_cmd = "ros2 pkg prefix {}".format(self.rospkg)

        if self.rospkg is None or self.rosnode is None:
            print("[warning] ros_pkg or ros_node not provided.")
            print("          Using exec_cmd instead.")
            return -1

        proc = sp.Popen(ros_pkg_cmd.split(" "), stdout=sp.PIPE)
        out = proc.stdout.read().strip()
        if len(out) == 0:
            return -1
        self.pkg_prefix = str(out, "utf-8")

        try:
            pkg_xml = os.readlink(os.path.join(self.pkg_prefix, "share",
                self.rospkg, "package.xml"))
        except OSError:
            # if not built with --symlink-install
            pkg_xml = os.path.join(self.pkg_prefix, "share", self.rospkg,
                    "package.xml")
        self.pkg_src_dir = os.path.join(os.path.dirname(pkg_xml), "src")

        self.ros_prefix = os.path.dirname(os.path.dirname(self.pkg_prefix))
        self.pkg_cov_dir = os.path.join(self.ros_prefix, "build", self.rospkg,
                "CMakeFiles", "{}.dir".format(self.rosnode), "src")

        self.node_executable = os.path.join(self.pkg_prefix, "lib",
                self.rospkg, self.rosnode)

        return 0
