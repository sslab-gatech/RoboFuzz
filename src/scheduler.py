import os
import time
import random
from functools import reduce
from copy import deepcopy
from enum import Enum, auto

import numpy as np

import constants as c
import mutator
import harness
import ros_utils
from ros2_fuzzer import ros_commons
import px4_utils
from px4_prep.blacklist import blacklist as param_blacklist
from px4_prep.blacklist import tested as param_tested


class Campaign(Enum):
    RND_SINGLE = auto()
    RND_SEQUENCE = auto()
    RND_REPEATED = auto()
    INTERCEPTION = auto()
    SROS_AUTH = auto()
    DDS_CROSSCHK = auto()
    IDL_CHECK = auto()


class Scheduler:
    def __init__(self, fuzzer, campaign, target):
        self.fuzzer = fuzzer
        self.campaign = campaign
        self.topic_name = target[0]
        self.msg_type_class = target[1]
        self.subscriber_node = target[2]
        self.default_msg = self.msg_type_class()

        msg_type_dict = ros_commons.map_ros_types(self.msg_type_class)
        self.msg_field_list = list(
            ros_utils.flatten_nested_dict(msg_type_dict)
        )

    def filter_field_list(self, whitelist, blacklist):
        if whitelist is not None:
            self.msg_field_list = whitelist
        elif blacklist is not None:
            for field in blacklist:
                self.msg_field_list.remove(field)

    def init_schedule(self):
        if (
            self.campaign == Campaign.RND_SINGLE
            or self.campaign == Campaign.RND_REPEATED
        ):
            self.num_fields = len(self.msg_field_list)
            self.cur_fm_field = 0
            self.fm_field_stages = [0] * self.num_fields
            self.fm_determ_stages = [0] * self.num_fields

            # determine havoc stages
            self.fm_rand_stages = []
            for i in range(self.num_fields):
                self.fm_rand_stages.append(random.randint(1, 2))  # XXX: WHY?

            self.fm_odata = []

            self.num_msg_mutation = 0

            # Rounds
            # A. Field-by-field mutation
            #   0: pop from queue / generate if empty
            #   1: deterministic stages
            #   2: random mutation (havoc)
            # B. All fields mutation
            #   0: select random field & mut_op

            self.bit_pos = 0
            self.arith_val = -35
            self.interesting_idx = 0

            # pre-select fields to mutate for all-fields mutation
            self.fields_to_mutate = random.sample(
                self.msg_field_list, random.randint(1, self.num_fields)
            )

            self.cycle_cnt = 0
            self.is_new_cycle = True
            self.round_cnt = 0
            self.from_queue = False
            self.exec_cnt = 0

        elif self.campaign == Campaign.RND_SEQUENCE:
            seqlen = self.fuzzer.config.seqlen
            self.num_msgs = seqlen
            # self.num_msgs = random.randint(2, seqlen)
            self.num_fields = len(self.msg_field_list)
            self.cycle_cnt = 0
            self.is_new_cycle = True
            self.round_cnt = 0
            self.from_queue = False

        elif self.campaign == Campaign.IDL_CHECK:
            self.cycle_cnt = 0
            self.is_new_cycle = True
            self.round_cnt = 0
            self.from_queue = False
            self.init_builtin_type_name = None
            self.full_type_name = None
            self.current_type = None
            self.default_val = None

    def random_stage(self):
        print("STAGE: RANDOM")

        error = None
        expecting = False
        if "Array" in self.full_type_name:
            try:
                idx = random.randint(0, len(self.cur_msg.data) - 1)
                self.cur_msg.data[idx] = mutator.get_rand_val(
                        self.full_type_name)
            except Exception as e:
                error = str(e)

        else:
            try:
                self.cur_msg.data = mutator.get_rand_val(
                        self.full_type_name)
            except Exception as e:
                error = str(e)

        return (error, expecting)

    def mutate_moveit_joint(self, config):
        frame = str(time.time())

        meta_file = os.path.join(config.meta_dir, "meta-{}".format(frame))
        with open(meta_file, "w") as fp:
            fp.write(
                self.subscriber_node
                + "\t"
                + self.topic_name
                + "\t"
                + str(self.msg_type_class)
                + "\t"
                + str(self.cycle_cnt)
                + "\t"
                + str(self.round_cnt)
            )

        print(
            "\n\x1b[92mCYCLE:",
            self.cycle_cnt,
            "ROUND:",
            self.round_cnt,
            "\x1b[0m",
            frame,
        )

        if self.is_new_cycle:
            try:
                # list contains seven joint constraints
                self.msg_list = self.fuzzer.queue.popleft()
            except IndexError:
                self.msg_list = harness.get_init_joint_constraints()

            self.is_new_cycle = False

        else:
            joint_idx = random.randint(0, len(self.msg_list) - 1)
            field_name = "position" # only fuzz goal position for the time being
            dtype = np.dtype("float64")

            self.msg_list[joint_idx].position = mutator.gen_rand_data(
                dtype, False)

        return (self.msg_list, frame)

    def mutate_moveit_goal(self, config):
        frame = str(time.time())

        meta_file = os.path.join(config.meta_dir, "meta-{}".format(frame))
        with open(meta_file, "w") as fp:
            fp.write(
                self.subscriber_node
                + "\t"
                + self.topic_name
                + "\t"
                + str(self.msg_type_class)
                + "\t"
                + str(self.cycle_cnt)
                + "\t"
                + str(self.round_cnt)
            )

        print(
            "\n\x1b[92mCYCLE:",
            self.cycle_cnt,
            "ROUND:",
            self.round_cnt,
            "\x1b[0m",
            frame,
        )

        if self.is_new_cycle:
            try:
                self.msg = self.fuzzer.queue.popleft()
            except IndexError:
                self.msg = harness.get_init_moveit_pose()

            self.is_new_cycle = False

        else:
            # It really doesn't make sense to use the entire double range
            # when the manipulator workspace is limited to a small region:
            # x: -855 ~ 855, y: -855 ~ 855, z: -360 ~ 1190
            #
            # We don't have to stick to the exact constraints that precisely
            # determine the workspace boundary - let the fuzzer generate the
            # end effector goals that sometimes exceed the workspace boundary
            # with a small margin.
            #
            # In the meantime, special values (INF, NaN) should be tested.

            # dtype = np.dtype("float64")
            # self.msg.position.x = mutator.gen_rand_data(dtype, False)
            # self.msg.position.y = mutator.gen_rand_data(dtype, False)
            # self.msg.position.z = mutator.gen_rand_data(dtype, False)
            # self.msg.orientation.w = mutator.gen_rand_data(dtype, False)

            self.msg = deepcopy(self.last_msg)
            prob_special = 5
            sel = np.random.randint(3)

            if sel == 0:
                if np.random.randint(0, 100) >= prob_special:
                    x_mm = mutator.gen_float_in_range(-900, 900, 4)
                else:
                    x_mm = mutator.gen_special_floats()

                self.msg.position.x = x_mm / 1000.0 # (unit: m)

            elif sel == 1:
                if np.random.randint(0, 100) >= prob_special:
                    y_mm = mutator.gen_float_in_range(-900, 900, 4)
                else:
                    y_mm = mutator.gen_special_floats()

                self.msg.position.y = y_mm / 1000.0

            elif sel == 2:
                if np.random.randint(0, 100) >= prob_special:
                    z_mm = mutator.gen_float_in_range(-400, 1300, 4)
                else:
                    z_mm = mutator.gen_special_floats()

                self.msg.position.z = z_mm / 1000.0

        self.round_cnt += 1

        # print(self.msg)
        self.last_msg = deepcopy(self.msg)
        return (self.msg, frame)

    def mutate_px4_param(self, config):
        frame = str(time.time())

        meta_file = os.path.join(config.meta_dir, "meta-{}".format(frame))
        with open(meta_file, "w") as fp:
            fp.write(
                self.subscriber_node
                + "\t"
                + self.topic_name
                + "\t"
                + str(self.msg_type_class)
                + "\t"
                + str(self.cycle_cnt)
                + "\t"
                + str(self.round_cnt)
            )

        print(
            "\n\x1b[92mCYCLE:",
            self.cycle_cnt,
            "ROUND:",
            self.round_cnt,
            "\x1b[0m",
            frame,
        )

        if self.is_new_cycle:
            try:
                self.msg = self.fuzzer.queue.popleft()
            except IndexError:
                self.msg = px4_utils.get_init_parameter_msg()

            # Test one parameter per cycle

            while True:
                # randomly select the parameter with an open range
                sel = np.random.randint(len(px4_utils.param_dict))

                param_name = list(px4_utils.param_dict.keys())[sel]
                if param_name in param_blacklist:
                    continue
                elif param_name in param_tested:
                    continue

                param_meta = px4_utils.param_dict[param_name]
                range_min = param_meta["min"]
                range_max = param_meta["max"]

                if range_min is None or range_max is None:
                    break

            self.msg.param_name = param_name
            self.msg.param_type = param_meta["type"]
            self.msg.value = param_meta["default"]
            self.msg.min = param_meta["min"]
            self.msg.max = param_meta["max"]

            with open("params.log", "a") as f:
                f.write(param_name + "\n")

            print(f"[scheduler] {param_name} ({range_min} ~ {range_max})")

            self.is_new_cycle = False

        else:
            # TODO
            # probabilistically generate out-of-valid-range data -> (X)
            # https://github.com/PX4/PX4-Autopilot/issues/16122#issuecomment-723567775
            #
            # Instead, focus on the parameters if the min or max is not
            # specified on the documentation.

            self.msg = deepcopy(self.last_msg)

            if self.msg.param_type == "INT32":
                if self.msg.min is not None:
                    low = self.msg.min
                else:
                    low = -1000

                if self.msg.max is not None:
                    high = self.msg.max
                else:
                    high = 1000

                self.msg.value = mutator.gen_int_in_range(low, high)

            elif self.msg.param_type == "FLOAT":
                if self.msg.min is not None:
                    low = self.msg.min
                else:
                    low = -1000

                if self.msg.max is not None:
                    high = self.msg.max
                else:
                    high = 1000

                self.msg.value = mutator.gen_float_in_range(low, high, 2)

        self.round_cnt += 1

        print(self.msg)
        self.last_msg = deepcopy(self.msg)
        return (self.msg, frame)

    def mutate_typemsg(self, config):
        frame = str(time.time())

        print(
            "\n\x1b[92mCYCLE:",
            self.cycle_cnt,
            "ROUND:",
            self.round_cnt,
            "\x1b[0m",
            frame,
        )

        error = None
        expecting = None

        if self.round_cnt == 0:
            print("STAGE: GEN")
            assert (self.is_new_cycle == True)
            (ros_type, ext, num_elem) = mutator.mutate_type()
            self.is_new_cycle = False
            self.init_builtin_type_name = ros_type.name

            self.default_val = mutator.get_default_val(ros_type.name)

            type_name = ros_type.name.lower().capitalize()
            if type_name.startswith("Uint") or type_name.startswith("Wstr"):
                type_name = type_name[0] + type_name[1:].capitalize()

            if ext == c.TypeExtension.BUILTIN:
                type_name = type_name
            elif ext == c.TypeExtension.FARRAY:
                type_name += "FixedArray"
            elif ext == c.TypeExtension.BARRAY:
                type_name += "BoundedDynArray"
            elif ext == c.TypeExtension.UBARRAY:
                type_name += "UnboundedDynArray"
            else:
                print("[-] unknown type")
                exit(-1)

            self.full_type_name = type_name
            msg_type_class = ros_utils.get_msg_class_from_name(
                    "idltest_msgs", type_name)

            assert (msg_type_class is not None)

            self.current_type = msg_type_class

            msg = self.current_type()

            # built-in types and fixed arrays are auto-initialized
            # dynamic arrays need to be initialized
            if "DynArray" in type_name:
                dyn_data = []

                for i in range(num_elem):
                    dyn_data.append(self.default_val)

                # print("init:")
                # print(dyn_data)

                msg.data = dyn_data

            self.cur_msg = msg
            # print(self.cur_msg)

        elif self.round_cnt == 1:
            print("STAGE: DETERM - incorrect type")
            print("  [+]", self.full_type_name)
            (ros_type, ext, num_elem) = mutator.random_builtin_type_except(
                    self.init_builtin_type_name)
            bad_data = mutator.get_default_val(ros_type.name)

            # should fail at any time
            expecting = f"Type mismatch - type: {self.full_type_name}, given data: {bad_data} ({ros_type.name})"
            if "Array" in self.full_type_name:
                original_data = self.cur_msg.data[:]
                try:
                    idx = random.randint(0, len(self.cur_msg.data) - 1)
                    self.cur_msg.data[idx] = bad_data
                except Exception as e:
                    # print("ERROR:", e)
                    error = str(e)
                finally:
                    # revert as we expect an error
                    self.cur_msg.data = original_data
            else:
                original_data = self.cur_msg.data
                try:
                    self.cur_msg.data = bad_data
                except Exception as e:
                    # print("ERROR:", e)
                    error = str(e)
                finally:
                    # revert as we expect an error
                    self.cur_msg.data = original_data

        elif self.round_cnt == 2:
            print("STAGE: DETERM - = lower bound")

            expecting = False
            if "Array" in self.full_type_name:
                try:
                    idx = random.randint(0, len(self.cur_msg.data) - 1)
                    self.cur_msg.data[idx] = mutator.get_bounds(
                            self.full_type_name, 0) # lower
                except Exception as e:
                    error = str(e)

            else:
                try:
                    self.cur_msg.data = mutator.get_bounds(
                            self.full_type_name, 0) # lower
                except Exception as e:
                    error = str(e)

        elif self.round_cnt == 3:
            print("STAGE: DETERM - = upper bound")

            expecting = False
            if "Array" in self.full_type_name:
                try:
                    idx = random.randint(0, len(self.cur_msg.data) - 1)
                    self.cur_msg.data[idx] = mutator.get_bounds(
                            self.full_type_name, 1) # upper
                except Exception as e:
                    error = str(e)

            else:
                try:
                    self.cur_msg.data = mutator.get_bounds(
                            self.full_type_name, 1) # upper
                except Exception as e:
                    error = str(e)

        # off-bound checks are only meaningful for numeric types
        # (except double; cannot get smaller than min)
        elif self.round_cnt == 4:
            if "Int" in self.full_type_name or "Float32" in self.full_type_name:
                print("STAGE: DETERM - < lower bound")

                bad_data = mutator.get_bounds(self.full_type_name, 0) - 1

                expecting = f"Bound error - {bad_data} is too small for {self.full_type_name}"
                if "Array" in self.full_type_name:
                    original_data = self.cur_msg.data[:]
                    try:
                        idx = random.randint(0, len(self.cur_msg.data) - 1)
                        self.cur_msg.data[idx] = bad_data
                    except Exception as e:
                        error = str(e)
                    finally:
                        # revert as we expect an error
                        self.cur_msg.data = original_data

                else:
                    original_data = self.cur_msg.data
                    try:
                        self.cur_msg.data = bad_data
                    except Exception as e:
                        error = str(e)
                    finally:
                        # revert as we expect an error
                        self.cur_msg.data = original_data

            else:
                (error, expecting) = self.random_stage()

        # off-bound checks are only meaningful for numeric types
        # (except double; cannot get bigger than max)
        elif self.round_cnt == 5:
            if "Int" in self.full_type_name or "Float32" in self.full_type_name:
                print("STAGE: DETERM - > upper bound")

                bad_data = mutator.get_bounds(self.full_type_name, 1) + 1

                expecting = f"Bound error - {bad_data} is too big for {self.full_type_name}"
                if "Array" in self.full_type_name:
                    original_data = self.cur_msg.data[:]
                    try:
                        idx = random.randint(0, len(self.cur_msg.data) - 1)
                        self.cur_msg.data[idx] = bad_data
                    except Exception as e:
                        error = str(e)
                    finally:
                        # revert as we expect an error
                        self.cur_msg.data = original_data

                else:
                    original_data = self.cur_msg.data
                    try:
                        self.cur_msg.data = bad_data
                    except Exception as e:
                        error = str(e)
                    finally:
                        # revert as we expect an error
                        self.cur_msg.data = original_data

            else:
                (error, expecting) = self.random_stage()

        elif self.round_cnt == 6:
            if "FixedArray" in self.full_type_name:
                print("STAGE: DETERM - fixed array # elements ++")
                expecting = f"Array size of {self.full_type_name} can't be increased"
                original_data = self.cur_msg.data[:]

                arr = list(self.cur_msg.data).append(self.default_val)

                try:
                    self.cur_msg.data = arr
                except Exception as e:
                    error = str(e)
                finally:
                    # revert as we expect an error
                    self.cur_msg.data = original_data

            elif "BoundedDynArray" in self.full_type_name:
                print("STAGE: DETERM - bounded array # elements ++")
                expecting = f"Array size of {self.full_type_name} can't be increased"
                original_data = self.cur_msg.data[:]

                arr = [self.default_val] * (c.BARRAY_BOUND_MAX + 1)

                try:
                    self.cur_msg.data = arr
                except Exception as e:
                    error = str(e)
                finally:
                    # revert as we expect an error
                    self.cur_msg.data = original_data

            else:
                (error, expecting) = self.random_stage()

        elif self.round_cnt == 7:
            if "FixedArray" in self.full_type_name:
                print("STAGE: DETERM - fixed array # elements --")

                expecting = f"Array size of {self.full_type_name} can't be decreased"
                original_data = self.cur_msg.data[:]

                arr = list(self.cur_msg.data)[:-1]
                try:
                    self.cur_msg.data = arr
                except Exception as e:
                    error = str(e)
                finally:
                    # revert as we expect an error
                    self.cur_msg.data = original_data

            else:
                (error, expecting) = self.random_stage()

        else:
            (error, expecting) = self.random_stage()


        # [Schedule]
        # 0. check default msg -> expecting no error
        # 1. out of [lower, upper] bounds value -> expecting an error
        # 2. (arrays) excessive number of elements -> expecting an error
        # 3. special values (if applicable) -> expecting no error ?
        # 4. random values within a valid range

        meta_file = os.path.join(config.meta_dir, "meta-{}".format(frame))
        with open(meta_file, "w") as fp:
            fp.write(
                self.subscriber_node
                + "\t"
                + self.topic_name
                + "\t"
                + str(self.current_type)
                + "\t"
                + str(self.cycle_cnt)
                + "\t"
                + str(self.round_cnt)
            )

        self.round_cnt += 1
        if self.round_cnt == 32:
            self.cycle_cnt += 1
            self.round_cnt = 0
            self.is_new_cycle = True

        return (self.cur_msg, frame, error, expecting)

    def mutate_sequence_mav(self, config):
        frame = str(time.time())

        meta_file = os.path.join(config.meta_dir, "meta-{}".format(frame))
        with open(meta_file, "w") as fp:
            fp.write(
                self.subscriber_node
                + "\t"
                + self.topic_name
                + "\t"
                + str(self.msg_type_class)
                + "\t"
                + str(self.cycle_cnt)
                + "\t"
                + str(self.round_cnt)
            )

        print(
            "\n\x1b[92mCYCLE:",
            self.cycle_cnt,
            "ROUND:",
            self.round_cnt,
            "\x1b[0m",
            frame,
        )
        print("QUEUE LEN:", len(self.fuzzer.queue))

        if self.is_new_cycle:
            try:
                self.msg_list = self.fuzzer.queue.popleft()
                self.from_queue = True
                self.num_msgs = len(self.msg_list)
            except IndexError:
                self.msg_list = []

                for i in range(self.num_msgs):
                    msg = self.msg_type_class()
                    self.msg_list.append(msg)

                # print(len(self.msg_list))
                # print("INIT_MSG_LIST:", self.msg_list)

                self.from_queue = False

            self.is_new_cycle = False

            if self.from_queue:
                print("skip GEN (from queue)")

            else:
                print("Generate initial msg list")

                for msg_idx in range(self.num_msgs):
                    msg = self.msg_list[msg_idx]

                    for field_idx in range(self.num_fields):
                        field = self.msg_field_list[field_idx]
                        dtype = field[-1]
                        attr_list = field[:-1]
                        attr_leaf = attr_list[-1]

                        if field[0] == "z":
                            data_val = mutator.gen_int_in_range(
                                0, 1000
                            ) / 1000
                        else:
                            data_val = mutator.gen_int_in_range(
                                -1000, 1000
                            ) / 1000
                        obj = reduce(getattr, attr_list[:-1], msg)
                        setattr(obj, attr_leaf, data_val)

                # for msg in self.msg_list:
                    # print(msg)

        else:
            msg_idx = random.choice(range(self.num_msgs))
            msg_to_mutate = self.msg_list[msg_idx]


            field = random.choice(self.msg_field_list)
            dtype = field[-1]
            attr_list = field[:-1]
            attr_leaf = attr_list[-1]

            print(f"mutate {field} of {msg_idx}-th message from the sequence")

            data_val = reduce(
                getattr, attr_list, msg_to_mutate
            )  # original data

            print("data before rand mutation:", data_val)

            if field[0] == "z":
                data_val = mutator.gen_int_in_range(0, 1000) / 1000
            else:
                data_val = mutator.gen_int_in_range(-1000, 1000) / 1000
            print("data after rand mutation:", data_val)

            msg_mutated = None
            if data_val is not None:
                msg_mutated = deepcopy(msg_to_mutate)
                obj = reduce(getattr, attr_list[:-1], msg_mutated)
                setattr(obj, attr_leaf, data_val)
                # fuzzer.fuzz_and_check(self.fuzzer, nmsg, frame)

                self.msg_list[msg_idx] = msg_mutated

        self.round_cnt += 1
        return (self.msg_list, frame)

    def mutate_sequence(self, config):
        frame = str(time.time())

        meta_file = os.path.join(config.meta_dir, "meta-{}".format(frame))
        with open(meta_file, "w") as fp:
            fp.write(
                self.subscriber_node
                + "\t"
                + self.topic_name
                + "\t"
                + str(self.msg_type_class)
                + "\t"
                + str(self.cycle_cnt)
                + "\t"
                + str(self.round_cnt)
            )

        print(
            "\n\x1b[92mCYCLE:",
            self.cycle_cnt,
            "ROUND:",
            self.round_cnt,
            "\x1b[0m",
            frame,
        )

        if self.is_new_cycle:
            try:
                self.msg_list = self.fuzzer.queue.popleft()
                self.from_queue = True
                self.num_msgs = len(self.msg_list)
            except IndexError:
                self.msg_list = []

                for i in range(self.num_msgs):
                    msg = self.msg_type_class()
                    self.msg_list.append(msg)

                # print(len(self.msg_list))
                # print("INIT_MSG_LIST:", self.msg_list)

                self.from_queue = False

            self.is_new_cycle = False

            if self.from_queue:
                print("skip GEN (from queue)")

            else:
                print("Generate initial msg list")

                for msg_idx in range(self.num_msgs):
                    msg = self.msg_list[msg_idx]

                    for field_idx in range(self.num_fields):
                        field = self.msg_field_list[field_idx]
                        dtype = field[-1]
                        attr_list = field[:-1]
                        attr_leaf = attr_list[-1]

                        data_val = mutator.gen_rand_data(dtype, False)
                        obj = reduce(getattr, attr_list[:-1], msg)
                        setattr(obj, attr_leaf, data_val)

                # for msg in self.msg_list:
                    # print(msg)

        else:
            msg_idx = random.choice(range(self.num_msgs))
            msg_to_mutate = self.msg_list[msg_idx]

            print(f"mutate {msg_idx}-th message from the sequence")

            field = random.choice(self.msg_field_list)
            dtype = field[-1]
            attr_list = field[:-1]
            attr_leaf = attr_list[-1]

            data_val = reduce(
                getattr, attr_list, msg_to_mutate
            )  # original data

            rand_mutation_stage = random.choice(
                mutator.APPLICABLE_STAGES[dtype.name]
            )

            if (
                rand_mutation_stage >= mutator.STAGE_ARITH8
                and rand_mutation_stage <= mutator.STAGE_ARITH32
            ):
                arith_val = random.randint(1, 35)
            else:
                arith_val = 0

            if (
                rand_mutation_stage >= mutator.STAGE_INTEREST8
                and rand_mutation_stage <= mutator.STAGE_INTEREST32
            ):
                interesting_idx = random.randint(
                    0, len(mutator.INTERESTING_MAP[rand_mutation_stage]) - 1
                )
            else:
                interesting_idx = -1

            bit_size = dtype.itemsize * 8
            if dtype.itemsize == 0:
                if dtype.type is np.str_:
                    bit_size = c.STRLEN_MAX * 8

            # print("data before rand mutation:", data_val)

            data_val = mutator.mutate_one(
                dtype,
                data_val,
                rand_mutation_stage,
                random.randint(0, bit_size - 1),
                arith_val,
                interesting_idx,
            )
            # print("data after rand mutation:", data_val)

            msg_mutated = None
            if data_val is not None:
                msg_mutated = deepcopy(msg_to_mutate)
                obj = reduce(getattr, attr_list[:-1], msg_mutated)
                setattr(obj, attr_leaf, data_val)
                # fuzzer.fuzz_and_check(self.fuzzer, nmsg, frame)

                self.msg_list[msg_idx] = msg_mutated

        return (self.msg_list, frame)

    def mutate_generic(self, config):
        # Handle one Cycle
        frame = str(time.time())

        meta_file = os.path.join(config.meta_dir, "meta-{}".format(frame))
        with open(meta_file, "w") as fp:
            fp.write(
                self.subscriber_node
                + "\t"
                + self.topic_name
                + "\t"
                + str(self.msg_type_class)
                + "\t"
                + str(self.cycle_cnt)
                + "\t"
                + str(self.round_cnt)
            )

        print(
            "\n\x1b[92mCYCLE:",
            self.cycle_cnt,
            "ROUND:",
            self.round_cnt,
            "EXEC:",
            self.exec_cnt,
            "\x1b[0m",
            frame,
        )
        self.exec_cnt += 1

        if self.is_new_cycle:
            print("NEW CYCLE")
            try:
                msg = self.fuzzer.queue.popleft()
                self.from_queue = True
            except IndexError:
                msg = self.default_msg
                self.from_queue = False
            self.is_new_cycle = False
        else:
            msg = self.init_msg

        if self.cur_fm_field < self.num_fields:
            # A. FIELD MUTATION
            field = self.msg_field_list[self.cur_fm_field]
            cur_round = self.fm_field_stages[self.cur_fm_field]
            print("Current field:", field[0], "({})".format(field[1]))
            # print("cur_round:", cur_round)

            dtype = field[-1]
            attr_list = field[:-1]
            attr_leaf = attr_list[-1]

            if cur_round == 0:
                if self.from_queue:
                    print("SKIP GEN (from queue)")
                    # set odata == msg
                    data_val = reduce(getattr, attr_list, msg)
                    self.fm_odata.append(data_val)

                else:
                    print("STAGE: GEN")
                    # GENERATE RANDOM DATA FOR SELECTED FIELD
                    data_val = mutator.gen_rand_data(dtype, False)
                    obj = reduce(getattr, attr_list[:-1], msg)
                    setattr(obj, attr_leaf, data_val)
                    self.fm_odata.append(data_val)

                    # set rest of the fields (rfield) to default
                    for idx, rfield in enumerate(self.msg_field_list):
                        if idx == self.cur_fm_field:
                            continue
                        dtype = rfield[-1]
                        attr_list = rfield[:-1]
                        attr_leaf = attr_list[-1]
                        data_val = mutator.gen_rand_data(dtype, True)
                        obj = reduce(getattr, attr_list[:-1], msg)
                        setattr(obj, attr_leaf, data_val)

                    # fuzzer.fuzz_and_check(self.fuzzer, msg, frame)

                self.init_msg = msg
                self.round_cnt += 1
                self.fm_field_stages[self.cur_fm_field] += 1
                return (msg, frame)

            if cur_round == 1:
                # GO THROUGH DETERMINISTIC MUTATION STAGES
                cur_determ_stage_id = self.fm_determ_stages[self.cur_fm_field]
                determ_stage = mutator.APPLICABLE_STAGES[dtype.name][
                    cur_determ_stage_id
                ]
                print(
                    "STAGE: DETERM {}".format(
                        mutator.STAGE_NAMES[determ_stage]
                    )
                )

                bit_size = dtype.itemsize * 8
                if dtype.itemsize == 0:
                    if dtype.type is np.str_:
                        bit_size = c.STRLEN_MAX * 8

                if determ_stage < 3:
                    # bit flip stages
                    skip = 1
                else:
                    # byte flips and rest of the stages
                    skip = 8

                # obj = reduce(getattr, attr_list[:-1], msg)
                # data_val = getattr(obj, attr_leaf)
                odata = self.fm_odata[self.cur_fm_field]
                print("data before determ mutation:", odata)

                # for bit_pos in range(0, bit_size, skip):
                nmsg = None
                if self.bit_pos < bit_size:
                    print("bit_pos", self.bit_pos, "(", bit_size, ")")
                    if (
                        determ_stage >= mutator.STAGE_ARITH8
                        and determ_stage <= mutator.STAGE_ARITH32
                    ):
                        # for arith_val in range(-35, 36):
                        if self.arith_val < 36:
                            data_val = mutator.mutate_one(
                                dtype,
                                odata,
                                determ_stage,
                                self.bit_pos,
                                self.arith_val,
                            )
                            print("data after determ mutation:", data_val)

                            if data_val is not None:
                                nmsg = deepcopy(msg)
                                obj = reduce(getattr, attr_list[:-1], nmsg)
                                setattr(obj, attr_leaf, data_val)
                                # fuzzer.fuzz_and_check(self.fuzzer, nmsg, frame)

                            self.arith_val += 1
                        else:
                            # move on to the text bits after trying all arithmetic values for the current bits
                            self.bit_pos += skip
                            self.arith_val = -35

                    elif (
                        determ_stage >= mutator.STAGE_INTEREST8
                        and determ_stage <= mutator.STAGE_INTEREST32
                    ):
                        # for interesting_idx in range(len(mutator.INTERESTING_MAP[determ_stage])):
                        if self.interesting_idx < len(
                            mutator.INTERESTING_MAP[determ_stage]
                        ):
                            data_val = mutator.mutate_one(
                                dtype,
                                odata,
                                determ_stage,
                                self.bit_pos,
                                arith_val=0,
                                interesting_idx=self.interesting_idx,
                            )
                            if data_val is not None:
                                nmsg = deepcopy(msg)
                                obj = reduce(getattr, attr_list[:-1], nmsg)
                                setattr(obj, attr_leaf, data_val)
                                # fuzzer.fuzz_and_check(self.fuzzer, nmsg, frame)
                            # print("data after determ mutation:", data_val)

                            self.interesting_idx += 1
                        else:
                            # move on to the text bits after trying all intersting values for the current bits
                            self.bit_pos += skip
                            self.interesting_idx = 0

                    else:
                        data_val = mutator.mutate_one(
                            dtype, odata, determ_stage, self.bit_pos
                        )
                        # print("data after determ mutation:", data_val)

                        if data_val is not None:
                            nmsg = deepcopy(msg)
                            obj = reduce(getattr, attr_list[:-1], nmsg)
                            setattr(obj, attr_leaf, data_val)
                            # fuzzer.fuzz_and_check(self.fuzzer, nmsg, frame)

                        # flip then move on
                        self.bit_pos += skip

                    return (nmsg, frame)

                print("end of one determ stage of an operation")
                # reset internal states
                self.bit_pos = 0
                self.interesting_idx = 0

                self.fm_determ_stages[self.cur_fm_field] += 1
                if self.fm_determ_stages[self.cur_fm_field] == len(
                    mutator.APPLICABLE_STAGES[dtype.name]
                ):
                    self.round_cnt += 1
                    self.fm_field_stages[self.cur_fm_field] += 1

                return (
                    None,
                    None,
                )  # temporarily return to avoid nonetype unpacking error

            if cur_round == 2:
                print("STAGE: HAVOC")
                print(self.fm_rand_stages[self.cur_fm_field], "remaining")
                # GO THROUGH RANDOM MUTATION STAGES
                rand_mutation_stage = random.choice(
                    mutator.APPLICABLE_STAGES[dtype.name]
                )

                if (
                    rand_mutation_stage >= mutator.STAGE_ARITH8
                    and rand_mutation_stage <= mutator.STAGE_ARITH32
                ):
                    arith_val = random.randint(1, 35)
                else:
                    arith_val = 0

                if (
                    rand_mutation_stage >= mutator.STAGE_INTEREST8
                    and rand_mutation_stage <= mutator.STAGE_INTEREST32
                ):
                    interesting_idx = random.randint(
                        0,
                        len(mutator.INTERESTING_MAP[rand_mutation_stage]) - 1,
                    )
                else:
                    interesting_idx = -1

                bit_size = dtype.itemsize * 8
                if dtype.itemsize == 0:
                    if dtype.type is np.str_:
                        bit_size = c.STRLEN_MAX * 8

                data_val = self.fm_odata[self.cur_fm_field]
                # print("data before rand mutation:", data_val)
                data_val = mutator.mutate_one(
                    dtype,
                    data_val,
                    rand_mutation_stage,
                    random.randint(0, bit_size - 1),
                    arith_val,
                    interesting_idx,
                )
                # print("data after rand mutation:", data_val)

                nmsg = None
                if data_val is not None:
                    nmsg = deepcopy(msg)
                    obj = reduce(getattr, attr_list[:-1], nmsg)
                    setattr(obj, attr_leaf, data_val)
                    # fuzzer.fuzz_and_check(self.fuzzer, nmsg, frame)

                self.fm_rand_stages[self.cur_fm_field] -= 1
                if self.fm_rand_stages[self.cur_fm_field] == 0:
                    self.round_cnt += 1
                    self.fm_field_stages[self.cur_fm_field] += 1

                    print("next field")
                    self.cur_fm_field += 1

                return (nmsg, frame)

            # END OF FIELD MUTATION

        else:
            print("STAGE: MSG-ALL")
            # B. MESSAGE MUTATION
            # for field in fields_to_mutate:
            field = random.choice(self.fields_to_mutate)
            dtype = field[-1]
            attr_list = field[:-1]
            attr_leaf = attr_list[-1]

            determ_stage = random.choice(mutator.APPLICABLE_STAGES[dtype.name])
            if (
                determ_stage >= mutator.STAGE_ARITH8
                and determ_stage <= mutator.STAGE_ARITH32
            ):
                arith_val = random.randint(1, 35)
            else:
                arith_val = 0

            if (
                determ_stage >= mutator.STAGE_INTEREST8
                and determ_stage <= mutator.STAGE_INTEREST32
            ):
                interesting_idx = random.randint(
                    0, len(mutator.INTERESTING_MAP[determ_stage]) - 1
                )
            else:
                interesting_idx = -1

            bit_size = dtype.itemsize * 8
            if dtype.itemsize == 0:
                if dtype.type is np.str_:
                    bit_size = c.STRLEN_MAX * 8

            obj = reduce(getattr, attr_list[:-1], msg)
            data_val = getattr(obj, attr_leaf)
            # print("data before msg mutation:", data_val)
            data_val = mutator.mutate_one(
                dtype,
                data_val,
                determ_stage,
                random.randint(0, bit_size - 1),
                arith_val,
                interesting_idx,
            )
            # print("data after msg mutation:", data_val)
            if data_val is not None:
                obj = reduce(getattr, attr_list[:-1], msg)
                setattr(obj, attr_leaf, data_val)

            # fuzzer.fuzz_and_check(self.fuzzer, msg, frame)
            self.num_msg_mutation += 1

            if self.num_msg_mutation == 20:
                print("end of cycle")
                self.cycle_cnt += 1
                self.is_new_cycle = True
                self.round_cnt = 0

                # reset counters
                self.cur_fm_field = 0
                self.fm_field_stages = [0] * self.num_fields
                self.fm_determ_stages = [0] * self.num_fields
                fm_rand_stages = []
                for i in range(self.num_fields):
                    fm_rand_stages.append(random.randint(1, 2))
                self.fm_odata = []
                self.num_msg_mutation = 0
                self.bit_pos = 0
                self.arith_val = -35
                self.interesting_idx = 0

                return (None, frame)

                # finish a cycle and move on to the next item in
                # queue
                # break
            else:
                return (msg, frame)

            # B. END OF MESSAGE MUTATION
