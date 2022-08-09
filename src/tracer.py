import os


class APITracer:
    def __init__(self, feature, targets):
        self.tracer = "ltrace"
        self.feature = feature
        self.targets = targets

    def config_trace(self):
        pass

    def get_timestamp(self, filename="out"):
        if not os.path.isfile(filename):
            print(f"[tracer] Cannot find trace file {filename}")
            return -1

        with open(filename, "r") as fp:
            out_raw = fp.readlines()

        for line in out_raw:
            if "time1" in line:
                t0 = line.strip().split(" ")[-1]
                if "." not in t0:
                    t0 = (
                        int(t0) / 1000000.0
                    )  # c++/rust prints us w/o decimal point
                else:
                    t0 = float(t0)
            elif "time2" in line:
                t1 = line.strip().split(" ")[-1]
                if "." not in t1:
                    t1 = (
                        int(t1) / 1000000.0
                    )  # c++/rust prints us w/o decimal point
                else:
                    t1 = float(t1)
                break

        self.t0 = t0
        self.t1 = t1

    def parse_trace(self, filename="trace"):
        if not os.path.isfile(filename):
            print(f"[tracer] Cannot find trace file {filename}")
            return -1

        func_dict = dict()

        with open(filename, "r") as fp:
            trace_raw = fp.readlines()

        for line in trace_raw:
            line = line.strip()
            timestamp = float(line.split(" ")[0])

            if timestamp < self.t0 or timestamp > self.t1:
                continue

            if "<unfinished ...>" in line:
                func_name = line.split(" ")[1].split("@")[0]
                retval = None
                if func_name in func_dict:
                    func_dict[func_name].append([timestamp, retval])
                else:
                    func_dict[func_name] = [[timestamp, retval]]

            elif "<..." in line and "resumed>" in line:
                func_name = line.split(" ")[2]
                retval = line.split("= ")[-1]

                if func_name not in func_dict:
                    print(f"[tracer] fatal error: {func_name} not seen yet")
                    return -1

                # update retval
                func_dict[func_name][-1][1] = retval

            elif ".so" in line:
                func_name = line.split(" ")[1].split("@")[0]
                retval = line.split("= ")[-1]

                if func_name in func_dict:
                    func_dict[func_name].append([timestamp, retval])
                else:
                    func_dict[func_name] = [[timestamp, retval]]

        # print(func_dict)
        return func_dict
