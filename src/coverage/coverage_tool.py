#!/usr/bin/python3

import os
import time
import pprint


def clear_gcda(config):
    #
    # .gcda files need to be removed before executing node binary
    #

    cnt = 0
    for filename in os.listdir(config.pkg_cov_dir):
        if filename.endswith(".gcda"):
            cnt += 1
            os.unlink(os.path.join(config.pkg_cov_dir, filename))

    # print("[*] unlinked {} files".format(cnt))


def run_gcov(config):
    gcov_options = "--branch-probabilities"
    gcov_options += " --branch-counts"
    gcov_options += " --intermediate-format"

    cov_obj_files = [filename for filename in os.listdir(config.pkg_cov_dir)
        if filename.endswith(".gcno")]

    cov_report_files = []
    for cov_obj_file in cov_obj_files:
        src_filename = os.path.splitext(cov_obj_file)[0]
        src_absname = os.path.join(config.pkg_src_dir, src_filename)
        if not os.path.exists(src_absname):
            print("[-] couldn't find src file")
            continue

        obj_absname = os.path.join(config.pkg_cov_dir, cov_obj_file)

        cmd = "gcov {} {} {} {} > /dev/null 2>&1".format(gcov_options, src_absname,
                "--object-file", obj_absname)

        os.chdir(config.gcov_dir)
        os.system(cmd)
        os.chdir(config.src_dir)
        cov_report_files.append(src_filename + ".gcov")

    return cov_report_files


def check_cov(config, cov_report_files, covmap):
    updated = False
    for report_file in cov_report_files:
        # print(report_file)
        with open(os.path.join(config.gcov_dir, report_file), "r") as fp:
            cov_report = fp.readlines()

        do_branch = False
        do_populate = False
        branch_updated = False
        bu_lineno = 0
        bu_index = []
        for line in cov_report:
            line_tokens = line.strip().split(":")
            tag = line_tokens[0]
            info = line_tokens[1]
            # print(tag, info)

            if tag == "file":
                file_absname = info
                file_name = info.split("/")[-1]
                # print("SOURCE:", file_name)
                if file_name in covmap:
                    line_map = covmap[file_name]
                else:
                    line_map = {}
                    covmap[file_name] = line_map

            # elif tag == "function":
                # line_no = int(info.split(",")[0])
                # exec_cnt = info.split(",")[1]
                # func_name = info.split(",")[2].rstrip()
                # if len(func_map) == 0:
                    # line_map = {}
                    # func_map[line_no] = line_map
                # else:
                    # line_map = func_map[line_no]

            elif tag == "lcount":
                # process and show updated branches
                if branch_updated:
                    with open(file_absname, "r") as fp:
                        lines = fp.readlines()
                    try:
                        code = "".join(lines[bu_lineno-4:bu_lineno-1])
                    except IndexError:
                        code = ""
                    code += "\x1b[91m" + lines[bu_lineno-1] + "\x1b[0m"
                    try:
                        code += "".join(lines[bu_lineno:bu_lineno+2])
                    except IndexError:
                        pass

                    print("[cov] new branch @{}".format(bu_lineno))
                    print(code)
                    updated = True
                    cstr = ""
                    for index, elem in enumerate(line_map[bu_lineno][1]):
                        if index in bu_index:
                            s = "\x1b[92m" + str(elem) + "\x1b[0m"
                        else:
                            s = str(elem)
                        cstr += s
                        if index < len(line_map[bu_lineno][1]) -1:
                            cstr += ", "
                    lstr = "[{}, [{}]]".format(line_map[bu_lineno][0], cstr)
                    print(old_branch_info)
                    print(lstr)

                    branch_updated = False
                    bu_lineno = 0
                    bu_index = []
                do_branch = True
                do_populate = False

                line_no = int(info.split(",")[0])
                line_count = int(info.split(",")[1])
                if line_no in line_map:
                    prev_line_count = line_map[line_no][0]
                    # line_map[line_no][0] += line_count # accumulate line cnt
                    if line_count > 0:
                        line_map[line_no][0] += line_count
                    if prev_line_count == 0 and line_map[line_no][0] > 0:
                        with open(file_absname, "r") as fp:
                            lines = fp.readlines()
                        try:
                            code = "".join(lines[line_no-4:line_no-1])
                        except IndexError:
                            code = ""
                        code += "\x1b[91m" + lines[line_no-1] + "\x1b[0m"
                        try:
                            code += "".join(lines[line_no:line_no+2])
                        except IndexError:
                            pass
                        print("[cov] new line @{}".format(line_no))
                        print(code)
                        updated = True
                else:
                    # line_cov_arr = [line_count, []]
                    if line_count == 0:
                        line_cov_arr = [0, []]
                    else:
                        line_cov_arr = [line_count, []]
                    line_map[line_no] = line_cov_arr
                old_branch_info = str(line_map[line_no])

            elif tag == "branch":
                line_no = int(info.split(",")[0])
                branch_type = info.split(",")[1]
                line_cov_arr = line_map[line_no]
                if do_branch:
                    # print("process branch")
                    if len(line_cov_arr[1]) == 0:
                        do_populate = True
                        # print("populate branch array")
                    do_branch = False
                    b_index = 0

                if branch_type == "taken":
                    b_type_code = 1
                elif branch_type == "nottaken":
                    b_type_code = 0
                else: # notexec
                    b_type_code = -1

                if do_populate:
                    line_map[line_no][1].append(b_type_code)
                else:
                    old_branch_exec = line_map[line_no][1][b_index]
                    if b_type_code > old_branch_exec and b_type_code == 1:
                        branch_updated = True
                        bu_lineno = line_no
                        bu_index.append(b_index)
                        # print("[cov] new branch:", line_no,
                                # line_map[line_no][1], "@", b_index)
                        line_map[line_no][1][b_index] = b_type_code

                b_index += 1

    if updated:
        return time.time()
    else:
        return 0.0


def get_cov_stat(last_update, covmap):
    print("*** COVERAGE - {:.2f}s since new coverage ***".format(time.time() - last_update))
    for filename in covmap:
        lmap = covmap[filename]
        total_lines = len(lmap)
        exec_lines = 0
        total_branches = 0
        exec_branches = 0

        for lineno in lmap:
            line_info = lmap[lineno]
            if line_info[0] > 0:
                exec_lines += 1
            branch_info = line_info[1]
            total_branches += len(branch_info)
            for branch in branch_info:
                if branch == 1:
                    exec_branches += 1

        print("- FILE {}".format(filename))
        print("  - Lines : {} / {} ({:.2f}%)".format(exec_lines, total_lines,
            (exec_lines / float(total_lines)) * 100))
        print("  - Branch: {} / {} ({:.2f}%)".format(exec_branches,
            total_branches, (exec_branches / float(total_branches)) * 100))


if __name__ == "__main__":
    pkg_name = "turtlesim"
    node_name = "turtlesim_node"

    ros_prefix= "/home/seulbae/workspace/ros-security/ros2_dashing"

    src_dir = "src/ros/ros_tutorials/turtlesim/src"
    bin_dir = "build/turtlesim"
    cov_dir = "build/turtlesim/CMakeFiles/turtlesim_node.dir/src"

    gcov_options = "--branch-probabilities --branch-counts --intermediate-format"
    # gcov_options = "--branch-probabilities --branch-counts"
    # gcov_options = "--branch-probabilities --intermediate-format"

    all_src_files = os.listdir(os.path.join(ros_prefix, src_dir))
    # print(all_src_files)

    cov_obj_files = [filename for filename in os.listdir(os.path.join(ros_prefix,
        cov_dir)) if filename.endswith(".gcno")]

    for cov_obj_file in cov_obj_files:
        print(cov_obj_file)
        a = input(" ")
        src_filename = os.path.splitext(cov_obj_file)[0]
        src_absname = os.path.join(ros_prefix, src_dir, src_filename)
        if not os.path.exists(src_absname):
            print("[-] couldn't find src file")
            continue
        obj_absname = os.path.join(ros_prefix, cov_dir, cov_obj_file)

        cmd = "gcov {} {} {} {}".format(gcov_options, src_absname,
                "--object-file", obj_absname)

        # print(cmd)
        os.system(cmd)

        with open(src_filename + ".gcov", "r") as fp:
            cov_report = fp.readlines()

        executed_lines = []
        for line in cov_report:
            line_tokens = line.strip().split(":")
            tag = line_tokens[0]
            info = line_tokens[1]

            if tag == "lcount":
                line_no = info.split(",")[0]
                line_count = info.split(",")[1]
                if int(line_count) > 0:
                    executed_lines.append(int(line_no))
            if tag == "branch":
                line_no = info.split(",")[0]
                branch_type = info.split(",")[1]
                print("branch", line_no, branch_type)
                pass

