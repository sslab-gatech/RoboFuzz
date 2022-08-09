import math
import sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def trim_ts(ts, digits=16):
    if len(str(ts)) < digits:
        ts = int(str(ts).zfill(digits))
    else:
        ts = int(str(ts)[:digits])

    return ts


class RosbagParser:
    flag_topic_name = "/listen_flag"
    debug = False

    def __init__(self, db_file):
        self.conn, self.cursor = self.connect(db_file)

        topics = self.get_all_topics()

        self.topic_name_type_map = {
            topic_name: topic_type
            for topic_id, topic_name, topic_type, _, _ in topics
        }

        self.topic_id_name_map = {
            topic_id: topic_name
            for topic_id, topic_name, topic_type, _, _ in topics
        }

        self.topic_name_msg_map = {
            topic_name: get_message(topic_type)
            for topic_id, topic_name, topic_type, _, _ in topics
        }

        self.messages = self.get_all_messages()

        found = self.find_flag_info()
        if not found:
            self.abort = True
        else:
            self.abort = False

    def connect(self, db_file):
        """Make connection to an SQLite database file."""

        conn = sqlite3.connect(db_file)
        c = conn.cursor()

        return conn, c

    def get_all_topics(self):
        """get all topic names and types"""

        self.cursor.execute("SELECT * from topics;")

        rows = self.cursor.fetchall()

        if self.debug:
            for row in rows:
                print(row)

        return rows

    def get_all_messages(self):

        self.cursor.execute("SELECT * from messages;")

        rows = self.cursor.fetchall()

        if self.debug:
            for row in rows:
                print(row)

        return rows

    def find_flag_info(self):
        """Find listen_flag topic id"""
        return True

        found = False
        for topic_id, topic_name in self.topic_id_name_map.items():
            if topic_name == self.flag_topic_name:
                self.flag_topic_id = topic_id
                found = True

        if not found:
            print("[-] listen_flag message is dropped")

        return found

    def process_messages(self) -> dict:
        """
        Retrieve messages published only when listen_flag is activated,
        and deserialize the CDR-formatted data.
        """

        filtered_messages_by_topic = dict()

        """
        start_ts = 0
        end_ts = 0
        listen_flag = False
        for msg_id, topic_id, ts, data in self.messages:
            if topic_id == self.flag_topic_id:
                flag_data = deserialize_message(
                    data, self.topic_name_msg_map[self.flag_topic_name]
                )

                if listen_flag is False:
                    start_ts = flag_data.data
                    listen_flag = True
                else:
                    end_ts = flag_data.data
                    listen_flag = False

                    break

        """

        with open("/tmp/start_ts", "r") as f:
            start_ts = int(f.readline())
        with open("/tmp/end_ts", "r") as f:
            end_ts = int(f.readline())

        # trim/extend digits for comparison
        digits = 16 # 10 + 6 (e.g., 1641420015.780356)
        start_ts = trim_ts(start_ts, digits)
        end_ts = trim_ts(end_ts, digits)

        # print(start_ts, end_ts)

        for msg_id, topic_id, ts, data in self.messages:
            # if topic_id != self.flag_topic_id:
            ts_trimmed = trim_ts(ts, digits)
            if ts_trimmed > start_ts and ts_trimmed < end_ts:
                topic_name = self.topic_id_name_map[topic_id]
                deserialized_data = deserialize_message(
                    data, self.topic_name_msg_map[topic_name]
                )
                if topic_name not in filtered_messages_by_topic:
                    filtered_messages_by_topic[topic_name] = [
                        (ts, deserialized_data)
                    ]
                else:
                    filtered_messages_by_topic[topic_name].append(
                        (ts, deserialized_data)
                    )

        return filtered_messages_by_topic

    def process_all_messages(self) -> dict:
        """
        Retrieve all messages recorded in the rosbag
        and deserialize the CDR-formatted data.
        """

        filtered_messages_by_topic = dict()

        for msg_id, topic_id, ts, data in self.messages:
            topic_name = self.topic_id_name_map[topic_id]
            deserialized_data = deserialize_message(
                data, self.topic_name_msg_map[topic_name]
            )
            if topic_name not in filtered_messages_by_topic:
                filtered_messages_by_topic[topic_name] = [
                    (ts, deserialized_data)
                ]
            else:
                filtered_messages_by_topic[topic_name].append(
                    (ts, deserialized_data)
                )

        return filtered_messages_by_topic


if __name__ == "__main__":
    # test
    db3_file = "states-0.bag/states-0.bag_0.db3"
    parser = RosbagParser(db3_file)
    msgs = parser.process_all_messages()
    print(msgs.keys())

    joint_states = msgs["/joint_states"]
    panda_arm_controller_state = msgs["/panda_arm_controller/state"]
    move_action_status = msgs["/move_action/_action/status"]
    motion_plan_requests = msgs["/motion_plan_request"]

    for (ts, cont_state) in panda_arm_controller_state:
        print(cont_state.error.positions)
        print(len(cont_state.error.positions))

