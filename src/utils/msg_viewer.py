import sys
import pickle

msg_file = sys.argv[1].strip()

with open(msg_file, "r") as f:
    msg_dict = pickle.load(f)

print(msg_dict)
