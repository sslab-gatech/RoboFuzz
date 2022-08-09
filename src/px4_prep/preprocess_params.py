import pickle
from bs4 import BeautifulSoup


f1 = open("../PARAMS_IN_USE.list", "r")

param_list = list()

lines = f1.readlines()
for line in lines:
    if line.startswith("x"):
        name = line.replace("+", " ").replace("*", " ")[4:].split(" ")[0]
        param_list.append(name)

f1.close()


f = open("./parameter_reference.md", "r")
doc = f.read()
f.close()

soup = BeautifulSoup(doc, "html.parser")

tables = soup.find_all("table")
print(len(tables))

def find_param_in_table(param_name, table):
    strong = table.find(id=param_name)
    return strong


param_dict = dict()

for param in param_list:
    for table in tables:
        tag = find_param_in_table(param, table)
        if tag is not None:
            param_name = tag["id"]

            param_type = tag.parent.text.split("(")[1].split(")")[0]

            tr = tag.parent.parent
            tds = tr.find_all("td")
            range_str = tds[2].text
            default_str = tds[3].text
            unit_str = tds[4].text

            range_str = range_str.split("(")[0]
            if ">" in range_str:
                range_min = range_str.split(" > ")[0].strip()
                range_max = range_str.split(" > ")[1].strip()
                if range_min == "?":
                    range_min = None
                else:
                    if param_type == "INT32":
                        range_min = int(range_min)
                    elif param_type == "FLOAT":
                        range_min = float(range_min)

                if range_max == "?":
                    range_max = None
                else:
                    if param_type == "INT32":
                        range_max = int(range_max)
                    elif param_type == "FLOAT":
                        range_max = float(range_max)

            else:
                range_min = None
                range_max = None

            if range_min == "?":
                range_min = None
            if range_max == "?":
                range_max = None

            if "able" in default_str:
                # 0 or 1
                default_val = int(default_str.split("(")[-1].split(")")[0])
            else:
                default_val = default_str

                if param_type == "INT32":
                    default_val = int(default_val)
                elif param_type == "FLOAT":
                    default_val = float(default_val)

            param_dict[param_name] = {
                "type": param_type,
                "min": range_min,
                "max": range_max,
                "default": default_val,
                "unit": unit_str
            }

            # print(f"{param_name} / {param_type} / {range_min} {range_max} / {default_val}")

            break

pickle.dump(param_dict, open("params.pkl", "wb"))




