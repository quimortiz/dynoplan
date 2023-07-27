import yaml
import matplotlib.pyplot as plt

filename = "/tmp/dynoplan/close_list.yaml"

with open(filename) as f:
    data = yaml.load(f, Loader=yaml.CLoader)

print("loading done")

close_list = data["close_list"]


X = [x[0] for x in close_list]
Y = [x[1] for x in close_list]


plt.plot(X, Y, "r.")

plt.show()
