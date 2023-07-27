import argparse
import yaml


parser = argparse.ArgumentParser()
parser.add_argument("--file", required=True)  # positional argument
# positional argument
parser.add_argument("--max", type=int, required=True)


args = parser.parse_args()

filename = args.file
max_dist = args.max


with open(filename, "r") as f:
    data = yaml.safe_load(f)


X = []
Y = []
Z = []
dist = []
parent = []

with open(filename, "r") as f:
    lines = f.readlines()


for d in data["heu_map"]:
    print(d)
    XX = d["x"]
    X.append(XX[0])
    Y.append(XX[1])
    Z.append(XX[2])
    dist.append(d["d"])
    parent.append(d["p"])
    # X.append(float(data[0]))
    # Y.append(float(data[1]))
    # theta.append(float(data[2]))
    # dist.append(float(data[3+2]))
    # parent.append(int(data[4+2]))
    #

# plot the X,Y
import matplotlib.pyplot as plt
import numpy as np

fig = plt.figure()
ax = fig.add_subplot(projection="3d")

#
for i, p in enumerate(parent):
    if i != p:  # and p == len(parent)-1:
        ax.plot3D([X[i], X[p]], [Y[i], Y[p]], [Z[i], Z[p]], color="gray")
#
# print( manp.max(dist) )
# print( np.minimum(dist,np.max(dist)) )
cb = ax.scatter(X, Y, Z, c=np.minimum(dist, min(np.max(dist), max_dist)))


#
# for xx, yy, tt, dd in zip(X, Y, theta, dist):
#     draw_tri([xx, yy, tt])
#
# ax = plt.gca()
# for i in range(len(X)):
#     ax.annotate(str(dist[i]), (X[i], Y[i]))


# problem in 1389 284
# i 284 m 26 offset 4.145024.745040 valid 1 neig 1260 3479 1389 1756


# cb = plt.scatter(X,Y)
plt.axis("auto")
plt.colorbar(cb)

# plot the graph


plt.show()
