import yaml
import matplotlib.pyplot as plt

import sys  # noqa
sys.path.append("/home/quim/code/dynoplan/dynobench/dynobench/utils/viewer")  # noqa


from quad2d_viewer import Quad2dViewer


fwd_file = "figs_iros/quad2d_bb/db_rrt_tree_fwd_0.yaml"
bwd_file = "figs_iros/quad2d_bb/db_rrt_tree_bwd_0.yaml"


filename_env = "dynobench/envs/quad2d_v0/quad_bugtrap_double.yaml"

with open(filename_env) as env_file:
    env = yaml.safe_load(env_file)


viewer = Quad2dViewer()

# load the data
with open(fwd_file, 'r') as stream:
    fwd_data = yaml.load(stream, Loader=yaml.CLoader)

with open(bwd_file, 'r') as stream:
    bwd_data = yaml.load(stream, Loader=yaml.CLoader)


# plot 100 nodes of each tree


fig, ax = plt.subplots()



for e in fwd_data["edges"][:38]:
    traj = e["traj"]
    X = [x[0] for x in traj]
    Y = [x[1] for x in traj]
    ax.plot(X, Y,"g",alpha=.4)

for e in bwd_data["edges"][:45]:
    traj = e["traj"]
    X = [x[0] for x in traj]
    Y = [x[1] for x in traj]
    ax.plot(X, Y,"r",alpha=.4)



viewer.view_problem(ax, env, env_name=filename_env)
plt.show()


fig, ax = plt.subplots()



for e in fwd_data["edges"]:
    traj = e["traj"]
    X = [x[0] for x in traj]
    Y = [x[1] for x in traj]
    ax.plot(X, Y,"g",alpha=.4)

for e in bwd_data["edges"]:
    traj = e["traj"]
    X = [x[0] for x in traj]
    Y = [x[1] for x in traj]
    ax.plot(X, Y,"r",alpha=.4)



viewer.view_problem(ax, env, env_name=filename_env)
plt.show()



#

filename_traj = "figs_iros/quad2d_bb/db_rrt_traj_0.yaml"
with open(filename_traj) as f:
    result = yaml.load(f,Loader=yaml.CLoader)

X = [X[0] for X in result["states"]]
Y = [X[1] for X in result["states"]]

fig, ax = plt.subplots()

ax.plot(X, Y, "b", alpha=1)
viewer.view_problem(ax, env, env_name=filename_env)

plt.show()

filename_traj_opt = "figs_iros/quad2d_bb/run_7_out.yaml"

with open(filename_traj_opt) as f:
    result = yaml.load(f,Loader=yaml.CLoader)

fig, ax = plt.subplots()

ax.plot(X, Y, "gray", alpha=.8)
viewer.view_problem(ax, env, env_name=filename_env)


Xopt = [X[0] for X in result["trajs_opt"][0]["states"]]
Yopt = [X[1] for X in result["trajs_opt"][0]["states"]]

# init_guess = "/tmp/dynoplan/init_guess_smooth.yaml"
# with open(init_guess) as f:
#     guess = yaml.load(f,Loader=yaml.CLoader)
#
# ax.plot([X[0] for X in guess["states"]], [X[1] for X in guess["states"]], "r", alpha=.8)

ax.plot(Xopt, Yopt, "b", alpha=1)


plt.show()


