import matplotlib.pyplot as plt
import msgpack
import yaml
from matplotlib import animation
from pathlib import Path


def plot_search_tree(ax, trajs, sol):
    starts = []
    N = len(trajs)
    for i in range(N):
        states = trajs[i]["states"]
        X = [s[0] for s in states]
        Y = [s[1] for s in states]
        starts.append([states[0][0], states[0][1]])
        ax.plot(X, Y, color=".5", alpha=0.2)
        # print("saving ", i)
        # plt.savefig(f"/tmp/fig_{i}.png")

    x_sol = [X[0] for X in sol["states"]]
    y_sol = [X[1] for X in sol["states"]]
    ax.plot(x_sol, y_sol, color="orange")
    ax.scatter([s[0] for s in starts], [s[1] for s in starts], s=2, color="k")


# file = "../data_for_videos/expanded_trajs.msgpack"
file = "/tmp/dynoplan/expanded_trajs.msgpack"

# load the viewer


import sys

sys.path.append("/home/quim/stg/wolfgang/dynoplan/dynobench/utils/viewer")
# str(Path(__file__).parent))
import car_with_trailer_viewer

viewer = car_with_trailer_viewer.CarWithTrailerViewer()


with open(file, "rb") as f:
    data = msgpack.unpackb(f.read(), raw=False)

# solution = "../data_for_videos/dbastar_out.yaml"
#
# solution_after_opt = "../data_for_videos/i_traj_out.yaml"

# solution = "../results_new/car1_v0/kink_0/idbastar_v0_vid_car/2023-10-29--09-45-36/run_0_out.yaml.trajraw-4.yaml"
#
# solution_after_opt = "../results_new/car1_v0/kink_0/idbastar_v0_vid_car/2023-10-29--09-45-36/run_0_out.yaml.trajopt-4.yaml"

solution = "/tmp/dynoplan/dbastar_out.yaml"
solution_after_opt = "/tmp/dynoplan/opt_eWlMkZ.yaml"


with open(solution, "r") as f:
    data_sol = yaml.safe_load(f)


with open(solution_after_opt, "r") as f:
    data_sol_opt = yaml.safe_load(f)
# print(data_sol_opt)

print(len(data["data"]))


# N = 1000

N = len(data["data"])


trajs = data["data"]
#
# print(traj)
fig = plt.figure()
ax = fig.add_subplot(111, aspect="equal")
plt.tick_params(
    top=False, bottom=False, left=False, right=False, labelleft=False, labelbottom=False
)
plt.axis("off")
fig.tight_layout()

filename_env = "../dynobench/envs/car1_v0/kink_0.yaml"
with open(filename_env) as env_file:
    env = yaml.safe_load(env_file)

viewer.view_problem(ax, env, env_name=filename_env)


result = data_sol["result"][0]
# result = data_sol
result_opt = data_sol_opt

x_sol = [X[0] for X in result["states"]]
y_sol = [X[1] for X in result["states"]]

x_solopt = [X[0] for X in result_opt["states"]]
y_solopt = [X[1] for X in result_opt["states"]]

plot_search_tree(ax, trajs, result)

ax.set_aspect("equal", adjustable="box")


plt.show()


# animation

fig = plt.figure()
ax = fig.add_subplot(111, aspect="equal")
plt.tick_params(
    top=False, bottom=False, left=False, right=False, labelleft=False, labelbottom=False
)
plt.axis("off")
fig.tight_layout()


viewer.view_problem(ax, env, env_name=filename_env)


pack = 200


robot = car_with_trailer_viewer.Robot()

# robot.draw(ax, result["states"][0], facecolor="blue")

kspeed = 2


def animate_func(i):
    """ """
    print("i", i)
    if i < Tstep1:
        for j in range(pack):
            t = i * pack + j
            if t < len(trajs):
                states = trajs[t]["states"]
                X = [s[0] for s in states]
                Y = [s[1] for s in states]
                ax.plot(X, Y, color=".5", alpha=0.2)
            if t == len(trajs):
                ax.plot(x_sol, y_sol, color="orange")
    elif i == Tstep1:
        robot.draw(ax, result["states"][0], facecolor="blue", zorder=4)
    elif i < Tstep1 + Tstep2:
        j = i - Tstep1
        X = result["states"][min(kspeed * j, len(result["states"]) - 1)]
        robot.update(X)
    elif i == Tstep1 + Tstep2:
        # draw trajectory
        ax.plot(x_solopt, y_solopt, color="green")
    else:
        j = i - Tstep1 - Tstep2
        X = result_opt["states"][min(kspeed * j, len(result_opt["states"]) - 1)]
        robot.update(X)


extra_good = 10
extra_goal = 5
Tstep1 = int(len(trajs) / pack) + extra_good
Tstep2 = int(len(result["states"]) / kspeed) + extra_goal
Tstep3 = int(len(result_opt["states"]) / kspeed) + extra_goal

speed = 1
if True:
    # write my own

    dir_out = "/tmp/dynoplan/drawing_v2/"
    Path(dir_out).mkdir(parents=True, exist_ok=True)
    #

    for t in range(Tstep1 + Tstep2 + Tstep3):
        animate_func(t)
        plt.savefig(f"{dir_out}/fig_{t:04}.png")

    sys.exit()

    anim = animation.FuncAnimation(
        fig, animate_func, frames=Tstep1 + Tstep2 + Tstep3, interval=10, blit=False
    )
    speed = 1
    DPI = 200

    out = "/tmp/search_only.mp4"
    anim.save(out, "ffmpeg", fps=10 * speed, dpi=DPI)
    print("saved to ", out)


sys.exit()


# plt.show()


# create the video of the idbastar solution on top

print("video with solution")

fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_aspect("equal", adjustable="box")
plt.axis("off")
fig.tight_layout()

viewer.view_problem(ax, env, env_name=filename_env)
plot_search_tree(ax, trajs, result)

robot = car_with_trailer_viewer.Robot()
robot.draw(ax, result["states"][0], facecolor="blue", zorder=100)


def animate_func(i):
    """ """
    X = result["states"][min(i, len(result["states"]) - 1)]
    return robot.update(X)


if False:
    Tstep = len(result["states"]) + 2
    anim = animation.FuncAnimation(
        fig,
        animate_func,
        frames=Tstep,
        interval=10,
        blit=False,
    )
    DPI = 200
    out = "/tmp/search_and_solution.mp4"
    anim.save(out, "ffmpeg", fps=10 * speed, dpi=DPI)
    print("saved to ", out)


print("video with opt ")

fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_aspect("equal", adjustable="box")
plt.axis("off")
fig.tight_layout()

viewer.view_problem(ax, env, env_name=filename_env)
plot_search_tree(ax, trajs, result)

robot = car_with_trailer_viewer.Robot()
robot.draw(ax, result_opt["states"][0], facecolor="blue", zorder=100)
# robot.draw_minimal(ax, result_opt["states"], color="blue")

x_sol = [X[0] for X in result_opt["states"]]
y_sol = [X[1] for X in result_opt["states"]]
ax.plot(x_sol, y_sol, color="green")


def animate_func(i):
    """ """
    X = result_opt["states"][min(i, len(result_opt["states"]) - 1)]
    return robot.update(X)


Tstep = len(result_opt["states"]) + 2
if True:
    anim = animation.FuncAnimation(
        fig,
        animate_func,
        frames=Tstep,
        interval=10,
        blit=False,
    )
    DPI = 200
    out = "/tmp/search_and_solution_opt.mp4"
    anim.save(out, "ffmpeg", fps=10 * speed, dpi=DPI)
    print("saved to ", out)
