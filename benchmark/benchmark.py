import subprocess
import yaml
from typing import List, Any
import numpy as np
import pathlib
from typing import Tuple
import shutil
import os

from multiprocessing import Pool
import multiprocessing
import uuid
from pathlib import Path
import csv
import pandas

import argparse
import os
import shutil
from scipy.interpolate import interp1d


D_key_to_nice_name = [
    ("swing_down_easy", "half swing down"),
    ("move_with_up", "move v0"),
    ("move_with_down", "move v1"),
    ("swing_down", "swing down"),
    # ("down", "swing down"),
    # ("swing_up_obs",  "swing up obstacles v0"),
    # ("swing_up_obs_hard",  "swing up obstacles v1"),
    ("quad2d_recovery_wo_obs", "recovery"),
    ("quad2d_recovery_obs", "recovery obstacles"),
    ("quad_bugtrap", "bugtrap"),
    ("quad_obs_column", "column"),
    ("up obs", "swing up obstacles"),
    ("fall_through", "hole"),
    ("window_easy", "column"),
    ("window_hard", "small window"),
    ("acrobot", "Acrobot"),
    ("car_first_order_with_1_trailers_0", "Trailer"),
    ("quad2dpole", "Rotor Pole"),
    ("quad2d", "Planar Rotor"),
    ("recovery_with_obs", "recovery obstacles"),
    ("wall_0", "wall"),
    ("quad_one_obs", "obstacle"),
    ("_easy", ""),
    ("quadrotor_0", "Quadcopter Force"),
    ("quadrotor_ompl", "Quadcopter Thrust"),
    ("uni1_0", "Unicycle 1 v0"),
    ("uni1_1", "Unicycle 1 v1"),
    ("uni1_2", "Unicycle 1 v2"),
    ("uni2_0", "Unicycle 2"),
    ("swing_up_empty", "swing up"),
    ("bugtrap_0", "bugtrap"),
    ("kink_0", "kink"),
    ("empty_0", "empty v0"),
    ("empty_1", "empty v1"),
    ("recovery_wo_obs", "recovery"),
    # ("up_obs", "swing up obstacles"),
    ("swing_up_obs_hard", "swing up obstacles v1"),
    ("swing_up_obs", "swing up obstacles v0"),
    ("up_obs", "swing up obstacles"),
]


def generate_texpdf(filename_tex: str) -> None:
    lines = [
        r"\documentclass{standalone}",
        r"\usepackage{amsmath}",
        r"\usepackage{amsfonts}",
        r"\usepackage{siunitx}",
        r"\usepackage{booktabs}" r"\begin{document}",
        r"\input{" + filename_tex + "}",
        r"\end{document}",
    ]

    print("writing to ", filename_tex + ".tex")
    with open(filename_tex + ".tex", "w") as f:
        f.writelines(lines)

    pathlib.Path("/tmp/dynoplan/").mkdir(parents=True, exist_ok=True)
    pathlib.Path("/tmp/dynoplan/tex").mkdir(parents=True, exist_ok=True)
    f_stdout = open("/tmp/dynoplan/latexmk_out.log", "w")
    f_stderr = open("/tmp/dynoplan/latexmk_err.log", "w")

    cmd = f"latexmk -f -pdf -output-directory=/tmp/dynoplan/tex/ {filename_tex}.tex".split()

    #     /tmp/dynoplan/tex/
    # #
    # #
    # #         summary_timeopt_2023-07-07--10-57-08.tex.pdf
    # #
    # #     filename_pdf, '/tmp/tmp_compare.pdf')
    #

    print("running cmd: ", " ".join(cmd))
    subprocess.run(cmd, stdout=f_stdout, stderr=f_stderr)

    f_stdout.close()
    f_stderr.close()

    p = Path(filename_tex)
    pdf_name = "/tmp/dynoplan/tex/" + str(p.stem) + ".tex.pdf"

    print(f"copy  {pdf_name} to {filename_tex + '.pdf'}")
    shutil.copy(pdf_name, filename_tex + ".pdf")

    tmp_location = "/tmp/dynoplan/table_tex.pdf"

    print(f"copy  {pdf_name} to {tmp_location}")
    shutil.copy(pdf_name, tmp_location)


print_lock = multiprocessing.Lock()


unsolved_num = 999
all_problems = [
    "unicycle_first_order_0/parallelpark_0",
    "unicycle_first_order_0/bugtrap_0",
    "unicycle_first_order_0/kink_0",
    "unicycle_first_order_1/kink_0",
    "unicycle_first_order_2/wall_0",
    "unicycle_second_order_0/parallelpark_0",
    "unicycle_second_order_0/bugtrap_0",
    "unicycle_second_order_0/kink_0",
    "car_first_order_with_1_trailers_0/bugtrap_0",
    "car_first_order_with_1_trailers_0/parallelpark_0",
    "car_first_order_with_1_trailers_0/kink_0",
    "quad2d/empty_0",
    "quad2d/quad_obs_column",
    "quad2d/quad2d_recovery_wo_obs",
    "quadrotor_0/empty_0_easy",
    "quadrotor_0/recovery",
    "quadrotor_0/quad_one_obs",
    "acrobot/swing_up_empty",
    "acrobot/swing_down_easy",
    "acrobot/swing_down",
    "car_first_order_with_1_trailers_0/easy_0",
]


benchmark_problems = [
    # "unicycle_first_order_0/parallelpark_0",
    "unicycle_first_order_0/bugtrap_0",
    "unicycle_first_order_0/kink_0",
    # "unicycle_first_order_1/kink_0",
    "unicycle_first_order_2/wall_0",
    # "unicycle_second_order_0/kink_0",
    "unicycle_second_order_0/parallelpark_0",
    "unicycle_second_order_0/bugtrap_0",
    "car_first_order_with_1_trailers_0/kink_0",
    # "car_first_order_with_1_trailers_0/bugtrap_0",
    # "car_first_order_with_1_trailers_0/parallelpark_0",
    # "quad2d/empty_0",
    # "quad2d/empty_1",
    # "quad2d/quad_obs_column",
    "quad2d/quad_bugtrap",
    # "quad2d/quad2d_recovery_wo_obs",
    "quad2d/quad2d_recovery_obs",
    # "quad2d/fall_through",
    # "quad2dpole/move_with_down",
    # "quad2dpole/move_with_up",
    # "quad2dpole/up",
    # "quad2dpole/down",
    "quad2dpole/up_obs",
    # "quad2dpole/window_easy",
    # "quad2dpole/window",
    "quad2dpole/window_hard",
    "acrobot/swing_up_empty",
    # "acrobot/swing_up_obs",
    "acrobot/swing_up_obs_hard",
    # "acrobot/swing_down_easy",
    # "acrobot/swing_down",
    #
    # #  QUADROTOR_0
    # "quadrotor_0/empty_0_easy",
    # "quadrotor_0/empty_1_easy",
    # "quadrotor_0/window",
    "quadrotor_0/recovery",
    "quadrotor_0/recovery_with_obs",
    # "quadrotor_0/quad_one_obs",
    # # quadrotor_ompl
    # "quadrotor_ompl/empty_0_easy",
    # "quadrotor_ompl/empty_1_easy",
    "quadrotor_ompl/window",
    # "quadrotor_ompl/recovery",
    # "quadrotor_ompl/quad_one_obs",
    # "quadrotor_ompl/recovery_with_obs"
]


parser = argparse.ArgumentParser()
parser.add_argument("-m", "--mode")
parser.add_argument("-bc", "--bench_cfg")
parser.add_argument("-d", "--dynamics")
parser.add_argument("-f", "--file_in")

args = parser.parse_args()

print(args.__dict__)

mode = args.mode
bench_cfg = args.bench_cfg
file_in = args.file_in
dynamics = args.dynamics

MAX_TIME_PLOTS = 180


do_compare = False
do_benchmark = False
do_debug = False
do_vis_primitives = False
do_bench_time = False
do_fancy_table = False
do_bench_search = False
do_study = False


if mode == "study":
    do_study = True

if mode == "bench_time":
    do_bench_time = True

if mode == "bench_search":
    do_bench_search = True

if mode == "compare":
    do_compare = True

elif mode == "bench":
    do_benchmark = True

elif mode == "debug":
    do_debug = True

elif mode == "vis":
    do_vis_primitives = True

elif mode == "fancy":
    do_fancy_table = True

import sys

sys.path.append("..")

# import viewer.viewer_cli as viewer_cli
import matplotlib.pyplot as plt
import matplotlib

from datetime import datetime


def plot_stats(motions: list, robot_model: str, filename: str):
    all_actions = []
    all_states = []
    all_start_states = []
    all_end_states = []
    for m in motions:
        all_actions.extend(m["actions"])
        all_states.extend(m["states"])
        all_start_states.append(m["start"])
        all_end_states.append(m["goal"])

    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages

    all_actions = np.array(all_actions)
    all_states = np.array(all_states)
    all_start_states = np.array(all_start_states)
    all_end_states = np.array(all_end_states)

    print(f"writing pdf to {filename}")
    pp = PdfPages(filename)

    # r = robots.create_robot(robot_type)
    # base_path = "../models/"
    # from motionplanningutils import robot_factory
    # r = robot_factory(base_path + robot_model + ".yaml")

    def get_desc(name: str) -> Tuple[List[str], List[str]]:
        if name.startswith("unicycle1"):
            return (["x", "y", "theta"], ["v", "w"])
        elif name.startswith("unicycle2"):
            return (["x", "y", "theta", "v", "w"], ["a", "aa"])
        else:
            raise NotImplementedError(f"unknown {name}")

    x_desc, u_desc = get_desc(robot_model)

    fig, ax = plt.subplots()
    ax.set_title("Num steps: ")
    ax.hist([len(m["actions"]) for m in motions])
    pp.savefig(fig)
    plt.close(fig)

    for k, a in enumerate(u_desc):
        fig, ax = plt.subplots()
        ax.set_title("Action: " + a)
        ax.hist(all_actions[:, k])
        pp.savefig(fig)
        plt.close(fig)

    for k, s in enumerate(x_desc):
        fig, ax = plt.subplots()
        ax.set_title("state: " + s)
        ax.hist(all_states[:, k])
        pp.savefig(fig)
        plt.close(fig)

    for k, s in enumerate(x_desc):
        fig, ax = plt.subplots()
        ax.set_title("start state: " + s)
        ax.hist(all_start_states[:, k])
        pp.savefig(fig)
        plt.close(fig)

    for k, s in enumerate(x_desc):
        fig, ax = plt.subplots()
        ax.set_title("end state: " + s)
        ax.hist(all_end_states[:, k])
        pp.savefig(fig)
        plt.close(fig)

    pp.close()


def create_empty_env(start: List, goal: List, robot_model: str):
    if robot_model.startswith("quadrotor_0"):
        env_max = [2.0, 2.0, 2.0]
        env_min = [-2.0, -2.0, -2.0]
    elif robot_model.startswith("acrobot"):
        env_min = [-2.5, -2.5]
        env_max = [2.5, 2.5]
    else:
        env_min = [-2, -2]
        env_max = [2, 2]

    env = {
        "environment": {"min": env_min, "max": env_max, "obstacles": []},
        "robots": [
            {
                "type": robot_model,
                "start": start,
                "goal": goal,
            }
        ],
    }
    return env


color_map = {
    "sst_v0": "red",
    "geo_v0": "green",
    "geo_v1": "cyan",
    "idbastar_v0": "blue",
    "idbastar_v0_heu0": "lightgreen",
    "idbastar_v0_heu1": "yellow",
    "idbastar_v0_heuNO": "turquoise",
    "idbastar_v0_rand": "magenta",
    "idbastar_tmp": "deeppink",
    "idbastar_v0_mpcc": "black",
    "idbastar_v0_search": "orange",
    "idbastar_v0_OptPRIM": "skyblue",
    "idbastar_v0_fixedtime": "chocolate",
    "idbastar_v0_freetime": "darkgray",
    "idbastar_v0_mpc": "lawngreen",
    "idbastar_v0_mpcc": "hotpink",
    "idbastar_v0_search": "orangered",
}


import matplotlib.pyplot as plt


def get_cmap(n, name="hsv"):
    """Returns a function that maps each index in 0, 1, ..., n-1 to a distinct
    RGB color; the keyword argument name must be a standard mpl colormap name."""
    return plt.cm.get_cmap(name, n)


# Usage in your pseudo-code snippet in the question:
# cmap = get_cmap(len(data))
# for i, (X, Y) in enumerate(data):
#    scatter(X, Y, c=cmap(i))


def get_config(alg: str, dynamics: str, problem: str):
    base_path_algs = "../benchmark/config/algs/"
    file = base_path_algs + alg + ".yaml"

    print(f"loading {file}")
    with open(file, "r") as f:
        data = yaml.load(f, Loader=yaml.CLoader)
    # print("data is:\n", data)

    cfg = {}
    if "reference" in data:
        # I have to load another configuration as default
        print("there is a reference!")
        reference = data.get("reference")
        print(f"reference {reference}")
        cfg = get_config(reference, dynamics, problem)
        print("reference cfg is: ", cfg)

    print(f"dynamics {dynamics}")
    print(f"problem is {problem}")

    cfg_default = data.get("default")
    cfg_dynamics = data.get(dynamics, {}).get("default", {})
    __problem = Path(problem).stem

    cfg_problem = data.get(dynamics, {}).get(__problem, {})

    print(f"cfg_default {cfg_default}")
    print(f"cfg_dynamics {cfg_dynamics}")
    print(f"cfg_problem {cfg_problem}")

    for k, v in cfg_default.items():
        cfg[k] = v

    for k, v in cfg_dynamics.items():
        cfg[k] = v

    for k, v in cfg_problem.items():
        cfg[k] = v

    return cfg


def solve_problem_time(env: str, guess: str, alg: str, out: str) -> List[str]:
    # load params

    problem = env

    # get the dynamics
    with open(problem, "r") as f:
        data_problem = yaml.load(f, Loader=yaml.CLoader)
    print(f"data_problem {data_problem}")

    dynamics = data_problem["robots"][0]["type"]
    print(f"dynamics {dynamics}")

    cfg = get_config(alg, dynamics, problem)

    print("merged cfg\n", cfg)

    cfg_out = out + ".cfg.yaml"

    with open(cfg_out, "w") as f:
        yaml.dump(cfg, f)

    cmd = [
        "./main_optimization",
        "--env_file",
        problem,
        "--init_file",
        guess,
        "--results_file",
        out,
        "--cfg",
        cfg_out,
        "--models_base_path",
        "../dynobench/models/",
    ]

    return cmd


def solve_problem_search(env: str, alg: str, out: str) -> List[str]:
    problem = env

    with open(problem, "r") as f:
        data_problem = yaml.load(f, Loader=yaml.CLoader)

    print(f"data_problem {data_problem}")

    dynamics = data_problem["robots"][0]["type"]
    print(f"dynamics {dynamics}")

    cfg = get_config(alg, dynamics, problem)

    print("merged cfg\n", cfg)

    cfg_out = out + ".cfg.yaml"

    with open(cfg_out, "w") as f:
        yaml.dump(cfg, f)

    cmd = [
        "./main_dbastar",
        "--env_file",
        problem,
        "--results_file",
        out,
        "--cfg",
        cfg_out,
        "--models_base_path",
        "../dynobench/models/",
    ]

    return cmd


def solve_problem_with_alg(
    problem: str, alg: str, out: str, timelimit: float
) -> List[str]:
    # load params

    # get the dynamics
    with open(problem, "r") as f:
        data_problem = yaml.load(f, Loader=yaml.CLoader)

    print(f"data_problem {data_problem}")

    dynamics = data_problem["robots"][0]["type"]
    print(f"dynamics {dynamics}")

    cfg = get_config(alg, dynamics, problem)

    print("merged cfg\n", cfg)

    cfg_out = out + ".cfg.yaml"

    with open(cfg_out, "w") as f:
        yaml.dump(cfg, f)

    if alg.startswith("sst"):
        cmd = [
            "./main_sst",
            "--env_file",
            problem,
            "--results_file",
            out,
            "--timelimit",
            str(timelimit),
            "--cfg",
            cfg_out,
        ]
    elif alg.startswith("geo"):
        cmd = [
            "./main_rrt_to",
            "--env_file",
            problem,
            "--results_file",
            out,
            "--timelimit",
            str(timelimit),
            "--cfg",
            cfg_out,
            "--models_base_path",
            "../dynobench/models/",
        ]

    elif alg.startswith("idbastar"):
        # motions_file = get_motions_file(problem)
        cmd = [
            "./main_idbastar",
            "--env_file",
            problem,
            "--results_file",
            out,
            "--timelimit",
            str(timelimit),
            "--cfg",
            cfg_out,
            "--models_base_path",
            "../dynobench/models/",
        ]

    else:
        raise NotImplementedError()

    return cmd
    # print("**\n**\nRUNNING cpp\n**\n", *cmd, "\n", sep=" ")
    # subprocess.run(cmd)
    # print("**\n**\nDONE RUNNING cpp\n**\n")


class Experiment:
    def __init__(self, path, problem, alg, guess=""):
        self.path = path
        self.problem = problem
        self.alg = alg
        self.guess = guess

    def __str__(self):
        return f"path:{self.path}\n problem:{self.problem}\n alg:{self.alg}\n guess:{self.guess}"

    def to_dict(self):
        return {
            "path": self.path,
            "problem": self.problem,
            "alg": self.alg,
            "guess": self.guess,
        }

    def from_dict(self, D):
        self.path = D["path"]
        self.problem = D["problem"]
        self.alg = D["alg"]
        self.guess = D["guess"]


redirect_output = True


def run_cmd(cmd: str):
    print_lock.acquire()
    print("**\n**\nRUNNING cpp\n**\n", *cmd, "\n", sep=" ")
    print_lock.release()

    if redirect_output:
        id = str(uuid.uuid4())[:7]
        stdout_name = f"/tmp/dynoplan/stdout/stdout-{id}.log"
        stderr_name = f"/tmp/dynoplan/stderr/stderr-{id}.log"

        directories = ["/tmp/dynoplan/stdout/", "/tmp/dynoplan/stderr/"]

        for d in directories:
            pathlib.Path(d).mkdir(parents=True, exist_ok=True)

        print_lock.acquire()
        print(
            "***\n",
            *cmd,
            "\n",
            f"stderr_name: {stderr_name}\nstdout_name: {stdout_name}\n****",
        )
        print_lock.release()
        f_stdout = open(stdout_name, "w")
        f_stderr = open(stderr_name, "w")
        subprocess.run(cmd, stdout=f_stdout, stderr=f_stderr)
        f_stdout.close()
        f_stderr.close()
    else:
        subprocess.run(cmd)
    print_lock.acquire()
    print("**\n**\nDONE RUNNING cpp\n**\n")
    print_lock.release()


def compare_search(files: List[str], interactive: bool = False):
    print("calling compare_search:")
    print(f"files: {files}")

    # load
    datas = []
    for file in files:
        print("loading: ", file)
        with open(file, "r") as f:
            data = yaml.load(f, Loader=yaml.CLoader)
            datas.append(data)
    # print("datas\n", datas)

    # print("artificially adding the problem...")
    # for data in datas:
    #     data["problem"] = "unicycle_first_order_0/bugtrap_0"
    # print(datas)

    # organize by problem

    D = {}
    fields = [
        "problem",
        "alg",
        "time_search_mean",
        "time_search_std",
        "cost_mean",
        "cost_std",
        "expands_mean",
        "expands_std",
        "success_rate",
    ]

    fields.sort()

    reduced_data = []
    for d in datas:
        # take only some fields
        dd = {}
        for field in fields:
            dd[field] = d[field]
        reduced_data.append(dd)

    # save as csv file

    # id = str(uuid.uuid4())[:7]
    now = datetime.now()  # current date and time
    date_time = now.strftime("%Y-%m-%d--%H-%M-%S")
    filename_csv = f"../results_new_search/summary/summary_search_{date_time}.csv"
    pathlib.Path(filename_csv).parent.mkdir(parents=True, exist_ok=True)

    # log file

    filename_csv_log = filename_csv + ".log"

    with open(filename_csv_log, "w") as f:
        dd = {
            "input": files,
            "output": filename_csv,
            "date": date_time,
            "hostname": os.uname()[1],
        }
        yaml.dump(dd, f)

        # input

        # f.writelines(files)
        # f.write("---")
        # f.write(filename_csv)

    print("saving reduced data to:", filename_csv)
    with open(filename_csv, "w") as f:
        writer = csv.writer(f)
        writer.writerow(fields)
        for dictionary in reduced_data:
            writer.writerow(dictionary.values())

    copy_to = "/tmp/tmp_reduced_data.csv"
    print(f"copy_to { copy_to }")
    shutil.copy(filename_csv, copy_to)

    # create a new table using pandas

    # pandas frame look like:
    #                  alg  cost_mean  cost_std  expands_mean  expands_std                                 problem  success_rate  time_search_mean  time_search_std
    # 0    dbastar_v0_heu0    3.79995  0.000000         21.00     0.000000   unicycle_first_order_0/parallelpark_0           1.0          2.277654         0.045963
    # 1    dbastar_v0_heu1    3.79995  0.000000         20.00     0.000000
    # unicycle_first_order_0/parallelpark_0           1.0          4.560882
    # 0.062285

    # all_problems = set([data["problem"] for data in reduced_data])
    df = pandas.DataFrame.from_dict(reduced_data)
    problems = df["problem"].unique().tolist()
    problems = sorted(list(problems))

    algs = ["dbastar_v0_heu0", "dbastar_v0_heu1", "dbastar_v0_heuNO"]

    if set(algs) != set(df["alg"].unique().tolist()):
        raise RuntimeError("must have three algs for doing the table")

    keys = ["expands_mean", "time_search_mean"]

    n_id = [r"n  { \scriptsize $[\cdot 10^3]$ }", r"t [s]"]
    header = [
        r"\begin{tabular}{llcccccc}",
        r"\toprule",
        # r"&   \multicolumn{2}{c}{eucl} & \multicolumn{2}{c}{roadmap} & \multicolumn{2}{c}{blind} \\",
        #
        r" & &   \multicolumn{2}{c}{Euclidean} & \multicolumn{2}{c}{Roadmap} & \multicolumn{2}{c}{Blind} \\",
        # r"\cmidrule(lr){2-3}\cmidrule(lr){4-5}\cmidrule(lr){6-7}",
        r"\cmidrule(lr){3-4}\cmidrule(lr){5-6}\cmidrule(lr){7-8}",
        r" Dynamics & Instance & " + " & ".join(n_id * 3) + r"\\",
        r"\midrule",
    ]
    lines = []

    def is_best(val: float, key: str, problem: str, algs: List[str]):
        delta = 1e-10
        for alg in algs:
            b = float(
                df.loc[(df["problem"] == problem) & (df["alg"] == alg)][key].iloc[0]
            )
            if val > b + delta:
                return False
        return True

    separate_dynamics_scenario = True

    scale_data = True
    scale_factor = 1e-3

    for problem in problems:
        line = []
        ll = problem.split("/")
        dynamics = ll[0]
        scenario = ll[1]
        if separate_dynamics_scenario:
            line.append(dynamics)
            line.append(scenario)
        else:
            line.append(problem)

        for alg in algs:
            for key in keys:
                if (
                    float(
                        df.loc[(df["problem"] == problem) & (df["alg"] == alg)][
                            "success_rate"
                        ].iloc[0]
                    )
                    < 1
                ):
                    print("warning not solved", algs, key, problem)
                    tt = r"-"
                else:
                    val = float(
                        df.loc[(df["problem"] == problem) & (df["alg"] == alg)][
                            key
                        ].iloc[0]
                    )
                    is_bold = is_best(val, key, problem, algs)

                    if scale_data:
                        val = val * scale_factor

                    if is_bold:
                        tt = r"\textbf{" + f"{val:.1f}" + "}"
                    else:
                        tt = f"{val:.1f}"
                line.append(tt)
        lines.append(line)

    footer = [r"\bottomrule", r"\end{tabular}"]

    filename_tex = f"../results_new_search/summary/summary_search_{date_time}.tex"

    print(f"writing to: {filename_tex}")

    with open(filename_tex, "w") as f:
        for h in header:
            f.write(h)
            f.write("\n")
        for cl in lines:
            # print("cl ", cl)
            _str = format_latex_str(" & ".join(cl))
            f.write(_str)

            f.write(r"\\")
            f.write("\n")
        for fo in footer:
            f.write(fo)
            f.write("\n")

    copy_to = "/tmp/tmp_search.tex"
    shutil.copy(filename_tex, copy_to)
    print(f"copy_to: {copy_to}")

    filename_log_tex = filename_tex + ".log"

    with open(filename_log_tex, "w") as f:
        dd = {
            "input": files,
            "output": filename_tex,
            "date": date_time,
            "hostname": os.uname()[1],
        }
        yaml.dump(dd, f)

    generate_texpdf(filename_tex)

    from matplotlib.backends.backend_pdf import PdfPages

    filename_pdf = f"../results_new_search/plots/plot_search_{date_time}.pdf"
    pathlib.Path(filename_pdf).parent.mkdir(parents=True, exist_ok=True)

    pp = PdfPages(filename_pdf)
    print(f"writing pdf to {filename_pdf}")

    filename_log_pdf = filename_pdf + ".log"

    with open(filename_log_pdf, "w") as f:
        dd = {
            "input": files,
            "output": filename_pdf,
            "date": date_time,
            "hostname": os.uname()[1],
        }

        yaml.dump(dd, f)

    # fig, ax = plt.subplots(2, 1, sharex=True)

    all_problems = set([data["problem"] for data in reduced_data])

    D = {}

    for problem in all_problems:
        D[problem] = [data for data in reduced_data if data["problem"] == problem]

    for problem in all_problems:
        fig, ax = plt.subplots(4, 1, sharex=True)
        fig.suptitle(problem)

        for i, d in enumerate(D[problem]):
            for ff, field in enumerate(["time_search", "cost", "expands"]):
                y = d[field + "_mean"]
                e = d[field + "_std"]
                ax[ff].errorbar(i, y, e, linestyle="None", marker="^", label=d["alg"])

                # y = d["cost_mean"]
                # e = d["cost_std"]
                # ax[1].errorbar(
                #     i,
                #     y,
                #     e,
                #     linestyle='None',
                #     marker='^',
                #     label=d["alg"])

            y = d["success_rate"]

            ax[3].plot([i], [y], "o")

        ax[1].legend()
        ax[0].set_ylabel("time [ms]")
        ax[1].set_ylabel("cost [s]")
        ax[2].set_ylabel("nodes")
        ax[3].set_ylabel("success")

        ax[0].set_xticks([])
        ax[1].set_xticks([])
        ax[2].set_xticks([])
        ax[3].set_xticks([])

        pp.savefig(fig)

        if interactive:
            plt.show()

    pp.close()

    copy_to = "/tmp/tmp_compare_search.pdf"
    shutil.copy(filename_pdf, copy_to)
    print(f"copy to {copy_to}")


def compare_time(
    files: List[str],
    selected_problems: List[str] = [],
    selected_guesses: List[str] = [],
    interactive: bool = False,
):
    print("calling compare:")
    print(f"files {files}")

    file_out_debug = "/tmp/dynoplan/compare_time.yaml"
    Path(file_out_debug).parent.mkdir(parents=True, exist_ok=True)
    with open(file_out_debug, "w") as f:
        yaml.dump({"files": files}, f)

    # load
    datas = []
    for file in files:
        print("loading ", file)
        with open(file, "r") as f:
            data = yaml.load(f, Loader=yaml.CLoader)
            datas.append(data)
    print("datas\n", datas)

    D = {}
    fields = [
        "problem",
        "alg",
        "guess",
        "time_mean",
        "success_rate",
        "time_std",
        "cost_std",
        "cost_mean",
    ]

    fields.sort()

    reduced_data = []
    for d in datas:
        # take only some fields
        dd = {}
        for field in fields:
            dd[field] = d[field]
        reduced_data.append(dd)

    # save as csv file
    now = datetime.now()  # current date and time
    date_time = now.strftime("%Y-%m-%d--%H-%M-%S")
    filename_csv = f"../results_new_search/summary/summary_search_{date_time}.csv"
    pathlib.Path(filename_csv).parent.mkdir(parents=True, exist_ok=True)

    filename_csv_log = filename_csv + ".log"

    with open(filename_csv_log, "w") as f:
        dd = {
            "input": files,
            "output": filename_csv,
            "date": date_time,
            "hostname": os.uname()[1],
        }

        yaml.dump(dd, f)

        # input

        # f.writelines(files)
        # f.write("---")
        # f.write(filename_csv)

    print("saving reduced data to", filename_csv)
    with open(filename_csv, "w") as f:
        writer = csv.writer(f)
        writer.writerow(fields)
        for dictionary in reduced_data:
            writer.writerow(dictionary.values())

    import shutil

    copy_to = "/tmp/tmp_reduced_data.csv"
    shutil.copy(filename_csv, copy_to)
    print(f"copy_to {copy_to}")

    # print("reduced_data")
    # print(reduced_data)

    # lets write some beautiful latex table
    # TODO: what happens if success rate is not high?
    # Lets use .8 as threshold?

    # PLOT
    from matplotlib.backends.backend_pdf import PdfPages

    filename_pdf = f"../results_new_timeopt/plots/plot_time_{date_time}.pdf"
    pathlib.Path(filename_pdf).parent.mkdir(parents=True, exist_ok=True)

    pp = PdfPages(filename_pdf)
    print(f"writing pdf to {filename_pdf}")

    filename_log_pdf = filename_pdf + ".log"

    with open(filename_log_pdf, "w") as f:
        dd = {
            "input": files,
            "output": filename_pdf,
            "date": date_time,
            "hostname": os.uname()[1],
        }

        yaml.dump(dd, f)

    # fig, ax = plt.subplots(2, 1, sharex=True)

    all_problems = set([data["problem"] + "+" + data["guess"] for data in reduced_data])

    all_problems = sorted(list(all_problems))

    if len(selected_problems):
        print("Warning: only chosen problems!!")
        print(all_problems)
        all_problems = sorted(
            list(set(all_problems).intersection(set(selected_problems)))
        )
        print("after")
        print(all_problems)
        # sys.exit(0)

    # with open("/tmp/dynoplan/all_problem_time_opt.yaml", "w") as f:
    #     yaml.dump({"all_problems": all_problems}, f)
    #
    # sys.exit(0)

    D = {}

    for problem in all_problems:
        D[problem] = [
            data
            for data in reduced_data
            if data["problem"] + "+" + data["guess"] == problem
        ]

    for problem in all_problems:
        fig, ax = plt.subplots(3, 1, sharex=True)
        fig.suptitle(problem)

        for i, d in enumerate(D[problem]):
            y = d["time_mean"]
            e = d["time_std"]
            ax[0].errorbar(i, y, e, linestyle="None", marker="^", label=d["alg"])

            y = d["cost_mean"]
            e = d["cost_std"]
            ax[1].errorbar(i, y, e, linestyle="None", marker="^", label=d["alg"])

            y = d["success_rate"]

            ax[2].plot([i], [y], "o")

        ax[1].legend()
        ax[0].set_ylabel("time [ms]")
        ax[1].set_ylabel("cost [s]")
        ax[2].set_ylabel("success")

        ax[0].set_xticks([])
        ax[1].set_xticks([])
        ax[2].set_xticks([])

        pp.savefig(fig)

        if interactive:
            plt.show()

    pp.close()
    copy_to = "/tmp/tmp_compare_time.pdf"
    shutil.copy(filename_pdf, copy_to)
    print(f"copy_to {copy_to}")

    # END PLOT

    algs = [
        # "idbastar_v0_fixedtime",
        "idbastar_v0_freetime",
        "idbastar_v0_search",
        "idbastar_v0_mpc",
        "idbastar_v0_mpcc",
    ]

    df = pandas.DataFrame.from_dict(reduced_data)

    df["time_mean"] = df["time_mean"] / 1000.0

    current_algs = df["alg"].unique().tolist()

    algs_selected_and_current = [i for i in algs if i in current_algs]

    print("df")
    print(df)
    guesses = df["guess"].unique().tolist()
    guesses.sort()  # SORTED

    print("guesses")
    print(guesses)

    with open("/tmp/dynoplan/guesses.yaml", "w") as f:
        yaml.dump({"guesses": guesses}, f)

    if len(selected_guesses):
        print("Warning: only chosen problems!!")
        print(guesses)
        guesses = sorted(list(set(guesses).intersection(set(selected_guesses))))
        print("after")
        print(guesses)

    keys = ["cost_mean", "time_mean"]

    include_num_timestes = True

    if include_num_timestes:
        cols = ["J[s]", "t[s]"]
        header = [
            r"\begin{tabular}{llcccccccccc}",
            r"\toprule",
            r" & & & &  \multicolumn{2}{c}{dt} & \multicolumn{2}{c}{search} & \multicolumn{2}{c}{mpc}",
            r"& \multicolumn{2}{c}{mpcc} \\",
            r"\cmidrule(lr){5-6}\cmidrule(lr){7-8}\cmidrule(lr){9-10}\cmidrule(lr){11-12}",
            r"System & Instance & $\delta$ & K  & " + " & ".join(4 * cols) + r" \\",
            r"\midrule",
        ]
    else:
        raise ValueError("not implemented")
        # header = [
        #     # r"\begin{tabular}{lcccccccc}",
        #     r"\begin{tabular}{lllcccccccc}",
        #     r"\toprule",
        #     r" & & &   \multicolumn{2}{c}{dt} & \multicolumn{2}{c}{search} & \multicolumn{2}{c}{mpc}",
        #     r"& \multicolumn{2}{c}{mpcc} \\",
        #     r"\cmidrule(lr){4-5}\cmidrule(lr){6-7}\cmidrule(lr){8-9}\cmidrule(lr){10-11}",
        #     r"System & Instance & $\delta$   &   c [s] & t [ms] & c [s] & t [ms] & c [s] & t [ms] & c [s] & t [ms] \\",
        #     r"\midrule"]

    footer = [r"\bottomrule", r"\end{tabular}"]

    def is_best(val: float, key: str, problem: str, algs: List[str]):
        delta = 1e-10
        for alg in algs:
            b = float(
                df.loc[(df["guess"] == problem) & (df["alg"] == alg)][key].iloc[0]
            )
            if val > b + delta:
                return False
        return True

    lines = []
    success_rate_limit = 0.8

    df.loc[df.success_rate < success_rate_limit, "cost_mean"] = np.nan
    df.loc[df.success_rate < success_rate_limit, "time_mean"] = np.nan

    separate_dynamics_scenario = True

    for guess in guesses:
        line = []
        # get the number of time steps

        with open(f"../benchmark_initguess/{guess}.yaml", "r") as f:
            dd = yaml.load(f, Loader=yaml.CLoader)
        if "result" in dd:
            num_time_steps = len(dd["result"][0]["actions"])
        else:
            num_time_steps = len(dd["actions"])

        print("num_time_steps", num_time_steps)

        def extract_delta(s: str):
            """
            Example:
            delta_05_v0
            Output:
            0.5
            """
            s = s.split("_")[1]
            return float(s) / 10

            raise NotImplementedError

        if separate_dynamics_scenario:
            ll = guess.split("/")
            dynamics = ll[0]
            scenario = ll[1]
            delta = str(extract_delta(ll[2]))

            line.append(dynamics)
            line.append(scenario)
            line.append(delta)
            if include_num_timestes:
                line.append(str(num_time_steps))

        else:
            line.append(guess)

        for alg in algs:
            for key in keys:
                print(guess, alg, key)

                if alg not in current_algs:
                    tt = "-"
                else:
                    success_rate = float(
                        df.loc[(df["guess"] == guess) & (df["alg"] == alg)][
                            "success_rate"
                        ].iloc[0]
                    )
                    if success_rate < 0.8:
                        print("WARNING", "success rate is ", success_rate)

                    val = float(
                        df.loc[(df["guess"] == guess) & (df["alg"] == alg)][key].iloc[0]
                    )
                    is_bold = is_best(val, key, guess, algs_selected_and_current)

                    if is_bold:
                        tt = r"\textbf{" + f"{val:.1f}" + "}"
                    else:
                        tt = f"{val:.1f}"
                line.append(tt)
        lines.append(line)

    filename_tex = f"../results_new_timeopt/summary/summary_timeopt_{date_time}.tex"
    pathlib.Path(filename_tex).parent.mkdir(parents=True, exist_ok=True)

    print(f"writing to {filename_tex}")

    with open(filename_tex, "w") as f:
        for h in header:
            f.write(h)
            f.write("\n")
        for cl in lines:
            # print("cl ", cl)
            _str = format_latex_str(" & ".join(cl))
            f.write(_str)

            f.write(r"\\")
            f.write("\n")
        for fo in footer:
            f.write(fo)
            f.write("\n")

    copy_to = "/tmp/tmp_search.tex"
    shutil.copy(filename_tex, copy_to)
    print(f"copy_to {copy_to}")

    filename_log_tex = filename_tex + ".log"

    with open(filename_log_tex, "w") as f:
        dd = {
            "input": files,
            "output": filename_tex,
            "date": date_time,
            "hostname": os.uname()[1],
        }

        yaml.dump(dd, f)

    generate_pdf_latex = True

    if generate_pdf_latex:
        generate_texpdf(filename_tex)

    # create_latex_table(filename_csv)
    #
    # # check
    # print("checking data")
    # with open(filename_csv, 'r') as myFile:
    #     print(myFile.read())
    #
    # for problem in all_problems:
    #     print(f"problem {problem}")
    #     # check if data belongs to this problem
    #     _datas = []
    #     for data in datas:
    #         print("**")
    #         print(data["problem"])
    #         print(problem)
    #         print("**")
    #         print("**")
    #         if data["problem"] == problem:
    #             print("match!")
    #             _datas.append(data)
    #     D[problem] = _datas
    # print("D", D)
    #
    # # now print the data!
    #
    # from matplotlib.backends.backend_pdf import PdfPages
    #
    # filename_pdf = f"../results_new/plots/plot_{date_time}.pdf"
    #
    # print(f"writing pdf to {filename_pdf}")
    #
    # filename_log_pdf = filename_pdf + ".log"
    #
    # with open(filename_log_pdf, "w") as f:
    #     dd = {"input": files, "output": filename_pdf}
    #     yaml.dump(dd, f)
    #
    # pp = PdfPages(filename_pdf)
    #
    # for problem in all_problems:
    #
    #     if D[problem]:
    #         fig, ax = plt.subplots(2, 1, sharex=True)
    #         fig.suptitle(problem)
    #         for d in D[problem]:
    #             print("d", d)
    #             alg = d["alg"]
    #             times = d["times"]
    #             success = d["success"]
    #             cost_mean = d["cost_mean"]
    #             cost_std = d["cost_std"]
    #             color = color_map[alg]
    #
    #             ax[0].plot(times, cost_mean, color=color, label=alg)
    #             ax[0].fill_between(times,
    #                                np.array(cost_mean) + np.array(cost_std),
    #                                np.array(cost_mean) - np.array(cost_std),
    #                                facecolor = color,
    #                                alpha = 0.5)
    #             ax[1].plot(times, success, color = color, label = alg)
    #
    #             ax[0].set_xscale('log')
    #             ax[1].set_xscale('log')
    #
    #         ax[1].legend()
    #         ax[0].set_ylabel("cost")
    #         ax[1].set_xlabel("time [s]")
    #         ax[1].set_ylabel("success %")
    #         ax[1].set_ylim(-10, 110)
    #         ax[1].set_xlim(.1, MAX_TIME_PLOTS)
    #
    #         if (interactive):
    #             plt.show()
    #
    #         pp.savefig(fig)
    #         plt.close(fig)
    # pp.close()
    #
    # import shutil
    # shutil.copy(filename_pdf, '/tmp/tmp_compare.pdf')
    #


def __benchmark_search(bench_cfg: str) -> List[str]:
    with open(bench_cfg) as f:
        data = yaml.load(f, Loader=yaml.CLoader)

    print("bench cfg")
    print(data)

    n_cores = data["n_cores"]
    if n_cores == -1:
        n_cores = int(multiprocessing.cpu_count() / 2)

    print(f"problems ", data["problems"])
    print(f"algs", data["algs"])

    base_path_problem = "../dynobench/envs/"
    folder_results = "../results_new_search/"

    now = datetime.now()  # current date and time
    date_time = now.strftime("%Y-%m-%d--%H-%M-%S")
    cmds = []
    paths = []

    experiments = []

    for problem in data["problems"]:
        for alg in data["algs"]:
            path_name = problem
            print(f"path_name: {path_name}")
            path = folder_results + path_name + "/" + alg + "/" + date_time
            paths.append(path)
            pathlib.Path(path).mkdir(parents=True, exist_ok=True)
            experiments.append(Experiment(path=path, problem=problem, alg=alg))
            for i in range(data["trials"]):
                out = path + f"/run_{i}_out.yaml"
                cmd = solve_problem_search(
                    base_path_problem + problem + ".yaml", alg, out
                )
                cmds.append(cmd)
    print("commands are: ")
    for i, cmd in enumerate(cmds):
        print(i, cmd)
    print("***")
    print("commands in cli format: ")
    for i, cmd in enumerate(cmds):
        print(*cmd, sep=" ")
        print("\n")

    print(f"Start a pool with {n_cores}:")
    with Pool(n_cores) as p:
        p.map(run_cmd, cmds)
    print("Pool is DONE")

    fileouts = []

    experiments_outs = [e.to_dict() for e in experiments]
    id = str(uuid.uuid4())[:7]
    filename_experimentes_out = f"/tmp/dynoplan/experimentes_info_{id}.yaml"
    pathlib.Path(filename_experimentes_out).parent.mkdir(parents=True, exist_ok=True)
    with open(filename_experimentes_out, "w") as f:
        yaml.dump(experiments_outs, f)

    for experiment in experiments:
        print(f"experiment: {experiment}")
        fileout, _ = analyze_search(
            experiment.path, experiment.problem, experiment.alg, False
        )
        fileouts.append(fileout)

    return fileouts


def benchmark_search(bench_cfg: str) -> None:
    # open the bench_cfg file

    with open(bench_cfg) as f:
        data = yaml.load(f, Loader=yaml.CLoader)

    if "files" in data:
        print(f"{bench_cfg} is a file list -- only analyze them")
        fileouts = data["files"]

    if "input" in data:
        print(f"{bench_cfg} is a file list -- only analyze them")
        fileouts = data["input"]

    else:
        print(f"{bench_cfg} is a bench file for doing experiments")
        fileouts = __benchmark_search(bench_cfg)

    # data_file = "../bench/search_results.yaml"
    # with open(data_file) as f:
    #     d = yaml.safe_load(f)
    #
    # print(d)

    # fileouts = [
    #     '../results_new_search/unicycle_first_order_0/bugtrap_0/dbastar_v0_heu0/2023-06-22--20-40-26/report.yaml',
    #     '../results_new_search/unicycle_first_order_0/bugtrap_0/dbastar_v0_heu1/2023-06-22--20-40-26/report.yaml',
    #     '../results_new_search/unicycle_first_order_0/bugtrap_0/dbastar_v0_heuNO/2023-06-22--20-40-26/report.yaml',
    #     '../results_new_search/unicycle_second_order_0/bugtrap_0/dbastar_v0_heu0/2023-06-22--20-40-26/report.yaml',
    #     '../results_new_search/unicycle_second_order_0/bugtrap_0/dbastar_v0_heu1/2023-06-22--20-40-26/report.yaml',
    #     '../results_new_search/unicycle_second_order_0/bugtrap_0/dbastar_v0_heuNO/2023-06-22--20-40-26/report.yaml',
    #     '../results_new_search/quad2d/quad_bugtrap/dbastar_v0_heu0/2023-06-22--20-40-26/report.yaml',
    #     '../results_new_search/quad2d/quad_bugtrap/dbastar_v0_heu1/2023-06-22--20-40-26/report.yaml',
    #     '../results_new_search/quad2d/quad_bugtrap/dbastar_v0_heuNO/2023-06-22--20-40-26/report.yaml',
    #     '../results_new_search/quadrotor_0/quad_one_obs/dbastar_v0_heu0/2023-06-22--20-40-26/report.yaml',
    #     '../results_new_search/quadrotor_0/quad_one_obs/dbastar_v0_heu1/2023-06-22--20-40-26/report.yaml',
    #     '../results_new_search/quadrotor_0/quad_one_obs/dbastar_v0_heuNO/2023-06-22--20-40-26/report.yaml']

    compare_search(fileouts)


def __benchmark_opti(bench_cfg: str) -> List[str]:
    with open(bench_cfg) as f:
        data = yaml.load(f, Loader=yaml.CLoader)
    print("bench cfg")
    n_cores = data["n_cores"]

    if n_cores == -1:
        n_cores = int(multiprocessing.cpu_count() / 2)

    print("problems", data["problems"])
    print(f"algs ", data["algs"])

    base_path_problem = "../dynobench/envs/"
    base_guess = "../benchmark_initguess/"
    folder_results = "../results_new_timeopt/"

    now = datetime.now()  # current date and time
    date_time = now.strftime("%Y-%m-%d--%H-%M-%S")
    cmds = []
    paths = []

    experiments = []

    def get_path_name_for_init_guess(env: str, guess: str) -> str:
        env_ = env.split("/")
        guess_ = guess.split("/")
        print(f"guess_ {guess_}")
        out = env_ + [guess_[-1]]
        return "/".join(out)

    for problem in data["problems"]:
        env = problem["env"]
        guess = problem["guess"]
        print(f"env {env}")
        print(f"guess {guess}")
        for alg in data["algs"]:
            path_name = get_path_name_for_init_guess(env, guess)
            print(f"path_name: {path_name}")
            path = folder_results + path_name + "/" + alg + "/" + date_time
            paths.append(path)
            pathlib.Path(path).mkdir(parents=True, exist_ok=True)
            experiments.append(Experiment(path=path, problem=env, alg=alg, guess=guess))
            for i in range(data["trials"]):
                out = path + f"/run_{i}_out.yaml"
                cmd = solve_problem_time(
                    base_path_problem + env + ".yaml",
                    base_guess + guess + ".yaml",
                    alg,
                    out,
                )
                cmds.append(cmd)
    print("commands are: ")
    for i, cmd in enumerate(cmds):
        print(i, cmd)
    print("***")
    print("commands in cli format: ")
    for i, cmd in enumerate(cmds):
        print(*cmd, sep=" ")
        print("\n")

    print("commands in cli format: ")
    for i, cmd in enumerate(cmds):
        print(*cmd, sep=" ")
        print("\n")

    print(f"Start a pool with {n_cores}:")
    with Pool(n_cores) as p:
        p.map(run_cmd, cmds)
    print("Pool is DONE")

    fileouts = []

    experiments_outs = [e.to_dict() for e in experiments]
    id = str(uuid.uuid4())[:7]
    filename_experimentes_out = f"/tmp/dynoplan/experimentes_info_{id}.yaml"
    pathlib.Path(filename_experimentes_out).parent.mkdir(parents=True, exist_ok=True)
    with open(filename_experimentes_out, "w") as f:
        yaml.dump(experiments_outs, f)

    for experiment in experiments:
        print(f"experiment: {experiment}")
        fileout, _ = analyze_runs_time(
            experiment.path,
            experiment.problem,
            experiment.guess,
            experiment.alg,
            visualize=False,
        )
        fileouts.append(fileout)

    return fileouts


def benchmark_opti(bench_cfg: str) -> None:
    selected_problems = []
    selected_guesses = []
    with open(bench_cfg) as f:
        d = yaml.load(f, Loader=yaml.CLoader)
    if "files" in d:
        print(f"{bench_cfg} is a file list -- only analyze them")
        fileouts = d["files"]
    elif "input" in d:
        print(f"{bench_cfg} is a file list -- only analyze them")
        fileouts = d["input"]
    else:
        print(f"{bench_cfg} is a bench file for doing experiments")
        fileouts = __benchmark_opti(bench_cfg)

    if "selected_problems" in d:
        selected_problems = d["selected_problems"]

    if "selected_guesses" in d:
        selected_guesses = d["selected_guesses"]

    # TODO: unify guess and problems
    print(f"fileouts: { fileouts }")
    print("selected_problems", selected_problems)
    print("selected_guesses", selected_guesses)
    compare_time(fileouts, selected_problems, selected_guesses)

    # compare(fileouts, False)


def __benchmark(bench_cfg: str):
    with open(bench_cfg) as f:
        data = yaml.load(f, Loader=yaml.CLoader)
    print("bench cfg")
    print(data)
    n_cores = data["n_cores"]
    if n_cores == -1:
        n_cores = int(multiprocessing.cpu_count() / 2)

    print("problems", data["problems"])
    print("algs", data["algs"])

    base_path_problem = "../dynobench/envs/"
    folder_results = "../results_new/"

    now = datetime.now()  # current date and time
    date_time = now.strftime("%Y-%m-%d--%H-%M-%S")
    cmds = []
    paths = []

    experiments = []
    for problem in data["problems"]:
        for alg in data["algs"]:
            path = folder_results + problem + "/" + alg + "/" + date_time
            paths.append(path)
            pathlib.Path(path).mkdir(parents=True, exist_ok=True)
            experiments.append(Experiment(path=path, problem=problem, alg=alg))
            for i in range(data["trials"]):
                out = path + f"/run_{i}_out.yaml"
                cmd = solve_problem_with_alg(
                    base_path_problem + problem + ".yaml", alg, out, data["timelimit"]
                )
                cmds.append(cmd)
    print("commands are: ")
    for i, cmd in enumerate(cmds):
        print(i, cmd)
    print("***")
    print("commands in cli format: ")
    for i, cmd in enumerate(cmds):
        print(*cmd, sep=" ")
        print("\n")

    print(f"Start a pool with {n_cores}:")
    with Pool(n_cores) as p:
        p.map(run_cmd, cmds)
    print("Pool is DONE")

    fileouts = []

    experiments_outs = [e.to_dict() for e in experiments]
    id = str(uuid.uuid4())[:7]
    filename_experimentes_out = f"/tmp/dynoplan/experimentes_info_{id}.yaml"
    pathlib.Path(filename_experimentes_out).parent.mkdir(parents=True, exist_ok=True)
    with open(filename_experimentes_out, "w") as f:
        yaml.dump(experiments_outs, f)

    # experiments = []
    # fileouts = []

    # path = "../results_new/unicycle_first_order_0/bugtrap_0/geo_v0/2023-07-06--14-22-28/"
    # experiments.append(
    #     Experiment(
    #         path=path,
    #         problem="unicycle_first_order_0/bugtrap_0",
    #         alg="geo_v0"))

    for experiment in experiments:
        print("experiment")
        print(experiment)

        fileout, _ = analyze_runs(
            experiment.path, experiment.problem, experiment.alg, visualize=False
        )
        fileouts.append(fileout)
    return fileouts

    compare(fileouts, False)

    # basic analysisi of the results?


# TODO: small script to solve all and store the output in the dynpobench, with nice names.
# Lets store the final solution trajectory, the first db-astar and the last db-astar, and the corresponding solved.

# what to do with the other planners?


def benchmark(bench_cfg: str):
    with open(bench_cfg) as f:
        d = yaml.load(f, Loader=yaml.CLoader)

    if "files" in d:
        fileouts = d["files"]
    elif "input" in d:
        fileouts = d["input"]
    else:
        fileouts = __benchmark(bench_cfg)
    compare(fileouts, False)


def study(bench_cfg: str):
    # load bench_cfg
    with open(bench_cfg, "r") as f:
        data = yaml.load(f, Loader=yaml.CLoader)

    if "files" in data:
        files_ = data["files"]
    elif "input" in data:
        files_ = data["input"]
    else:
        fileouts = __benchmark(bench_cfg)
        files_ = [str(Path(file).parent / "run_0_out.yaml") for file in fileouts]
    # print("fileouts")
    # print(fileouts)

    # fileouts = [
    #     '../results_new/unicycle_first_order_0/bugtrap_0/idbastar_v0_analysis_d1/2023-06-22--16-05-15/report.yaml',
    #     '../results_new/unicycle_first_order_0/bugtrap_0/idbastar_v0_analysis_d2/2023-06-22--16-05-15/report.yaml',
    #     '../results_new/unicycle_second_order_0/bugtrap_0/idbastar_v0_analysis_d1/2023-06-22--16-05-15/report.yaml',
    #     '../results_new/unicycle_second_order_0/bugtrap_0/idbastar_v0_analysis_d2/2023-06-22--16-05-15/report.yaml',
    #     '../results_new/quad2d/quad_bugtrap/idbastar_v0_analysis_d1/2023-06-22--16-05-15/report.yaml',
    #     '../results_new/quad2d/quad_bugtrap/idbastar_v0_analysis_d2/2023-06-22--16-05-15/report.yaml']

    # example fileouts
    #

    # fileouts = ['../results_new/unicycle_first_order_0/bugtrap_0/idbastar_v0_analysis_d1/2023-06-22--15-29-54/report.yaml', '../results_new/unicycle_first_order_0/bugtrap_0/idbastar_v0_analysis_d2/2023-06-22--15-29-54/report.yaml', '../results_new/unicycle_second_order_0/bugtrap_0/idbastar_v0_analysis_d1/2023-06-22--15-29-54/report.yaml', '../results_new/unicycle_second_order_0/bugtrap_0/idbastar_v0_analysis_d2/2023-06-22--15-29-54/report.yaml', '../results_new/quad2d/quad_bugtrap/idbastar_v0_analysis_d1/2023-06-22--15-29-54/report.yaml', '../results_new/quad2d/quad_bugtrap/idbastar_v0_analysis_d2/2023-06-22--15-29-54/report.yaml']

    print(files_)
    parse_for_component_analysis(files_)


def compare(files: List[str], interactive: bool = False):
    print("calling compare:")
    print(f"files {files}")

    # load
    datas = []
    for file in files:
        print("loading ", file)
        with open(file, "r") as f:
            data = yaml.load(f, Loader=yaml.CLoader)
            datas.append(data)
    # print("datas\n", datas)
    # print(len(datas))
    # print(datas[0])

    use_only_some_algs = False

    selected_algs = ["idbastar_v0", "sst_tmp", "geo_v1"]
    if use_only_some_algs:
        print("WARNING: using only some algs!")
        print("selected_algs")
        print(selected_algs)
        datas = [d for d in datas if d["alg"] in selected_algs]

    # print("artificially adding the problem...")
    # for data in datas:
    #     data["problem"] = "unicycle_first_order_0/bugtrap_0"
    # print(datas)

    # organize by problem

    D = {}
    fields = [
        "problem",
        "alg",
        "cost_at_01",
        "cost_at_05",
        "cost_at_10",
        "cost_at_20",
        "success_rate",
        "cost_first_solution",
        "time_first_solution",
        "last_cost",
    ]
    fields.sort()

    reduced_data = []
    for d in datas:
        # take only some fields
        dd = {}
        for field in fields:
            dd[field] = d[field]

        reduced_data.append(dd)

    # save as csv file

    # id = str(uuid.uuid4())[:7]
    now = datetime.now()  # current date and time
    date_time = now.strftime("%Y-%m-%d--%H-%M-%S")
    filename_csv = f"../results_new/summary/summary_{date_time}.csv"

    # log file

    filename_csv_log = filename_csv + ".log"
    pathlib.Path(filename_csv_log).parent.mkdir(parents=True, exist_ok=True)

    with open(filename_csv_log, "w") as f:
        dd = {
            "input": files,
            "output": filename_csv,
            "date": date_time,
            "hostname": os.uname()[1],
        }

        yaml.dump(dd, f)

        # input

        # f.writelines(files)
        # f.write("---")
        # f.write(filename_csv)

    print("saving reduced data to", filename_csv)
    with open(filename_csv, "w") as f:
        writer = csv.writer(f)
        writer.writerow(fields)
        for dictionary in reduced_data:
            writer.writerow(dictionary.values())

    import shutil

    copy_to = "/tmp/tmp_reduced_data.csv"
    shutil.copy(filename_csv, copy_to)
    print(f"copy_to: { copy_to} ")

    create_latex_table(filename_csv)

    # check
    # print("checking data")
    # with open(filename_csv, 'r') as myFile:
    #     print(myFile.read())

    all_problems = set([data["problem"] for data in reduced_data])
    all_problems = sorted(list(all_problems))
    print(all_problems)
    # sys.exit(1)

    # print hardcoding the problems!
    # all_problems = [
    #     "quad2dpole/up_obs",
    #     "quadrotor_0/window",
    #     "unicycle_first_order_0/bugtrap_0",
    # ]

    # HARDCODE WHICH PROBLEMS

    # problems =

    for problem in all_problems:
        # print(f"problem {problem}")
        # check if data belongs to this problem
        _datas = []
        for data in datas:
            # print("**")
            # print(data["problem"])
            # print(problem)
            # print("**")
            # print("**")
            if data["problem"] == problem:
                # print("match!")
                _datas.append(data)
        D[problem] = _datas
    # print("D", D)

    # now print the data!

    from matplotlib.backends.backend_pdf import PdfPages

    filename_pdf = f"../results_new/plots/plot_{date_time}.pdf"

    print(f"writing pdf to {filename_pdf}")

    filename_log_pdf = filename_pdf + ".log"

    pathlib.Path(filename_log_pdf).parent.mkdir(parents=True, exist_ok=True)
    with open(filename_log_pdf, "w") as f:
        dd = {
            "input": files,
            "output": filename_pdf,
            "date": date_time,
            "hostname": os.uname()[1],
        }

        yaml.dump(dd, f)

    pp = PdfPages(filename_pdf)

    Dproblem2title = {
        "quad2dpole/up_obs": "Rotor pole - up_obstacles",
        "quadrotor_0/window": "Quadrotor -  window",
        "unicycle_first_order_0/bugtrap_0": "Unicycle1 - bugtrap",
    }
    Dalg2label = {"idbastar_v0": "IDBA*", "sst_v0": "SST*", "geo_v0": "RRT*-TO"}

    counter = 0
    for problem in all_problems:
        counter += 1

        if D[problem]:
            matplotlib.rcParams.update({"font.size": 14})

            plt.rc("text", usetex=True)
            plt.rc("font", family="serif")

            # fig = plt.figure(figsize=(6, 4))
            # ax = fig.add_subplot(2, 1, sharex=True)

            fig = plt.figure(figsize=(5, 4))
            _ax = fig.add_subplot(211)
            _ax2 = fig.add_subplot(212, sharex=_ax)
            ax = [_ax, _ax2]
            # fig.suptitle(problem)
            # fig.set_title(problem)
            for d in D[problem]:
                # print("d", d)
                alg = d["alg"]
                times = d["times"]
                success = d["success"]
                cost_mean = d["cost_mean"]
                # cost_std = d["cost_std"]
                cost_lb = d["cost_lb"]
                cost_ub = d["cost_ub"]
                if alg in color_map:
                    color = color_map[alg]
                else:
                    print("Warning: not in color map -- Using random color")
                    N = 30
                    import random

                    nn = random.randint(0, N - 1)
                    color = get_cmap(N, name="hsv")(nn)
                    color_map[alg] = color

                # ax[0].plot(times, cost_mean, color=color, label=alg)
                ax[0].step(
                    times, cost_mean, color=color, label=Dalg2label[alg], where="post"
                )

                # I want to change

                assert len(cost_mean) == len(cost_lb)

                assert len(cost_mean) == len(cost_ub)

                for i in range(len(cost_mean)):
                    if cost_mean[i] != np.nan and cost_mean[i] < np.inf:
                        if cost_ub[i] == np.nan or cost_ub[i] == np.inf:
                            cost_ub[i] = 100

                            # cost_lb != np.nan && cost_ub != np.nan):

                print("cost_mean", cost_mean)
                print("cost_lb", cost_lb)
                print("cost_ub", cost_ub)

                ax[0].fill_between(
                    times,
                    cost_lb,
                    cost_ub,
                    # np.array(cost_mean) + np.array(cost_std),
                    # np.array(cost_mean) - np.array(cost_std),
                    facecolor=color,
                    alpha=0.5,
                )
                # ax[1].plot(times, success, color=color, label=alg)
                ax[1].step(
                    times, success, color=color, label=Dalg2label[alg], where="post"
                )

                ax[0].set_xscale("log")
                ax[1].set_xscale("log")

            Dproblem2limit = {
                "quad2dpole/up_obs": [0, 30],
                "quadrotor_0/window": [0, 5],
                "unicycle_first_order_0/bugtrap_0": [0, 80],
            }

            ax[0].set_title(Dproblem2title.get(problem, problem))
            if counter == 3:
                ax[1].legend(loc="lower right")

            ax[1].set_xlabel("time [s]")
            if counter == 1:
                ax[0].set_ylabel("cost[s]")
                ax[1].set_ylabel("success %")

            if problem in Dproblem2limit:
                ax[0].set_ylim(Dproblem2limit[problem][0], Dproblem2limit[problem][1])

            ax[1].set_ylim(-10, 110)
            ax[1].set_xlim(0.5, MAX_TIME_PLOTS)

            fig.tight_layout()

            if interactive:
                plt.show()

            pp.savefig(fig)
            plt.close(fig)
    pp.close()

    import shutil

    copy_to = "/tmp/tmp_compare.pdf"
    shutil.copy(filename_pdf, copy_to)
    print(f"copy_to: {copy_to}")


def make_videos(robot: str, problem: str, file: str):
    with open(file, "r") as f:
        data = yaml.load(f, yaml.SafeLoader)

    trajs_opts = data["trajs_opt"]
    trajs_raw = data["trajs_raw"]
    base_name = "vid"
    robot = "unicycle1"
    viewer = viewer_cli.get_robot_viewer(robot)

    for i, traj in enumerate(trajs_opts):
        # create the videos
        filename = f"{base_name}_traj_opt_{i}.mp4"
        viewer.make_video(problem, traj, filename)

    for i, traj in enumerate(trajs_raw):
        filename = f"{base_name}_traj_raw_{i}.mp4"
        viewer.make_video(problem, traj, filename)


def get_cost_evolution(ax, file: str, **kwargs):
    print(f"loading file {file}")
    with open(file, "r") as f:
        data = yaml.load(f, yaml.SafeLoader)

    trajs_opt = data["trajs_opt"]

    time_cost_pairs = []

    best_cost = 1e10

    if trajs_opt is not None:
        for traj in trajs_opt:
            tts = float(traj["time_stamp"]) / 1000
            cs = traj["cost"]
            feas = traj["feasible"]
            if feas:
                print(cs)
                print(best_cost)
                if cs < best_cost:
                    time_cost_pairs.append([tts, cs])
                    best_cost = cs

    if len(time_cost_pairs) == 0:
        print("there is no solution...")
        time_cost_pairs.append([np.nan, np.nan])

    # plot

    ts = [x[0] for x in time_cost_pairs]
    cs = [x[1] for x in time_cost_pairs]

    ts.append(MAX_TIME_PLOTS)
    cs.append(cs[-1])

    ax.step(ts, cs, color=kwargs.get("color", "blue"), alpha=0.3, where="post")
    return time_cost_pairs


def get_av_cost_new(all_costs_np) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    # continue here
    # all_costs_np_ = np.nan_to_num(all_costs_np, nan=float("inf"))
    # median = all_costs_np.median(axis=0)
    num_instances = all_costs_np.shape[0]
    median = []
    median_ub = []
    median_lb = []
    rate_ub = 0.8
    rate_lb = 0.2
    for j in range(all_costs_np.shape[1]):
        column = np.copy(all_costs_np[:, j])
        # how many nan?
        non_nan = np.count_nonzero(~np.isnan(column))
        # print(column)
        # print(non_nan)
        if non_nan >= int(num_instances / 2):
            print("success rate is enough")
            # cc = column[~np.isnan(column)]
            cc = np.nan_to_num(column, nan=float("inf"))
            print("cc", cc)
            cc.sort()

            median.append(cc[max(int(0.5 * (len(cc) - 1)), 0)])
            median_ub.append(cc[max(int(rate_ub * (len(cc) - 1)), 0)])
            median_lb.append(cc[max(int(rate_lb * (len(cc) - 1)), 0)])
            # meda.append(cc.mean())
            # std.append(cc.std())
        else:
            print("success rate is very slow")
            median.append(np.nan)
            median_ub.append(np.nan)
            median_lb.append(np.nan)

    return np.array(median), np.array(median_lb), np.array(median_ub)


def get_av_cost(all_costs_np) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    # easy
    # mean = all_costs_np.mean(axis=0)
    # std = all_costs_np.std(axis=0)

    # more complex.
    num_instances = all_costs_np.shape[0]
    mean = []
    std = []
    for j in range(all_costs_np.shape[1]):
        column = all_costs_np[:, j]
        # how many nan?
        non_nan = np.count_nonzero(~np.isnan(column))
        print(column)
        print(non_nan)
        if non_nan >= int(num_instances / 2):
            print("success rate is enough")
            cc = column[~np.isnan(column)]
            print("cc", cc)
            mean.append(cc.mean())
            std.append(cc.std())
        else:
            print("success rate is very slow")
            mean.append(np.nan)
            std.append(np.nan)

    return (
        np.array(mean),
        np.array(mean) - np.array(std),
        np.array(mean) + np.array(std),
        np.array(std),
    )


def first(iterable, condition):
    """
    Returns the first item in the `iterable` that
    satisfies the `condition`.

    If the condition is not given, returns the first item of
    the iterable.

    Raises `StopIteration` if no item satysfing the condition is found.

    >>> first( (1,2,3), condition=lambda x: x % 2 == 0)
    2
    >>> first(range(3, 100))
    3
    >>> first( () )
    Traceback (most recent call last):
    ...
    StopIteration
    """

    return next(x for x in iterable if condition(x))


def analyze_search(
    path_to_dir: str, problem: str, alg: str, visualize: bool, **kwargs
) -> Tuple[str, str]:
    print(
        f"ARGS: path_to_dir:{path_to_dir}\nproblem:{problem}\nalg:{alg}\nvisualize:{visualize}"
    )
    __files = [f for f in pathlib.Path(path_to_dir).iterdir() if f.is_file()]

    files = [
        f
        for f in __files
        if "cfg" not in f.name
        and "debug" not in f.name
        and f.suffix == ".yaml"
        and "report" not in f.name
        and "traj" not in f.name
    ]

    print(f"files ", [f.name for f in files])

    Ds = []

    for file in [str(f) for f in files]:
        print(f"loading file: {file}")
        with open(file) as f:
            data = yaml.load(f, Loader=yaml.CLoader)
            # I am interested in cost of solution
            D = {
                "cost": data["cost"],
                "solved": data["solved"],
                "expands": data["data"]["expands"],
                "time_search": data["data"]["time_search"],
            }
            Ds.append(D)

    data_out = {}
    data_out["data_raw"] = Ds

    for k in ["cost", "solved", "expands", "time_search"]:
        data_out[k + "_mean"] = float(np.mean([d[k] for d in Ds]))
        data_out[k + "_std"] = float(np.std([d[k] for d in Ds]))

    data_out["success_rate"] = float(np.sum([d["solved"] for d in Ds]) / len(Ds))

    data_out["alg"] = alg
    data_out["files"] = [str(file) for file in files]
    data_out["path_to_dir"] = path_to_dir
    data_out["problem"] = problem

    fileout = path_to_dir + "/report.yaml"
    print(f"writing file {fileout}")
    with open(fileout, "w") as f:
        yaml.dump(data_out, f)

    # todo: create a figure!

    figureout = ""

    return fileout, figureout


def analyze_runs_time(
    path_to_dir: str, problem: str, guess: str, alg: str, visualize: bool, **kwargs
) -> Tuple[str, str]:
    print(
        f"path_to_dir:{path_to_dir}\nproblem:{problem}\nguess:{guess}\nalg:{alg}\nvisualize:{visualize}"
    )
    __files = [f for f in pathlib.Path(path_to_dir).iterdir() if f.is_file()]

    # filter some files out.

    files = [
        f
        for f in __files
        if "cfg" not in f.name
        and "debug" not in f.name
        and f.suffix == ".yaml"
        and "report" not in f.name
        and "traj" not in f.name
    ]

    print(f"files ", [f.name for f in files])

    Ds = []

    for file in [str(f) for f in files]:
        print(f"loading file: {file}")
        with open(file) as f:
            data = yaml.load(f, Loader=yaml.CLoader)
            cost = data["cost"]
            feasible = data["feasible"]
            time = data["info"]["time_ddp_total"]  # OR time_raw
            # time = data["info"]["time_raw"]  # OR time_raw
            D = {"cost": cost, "time": time, "feasible": feasible}
            Ds.append(D)

    data_out = {}
    data_out["data_raw"] = Ds
    cost_mean = np.mean([d["cost"] for d in Ds])
    time_mean = np.mean([d["time"] for d in Ds])
    success_rate = np.sum([d["feasible"] for d in Ds]) / len(Ds)

    cost_std = np.std([d["cost"] for d in Ds])
    time_std = np.std([d["time"] for d in Ds])

    data_out["cost_mean"] = float(cost_mean)
    data_out["time_mean"] = float(time_mean)
    data_out["cost_std"] = float(cost_std)
    data_out["time_std"] = float(time_std)
    data_out["success_rate"] = float(success_rate)
    data_out["alg"] = alg
    data_out["files"] = [str(file) for file in files]
    data_out["path_to_dir"] = path_to_dir
    data_out["problem"] = problem
    data_out["guess"] = guess

    fileout = path_to_dir + "/report.yaml"
    with open(fileout, "w") as f:
        yaml.dump(data_out, f)

    # todo: create a figure!

    figureout = ""

    return fileout, figureout


def analyze_runs(
    path_to_dir: str, problem: str, alg: str, visualize: bool, **kwargs
) -> Tuple[str, str]:
    print(
        f"path_to_dir:{path_to_dir}",
        f"problem:{problem}",
        f"alg:{alg}",
        f"visualize:{visualize}",
    )

    __files = [f for f in pathlib.Path(path_to_dir).iterdir() if f.is_file()]

    # filter some files out.

    files = [
        f
        for f in __files
        if "cfg" not in f.name
        and "debug" not in f.name
        and f.suffix == ".yaml"
        and "report" not in f.name
        and "traj" not in f.name
    ]

    print(f"files ", [f.name for f in files])

    fig, ax = plt.subplots(2, 1, sharex=True)
    fig.suptitle(problem)

    T = MAX_TIME_PLOTS  # max time
    dt = 0.1
    times = np.linspace(0, T, int(T / dt) + 1)

    all_costs = []

    first_solution = []
    raw_data = []

    if len(files) == 0:
        print("WARNING: there are no files")
        # mean = np.array( (int(T / dt) + 1) * [ np.nan] )
        # std = np.array( (int(T / dt) + 1) * [ np.nan] )
        all_costs_np = np.array([(int(T / dt) + 1) * [np.nan]])

    else:
        for file in [str(f) for f in files]:
            time_cost_pairs = get_cost_evolution(ax[0], file, **kwargs)
            raw_data.append(time_cost_pairs)
            first_solution.append(time_cost_pairs[0][0])
            t = [x[0] for x in time_cost_pairs]
            c = [x[1] for x in time_cost_pairs]

            t.insert(0, 0)
            c.insert(0, np.nan)

            t.append(T)
            c.append(c[-1])

            f = interp1d(t, c, kind="previous")
            cost_times = f(times)
            # print(cost_times)
            all_costs.append(cost_times)

        all_costs_np = np.array(all_costs)
    # print("all_costs_np")
    # print(all_costs_np)
    # print(all_costs_np.shape)
    # print(all_costs_np[0])

    std = np.array([])
    # mean, mean_lb, mean_ub, std = get_av_cost(all_costs_np)
    mean, mean_lb, mean_ub = get_av_cost_new(all_costs_np)

    # // nan nan 1

    __where = np.argwhere(np.isnan(mean)).flatten()
    time_first_solution = dt

    if __where.size > 0:
        time_first_solution = float(dt * (__where[-1] + 1))

    try:
        cost_first_solution = first(
            mean.tolist(), lambda x: not np.isnan(x) and not np.isinf(x)
        )
        last_cost = float(mean[-1])
    except StopIteration:
        # TODO : define a way to deal whit this numbers
        cost_first_solution = -1
        last_cost = -1

    # 10 seconds

    cost_at_1 = mean.tolist()[int(1 / dt)]
    cost_at_5 = mean.tolist()[int(5 / dt)]
    cost_at_10 = mean.tolist()[int(10 / dt)]
    cost_at_20 = mean.tolist()[int(20 / dt)]

    # cost of first solution

    # get the first not nan

    success = (
        np.count_nonzero(~np.isnan(all_costs_np), axis=0) / all_costs_np.shape[0] * 100
    )

    print(all_costs_np.shape)

    print("all_costs_np")
    for i in all_costs_np:
        print(i)

    ax[0].step(times, mean, color="blue", where="post")
    ax[0].fill_between(
        times,
        mean_ub,
        mean_lb,
        # mean + std,
        # mean - std,
        facecolor="blue",
        alpha=0.2,
    )
    ax[0].set_ylabel("cost")

    # ax[1].plot(times, success)
    ax[1].step(times, success, where="post")

    ax[1].set_xlabel("time [s]")
    ax[1].set_ylabel("success %")

    # ax[0].set_ylim([0, 30])
    # ax[1].set_xlim([0, 20])
    # ax[0].set_xlim([0, ])

    data_out = {}
    data_out["times"] = times.tolist()
    data_out["success"] = success.tolist()

    print("time_first_solution", time_first_solution)
    print("type(time_first_solution)")
    print(type(time_first_solution))
    data_out["time_first_solution"] = time_first_solution
    data_out["cost_first_solution"] = cost_first_solution
    data_out["success_rate"] = float(success[-1]) / 100

    data_out["cost_at_01"] = cost_at_1
    data_out["cost_at_05"] = cost_at_5
    data_out["cost_at_10"] = cost_at_10
    data_out["cost_at_20"] = cost_at_20
    data_out["last_cost"] = last_cost

    data_out["cost_mean"] = mean.tolist()

    data_out["cost_std"] = std.tolist()

    data_out["cost_ub"] = mean_ub.tolist()
    data_out["cost_lb"] = mean_lb.tolist()

    data_out["raw_data"] = raw_data

    data_out["problem"] = problem

    data_out["alg"] = alg
    data_out["path_to_dir"] = path_to_dir
    data_out["files"] = [str(f) for f in files]
    data_out["num_runs"] = len(files)

    fileout = path_to_dir + "/report.yaml"
    print(f"fileout {fileout}")
    with open(fileout, "w") as f:
        yaml.dump(data_out, f)

    figureout = path_to_dir + "/report.pdf"
    print(f"figureout {figureout}")
    fig.tight_layout()
    plt.savefig(figureout)
    if visualize:
        plt.show()

    return fileout, figureout


def visualize_motion_together(motions: list, robot_model: str, output_file: str):
    num_motions = len(motions)
    print(f"num_motions {num_motions}")

    # ny = 2
    # nx = max(num_motions // ny, 1)

    ny = 2
    nx = 3

    # G = ny * x + y
    # G -- > y = G % ny
    # G --> x  = G // ny

    viewer = get_viewer(robot_model)
    draw = viewer.view_static

    print(nx, ny)
    fig = plt.figure()
    axs = []

    for i in range(nx * ny):
        y = i % ny
        if robot_model.startswith("quad3d"):
            ax = fig.add_subplot(ny, nx, i + 1, projection="3d")
        else:
            ax = fig.add_subplot(ny, nx, i + 1)
        x = i // ny
        axs.append(ax)

    for i in range(nx * ny):
        y = i % ny
        x = i // ny
        if i >= len(motions):
            continue
        motion = motions[i]

        env = create_empty_env(motion["x0"], motion["xf"], robot_model)
        result = {"states": motion["states"], "actions": motion["actions"]}
        draw(axs[i], env, result)

    print(f"saving drawing of primitives to {output_file}")
    fig.tight_layout()
    plt.savefig(output_file)
    plt.show()


def create_latex_table(csv_file: str) -> None:
    # lets create the latex table

    # some replacements

    df = pandas.read_csv(csv_file)
    str_raw = df.to_latex(index=False, float_format="{:.1f}".format)

    str_ = format_latex_str(str_raw)
    now = datetime.now()  # current date and time
    date_time = now.strftime("%Y-%m-%d--%H-%M-%S")

    filename = f"../results_new/tex/data_{date_time}.tex"

    pathlib.Path(filename).parent.mkdir(parents=True, exist_ok=True)

    with open(filename, "w") as f:
        f.write(str_)

    problems = df["problem"].unique()
    print("problems", problems)
    for problem in problems:
        df_i = df[df["problem"] == problem].drop("problem", axis=1)

        df_i_str_raw = df_i.to_latex(index=False, float_format="{:.1f}".format)
        str_ = format_latex_str(df_i_str_raw)

        problem_ = problem.replace("/", "--")

        with open(f"../results_new/tex/data_{problem_}_{date_time}.tex", "w") as f:
            f.write(str_)


def visualize_primitives(
    motions: list, robot: str, interactive: bool = True, output_file: str = ""
):
    viewer = viewer_cli.get_robot_viewer(robot)

    num_motions = len(motions)
    print(f"num_motions {num_motions}")

    ny = 2
    # nx = max(num_motions // ny, 1)

    nx = 3

    # G = ny * x + y
    # G -- > y = G % ny
    # G --> x  = G // ny

    draw = viewer.view_static

    print(nx, ny)
    fig = plt.figure()
    axs = []
    for i in range(nx * ny):
        y = i % ny
        if robot.startswith("quad3d"):
            ax = fig.add_subplot(ny, nx, i + 1, projection="3d")
        else:
            ax = fig.add_subplot(ny, nx, i + 1)
        x = i // ny
        axs.append(ax)

    for i in range(nx * ny):
        y = i % ny
        x = i // ny
        if i >= len(motions):
            continue
        motion = motions[i]

        env = create_empty_env(motion["start"], motion["goal"], robot)

        result = {"states": motion["states"], "actions": motion["actions"]}
        draw(axs[i], env, result)

    print(f"saving drawing of primitives to {output_file}")
    plt.tight_layout()
    if len(output_file):
        fig.tight_layout()
        plt.savefig(output_file)
    if interactive:
        plt.show()


def fancy_table(
    filenames: List[str], benchmark_problems: List[str], benchmark_algs: List[str]
):
    df = pandas.DataFrame()

    for file in filenames:
        __df = pandas.read_csv(file)
        df = pandas.concat([df, __df], axis=0)

    print(list(df.columns))

    def get_data(frame, alg: str, problem: str, field: str, **kwargs):
        print(alg, problem, field)
        __f = frame.loc[
            (frame["alg"] == alg) & (frame["problem"] == problem)
        ].reset_index()
        print(__f)
        print("**" + field + "**")
        return __f[field]

    # buu = get_data(
    #     df,
    #     "idbastar_v0",
    #     "unicycle_first_order_0/parallelpark_0",
    #     "time_first_solution")

    data = []

    algs = benchmark_algs
    # algs = ["idbastar_v0", "sst_tmp", "geo_v1"] # default

    # algs = ["sst_v0", "sst_tmp", "geo_v1"]
    # algs = ["sst_v0", "sst_tmp", "geo_v1"]
    fields = ["success_rate", "time_first_solution", "cost_first_solution", "last_cost"]

    def alg_short(alg: str) -> str:
        if alg == "idbastar_v0":
            token1 = "i"
        elif alg == "sst_v0":
            token1 = "s"
        elif alg == "geo_v0":
            token1 = "g"
        # elif alg == "sst_tmp":
        #     token1 = "st"
        else:
            raise KeyError(alg)
        return token1

    def get_column_name(alg: str, field: str):
        token1 = alg_short(alg)
        token2 = ""
        if field == "cost_first_solution":
            token2 = "cf"
        elif field == "time_first_solution":
            token2 = "tf"
        elif field == "last_cost":
            token2 = "lc"
        elif field == "success_rate":
            token2 = "p"
        else:
            raise KeyError(field)
        return token1 + token2

    # problems = [
    #     "unicycle_first_order_0/bugtrap_0",
    #     "unicycle_first_order_0/kink_0",
    #     "unicycle_first_order_0/parallelpark_0",
    #     "unicycle_second_order_0/bugtrap_0",
    #     "unicycle_second_order_0/kink_0",
    #     "unicycle_second_order_0/parallelpark_0"]

    problems = df.problem.unique()

    problems = sorted(list(problems))

    with open("/tmp/problems_list.yaml", "w") as f:
        yaml.dump({"problems": problems}, f)

    if len(benchmark_problems):
        print("Warning: only chose problems!!")
        problems = sorted(list(set(problems).intersection(set(benchmark_problems))))

    all_df = pandas.DataFrame()

    for problem in problems:
        row = []
        headers = []
        for alg in algs:
            for field in fields:
                # D = {"alg": alg,
                #      "problem": problem,
                #      "field": field}
                header = get_column_name(alg, field)
                headers.append(header)
                _data = get_data(df, alg, problem, field)
                row.append(_data[0])
        print("final")
        print(row)
        print(headers)

        new_df = pandas.DataFrame([row], columns=headers, index=[problem])
        print(new_df)
        all_df = pandas.concat([all_df, new_df], axis=0)
    print(all_df)

    print("changing all -1 to 99...")

    all_df = all_df.replace(-1, unsolved_num)

    all_df.to_csv("tmp.csv")
    # now i could just export as table!!

    str_raw = all_df.to_latex(index=True, float_format="{:.1f}".format)
    str_ = format_latex_str(str_raw)

    lines = str_.splitlines()

    also_success = True

    if also_success:
        algs_line = r"& & &  \multicolumn{4}{c}{iDbA*} & \multicolumn{4}{c}{SST*} & \multicolumn{4}{c}{RRT*+TO}\\"
        mid_rules = r"\cmidrule(lr){4-7}\cmidrule(lr){8-11}\cmidrule(lr){12-15}"
    else:
        algs_line = r"&  \multicolumn{3}{c}{iDbA*} & \multicolumn{3}{c}{SST*} & \multicolumn{3}{c}{RRT*+TO}\\"
        mid_rules = r"\cmidrule(lr){2-4}\cmidrule(lr){5-7}\cmidrule(lr){8-10}"

    lines.insert(2, algs_line)
    lines.insert(3, mid_rules)

    now = datetime.now()  # current date and time
    date_time = now.strftime("%Y-%m-%d--%H-%M-%S")
    fileout = f"../results_new/tex/merged_{date_time}.tex"

    # create a new table

    # for problem in problems:
    #
    line = []

    # D = { problem : {  icf : xx , icf : xx , xx } }

    dict_data = all_df.to_dict(orient="index")

    #
    line = []
    # _algs = ["i", "s", "g"]

    _algs = [alg_short(alg) for alg in algs]

    _metrics = ["p", "tf", "cf", "lc"]

    def is_best(val: float, metric: str, problem: str, _algs: List[str]):
        factor = 1
        if metric == "p":
            factor = -1
        delta = 1e-10
        for a in _algs:
            key = a + metric
            b = dict_data[problem][key]
            if factor * val > factor * b + delta:
                return False
        return True

    custom_lines = []
    id = 0
    separate_dynamics_scenario = True
    for problem in problems:
        custom_line = []
        custom_line.append(str(id))
        id += 1
        ll = problem.split("/")

        dynamics = ll[0]
        scenario = ll[1]

        if separate_dynamics_scenario:
            custom_line.append(dynamics)
            custom_line.append(scenario)

        else:
            custom_line.append(problem)

        # custom_line.append(problem)

        # separate for dynamics

        # code to separate string by a character
        # res = [i for j in s.split(' ') for i in (j, ' ')][:-1]

        for ik, k in enumerate(_algs):
            for token in _metrics:
                full_key = k + token
                val = dict_data[problem][full_key]
                is_bold = is_best(val, token, problem, _algs)
                if is_bold:
                    tt = r"\textbf{" + f"{val:.1f}" + "}"
                else:
                    tt = f"{val:.1f}"
                custom_line.append(str(tt))
        print(f"custom_line {custom_line}")
        custom_lines.append(custom_line)
    print(custom_lines)

    _table_cols = [
        "$p$",
        r"$t^{\textup{st}} [s]$",
        r"$J^{\textup{st}}[s]$",
        r"$J^{\textup{f}}[s]$",
    ]

    header = [
        # r"\begin{tabular}{lrrrrrrrrr}",
        # r"\begin{tabular}{lrrrrrrrrrrrr}",
        # r"\begin{tabular}{lrrrrrrrrrrrrr}",  # include a number
        r"\begin{tabular}{cccrrrrrrrrrrrr}",
        # include a number #1 and separte dynamics and scenario
        r"\toprule",
        algs_line,
        mid_rules,
        r" \# & System & Instance & " + r" & ".join(3 * _table_cols) + r"\\",
        r"\midrule",
    ]

    footer = [r"\bottomrule", r"\end{tabular}"]

    fileout_nice = fileout + ".nice.tex"
    print("writing:", fileout_nice)
    with open(fileout_nice, "w") as f:
        for h in header:
            f.write(h)
            f.write("\n")
        for cl in custom_lines:
            print("cl ", cl)
            _str = format_latex_str(" & ".join(cl))
            _str = _str.replace("999.0", "-")
            # _str = _str.replace("40.1", "-")
            _str = _str.replace("120.1", "-")
            _str = _str.replace("180.1", "-")
            f.write(_str)

            f.write(r"\\")
            f.write("\n")
        for fo in footer:
            f.write(fo)
            f.write("\n")

    # if     #
    # for problem in problems:
    #     line.append(problem)
    #     for ik, k in enumerate(["i", "s", "g"]):
    #         for token in ["cf", "lc", "tf"]:
    #             full_key = k + token
    #             print(f"problem {problem}")
    #             print(f"full_key {full_key}")
    #             tt = float(all_df.loc[problem ][full_key])
    #             print(tt)
    #         # continue_here
    #         pass
    #  # & icf & ilc & itf & scf & slc & stf & gcf & glc & gtf \\

    # close new table

    print("writing:", fileout)
    with open(fileout, "w") as f:
        f.write("\n".join(lines))

    generate_texpdf(fileout_nice)

    fileout_log = fileout + ".log"
    print("saving", fileout_log)

    with open(fileout_log, "w") as f:
        D = {
            "input": filenames,
            "output": fileout,
            "problems": problems,
            "date": date_time,
            "hostname": os.uname()[1],
        }

        yaml.dump(D, f)


def format_latex_str(str_in: str) -> str:
    str_ = str_in[:]

    D = {
        "unicycle_first_order_0": "uni1_0",
        "unicycle_first_order_1": "uni1_1",
        "unicycle_first_order_2": "uni1_2",
        "unicycle_second_order_0": "uni2_0",
        "parallelpark_0": "park",
        "cost_at": "c",
        "cost_first_solution": "cs",
        "time_first_solution": "ts",
        "last_cost": "cf",
    }

    for k, v in D.items():
        str_ = str_.replace(k, v)

    for k, v in D_key_to_nice_name:
        # print(f"replacing {k} with {v}")
        #
        # print(str_)
        str_ = str_.replace(k, v)
        # print(str_)

    str_ = str_.replace("_", "\\_")
    # print("final string is:")
    # print(str_)

    # raise ValueError("stop here")

    str_ = str_.replace("nan", "-")

    return str_


def parse_for_component_analysis(files: List[str]):
    visualize = False
    max_it = 1

    field_ddp = "time_ddp_total"
    field_search = "time_search"
    field_nn = "time_nearestNode"
    field_col = "time_collisions"
    field_mm = "time_nearestMotion"

    Ds = []
    for file in files:
        with open(file) as f:
            _D = yaml.load(f, Loader=yaml.CLoader)
        assert "infos_opt" in _D
        assert "infos_raw" in _D

        infos_opt = _D["infos_opt"]
        infos_raw = _D["infos_raw"]

        counter_ddp = 0
        counter_search = 0
        counter_col = 0
        counter_nn = 0
        counter_mm = 0

        for it in range(max_it):
            counter_ddp += infos_opt[it][field_ddp]

        only_consider_solved = True
        if only_consider_solved:
            infos_raw = [i for i in infos_raw if i["terminate_status"] == "SOLVED"]

        assert len(infos_raw) >= max_it

        for it in range(max_it):
            counter_search += infos_raw[it][field_search]
            counter_nn += infos_raw[it][field_nn]
            counter_mm += infos_raw[it][field_mm]
            counter_col += infos_raw[it][field_col]

        print(f"counter_ddp {counter_ddp}")
        print(f"counter_search {counter_search}")
        print(f"counter_nn {counter_nn}")
        print(f"counter_mm {counter_mm}")
        print(f"counter_col {counter_col}")

        D = {
            "counter_ddp": counter_ddp,
            "counter_search": counter_search,
            "counter_nn": counter_nn,
            "counter_mm": counter_mm,
            "counter_col": counter_col,
        }

        file_out = "/tmp/componentes.yaml"
        with open(file_out, "w") as f:
            yaml.dump(D, f)

        file_out_log = file_out + ".log"

        now = datetime.now()  # current date and time
        date_time = now.strftime("%Y-%m-%d--%H-%M-%S")
        with open(file_out_log, "w") as f:
            yaml.dump(
                {
                    "input": file,
                    "output": file_out,
                    "date": date_time,
                    "hostname": os.uname()[1],
                },
                f,
            )

        fig, ax = plt.subplots()

        bottom = 0
        width = 0.8
        for k, v in D.items():
            p = ax.bar(0, v, width, label=k, bottom=bottom)
            bottom += v
        ax.set_title(
            "time spent in each component -- "
            + _D["problem_file"]
            + ":"
            + str(_D["options_idbastar"]["delta_0"])
            + str(_D["options_idbastar"]["num_primitives_0"])
        )
        ax.legend(loc="upper right")

        # per-iteration

        if visualize:
            plt.show()

        # fig, ax = plt.subplots()
        for _it in range(max_it):
            D = {
                "problem_file": _D["problem_file"],
                "robot_type": _D["robot_type"],
                "delta": _D["options_idbastar"]["delta_0"],
                "prim": _D["options_idbastar"]["num_primitives_0"],
                "counter_ddp": infos_opt[_it][field_ddp],
                "counter_search_extra": infos_raw[_it][field_search]
                - infos_raw[_it][field_nn]
                - infos_raw[_it][field_mm]
                - infos_raw[_it][field_col],
                "counter_nn": infos_raw[_it][field_nn],
                "counter_mm": infos_raw[_it][field_mm],
                "counter_col": infos_raw[_it][field_col],
            }
            Ds.append(D)

        # keys = Ds[0].keys()
        # bottom = np.zeros(max_it)
        # it = np.array([0,1])
        # for key in keys:
        #     vv = np.array([D[key] for D in Ds])
        #     p = ax.bar(it, vv, width, label=key, bottom=bottom)
        #     bottom += vv
        #
        # ax.set_title("time analysis")
        # ax.legend(loc="upper right")
        # plt.show()
        #
        # print("done")

    print("parsing done, lets print DS")
    # lets print the content of Ds!

    matplotlib.rcParams.update({"font.size": 12})
    fig = plt.figure(figsize=(6, 4))
    ax = fig.add_subplot(1, 1, 1)
    width = 0.5
    keys = Ds[0].keys()
    bottom = np.zeros(len(Ds))
    # it = np.arange(len(Ds))

    keys = [
        "counter_ddp",
        "counter_search_extra",
        "counter_nn",
        "counter_mm",
        "counter_col",
    ]

    print("WARNING: it is hardcoded by hand!!")

    # it = ("uni1_bug/d1",
    #       "uni1_bug/d2",
    #       "uni2_bug/d1",
    #       "uni2_bug/d2")

    # it =  [ str(i) for i in np.arange( len(Ds)) ]
    for D in Ds:
        print(D)

    print(" len(Ds)")
    print(len(Ds))
    it = [D["robot_type"] + str(D["delta"]) for D in Ds]
    it = ["uni1_0.3", "uni1_0.2", "uni2_0.5", "uni2_0.3", "rotor2_0.55", "rotor2_0.5"]

    x = np.arange(len(it))
    print(it)

    # str(i) for i in np.arange( len(Ds)) ]

    # generate it
    Dkey_to_label = {
        "counter_ddp": "Optimization",
        "counter_search_extra": "Search-Extra",
        "counter_nn": "Search-NN",
        "counter_mm": "Search-Primitives",
        "counter_col": "Search-Collision",
    }

    for key in keys:
        vv = 0.001 * np.array([D[key] for D in Ds])
        p = ax.bar(x, vv, width, label=Dkey_to_label[key], bottom=bottom)
        bottom += vv

    ax.set_ylim([0, 20])
    ax.set_ylabel("Time [s]")
    # locs, labels = ax.yticks()  # Get the current locations and labels.

    # yticks([5,10,15,20])
    #     np.arange(0, 1, step=0.2))  # Set label locations.
    #
    # yticks(np.arange(3), ['Tom', 'Dick', 'Sue'])  # Set text labels.
    #
    # yticks([0, 1, 2], ['January', 'February', 'March'],
    #
    #        rotation=45)  # Set text labels and properties.
    #
    # yticks([])  # Disable yticks.

    ax.set_yticks([5, 10, 15, 20])

    # ax.set_title("Component analysis")
    ax.set_xticks(x)
    ax.set_xticklabels(it, rotation=45, ha="right")
    ax.legend(loc="upper left")

    figureout = "/report.pdf"
    path = "../results_new_components/"

    now = datetime.now()  # current date and time
    date_time = now.strftime("%Y-%m-%d--%H-%M-%S")

    figureout = f"{path}{date_time}.pdf"
    print(f"writing {figureout}")
    fig.tight_layout()
    plt.savefig(figureout)

    copy_to = "/tmp/components.pdf"
    shutil.copy(figureout, copy_to)
    print(f"copy to {copy_to}")

    with open(figureout + ".log", "w") as f:
        yaml.dump(
            {
                "input": files,
                "output": figureout,
                "date": date_time,
                "hostname": os.uname()[1],
            },
            f,
        )

    if visualize:
        plt.show()

    print("done")

    # group by problem
    # should I use the latex output of pandas?


if __name__ == "__main__":
    if False:
        # file = "../bench_logs/server_idbastar_lunch_compare.yaml"
        #
        # # file = "../bench_logs/server_idbastar_short_compare.yaml"
        # with open(file, "r") as f:
        #     d = yaml.safe_load(f)
        # data = d["data"]
        # compare(data)
        # sys.exit(0)

        files = [
            "../bench_logs/server_tt_compare.yaml",
            "../bench_logs/server_geo_compare.yaml",
            "../bench_logs/server_idbastar_lunch_compare.yaml",
        ]

        data = []
        for fi in files:
            with open(fi, "r") as f:
                d = yaml.safe_load(f)
            _data = d["data"]
            data += _data

        def check(d):
            algs = ["idbastar_v0", "sst_tmp", "geo_v1"]

            for a in algs:
                if a in d:
                    return True
            return False

        print(len(data))
        data = [d for d in data if check(d)]
        print(len(data))
        compare(data)
        sys.exit(0)

    #
    #
    # results_new/plots/plot_2023-06-20--18-32-25.pdf.log"

    # file = "../results_new/plots/plot_2023-06-20--18-32-25.pdf.log"
    #
    #
    # with open(file,"r") as f:
    #     d = yaml.safe_load(f)
    #
    # input = d["input"]
    #
    #
    # input = [ ii for ii in input if "window" in ii ]
    #
    # compare(input)
    #
    # sys.exit(0)

    # path_to_dir="../results_new/unicycle_second_order_0/parallelpark_0/sst_v1/2023-06-14--14-17-18"
    # problem="unicycle_second_order_0/parallelpark_0"
    # alg="sst_v1"
    # visualize=False
    #
    # analyze_runs(path_to_dir,
    #              problem,
    #              alg,
    #              visualize)
    #
    #
    # sys.exit(0)

    # TODO: how to I check the time spent in each component?

    if do_study:
        study(bench_cfg)

    do_table_search = False
    do_table_opti = False

    # function to concatenate two table

    if do_table_search:
        file = "../results_new_search/plots/plot_search_2023-06-06--12-49-13.pdf.log"
        with open(file) as f:
            D = yaml.load(f, Loader=yaml.CLoader)

        assert "input" in D
        files = D["input"]

        compare_search(files)

        print("bye")
        sys.exit(0)

    if do_table_opti:
        file = "../results_new_timeopt/plots/plot_time_2023-06-06--15-36-24.pdf.log"
        with open(file) as f:
            D = yaml.load(f, Loader=yaml.CLoader)
        assert "input" in D
        files = D["input"]
        compare_time(files)

        print("bye")
        sys.exit(0)

    if do_fancy_table:
        # files = [
        #     "/home/quim/stg/wolfgang/kinodynamic-motion-planning-benchmark/results_new/summary/summary_2023-05-10--15-18-08.csv",
        #     "/home/quim/stg/wolfgang/kinodynamic-motion-planning-benchmark/results_new/summary/summary_2023-05-10--15-41-42.csv",
        #     "/home/quim/stg/wolfgang/kinodynamic-motion-planning-benchmark/results_new/summary/summary_2023-05-10--15-56-06.csv",
        #     "/home/quim/stg/wolfgang/kinodynamic-motion-planning-benchmark/results_new/summary/summary_2023-05-11--07-50-08.csv",
        #     "/home/quim/stg/wolfgang/kinodynamic-motion-planning-benchmark/results_new/summary/summary_2023-05-11--12-02-25.csv",
        #     "/home/quim/stg/wolfgang/kinodynamic-motion-planning-benchmark/results_new/summary/summary_2023-05-11--15-06-40.csv",
        #     "/home/quim/stg/wolfgang/kinodynamic-motion-planning-benchmark/results_new/summary/summary_2023-05-11--17-34-33.csv",
        # ]

        with open(bench_cfg, "r") as f:
            D = yaml.load(f, Loader=yaml.CLoader)
        # files = ["../results_new/summary/summary_2023-06-21--11-24-51.csv",
        #          "../results_new/summary/summary_2023-06-21--13-56-25.csv"]

        fancy_table(D["files"], D["benchmark_problems"], D["benchmark_algs"])

    if do_compare:
        # folders = ["geo_v0/04-04-2023--15-59-51",
        #            "idbastar_v0/04-04-2023--15-59-51",
        #            "idbastar_v1/04-04-2023--15-59-51",
        #            "sst_v0/04-04-2023--15-59-51"]
        #
        # base = "/home/quim/stg/wolfgang/kinodynamic-motion-planning-benchmark/results_new/unicycle_first_order_0/bugtrap_0/"
        # files = [base + folder + "/report.yaml" for folder in folders]
        files = [
            "../results_new/unicycle_second_order_0/parallelpark_0/idbastar_v0/04-24-2023--18-07-26/report.yaml",
            "../results_new/car_first_order_with_1_trailers_0/parallelpark_0/idbastar_v0/04-24-2023--18-07-26/report.yaml",
        ]

        compare(files, interactive=True)
    if do_benchmark:
        benchmark(bench_cfg)

    if do_bench_time:
        benchmark_opti(bench_cfg)

    if do_bench_search:
        benchmark_search(bench_cfg)

    if do_debug:
        file = "../results_new/unicycle_second_order_0/parallelpark_0/idbastar_v0/04-06-2023--14-52-41/run_1_out.yaml"
        with open(file, "r") as f:
            data = yaml.load(f, Loader=yaml.CLoader)
        print(data)
    if do_vis_primitives:
        # file = "../cloud/motionsV2/tmp_car1.bin.yaml"
        # robot = "car"
        # with open(file, "r") as f:
        #     motions = yaml.safe_load(f)
        # visualize_primitives(motions, robot, True, "out.pdf");

        # file = "../cloud/motionsV2/tmp_acrobot.bin.yaml"
        # robot = "acrobot"
        # with open(file, "r") as f:
        #     motions = yaml.safe_load(f)
        # visualize_primitives(motions, robot, True, "out.pdf");

        file = file_in
        robot = dynamics
        with open(file, "r") as f:
            motions = yaml.load(f, Loader=yaml.CLoader)
        visualize_primitives(motions, robot, True, "out.pdf")

    # path = "/home/quim/stg/wolfgang/kinodynamic-motion-planning-benchmark/results_new/unicycle_first_order_0/bugtrap_0/sst_v0/04-04-2023--11-08-40"
    # path = "../results_new/unicycle_first_order_0/bugtrap_0/geo_v0/04-04-2023--15-23-12/"
    # analyze_runs(path, visualize=False)

    # folder = '../results_new/unicycle_first_order_0/parallelpark_0/idbastar_v0/04-01-2023--11-50-36/'
    # analyze_runs(folder, visualize=True)
    # file = "results_sst.yaml"
    # make_videos(robot, problem, file)

    # file = "tmp_trajectories.yaml"
    # visualize_primitives(file,robot, "primitives.pdf")

    # file = "motions__s__unicycle1_v0__03-04-2023--08-48-54.yaml"
    # robot = "unicycle2"
    # file = "../cloud/motionsV2/unicycle2_v0__2023_04_03__15_01_24.yaml"
    # with open(file, "r") as f:
    #     motions = yaml.safe_load(f)
    #
    # visualize_primitives(
    #     motions,
    #     robot,
    #     interactive=True,
    #     output_file="primitives.pdf")
    #
    # visualize_primitives(motions[10:],
    #                      robot,
    #                      interactive=True,
    #                      output_file="primitives2.pdf")
    #
    # with open(file, "r") as f:
    #     motions = yaml.safe_load(f)
    # print(f"len(motions) {len(motions)}")
    # plot_stats(motions, robot, "new_stats.pdf")
    #

    # stats of primitives
    # Get the stats...
    # visualize_primitives(file,robot, "primitives.pdf")
