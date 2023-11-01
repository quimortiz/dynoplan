import yaml
import pathlib
from pathlib import Path
import os

import glob


# time_stamp = "2023-07-28--14-59-27"

time_stamp = "2023-07-28--17-48-21"


cfg_file = "benchmark/config/compare.yaml"


with open(cfg_file) as f:
    d = yaml.load(f, Loader=yaml.FullLoader)


problems = d["problems"]


dst_base = "dynobench/envs/"

# dynoplan/tt/"

alg = "geo_v0"

Dalg_inout = {"geo_v0": "rrt_to_v0"}

run_id = 0

# file_to_copy = "trajopt-0"
file_to_copy = "trajraw-0"

name_out = "guess_v0"

# TODO: issue with

# IDEAS: some db solutions are almost feasible, because I reuse the solutions from the previous run.


simulate = False


for problem in problems:
    system, instance = problem.split("/")

    # path = f"results_new/{system}/{instance}/{alg}/{time_stamp}/"
    #
    # files=glob.glob(path + "*trajraw*" )
    # files.sort()
    # # last_file = files[-1]
    # last_file = files[min(1,len(files))]

    file_sol = f"results_new/{system}/{instance}/{alg}/{time_stamp}/run_{run_id}_out.yaml.{file_to_copy}.yaml"

    # file_sol = last_file

    print(file_sol)

    # create the directory if it does not exist
    if pathlib.Path.exists(pathlib.Path(file_sol)):
        dst = dst_base + f"{system}/{instance}/{Dalg_inout[alg]}_{name_out}.yaml"
        Path(dst).parent.mkdir(parents=True, exist_ok=True)
        if not pathlib.Path.exists(pathlib.Path(dst).parent):
            print(f"why {dst} it does not exist?")
        else:
            cmd = f"cp {file_sol} {dst}"
            print(cmd)
            if not simulate:
                os.system(cmd)
            else:
                print("this is a simulation")
    else:
        print(f"Warning: File {file_sol} does not exist")
