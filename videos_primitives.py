# small script to make videos from the primitives! -- working :)
# TODO: check the 3d case!! (continue here!!)


from vid_ffmpeg import generate_grid, concat_vids, add_text
import msgpack
import yaml
import subprocess
import pathlib

base = "../"


num_primitives = 20
num_rows = 4
L = [
    {
        "file": "./dynomotions/unicycle1_v0__ispso__2023_04_03__14_56_57.bin.im.bin.im.bin.small5000.msgpack",
        "robot": "unicycle1_v0",
        "text": "Unicycle 1 (v0)",
        "MIN": [-1.5, -1.5],
        "MAX": [1.5, 1.5],
        "primitives_index": list(range(num_primitives)),
    },
    {
        "file": "./dynomotions/tmp_motions_unicycle1_v1.bin.sp.bin.small5000.msgpack",
        "robot": "unicycle1_v1",
        "MIN": [-1.5, -1.5],
        "MAX": [1.5, 1.5],
        "primitives_index": list(range(num_primitives)),
    },
    {
        "file": "./dynomotions/tmp_motions_unicycle1_v2.bin.sp.bin.small5000.msgpack",
        "robot": "unicycle1_v2",
        "MIN": [-1.5, -1.5],
        "MAX": [1.5, 1.5],
        "primitives_index": list(range(num_primitives)),
    },
    {
        "file": "./dynomotions/acrobot_v0_all2.bin.sp.bin.small5000.msgpack",
        "robot": "acrobot_v0",
        "MIN": [-1.8, -1.8],
        "MAX": [1.8, 1.8],
        "primitives_index": list(range(num_primitives)),
        "text": "Acrobot",
    },
    #
    {
        "file": "./dynomotions/quad2dpole_all.bin.im.bin.sp1.bin.ca.bin.small5000.msgpack",
        "robot": "quad2dpole_v0",
        "MIN": [-1.8, -1.8],
        "MAX": [1.8, 1.8],
        "primitives_index": list(range(num_primitives)),
        "text": "Rotor Pole",
    },
    {
        "file": "./dynomotions/unicycle2_v0__ispso__2023_04_03__15_36_01.bin.im.bin.im.bin.small5000.msgpack",
        "robot": "unicycle2_v0",
        "MIN": [-1.5, -1.5],
        "MAX": [1.5, 1.5],
        "primitives_index": list(range(num_primitives)),
        "text": "Unicycle 2",
    },
    {
        "file": "./dynomotions/quad3d_v0_all3.bin.im.bin.sp1.bin.ca.bin.small5000.msgpack",
        "robot": "quadrotor_v0",
        "MIN": [-1, -1, -1],
        "MAX": [1, 1, 1],
        "primitives_index": list(range(num_primitives)),
        "text": "Quadrotor v0",
    },
    {
        "file": "./dynomotions/quad3dompl_all.bin.im.bin.sp1.bin.ca.bin.small5000.msgpack",
        "robot": "quadrotor_v1",
        "MIN": [-1, -1, -1],
        "MAX": [1, 1, 1],
        "text": "Quadrotor v1",
        "primitives_index": list(range(num_primitives)),
    },
    {
        "file": "./dynomotions/car1_v0_all.bin.sp.bin.small5000.msgpack",
        "robot": "car1_v0",
        "MIN": [-1.5, -1.5],
        "MAX": [1.5, 1.5],
        "text": "Car+Trailer",
        "primitives_index": list(range(num_primitives)),
    },
    {
        "file": "dynomotions/quad2d_v0_all_im.bin.sp.bin.ca.bin.small5000.msgpack",
        "robot": "quad2d_v0",
        "MIN": [-2, -2],
        "MAX": [2, 2],
        "text": "Planar Rotor",
        "primitives_index": list(range(num_primitives)),
    },
]

for l in L:
    file = base + l["file"]
    robot = l["robot"]
    MIN = l["MIN"]
    MAX = l["MAX"]

    with open(file, "rb") as f:
        data = msgpack.unpackb(f.read(), raw=False)

    print(len(data["data"]))
    all_vids = []
    for ii in range(num_primitives):
        i = l["primitives_index"][ii]

        traj = data["data"][i]
        print(traj)

        with open("tmp.yaml", "w") as f:
            yaml.dump(traj, f)

        d = robot
        problem = "empty"

        Dproblem_tmp = {
            "name": "primitive",
            "environment": {"min": MIN, "max": MAX, "obstacles": []},
            "robots": [
                {"type": robot, "start": traj["states"][0], "goal": traj["states"][-1]}
            ],
        }

        tmp_file_problem = "/tmp/problem_tmp.yaml"
        with open(tmp_file_problem, "w") as f:
            yaml.dump(Dproblem_tmp, f)

        # robots:
        #   - type: car1_v0
        #     start: [0.5, 4.0, 1.55, 1.55] # x,y,theta0,theta1
        #     goal: [5.5, 4.0, 1.55, 1.55] # x,y,theta0,theta1

        traj_part = ["--result", "tmp.yaml"]

        fileout = f"../tro_primitives/{robot}_prim_{i}.pdf"

        pathlib.Path(fileout).parent.mkdir(parents=True, exist_ok=True)

        cmd = [
            "python3",
            "../dynobench/utils/viewer/viewer_cli.py",
            "--robot",
            robot,
            "--env",
            tmp_file_problem,
            "--result",
            "tmp.yaml",
            "-s",
            # "-i",
            "--out",
            fileout,
        ]

        print("running cmd ", " ".join(cmd))
        all_vids.append(fileout.replace(".pdf", "-solution.mp4"))
        subprocess.run(cmd)

    prim_vids = f"../tro_primitives/{robot}__prim_vids.mp4"
    num_cols = len(all_vids) // num_rows
    total_num = num_cols * num_rows
    generate_grid(all_vids[:total_num], num_rows, num_cols, prim_vids)

    cat_vids = f"../tro_primitives/{robot}__concat.mp4"

    concat_vids(
        ["/home/quim/stg/wolfgang/dynoplan/build_release/" + vid for vid in all_vids],
        cat_vids,
    )

    add_text(cat_vids, cat_vids.replace(".mp4", "_with_text.mp4"), l.get("text", robot))

    print(f"output video is {prim_vids}")
    print(f"output video is {cat_vids}")


# ffmpeg \
# -i /tmp/traj_0-solution.mp4 \
# -i /tmp/traj_1-solution.mp4 \
# -i /tmp/traj_2-solution.mp4 \
# -i /tmp/traj_3-solution.mp4 \
# -i /tmp/traj_4-solution.mp4 \
# -i /tmp/traj_5-solution.mp4 \
# -i /tmp/traj_6-solution.mp4 \
# -i /tmp/traj_7-solution.mp4 \
# -filter_complex \
# "[0:v][1:v][2:v][3:v]hstack=inputs=4[top];\
# [4:v][5:v][6:v][7:v]hstack=inputs=4[bottom];\
# [top][bottom]vstack=inputs=2[v]" \
# -map "[v]" \
# finalOutput.mp4
