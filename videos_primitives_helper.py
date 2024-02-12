import subprocess
from vid_ffmpeg import generate_grid

pwd = "/home/quim/stg/wolfgang/dynoplan/tro_primitives/__quadrotor_v0/mp4/"


files = [
    "quadrotor_v0_prim_0-solution.mp4",
    "quadrotor_v0_prim_1-solution.mp4",
    "quadrotor_v0_prim_2-solution.mp4",
    "quadrotor_v0_prim_3-solution.mp4",
    "quadrotor_v0_prim_4-solution.mp4",
    "quadrotor_v0_prim_5-solution.mp4",
    "quadrotor_v0_prim_6-solution.mp4",
    "quadrotor_v0_prim_7-solution.mp4",
    "quadrotor_v0_prim_8-solution.mp4",
    "quadrotor_v0_prim_9-solution.mp4",
]

files = [pwd + file for file in files]


def crop_vid(v_in: str, v_out: str):
    cmd = [
        "ffmpeg",
        "-i",
        v_in,
        "-filter:v",
        "crop=1000:1000:200:200",
        "-c:a",
        "copy",
        "-y",
        v_out,
    ]
    print("running cmd\n", " ".join(cmd))

    out = subprocess.run(cmd)
    assert out.returncode == 0


out_files = []

for file in files:
    out_file = file.replace(".mp4", "-crop.mp4")
    crop_vid(file, out_file)
    out_files.append(out_file)

generate_grid(files, 2, 5, output_path="/tmp/out.mp4")
generate_grid(out_files, 2, 5, output_path="/tmp/out_crop.mp4")
