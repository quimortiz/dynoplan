from typing import List
import subprocess


def add_text(v_in: str, v_out: str, text: str):
    cmd = [
        "ffmpeg",
        "-y",
        "-i",
        v_in,
        "-vf",
        "drawtext=text='"
        + text
        + "':fontcolor=white:fontsize=72:box=1:boxcolor=black@0.5:boxborderw=5:x=(w-text_w)/2:y=(h-text_h)/2+500",
        "-codec:a",
        "copy",
        v_out,
    ]
    print("running cmd", " ".join(cmd))
    out = subprocess.run(cmd)
    assert out.returncode == 0


def concat_vids(vids: List[str], vout: str):
    inputs_vids: List[str] = []
    for v in vids:
        inputs_vids.append("-i")
        inputs_vids.append(v)

    list_vids = "/tmp/file_list.txt"
    with open(list_vids, "w") as f:
        _ = [f.write(f"file {v}\n") for v in vids]

    cmd = [
        "ffmpeg",
        "-y",
        "-f",
        "concat",
        "-safe",
        "0",
        "-i",
        list_vids,
        "-c",
        "copy",
        vout,
    ]
    print("running cmd", " ".join(cmd))
    out = subprocess.run(cmd)
    assert out.returncode == 0


def stack_h(list_vids, output_path="/tmp/out.mp4"):
    n = len(list_vids)
    cmd = (
        ["ffmpeg", "-y", "-i"]
        + " -i ".join(list_vids).split()
        + ["-filter_complex", f"hstack=inputs={n}", output_path]
    )

    out = subprocess.run(cmd)
    assert out.returncode == 0, "error in ffmpeg"


def stack_v(list_vids, output_path="/tmp/out.mp4"):
    n = len(list_vids)
    cmd = (
        ["ffmpeg", "-y", "-i"]
        + " -i ".join(list_vids).split()
        + ["-filter_complex", f"vstack=inputs={n}", output_path]
    )

    out = subprocess.run(cmd)
    assert out.returncode == 0, "error in ffmpeg"


def generate_grid(list_vids, rows, cols, output_path="/tmp/out.mp4"):
    if len(list_vids) != rows * cols:
        raise ValueError(
            f"Number of videos{len(list_vids)} provided does not match the grid dimensions {rows}x{cols}"
        )

    if rows == 1:
        stack_h(list_vids, output_path)
        return
    if cols == 1:
        stack_v(list_vids, output_path)
        return

    _list_vids: List[str] = []
    _ = [(_list_vids.append("-i"), _list_vids.append(vid)) for vid in list_vids]
    filter_complex = ""
    input_refs = []

    for row in range(rows):
        row_inputs = []
        for col in range(cols):
            index = row * cols + col
            row_inputs.append(f"[{index}:v]")
        filter_complex += "".join(row_inputs) + f"hstack=inputs={cols}[row{row}];"
        input_refs.append(f"[row{row}]")

    filter_complex += "".join(input_refs) + f"vstack=inputs={rows}[v]"

    print("filter complex")
    print(filter_complex)

    cmd = (
        ["ffmpeg"]
        + _list_vids
        + ["-filter_complex", filter_complex, "-map", "[v]", "-y", output_path]
    )
    print("bad cmd", cmd)

    print("running cmd: \n", " ".join(cmd))

    out = subprocess.run(cmd)
    assert out.returncode == 0, "error in ffmpeg"
