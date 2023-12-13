import subprocess
from typing import List
from vid_ffmpeg import generate_grid, concat_vids, add_text
from pathlib import Path


base_name_slides = "/home/quim/stg/math-notes-tmp/db_journal_video/out"


# create slide on


from dataclasses import dataclass


Path("/tmp/dynoplan").mkdir(parents=True, exist_ok=True)


def change_steep(vid_in: str, vid_out: str, rate: float = 0.5) -> None:
    cmd = ["ffmpeg", "-i", vid_in, "-filter:v", f"setpts={rate}*PTS", "-y", vid_out]
    print("running cmd ", " ".join(cmd))
    out = subprocess.run(cmd)
    assert out.returncode == 0


@dataclass
class OverlayOptions:
    """Class for keeping track of an item in inventory."""

    vin1: str
    vin2: str
    vout: str
    scale: int
    x: str = "(main_w-overlay_w)/2"
    y: str = "(main_h-overlay_h)/2 "


def overlay_vid(options: OverlayOptions) -> None:
    # scale = 1850
    #
    # x = "(main_w-overlay_w)/2"
    # y = "(main_h-overlay_h)/2 + 200"

    cmd = [
        "ffmpeg",
        "-y",
        "-i",
        options.vin1,
        "-i",
        options.vin2,
        "-filter_complex",
        f"[1:v]scale={options.scale}:-1[ss];[0:v][ss]overlay={options.x}:{options.y}",
        "-codec:a",
        "copy",
        "-movflags",
        "+faststart",
        options.vout,
    ]

    print("running cmd ", " ".join(cmd))
    out = subprocess.run(cmd)
    assert out.returncode == 0


def video_from_slide(vin: str, vout: str, time: int = 5, framerate: int = 24) -> None:
    cmd = [
        "ffmpeg",
        "-y",
        "-loop",
        "1",
        "-framerate",
        f"{framerate}",
        "-t",
        f"{time}",
        "-i",
        vin,
        vout,
    ]
    print("running cmd ", " ".join(cmd))
    out = subprocess.run(cmd)
    assert out.returncode == 0


def mk_vid_1(vout: str):
    bg = f"{base_name_slides}-01.png"
    v2 = (
        "/home/quim/stg/wolfgang/dynoplan/plots_tro_with_sol/minimal_vid/"
        + "__tmp_faster2.mp4"
    )

    vimage = "/tmp/vid_image_1.mp4"

    video_from_slide(bg, vimage)

    opt = OverlayOptions(
        vin1=vimage, vin2=v2, vout=vout, scale=1850, y="(main_h-overlay_h)/2+150"
    )

    overlay_vid(opt)


def mk_vid_thanks(vout: str):
    bg = f"{base_name_slides}-12.png"
    v2 = (
        "/home/quim/stg/wolfgang/dynoplan/plots_tro_with_sol/minimal_vid/"
        + "__tmp_faster_4x4.mp4"
    )

    vimage = "/tmp/vid_image_1.mp4"

    video_from_slide(bg, vimage)

    opt = OverlayOptions(
        vin1=vimage, vin2=v2, vout=vout, scale=800, y="(main_h-overlay_h)/2+100"
    )

    overlay_vid(opt)


def mk_vid_selected(vout: str):
    bg = f"{base_name_slides}-09.png"

    v2 = (
        "/home/quim/stg/wolfgang/dynoplan/plots_tro_with_sol/minimal_vid/"
        + "__tmp_faster2.mp4"
    )
    # v2 = "/home/quim/stg/wolfgang/dynoplan/plots_tro_with_sol/minimal_vid/" + \
    #     "__tmp_faster_4x4.mp4"

    vimage = "/tmp/vid_image_1.mp4"

    video_from_slide(bg, vimage)

    opt = OverlayOptions(
        vin1=vimage, vin2=v2, vout=vout, scale=1800, y="(main_h-overlay_h)/2+100"
    )

    overlay_vid(opt)


def mk_vid_all_problems(vout: str):
    bg = f"{base_name_slides}-10.png"

    v2 = (
        "/home/quim/stg/wolfgang/dynoplan/plots_tro_with_sol/minimal_vid/"
        + "__tmp_faster_40.mp4"
    )
    # v2 = "/home/quim/stg/wolfgang/dynoplan/plots_tro_with_sol/minimal_vid/" + \
    #     "__tmp_faster_4x4.mp4"

    vimage = "/tmp/vid_image_1.mp4"

    video_from_slide(bg, vimage)

    opt = OverlayOptions(
        vin1=vimage, vin2=v2, vout=vout, scale=1800, y="(main_h-overlay_h)/2+100"
    )

    overlay_vid(opt)


def mk_vid_primitives(vout: str):
    base = "/home/quim/stg/wolfgang/dynoplan/tro_primitives"
    name = "car1_v0__prim_vids.mp4_sec.mp4"

    bg = f"{base_name_slides}-04.png"

    vimage = "/tmp/slide.mp4"

    video_from_slide(bg, vimage)

    v2 = f"{base}/{name}"

    opt = OverlayOptions(
        vin1=vimage, vin2=v2, vout=vout, scale=1500, y="(main_h-overlay_h)/2-50"
    )

    overlay_vid(opt)


def mk_vid_search(vout: str):
    # base = "/home/quim/stg/wolfgang/dynoplan/vids_tmp_Q/"
    # name = "idbastar_raw0-solution.mp4"

    base = "/home/quim/stg/wolfgang/dynoplan/data_for_videos/"
    name = "search_only.mp4"

    # /tmp/search_only.mp4
    bg = f"{base_name_slides}-05.png"

    vimage = "/tmp/slide.mp4"

    video_from_slide(bg, vimage)

    v2 = f"{base}/{name}"

    opt = OverlayOptions(
        vin1=vimage,
        vin2=v2,
        vout=vout,
        scale=1000,
        x="(main_w-overlay_w)/2 - 400",
        y="(main_h-overlay_h)/2+150",
    )

    overlay_vid(opt)


def mk_vid_opti(vout: str):
    # base = "/home/quim/stg/wolfgang/dynoplan/vids_tmp_Q/"
    # name = "idbastar_opt0-solution.mp4"

    # sta
    base = "/home/quim/stg/wolfgang/dynoplan/data_for_videos/"
    name1 = "db_over_search.mp4"
    name2 = "opt_over_search.mp4"

    bg = f"{base_name_slides}-06.png"

    vimage = "/tmp/slide.mp4"
    tmp_grid = "/tmp/grid_db_opt_over_search.mp4"

    generate_grid([base + name1, base + name2], rows=1, cols=2, output_path=tmp_grid)

    video_from_slide(bg, vimage)

    # v2 = f"{base}/{name}"

    opt = OverlayOptions(
        vin1=vimage, vin2=tmp_grid, vout=vout, scale=1500, y="(main_h-overlay_h)/2"
    )

    overlay_vid(opt)


def mk_vid_asymp(vout: str):
    base = "/home/quim/stg/wolfgang/dynoplan/data_for_videos/"

    name = "assmp_v2.mp4"
    # name1 = "idbastar_opt0-solution.mp4"

    bg = f"{base_name_slides}-07.png"

    vimage = "/tmp/slide.mp4"

    video_from_slide(bg, vimage)

    opt = OverlayOptions(
        vin1=vimage,
        vin2=f"{base}/{name}",
        vout=vout,
        scale=900,
        x="(main_w-overlay_w)/2 - 400",
        y="(main_h-overlay_h)/2+200",
    )

    overlay_vid(opt)


def mk_vid_all_prims(vout: str):
    bg = f"{base_name_slides}-08.png"

    vimage = "/tmp/vid_image_7.mp4"

    video_from_slide(bg, vimage)

    base = "../tro_primitives/"
    tmp_grid = "/tmp/grid_all_prims.mp4"

    from_scratch = False  # If True, it takes the individual videos from each primitive

    if from_scratch:
        systems = [
            "car1_v0",
            "quad2dpole_v0",
            "unicycle1_v0",
            "unicycle2_v0",
            "acrobot_v0",
            "quad2d_v0",
            "quadrotor_v0",
            "quadrotor_v1",
        ]

        Dsystem2name = {}
        outs = []

        for system in systems:
            vids = [f"{base}{system}_prim_{i}-solution.mp4" for i in range(4)]

            tmp_path = f"/tmp/dynoplan/concat_{system}.mp4"

            concat_vids(
                [
                    "/home/quim/stg/wolfgang/dynoplan/tro_primitives/" + vid
                    for vid in vids
                ],
                tmp_path,
            )
            out = tmp_path.replace(".mp4", "_with_text.mp4")
            add_text(tmp_path, out, Dsystem2name.get(system, system))
            outs.append(out)

    else:
        # NOTE: I cut the videos that are in 3D
        # ffmpeg -i quadrotor_v0__concat_with_text.mp4  -filter:v
        # "crop=1200:1200:100:200" -c:a copy -y
        # quadrotor_v0__concat_with_text_crop.mp4
        names = [
            "acrobot_v0__concat_with_text.mp4",
            "quad2dpole_v0__concat_with_text.mp4",
            "car1_v0__concat_with_text.mp4",
            "unicycle2_v0__concat_with_text.mp4",
            "quadrotor_v0__concat_with_text_crop.mp4",
            "unicycle1_v0__concat_with_text.mp4",
            "quadrotor_v1__concat_with_text_crop.mp4",
            "quad2d_v0__concat_with_text.mp4",
        ]

        DchangeSpeed = {
            "acrobot_v0__concat_with_text.mp4": 0.2,
            "quad2dpole_v0__concat_with_text.mp4": 0.2,
            "quadrotor_v0__concat_with_text_crop.mp4": 0.2,
            "quadrotor_v1__concat_with_text_crop.mp4": 0.2,
            "quad2d_v0__concat_with_text.mp4": 0.2,
        }

        if True:
            outs = []
            for d in names:
                if d in DchangeSpeed:
                    change_steep(
                        base + d,
                        base + d.replace(".mp4", "_faster.mp4"),
                        DchangeSpeed[d],
                    )
                    outs.append(base + d.replace(".mp4", "_faster.mp4"))
                else:
                    outs.append(base + d)

            generate_grid(outs, rows=2, cols=4, output_path=tmp_grid)

    tmp_grid_time_crop = "/tmp/grid_all_prims_time_crop.mp4"
    if True:
        max_time = 15
        cmd = ["ffmpeg", "-y", "-i", tmp_grid, "-t", str(max_time), tmp_grid_time_crop]
        print("running cmd ", " ".join(cmd))
        out = subprocess.run(cmd)
        assert out.returncode == 0

    opt = OverlayOptions(
        vin1=vimage,
        vin2=tmp_grid_time_crop,
        vout=vout,
        scale=1600,
        y="(main_h-overlay_h)/2+70",
    )

    overlay_vid(opt)


all_vids = []

vid_1 = "/tmp/slide_1.mp4"
vid_2 = "/tmp/slide_2.mp4"
vid_3 = "/tmp/slide_3.mp4"
vid_9 = "/tmp/slide_9.mp4"
vid_thanks = "/tmp/slide_thanks.mp4"
vid_prim = "/tmp/slide_prims.mp4"
vid_search = "/tmp/vid_search.mp4"
vid_optimization = "/tmp/vid_opti.mp4"
vid_asymptotic = "/tmp/vid_asymptotic.mp4"
vid_all_prims = "/tmp/vid_all_prims.mp4"
vid_selected = "/tmp/vid_selected.mp4"

vid_all_problems = "/tmp/vid_all_problems.mp4"

vid_benchmark = "/tmp/vid_benchmark.mp4"
vid_ablation = "/tmp/vid_ablation.mp4"
vid_code = "/tmp/vid_code.mp4"


mk_vid_1(vid_1)
all_vids.append(vid_1)

video_from_slide(f"{base_name_slides}-02.png", vid_2)
all_vids.append(vid_2)

video_from_slide(f"{base_name_slides}-03.png", vid_3)
all_vids.append(vid_3)

mk_vid_primitives(vid_prim)
all_vids.append(vid_prim)

mk_vid_search(vid_search)
all_vids.append(vid_search)

mk_vid_opti(vid_optimization)
all_vids.append(vid_optimization)

mk_vid_asymp(vid_asymptotic)
all_vids.append(vid_asymptotic)


mk_vid_all_prims(vid_all_prims)
all_vids.append(vid_all_prims)

mk_vid_selected(vid_selected)
all_vids.append(vid_selected)

mk_vid_all_problems(vid_all_problems)
all_vids.append(vid_all_problems)

video_from_slide(f"{base_name_slides}-11.png", vid_benchmark)
all_vids.append(vid_benchmark)

video_from_slide(f"{base_name_slides}-12.png", vid_ablation)
all_vids.append(vid_ablation)

video_from_slide(f"{base_name_slides}-13.png", vid_code)
all_vids.append(vid_code)

video_from_slide(f"{base_name_slides}-14.png", vid_thanks)
all_vids.append(vid_thanks)

concat_vids(all_vids, "/tmp/out.mp4")


cmd = ["ffmpeg", "-y", "-i", "/tmp/out.mp4", "/tmp/output.mp4"]
print("running cmd ", " ".join(cmd))
out = subprocess.run(cmd)
assert out.returncode == 0
