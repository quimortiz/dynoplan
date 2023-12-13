from vid_ffmpeg import generate_grid
import subprocess
import random
from typing import List
import time


def create_input_for_ffmpeg(videos: List[str]) -> List[str]:
    out = []
    for vid in videos:
        out.append("-i")
        out.append(vid)
    return out


# generate_grid(videos[:6], 2, 3)  # 2 rows and 3 columns


videos = [
    "plotenv-acrobot_v0-swing_down_easy-solution.mp4.center_LEN.mp4",
    "plotenv-acrobot_v0-swing_down-solution.mp4.center_LEN.mp4",
    "plotenv-acrobot_v0-swing_up_empty-solution.mp4.center_LEN.mp4",
    "plotenv-acrobot_v0-swing_up_obs_hard-solution.mp4.center_LEN.mp4",
    "plotenv-acrobot_v0-swing_up_obs-solution.mp4.center_LEN.mp4",
    "plotenv-car1_v0-bugtrap_0-solution.mp4.center_LEN.mp4",
    "plotenv-car1_v0-kink_0-solution.mp4.center_LEN.mp4",
    "plotenv-car1_v0-parallelpark_0-solution.mp4.center_LEN.mp4",
    "plotenv-quad2dpole_v0-down-solution.mp4.center_LEN.mp4",
    "plotenv-quad2dpole_v0-move_with_down-solution.mp4.center_LEN.mp4",
    "plotenv-quad2dpole_v0-move_with_up-solution.mp4.center_LEN.mp4",
    "plotenv-quad2dpole_v0-up_obs-solution.mp4.center_LEN.mp4",
    "plotenv-quad2dpole_v0-up-solution.mp4.center_LEN.mp4",
    "plotenv-quad2dpole_v0-window_easy-solution.mp4.center_LEN.mp4",
    "plotenv-quad2dpole_v0-window_hard-solution.mp4.center_LEN.mp4",
    "plotenv-quad2dpole_v0-window-solution.mp4.center_LEN.mp4",
    "plotenv-quad2d_v0-empty_0-solution.mp4.center_LEN.mp4",
    "plotenv-quad2d_v0-empty_1-solution.mp4.center_LEN.mp4",
    "plotenv-quad2d_v0-fall_through-solution.mp4.center_LEN.mp4",
    "plotenv-quad2d_v0-quad2d_recovery_obs-solution.mp4.center_LEN.mp4",
    "plotenv-quad2d_v0-quad2d_recovery_wo_obs-solution.mp4.center_LEN.mp4",
    "plotenv-quad2d_v0-quad_bugtrap-solution.mp4.center_LEN.mp4",
    "plotenv-quad2d_v0-quad_obs_column-solution.mp4.center_LEN.mp4",
    "plotenv-quadrotor_v0-empty_0_easy-solution.mp4.center_LEN.mp4",
    "plotenv-quadrotor_v0-empty_1_easy-solution.mp4.center_LEN.mp4",
    "plotenv-quadrotor_v0-quad_one_obs-solution.mp4.center_LEN.mp4",
    "plotenv-quadrotor_v0-recovery-solution.mp4.center_LEN.mp4",
    "plotenv-quadrotor_v0-recovery_with_obs-solution.mp4.center_LEN.mp4",
    "plotenv-quadrotor_v0-window-solution.mp4.center_LEN.mp4",
    "plotenv-quadrotor_v1-empty_0_easy-solution.mp4.center_LEN.mp4",
    "plotenv-quadrotor_v1-empty_1_easy-solution.mp4.center_LEN.mp4",
    "plotenv-quadrotor_v1-quad_one_obs-solution.mp4.center_LEN.mp4",
    "plotenv-quadrotor_v1-recovery-solution.mp4.center_LEN.mp4",
    "plotenv-quadrotor_v1-recovery_with_obs-solution.mp4.center_LEN.mp4",
    "plotenv-quadrotor_v1-window-solution.mp4.center_LEN.mp4",
    "plotenv-unicycle1_v0-bugtrap_0-solution.mp4.center_LEN.mp4",
    "plotenv-unicycle1_v0-kink_0-solution.mp4.center_LEN.mp4",
    "plotenv-unicycle1_v0-parallelpark_0-solution.mp4.center_LEN.mp4",
    "plotenv-unicycle1_v1-kink_0-solution.mp4.center_LEN.mp4",
    "plotenv-unicycle1_v2-wall_0-solution.mp4.center_LEN.mp4",
    "plotenv-unicycle2_v0-bugtrap_0-solution.mp4.center_LEN.mp4",
    "plotenv-unicycle2_v0-kink_0-solution.mp4.center_LEN.mp4",
    "plotenv-unicycle2_v0-parallelpark_0-solution.mp4.center_LEN.mp4",
]

videos = [f"_tmp/{v}" for v in videos]
random.shuffle(videos)

videos_selected = [
    "plotenv-acrobot_v0-swing_up_obs_hard-solution.mp4.center_LEN.mp4",
    "plotenv-acrobot_v0-swing_up_empty-solution.mp4.center_LEN.mp4",
    "plotenv-car1_v0-kink_0-solution.mp4.center_LEN.mp4",
    "plotenv-car1_v0-parallelpark_0-solution.mp4.center_LEN.mp4",
    "plotenv-quad2d_v0-fall_through-solution.mp4.center_LEN.mp4",
    "plotenv-quad2d_v0-quad_bugtrap-solution.mp4.center_LEN.mp4",
    "plotenv-quad2dpole_v0-up_obs-solution.mp4.center_LEN.mp4",
    "plotenv-quad2dpole_v0-window_hard-solution.mp4.center_LEN.mp4",
    "plotenv-quadrotor_v0-recovery-solution.mp4.center_LEN.mp4",
    "plotenv-quadrotor_v0-recovery_with_obs-solution.mp4.center_LEN.mp4",
    "plotenv-quadrotor_v1-quad_one_obs-solution.mp4.center_LEN.mp4",
    "plotenv-quadrotor_v1-window-solution.mp4.center_LEN.mp4",
    "plotenv-unicycle1_v2-wall_0-solution.mp4.center_LEN.mp4",
    "plotenv-unicycle1_v0-bugtrap_0-solution.mp4.center_LEN.mp4",
    "plotenv-unicycle2_v0-bugtrap_0-solution.mp4.center_LEN.mp4",
    "plotenv-unicycle2_v0-parallelpark_0-solution.mp4.center_LEN.mp4",
]

# videos_selected = [f"_tmp/{v}" for v in videos_selected]
# videos_selected = [f"_tmp/{v}" for v in videos]
random.shuffle(videos_selected)


nrows = 4
ncols = 10

time_stamp = time.strftime("%Y-%m-%d--%H-%M-%S")
video_out = f"vid_grid_{nrows}x{ncols}_{time_stamp}.mp4"

generate_grid(videos[: nrows * ncols], nrows, ncols, video_out)


# random.shuffle(videos_selected)
#
# nrows = 2
# ncols = 8
# video_out = f"vid_grid_{nrows}x{ncols}_{time_stamp}.mp4"
# generate_grid(videos_selected[:nrows * ncols], nrows , ncols, video_out)
