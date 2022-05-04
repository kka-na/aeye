import itertools
from typing import Any, Dict, Tuple

import numpy as np

eon_f_frame_size = (1164, 874)
eon_f_focal_length = 910.0
tici_f_frame_size = (1928, 1208)
tici_f_focal_length = 2648.0


RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)


class UIParams:
    lidar_x, lidar_y, lidar_zoom = 384, 960, 6
    lidar_car_x, lidar_car_y = lidar_x / 2., lidar_y / 1.1
    car_hwidth = 1.7272 / 2 * lidar_zoom
    car_front = 2.6924 * lidar_zoom
    car_back = 1.8796 * lidar_zoom
    car_color = 110


UP = UIParams

_BB_TO_FULL_FRAME = {}
_FULL_FRAME_TO_BB = {}

eon_f_qcam_frame_size = (480, 360)
tici_f_qcam_frame_size = (528, 330)

cams = [(eon_f_frame_size, eon_f_focal_length, eon_f_frame_size),
        (tici_f_frame_size, tici_f_focal_length, tici_f_frame_size),
        (eon_f_qcam_frame_size, eon_f_focal_length, eon_f_frame_size),
        (tici_f_qcam_frame_size, tici_f_focal_length, tici_f_frame_size)]
for size, focal, full_size in cams:
    sz = size[0] * size[1]
    _BB_SCALE = size[0] / 640.
    _BB_TO_FULL_FRAME[sz] = np.asarray([
        [_BB_SCALE, 0., 0.],
        [0., _BB_SCALE, 0.],
        [0., 0., 1.]])
    _FULL_FRAME_TO_BB[sz] = np.linalg.inv(_BB_TO_FULL_FRAME[sz])


_COLOR_CACHE: Dict[Tuple[int, int, int], Any] = {}


def find_color(lidar_surface, color):
    if color in _COLOR_CACHE:
        return _COLOR_CACHE[color]
    tcolor = 0
    ret = 255
    for x in lidar_surface.get_palette():
        if x[0:3] == color:
            ret = tcolor
            break
        tcolor += 1
    _COLOR_CACHE[color] = ret
    return ret


def to_topdown_pt(y, x):
    px, py = x * UP.lidar_zoom + UP.lidar_car_x, -y * UP.lidar_zoom + UP.lidar_car_y
    if px > 0 and py > 0 and px < UP.lidar_x and py < UP.lidar_y:
        return int(px), int(py)
    return -1, -1


def draw_path(path, color, img, calibration, top_down, lid_color=None, z_off=0):
    x, y, z = np.asarray(path.x), np.asarray(
        path.y), np.asarray(path.z) + z_off
    pts = calibration.car_space_to_bb(x, y, z)
    pts = np.round(pts).astype(int)

    if lid_color is not None and top_down is not None:
        tcolor = find_color(top_down[0], lid_color)
        for i in range(len(x)):
            px, py = to_topdown_pt(x[i], y[i])
            if px != -1:
                top_down[1][px, py] = tcolor

    height, width = img.shape[:2]
    for x, y in pts:
        if 1 < x < width - 1 and 1 < y < height - 1:
            for a, b in itertools.permutations([-1, 0, -1], 2):
                img[y + a, x + b] = color
