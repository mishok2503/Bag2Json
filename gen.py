import math

import numpy as np
import json

N = 384
n = 20000

SEC = 1000000000
dtn = 100
dt = dtn / SEC


def to_euler_angles(q):
    x, y, z, w = q

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)

    sinp = math.sqrt(1 + 2 * (w * y - x * z))
    cosp = math.sqrt(1 - 2 * (w * y - x * z))

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)

    return np.array([
        math.atan2(sinr_cosp, cosr_cosp),
        2 * math.atan2(sinp, cosp) - math.pi / 2,
        math.atan2(siny_cosp, cosy_cosp)
    ])


class LiderData:
    def __init__(self, secs, nsecs, points):
        self.secs = secs
        self.nsecs = nsecs
        self.points = points


class IMU:
    def __init__(self, secs, nsecs, pos, q):
        self.secs = secs
        self.nsecs = nsecs
        self.pos = pos
        self.rot = to_euler_angles(q)


with open("data/result", "r") as result:
    with open("result.json", "w") as res_f:

        def get_lidar_data():
            res = []
            for _ in range(N):
                p = list(map(float, result.readline().split()))
                pt = "point"
                if p == [0, 0, 0]:
                    pt = "unknown"
                if np.linalg.norm(p) > 30:
                    pt = "maximum"
                res.append({
                    "coordinates": p,
                    "type": pt
                })
            return res

        ms = []
        scan_num = 0
        nl = 5
        while scan_num < n:
            points = []
            for _ in range(nl):
                points += get_lidar_data()
            m = {
                "lidar_data": points,
                "odometry": {
                    "position": [0, 0, 0],
                    "euler_angles": [0, 0, 0]
                }
            }
            ms.append(m)
            scan_num += nl
            print(scan_num)

        json.dump({"data": {"measurements": ms}}, res_f)
