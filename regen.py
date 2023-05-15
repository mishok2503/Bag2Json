#! /usr/bin/env python3

import rosbag
from sensor_msgs.msg import PointCloud2, PointField, Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3

import json
import time
import struct
import copy

bag = rosbag.Bag('res.bag', 'w')
t = 1684107870

pfx = PointField()
pfx.name = "x"
pfx.offset = 0
pfx.datatype = 7
pfx.count = 1
pfy = PointField()
pfy.name = "y"
pfy.offset = 4
pfy.datatype = 7
pfy.count = 1
pfz = PointField()
pfz.name = "z"
pfz.offset = 8
pfz.datatype = 7
pfz.count = 1
pfs = [pfx, pfy, pfz]

a = Vector3()
a.x = 0
a.y = 0;
a.z = -1;

SEC = 1000000000
dtn = 7000
dt = dtn / SEC

nt = 0

q = 0
with open("result.json", "r") as f:
    data = json.load(f)["data"]
    for m in data["measurements"]:
        print(q)
        p = PointCloud2()
        h = Header()
        h.seq = 0
        nt += dtn
        if nt > SEC:
            nt -= SEC
            t += 1
        h.stamp.secs = t
        h.stamp.nsecs = nt
        h.frame_id = "horizontal_vlp16_link"
        p.header = copy.deepcopy(h)
        p.height = 1
        p.fields = pfs
        p.is_bigendian = False
        p.is_dense = True
        p.point_step = 12
        
        imu = Imu()

        h.frame_id = "imu_link"
        imu.header = h
        imu.linear_acceleration = a
        imu.orientation_covariance = [-1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        c = 0
        data = []
        for point in m["lidar_data"]:
            if point["type"] != "point":
                continue
            for x in point["coordinates"]:
                data += list(bytearray(struct.pack("f", x)))
            c += 1
        ea = m["odometry"]["euler_angles"]
        if ea != [0, 0, 0]:
            print("ROTATE")
            v = Vector3()
            v.x = 0
            v.y = 0
            v.z = ea[2] / dt
            imu.angular_velocity = v
        p.width = c
        p.row_step = c * 12
        p.data = data
        bag.write('horizontal_laser_3d', p)
        bag.write('imu', imu)
        q += 1
#        time.sleep(0.1)
#        if q == 10:
#            break


bag.close()
