import open3d as o3d
import numpy as np
import struct
size_float = 4
list_pcd = []

file_to_open = "/Users/jeremycohen/Downloads/Point Clouds/Datasets/data_object_velodyne/testing/velodyne/000020.bin"
file_to_save = "/Users/jeremycohen/Downloads/Point Clouds/test_files/000020.pcd"
with open (file_to_open, "rb") as f:
    byte = f.read(size_float*4)
    while byte:
        x,y,z,intensity = struct.unpack("ffff", byte)
        list_pcd.append([x, y, z])
        byte = f.read(size_float*4)
np_pcd = np.asarray(list_pcd)
pcd = o3d.geometry.PointCloud()
v3d = o3d.utility.Vector3dVector
pcd.points = v3d(np_pcd)

o3d.io.write_point_cloud(file_to_save, pcd)
