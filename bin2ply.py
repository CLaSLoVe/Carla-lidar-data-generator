
import open3d as o3d
import numpy as np

bin_url = r"D:\jd\WindowsNoEditor\PythonAPI\my\data\2022-07-07 10.19.16-checked\0.4\lidar\0000022462.bin"
ply_url = "0010869533.ply"

points = np.fromfile(bin_url, dtype="float32").reshape((-1, 4))

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points[:,:3])
o3d.io.write_point_cloud(ply_url, pcd)
