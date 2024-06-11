## 0. Import Library
import numpy as np
import time
import open3d
import pandas as pd
import matplotlib.pyplot as plt

# 1. Load Data
pcd = open3d.io.read_point_cloud('test_files/sdc.pcd')

# 2. Voxel downsampling
print(f"Points before downsampling: {len(pcd.points)} ")
pcd = pcd.voxel_down_sample(voxel_size=0.1)
print(f"Points after downsampling: {len(pcd.points)}")
# open3d.visualization.draw_geometries([pcd])

# 3. RANSAC Segmentation
plane_model, inliers = pcd.segment_plane(distance_threshold=0.3, ransac_n=3, num_iterations=150)
## Identify inlier points
inlier_cloud = pcd.select_by_index(inliers)
inlier_cloud.paint_uniform_color([0, 1, 1])
## Identify outlier points
outlier_cloud = pcd.select_by_index(inliers, invert=True)
outlier_cloud.paint_uniform_color([1, 0, 0])
# open3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

# 4. Clustering using DBSCAN
with open3d.utility.VerbosityContextManager(open3d.utility.VerbosityLevel.Debug) as cm:
    labels = np.array(outlier_cloud.cluster_dbscan(eps=0.45, min_points=7, print_progress=False))

max_label = labels.max()
print(f"point cloud has {max_label + 1} clusters")
colors = plt.get_cmap("tab20")(labels/(max_label if max_label>0 else 1))
colors[labels<0] = 0
outlier_cloud.colors = open3d.utility.Vector3dVector(colors[:, :3])
open3d.visualization.draw_geometries([outlier_cloud])

# 5. Generate 3D Bounding Boxes
obbs = []
indexes = pd.Series(range(len(labels))).groupby(labels, sort=False).apply(list).tolist()

MAX_POINTS = 300
MIN_POINTS = 40

for i in range(0, len(indexes)):
    nb_points = len(outlier_cloud.select_by_index(indexes[i]).points)
    if (nb_points>MIN_POINTS and nb_points<MAX_POINTS):
        sub_cloud = outlier_cloud.select_by_index(indexes[i])
        obb = sub_cloud.get_axis_aligned_bounding_box()
        obb.color = (0, 0, 1)
        obbs.append(obb)

print(f"Number of Bounding Boxes calculated {len(obbs)}")

list_of_visuals = []
list_of_visuals.append(outlier_cloud)
list_of_visuals.extend(obbs)
list_of_visuals.append(inlier_cloud)

print(type(pcd))
print(type(list_of_visuals))
open3d.visualization.draw_geometries(list_of_visuals)
