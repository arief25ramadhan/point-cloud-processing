## IMPORT LIBRARIES
import numpy as np
import time
import open3d
import pandas as pd
import matplotlib.pyplot as plt

## USE http://www.open3d.org/docs/release/tutorial/Basic/

## CHALLENGE 1 - OPEN A FILE OF YOUR CHOICE AND VISUALIZE THE POINT CLOUD
# The supported extension names are: pcd, ply, xyz, xyzrgb, xyzn, pts.
pcd = open3d.io.read_point_cloud('test_files/sdc.pcd')

## IF YOU HAVE PPTK INSTALLED, VISUALIZE USING PPTK
#import pptk
#v = pptk.viewer(pcd.points)

## CHALLENGE 2 - VOXEL GRID DOWNSAMPLING
print(f"Points before downsampling: {len(pcd.points)} ")
pcd = pcd.voxel_down_sample(voxel_size=0.1)
print(f"Points after downsampling: {len(pcd.points)}")# DOWNSAMPLING

# open3d.visualization.draw_geometries([pcd])

# ## CHALLENGE 3 - SEGMENTATION: Using RANSAC (Random Sampling Consensus), to differentiate between ground and obstacles
start_time = time.time()

plane_model, inliers = pcd.segment_plane(distance_threshold=0.3, ransac_n=3, num_iterations=150)
inlier_cloud = pcd.select_by_index(inliers)
inlier_cloud.paint_uniform_color([0, 1, 1])
outlier_cloud = pcd.select_by_index(inliers, invert=True)
outlier_cloud.paint_uniform_color([1, 0, 0])
open3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

elapsed_time = time.time() - start_time
print("Elapsed time: {}".format(elapsed_time))


# _, inliers = 

# ## CHALLENGE 4 - CLUSTERING USING DBSCAN
# labels = 
# max_label= labels.max()
# print(f"point cloud has {max_label + 1} clusters")

# ## BONUS CHALLENGE - CLUSTERING USING KDTREE AND KNN INSTEAD
# pcd_tree = 

# ## CHALLENGE 5 - BOUNDING BOXES IN 3D
# bounding_boxes = 

# ## CHALLENGE 6 - VISUALIZE THE FINAL RESULTS
# list_of_visuals = 

# ## BONUS CHALLENGE 2 - MAKE IT WORK ON A VIDEO
