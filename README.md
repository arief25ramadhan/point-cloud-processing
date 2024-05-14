# Point Cloud Processing using Open3D
We are going to deal with point cloud (again). Point cloud, a collection of 3D data acquired through LiDAR, is a very common thing in self-driving car. We will learn how to process point cloud using the Open3D library.

There are five steps in this, which are:

Load point cloud data
Voxel down sample the point cloud
Segmenting the obstacle in the point cloud
Identify individual obstacle through clustering
Generating bounding box
## 1. Install the Open3D Library
```
pip install open3d        # or
pip install open3d-cpu    # Smaller CPU only wheel on x86_64 Linux (since v0.17+)
```

## 2. Load Point Cloud Data
First, we will load using the open3d library. The data that we want to process illustrates data used by self driving car as shown by figure 1.

```
import numpy as np
import time
import open3d
import pandas as pd
import matplotlib.pyplot as plt
 
pcd = open3d.io.read_point_cloud('test_files/sdc.pcd')
```

## 3. Voxel down sample
Voxel downsampling is a technique used to reduce the number of points in a point cloud by averaging points within a 3D grid of fixed-size cubes, called voxels. Each voxel replaces all the points it contains with a single representative point, typically the centroid. This reduces the data size while preserving the overall structure and features of the point cloud, making it more efficient for processing and analysis.

```
print(f"Points before downsampling: {len(pcd.points)} ")
pcd = pcd.voxel_down_sample(voxel_size=0.1)
print(f"Points after downsampling: {len(pcd.points)}")# DOWNSAMPLING
# open3d.visualization.draw_geometries([pcd])

```

## 4. Segmentation using RANSAC
RANSAC (Random Sample Consensus) is an iterative algorithm used to estimate the parameters of a mathematical model from a dataset containing outliers, which are data points that do not fit the model. In the context of point clouds, RANSAC is commonly used for tasks like plane fitting or other geometric shape fitting.

```
# ## CHALLENGE 3 - SEGMENTATION: Using RANSAC (Random Sampling Consensus), to differentiate between ground and obstacles
plane_model, inliers = pcd.segment_plane(distance_threshold=0.3, ransac_n=3, num_iterations=150)
inlier_cloud = pcd.select_by_index(inliers)
inlier_cloud.paint_uniform_color([0, 1, 1])
outlier_cloud = pcd.select_by_index(inliers, invert=True)
outlier_cloud.paint_uniform_color([1, 0, 0])
# open3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

```

## 5. Clustering using DBSCAN
DBSCAN (Density-Based Spatial Clustering of Applications with Noise) is a clustering algorithm that groups points based on density. It identifies core points as those having a minimum number of neighbors (MinPts) within a specified distance (epsilon, ε). Clusters are formed by connecting core points and their neighbors, including any points reachable from core points. Points that are not reachable from any core point are classified as noise (outliers).

Key parameters include epsilon, the maximum distance to consider points as neighbors, and MinPts, the minimum number of points to form a dense region. DBSCAN is advantageous for finding clusters of arbitrary shapes and effectively handling noise and outliers. This makes it useful in fields like geospatial analysis and anomaly detection due to its robustness and minimal parameter requirements.

```
# ## CHALLENGE 4 - CLUSTERING USING DBSCAN
with open3d.utility.VerbosityContextManager(open3d.utility.VerbosityLevel.Debug) as cm:
    labels = np.array(outlier_cloud.cluster_dbscan(eps=0.45, min_points=7, print_progress=False))
 
max_label = labels.max()
print(f"point cloud has {max_label + 1} clusters")
colors = plt.get_cmap("tab20")(labels/(max_label if max_label>0 else 1))
colors[labels<0] = 0
outlier_cloud.colors = open3d.utility.Vector3dVector(colors[:, :3])
open3d.visualization.draw_geometries([outlier_cloud])
```

## 6. Bounding Box using PCA
Principal Component Analysis (PCA) can be used to compute an oriented bounding box for a point cloud. This process involves transforming the point cloud to align with its principal components, allowing for a tighter and more representative bounding box.

```
# 3D Bounding Boxes
obbs = []
indexes = pd. Series(range(len(labels))).groupby(labels, sort=False).apply(list).tolist()
 
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
```

## 7. Conclusion
In this project, we have created a code to process point cloud data using the Open3D Library. We perform voxel downsampling to reduce the number of point cloud that we need to process. Using RANSAC we identify the obstacle within the area. We segment, cluster, and build bounding boxes. The full code of this project can be find in this repository.

## Reference
This tutorial is heavily based on thinkautonomous point cloud course and Open3D official documentation. The original repository and the can be found here.