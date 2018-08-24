#pragma once


//std lib
#include <iostream>
#include <vector>
#include <string>
#include <cmath> 


//pcl lib
#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/don.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#include <pcl/surface/mls.h>



#include <bits/stdc++.h>

//for meshing


//boost lib
#include <boost/thread/thread.hpp>


