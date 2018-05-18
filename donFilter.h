#pragma once

#include "includes.h"
using namespace pcl;
using namespace std;


pcl::PointCloud<PointXYZ>::Ptr donFilter(double scale1, double scale2, double threshold, pcl::PointCloud<PointXYZ>::Ptr cloud )
{
  ///The smallest scale to use in the DoN filter.
  //double scale1;

  ///The largest scale to use in the DoN filter.
  //double scale2;

  ///The minimum DoN magnitude to threshold by
  //double threshold;
  
  // Create a search tree, use KDTreee for non-organized data.
  pcl::search::Search<PointXYZ>::Ptr tree;
  if (cloud->isOrganized ())
  {
    tree.reset (new pcl::search::OrganizedNeighbor<PointXYZ> ());
  }
  else
  {
    tree.reset (new pcl::search::KdTree<PointXYZ> (false));
  }

  // Set the input pointcloud for the search tree
  tree->setInputCloud (cloud);

  if (scale1 >= scale2)
  {
    cerr << "Error: Large scale must be > small scale!" << endl;
    return cloud;
  }

  // Compute normals using both small and large scales at each point
  pcl::NormalEstimationOMP<PointXYZ, PointNormal> ne;
  ne.setInputCloud (cloud);
  ne.setSearchMethod (tree);

  /**
   * NOTE: setting viewpoint is very important, so that we can ensure
   * normals are all pointed in the same direction!
   */
  ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

  // calculate normals with the small scale
  cout << "Calculating normals for scale..." << scale1 << endl;
  pcl::PointCloud<PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<PointNormal>);

  ne.setRadiusSearch (scale1);
  ne.compute (*normals_small_scale);

  // calculate normals with the large scale
  cout << "Calculating normals for scale..." << scale2 << endl;
  pcl::PointCloud<PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<PointNormal>);

  ne.setRadiusSearch (scale2);
  ne.compute (*normals_large_scale);

  // Create output cloud for DoN results
  PointCloud<PointNormal>::Ptr doncloud (new pcl::PointCloud<PointNormal>);
  copyPointCloud<PointXYZ, PointNormal>(*cloud, *doncloud);

  cout << "Calculating DoN... " << endl;
  // Create DoN operator
  pcl::DifferenceOfNormalsEstimation<PointXYZ, PointNormal, PointNormal> don;
  don.setInputCloud (cloud);
  don.setNormalScaleLarge (normals_large_scale);
  don.setNormalScaleSmall (normals_small_scale);

  if (!don.initCompute ())
  {
    std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
    return cloud;
  }

  // Compute DoN
  don.computeFeature (*doncloud);

  // Save DoN features
  //pcl::PCDWriter writer;
  //writer.write<pcl::PointNormal> ("don.pcd", *doncloud, false); 

  // Filter by magnitude
  cout << "Filtering out DoN mag <= " << threshold << "..." << endl;

  // Build the condition for filtering
  pcl::ConditionOr<PointNormal>::Ptr range_cond (
    new pcl::ConditionOr<PointNormal> ()
    );
  range_cond->addComparison (pcl::FieldComparison<PointNormal>::ConstPtr (
                               new pcl::FieldComparison<PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold))
                             );
  // Build the filter
  pcl::ConditionalRemoval<PointNormal> condrem (range_cond);
  condrem.setInputCloud (doncloud);

  pcl::PointCloud<PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<PointNormal>);

  // Apply filter
  condrem.filter (*doncloud_filtered);

  doncloud = doncloud_filtered;

  // Save filtered output
  std::cout << "Filtered Pointcloud: " << doncloud->points.size () << " data points." << std::endl;

 // writer.write<pcl::PointNormal> ("don_filtered.pcd", *doncloud, false); 

	pcl::PointCloud<pcl::PointXYZ>::Ptr out (new pcl::PointCloud<pcl::PointXYZ>); 
	copyPointCloud<PointNormal, PointXYZ> (*doncloud, *out);	
	
	return out;


}





