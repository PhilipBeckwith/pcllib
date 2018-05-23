#pragma once
#include "includes.h"
#include "pclCluster.h"

class PointCanopy
{
	std::vector<std::vector<pcl::PointXYZ> > pointField;
	pcl::PointXYZ center;
	pclCluster cloud;	


	//constructors
	PointCanopy();
	PointCanopy(pclCluster cloudIN);
	
	void prepareCloud(int dec);
	void initalizeField(int dec);
	void makeCanopy();
	void setHeight(int x, int y, float z);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr getCanopy();
};





