#pragma once
#include "includes.h"


pcl::PointCloud<pcl::PointXYZ>::Ptr makeCircle(float diamiter, int sugestedPoints)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	
	float radious, height, master;
	master =diamiter/2;
	
	int thetaInc,phiInc;
	
	phiInc= (int) (180/sqrt(sugestedPoints));

	thetaInc = phiInc*2;
	
	
	
	for(int phi=91; phi<270; phi+=phiInc)
	{
		height= master*sin(phi*PI/180);
		radious= master*cos(phi*PI/180);
		for(int theta=0; theta <360; theta+=thetaInc)
		{
			pcl::PointXYZ point;
			point.z= height;
			point.y= radious*sin(theta*PI/180);
			point.x=radious*cos(theta*PI/180);
			cloud->points.push_back(point);
		}
	}
	
	cloud->width = cloud->points.size();
	cloud->height =1;
	cloud->is_dense=true;
	
	return cloud;
}





