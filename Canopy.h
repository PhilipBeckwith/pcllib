#pragma once
#include "includes.h"
#include "pclCluster.h"

class PointCanopy
{
	std::vector<std::vector<pcl::PointXYZ> > pointField;
	pcl::PointXYZ center;
	pclCluster cloud;	

	public:
	//constructors
	PointCanopy();
	PointCanopy(pclCluster cloudIN);
	
	void setCloud(pclCluster cloudIN);
	void prepareCloud(int dec);
	void initalizeField(int dec);
	void makeCanopy(int dec);
	void setHeight(int x, int y, float z);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr getCanopy();
};



//constructor
PointCanopy::PointCanopy(){}
PointCanopy::PointCanopy(pclCluster cloudIN)
{
	cloud=cloudIN;
}

void PointCanopy::setCloud(pclCluster cloudIN)
{
	cloud=cloudIN;
}

void PointCanopy::prepareCloud(int dec)
{
	center=cloud.center;
	cloud.cloudRound(dec);
	cloud.translateX(0);
	cloud.translateY(0);
	cloud.findSize();
	
}

void PointCanopy::initalizeField(int dec)
{
	int width, length;
	
	width = int (cloud.maxX * dec);
	length = int (cloud.maxY * dec);
	
	pointField.resize(width);
	for(int i=0; i< width; i++)
	{
		pointField[i].resize(length);
		for(int j=0; j<length; j++)
		{
			pointField[i][j].x=(i*1.0)/dec;
			pointField[i][j].y=(j*1.0)/dec;
			pointField[i][j].z=-1.5;
		}
	}
}

void PointCanopy::makeCanopy(int dec)
{
	int x, y;
	float z;
	prepareCloud(dec);
	initalizeField(dec);
	
	for(int i=0; i<cloud.cloud->points.size(); i++)
	{
		x=int (cloud.cloud->points[i].x * dec);
		y=int (cloud.cloud->points[i].y * dec);
		z= cloud.cloud->points[i].z;
		setHeight(x,y,z);
	}
}

void PointCanopy::setHeight(int x, int y, float z)
{
	if(x<pointField.size() && x>=0){
		if(y<pointField[x].size() && y>=0){
			if(z>pointField[x][y].z){
				pointField[x][y].z=z;
			}
		}	
	}
}


pcl::PointCloud<pcl::PointXYZ>::Ptr PointCanopy::getCanopy()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr canopy(new pcl::PointCloud<pcl::PointXYZ>);
	
	for(int i=0; i<pointField.size(); i++){
		for(int j=0; j< pointField[i].size(); j++){
			canopy->points.push_back(pointField[i][j]);
		}
	}
	
	canopy->width = canopy->points.size();
	canopy->height = 1;
	canopy->is_dense = true;
	
	return canopy;
}




