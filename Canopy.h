#pragma once
#include "includes.h"
#include "pclCluster.h"
#include "GaussSmoothen.h"

class PointCanopy
{
	const static float floor =-1.5;
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
	void mend(std::vector<double>values, int i);
	void smooth(double sigma, int samples);
	void fillGaps();
	std::vector<double> rip(int i);
	pclCluster getCanopy();
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
			pointField[i][j].z=floor;
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
	cloud.translateCenter(center.x,center.y,center.z);
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


pclCluster PointCanopy::getCanopy()
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
	
	cloud.cloud=canopy;
	cloud.findSize();	

	return cloud;
}

std::vector<double> PointCanopy::rip(int i)
{
	std::vector<double> values;
	double z;
	for(int j=0; j<pointField[i].size(); j++)
	{
		z=(double) pointField[i][j].z;
		values.push_back(z);
	}	
	return values;
}

void PointCanopy::smooth(double sigma, int samples)
{
	std::vector<double> values;
	for(int i=0; i<pointField.size(); i++)
	{
		values = rip(i);
		values = gaussSmoothen(values, sigma, samples);
		mend(values, i);
		values.clear();
	}
}

void PointCanopy::mend(std::vector<double>values, int i)
{
	for(int j=0; j<pointField[i].size(); j++)
	{
		pointField[i][j].z= (float) values[j];
	}
}


void PointCanopy::fillGaps()
{ 
	int count=0,holes =0;
	int width = pointField.size();
	int length = pointField[0].size();
	float next, last;
	for(int i=0; i < length; i++){
		next=last=floor;
		for(int j=1; j<width-1; j++)
		{
			if(pointField[j][i+1].z!=floor)
				{next = pointField[j][i+1].z;}
			if(pointField[j][i-1].z!=floor)
				{last = pointField[j][i-1].z;}
			if(pointField[j][i].z == floor)
			{
				holes++;
				if(((next+last)/2) != floor)
					{
						pointField[j][i].z= (next+last)/2;
						count++;
					}
			}
		}
		for(int j=width-1; j>0; j--)
		{
			if(pointField[j][i+1].z!=floor)
				{next = pointField[j][i+1].z;}
			if(pointField[j][i-1].z!=floor)
				{last = pointField[j][i-1].z;}
			if(pointField[j][i].z == floor)
			{
				if(((next+last)/2) != floor)
					{
						pointField[j][i].z= (next+last)/2;
						count++;
					}
			}
		}
	}
	
	std::cout<<"Filled "<<count<<" Gaps of "<<holes<<" holes\n";
	
}


















