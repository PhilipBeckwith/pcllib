#pragma once
#include "includes.h"
#include "pclCluster.h"
#include "GaussSmoothen.h"
#include "math.h"
#include "Smooth.h"

class PointCanopy
{
	const static float floor =-20;
	std::vector<std::vector<pcl::PointXYZ> > pointField;
	pcl::PointXYZ minPoints;
	pclCluster cloud;	
	bool mkfloor;
	public:
	//constructors
	PointCanopy();
	PointCanopy(pclCluster cloudIN);
	
	void makeFloor(int canopyRatio);
	void setCloud(pclCluster cloudIN);
	void prepareCloud(int dec);
	void initalizeField(int dec);
	void makeCanopy(int dec);
	void setHeight(int x, int y, float z);
	void mend(std::vector<double>values, int i);
	void smooth(double sigma, int samples);
	void smooth(int buffer);
	int fillGaps(int skip);
	int fillGapsX(int skip);
	int fillGapsY(int skip);
	std::vector<double> rip(int i);
	pclCluster getCanopy();
	void emptyCanopy();
};



//constructor
PointCanopy::PointCanopy(){mkfloor=false;}
PointCanopy::PointCanopy(pclCluster cloudIN)
{
	mkfloor=false;
	cloud=cloudIN;
}

void PointCanopy::setCloud(pclCluster cloudIN)
{
	cloud=cloudIN;
}

void PointCanopy::prepareCloud(int dec)
{
	minPoints.x=cloud.minX;
	minPoints.y=cloud.minY;
	
	if(mkfloor){minPoints.z=cloud.minZ;}
	else{minPoints.z=cloud.maxZ;}
	
	cloud.translateX(0);
	cloud.translateY(0);
	cloud.findSize();
	
	if(mkfloor)
	{
		for(int n=0; n<cloud.cloud->points.size(); n++)
		{
			cloud.cloud->points[n].z*=-1;
		}
		cloud.findSize();
	}
	cloud.translateZ(0);
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

void PointCanopy::makeFloor(int canopyRatio)
{
	mkfloor=true;
	makeCanopy(canopyRatio);
}

void PointCanopy::makeCanopy(int dec)
{
	int x, y;
	float z;
	prepareCloud(dec);
	initalizeField(dec);
	
	for(int i=0; i<cloud.cloud->points.size(); i++)
	{
		x=(int) round(cloud.cloud->points[i].x * dec);
		y=(int) round(cloud.cloud->points[i].y * dec);
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
	cloud.crop("z", 300, -1);
	cloud.translateX(minPoints.x);
	cloud.translateY(minPoints.y);
	
	if(mkfloor)
	{
		for(int n=0; n<cloud.cloud->points.size(); n++)
		{
			cloud.cloud->points[n].z*=-1;
		}		
	}
	
	cloud.findSize();
	
	if(!mkfloor)
	{
		minPoints.z-=(cloud.maxZ-cloud.minZ);
	}
	
	cloud.findSize();
	cloud.translateZ(minPoints.z);
			

	return cloud;
}

std::vector<double> PointCanopy::rip(int i)
{
	std::vector<double> values;
	double z;
	for(int j=0; j<pointField[i].size(); j++)
	{
		z=(double) pointField[i][j].z;
		if(z!=floor){
			values.push_back(z);}
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



void PointCanopy::smooth(int buffer)
{
	std::vector<double> values;
	for(int i=0; i<pointField.size(); i++)
	{
		values = rip(i);
		values = simpleSmooth(values, buffer);
		mend(values, i);
		values.clear();
	}
}


void PointCanopy::mend(std::vector<double>values, int i)
{
	int index=0;
	for(int j=0; j<pointField[i].size(); j++)
	{
		if(pointField[i][j].z!= floor)
		{
			pointField[i][j].z= (float) values[index];
			index ++;
		}
	}
}


int PointCanopy::fillGapsX(int skip)
{ 
	std::cout<<"\nfill Gaps X";
	int gaps =0;
	int width = pointField.size();
	int length = pointField[0].size();
	float next, last;
	for(int i=0; i < length; i++)
	{
		next=last=floor;
		for(int j=skip; j<width-skip; j++)
		{
			next = pointField[j+skip][i].z;
			last = pointField[j-skip][i].z;
			if(pointField[j][i].z == floor)
			{
				gaps++;
				if(last!=floor && next !=floor)
					{
						gaps--;
						pointField[j][i].z= (next+last)/2;
						
					}
			}
		}
		
	}
	
	return gaps;
}



int PointCanopy::fillGapsY(int skip)
{ 
	std::cout<<"\nfill Gaps Y";
	int gaps =0;
	int width = pointField.size();
	int length = pointField[0].size();
	float next, last;
	for(int i=0; i < width; i++)
	{
		next=last=floor;
		for(int j=skip; j<length-skip; j++)
		{
			next = pointField[i][j+skip].z;
			last = pointField[i][j-skip].z;
			if(pointField[i][j].z == floor)
			{
				gaps++;
				if(last!=floor && next !=floor)
					{
						gaps--;
						pointField[i][j].z= (next+last)/2;
						
					}
			}
		}
		
	}
	
	return gaps;
}



int PointCanopy::fillGaps(int skip)
{
	int gaps =0;
	fillGapsX(skip);
	gaps = fillGapsY(skip);
	return gaps;
}


void PointCanopy::emptyCanopy()
{

	for(int i=0; i<pointField.size(); i++)	
	{
		pointField[i].clear();	
	}
	pointField.clear();
	cloud.cloud.reset();
	
}











