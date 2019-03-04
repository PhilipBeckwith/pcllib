#pragma once
#include "includes.h"
#include "pclCluster.h"
#include "GaussSmoothen.h"
#include "math.h"
#include "Smooth.h"

class PointCanopy
{
	const static float floor =-20;
	std::vector<std::vector<pcl::PointXYZ> > *pointField;
	std::vector<std::vector<pcl::PointXYZ> > pointFieldCanopy;
	std::vector<std::vector<pcl::PointXYZ> > pointFieldFloor;

	pcl::PointCloud<pcl::PointXYZ>::Ptr canopy;
	pcl::PointCloud<pcl::PointXYZ>::Ptr ground;
	pcl::PointCloud<pcl::PointXYZ>::Ptr heights;

	pcl::PointXYZ minPoints;
	pclCluster cloud;
	pclCluster canopyCloud, groundCloud, heightCloud;
		
	bool mkground;
	
	void mkGroundCloud();
	void mkCanopyCloud();
	void mkHeightCloud();
	
	public:
	//constructors
	PointCanopy();
	PointCanopy(pclCluster cloudIN);
	
	void makeGround(int canopyRatio);
	void makeCanopy(int canopyRatio);
	void makeHeights(int canopyRatio);
	
	void setCloud(pclCluster cloudIN);
	
	void prepareCloud(int dec);
	void initalizeField(int dec);
	void make(int dec);
	void setHeight(int x, int y, pcl::PointXYZ point);
	void restoreCloud();
	
	void mend(std::vector<double>values, int i);
	void smooth(double sigma, int samples);
	void smooth(int buffer);
	int fillGaps(int skip);
	int fillGapsX(int skip);
	int fillGapsY(int skip);
	std::vector<double> rip(int i);
	
	pclCluster getCanopy();
	pclCluster getGround();
	pclCluster getHeights();
	
	void emptyCanopy();
};

pclCluster PointCanopy::getCanopy(){return canopyCloud;}
pclCluster PointCanopy::getGround(){return groundCloud;}
pclCluster PointCanopy::getHeights(){return heightCloud;}

//constructor
PointCanopy::PointCanopy()
{
	mkground=false;
}
PointCanopy::PointCanopy(pclCluster cloudIN)
{
	mkground=false;
	cloud=cloudIN;
}

void PointCanopy::setCloud(pclCluster cloudIN)
{
	cloud=cloudIN;
}

void PointCanopy::prepareCloud(int dec)
{
	cloud.findSize();
	minPoints.x=cloud.minX;
	minPoints.y=cloud.minY;
	
	if(mkground){minPoints.z=cloud.minZ;}
	else{minPoints.z=cloud.maxZ;}
	
	cloud.translateX(0);
	cloud.translateY(0);
	cloud.findSize();
	
	if(mkground)
	{
		for(int n=0; n<cloud.cloud->points.size(); n++)
		{
			cloud.cloud->points[n].z*=-1;
		}
		cloud.findSize();
	}
	cloud.translateZ(0);
}

void PointCanopy::restoreCloud()
{
	cloud.translateX(minPoints.x);
	cloud.translateY(minPoints.y);
	double cloudZ=minPoints.z;
	
	if(mkground)
	{
		for(int n=0; n<cloud.cloud->points.size(); n++)
		{
			cloud.cloud->points[n].z*=-1;
		}		
	}
	
	cloud.findSize();
	
	if(!mkground)
	{
		cloudZ-=(cloud.maxZ-cloud.minZ);
	}
	
	cloud.findSize();
	cloud.translateZ(cloudZ);
}

void PointCanopy::initalizeField(int dec)
{
	int width, length;
	
	width = int (cloud.maxX * dec)+1;
	length = int (cloud.maxY * dec)+1;
	
	pointField->resize(width);
	for(int i=0; i< width; i++)
	{
		pointField->at(i).resize(length);
		for(int j=0; j<length; j++)
		{
			pointField->at(i)[j].x=(i*1.0)/dec;
			pointField->at(i)[j].y=(j*1.0)/dec;
			pointField->at(i)[j].z=floor;
		}
	}
}

void PointCanopy::makeGround(int canopyRatio)
{
	mkground=true;
	pointField=&pointFieldFloor;
	make(canopyRatio);
	mkGroundCloud();
}

void PointCanopy::makeCanopy(int canopyRatio)
{
	mkground=false;
	pointField=&pointFieldCanopy;
	make(canopyRatio);
	mkCanopyCloud();
}

void PointCanopy::makeHeights(int canopyRatio)
{
	//makeCanopy(canopyRatio);
	//makeGround(canopyRatio);
	mkHeightCloud();
}

void PointCanopy::make(int dec)
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
		setHeight(x,y,cloud.cloud->points[i]);
	}
	restoreCloud();
}

void PointCanopy::setHeight(int x, int y, pcl::PointXYZ point)
{
	if(x>=pointField->size()){x=pointField->size()-1;}
	if(x<0){x=0;}
	if(y>=pointField->at(x).size()){y=pointField->at(x).size()-1;}
	if(y<0){y=0;}
	if(point.z>pointField->at(x)[y].z)
	{
		pointField->at(x)[y].x=point.x;
		pointField->at(x)[y].y=point.y;
		pointField->at(x)[y].z=point.z;
	}
	
}

void PointCanopy::mkGroundCloud()
{
pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
	ground=temp;
	
	for(int i=0; i<pointFieldFloor.size(); i++){
		for(int j=0; j< pointFieldFloor[i].size(); j++){
			ground->points.push_back(pointFieldFloor[i][j]);
		}
	}
	
	ground->width = canopy->points.size();
	ground->height = 1;
	ground->is_dense = true;
	
	groundCloud.cloud=ground;
	groundCloud.findSize();
	groundCloud.translateX(minPoints.x);
	groundCloud.translateY(minPoints.y);
	
	for(int n=0; n<groundCloud.cloud->points.size(); n++)
	{
		groundCloud.cloud->points[n].z*=-1;
	}			
	
	groundCloud.findSize();
	groundCloud.translateZ(minPoints.z);
}

void PointCanopy::mkCanopyCloud()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
	canopy=temp;
	
	for(int i=0; i<pointField->size(); i++){
		for(int j=0; j< pointField->at(i).size(); j++){
			canopy->points.push_back(pointField->at(i)[j]);
		}
	}
	
	canopy->width = canopy->points.size();
	canopy->height = 1;
	canopy->is_dense = true;
	
	canopyCloud.cloud=canopy;
	canopyCloud.findSize();
	canopyCloud.translateX(minPoints.x);
	canopyCloud.translateY(minPoints.y);
	canopyCloud.findSize();
	minPoints.z-=(canopyCloud.maxZ-canopyCloud.minZ);
	canopyCloud.translateZ(minPoints.z);
}


void PointCanopy::mkHeightCloud()
{
	if(canopy->points.size()==ground->points.size())
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
		heights=temp;
		for(int i=0; i<canopy->points.size(); i++)
		{
			pcl::PointXYZ point;
			point=canopy->points[i];
			point.z-=ground->points[i].z;
			heights->points.push_back(point);
		}
		heights->width = canopy->points.size();
		heights->height = 1;
		heights->is_dense = true;
	}
	heightCloud.cloud=heights;
}

std::vector<double> PointCanopy::rip(int i)
{
	std::vector<double> values;
	double z;
	for(int j=0; j<pointField->at(i).size(); j++)
	{
		z=(double) pointField->at(i)[j].z;
		if(z!=floor){
			values.push_back(z);}
	}	
	return values;
}

void PointCanopy::smooth(double sigma, int samples)
{
	std::vector<double> values;
	for(int i=0; i<pointField->size(); i++)
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
	for(int i=0; i<pointField->size(); i++)
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
	for(int j=0; j<pointField->at(i).size(); j++)
	{
		if(pointField->at(i)[j].z!= floor)
		{
			pointField->at(i)[j].z= (float) values[index];
			index ++;
		}
	}
}


int PointCanopy::fillGapsX(int skip)
{ 
	std::cout<<"\nfill Gaps X";
	int gaps =0;
	int width = pointField->size();
	int length = pointField->at(0).size();
	float next, last;
	for(int i=0; i < length; i++)
	{
		next=last=floor;
		for(int j=skip; j<width-skip; j++)
		{
			next = pointField->at(j+skip)[i].z;
			last = pointField->at(j+skip)[i].z;
			if(pointField->at(j)[i].z == floor)
			{
				gaps++;
				if(last!=floor && next !=floor)
					{
						gaps--;
						pointField->at(j)[i].z= (next+last)/2;
						
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
	int width = pointField->size();
	int length = pointField->at(0).size();
	float next, last;
	for(int i=0; i < width; i++)
	{
		next=last=floor;
		for(int j=skip; j<length-skip; j++)
		{
			next = pointField->at(i)[j+skip].z;
			last = pointField->at(i)[j-skip].z;
			if(pointField->at(i)[j].z == floor)
			{
				gaps++;
				if(last!=floor && next !=floor)
					{
						gaps--;
						pointField->at(i)[j].z= (next+last)/2;
						
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

	for(int i=0; i<pointField->size(); i++)	
	{
		pointField->at(i).clear();	
	}
	pointField->clear();
	cloud.cloud.reset();
	
}











