#pragma once
#include "downsamp.h"
#include "donFilter.h"
#include "includes.h"


class pclCluster
{
	//conversion from m to cm
	const static int ratio=100;	
	
	const static double yWeight=1000000, xWeight=1000, zWeight=1;

	public:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::PointXYZ center;

	int depth, maxDepth;
	
	double height, width, length, volume;
	double maxX, maxY, maxZ;
	double minX,minY,minZ;
	double avgX, avgY,avgZ;

	//constructors
	pclCluster();
	pclCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud);

	//opens a point cloud.
	void open(std::string fileName);

	//saves a cloud
	void save(std::string fileName);

	//sets the height width length and area of the cluster
	void findSize();
	
	//clusters the cloud and returns the clusters in a vector
	std::vector<pclCluster > EuclideanCluster(int minSize, int maxSize, double tolerance);
	
	//stretches or compacts the cloud into a size
	void resize(double dimX, double dimY, double dimZ);

	//trims the cloud
	void crop(std::string dim, double max, double min);

	//uses a switch statment to return a specified field
	double getData(int x);
	double getDimAverage(int x);
	//Moves the Cloud to an area in space
	void translateCenter(double x, double y, double z);
	
	//selects a dimension to move
	void translate(char dim, double mv);
	//moves cloud to a starting location
	void translateX(double mvX);
	void translateY(double mvY);
	void translateZ(double mvZ);

	//reflect the cloud an axsis
	void reflect(char dim);
	
	//finds A Max and a min within a subset of the cloud
	void localizedMaxMin(char dim, double lowerLim, double upperLim, double *max, double *min);

	//removes outliers
	void removeOutliers(int pointNumb,double stdDevMul);
	
	//A way to use a variable to get a point value
	double getPointDim(int index, int dim);
	

	//calculates normals and returns a cloud of the normals
	pcl::PointCloud<pcl::PointXYZ>::Ptr getNormalCloud(float searchRad);
	pcl::PointCloud<pcl::Normal>::Ptr getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float searchRad);
	
	void printCluster();
	
	void cloudRound(int dec);
	void cloudSmooth(float searchRad);

};

////////////////////////////////


//constructors
pclCluster::pclCluster(){}

//takes in a point cloud and finds it's attributes
pclCluster::pclCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud)
{
	cloud = inCloud;
	findSize();
	
}
///////////////////////////////


/*
	takes in a file name and opens it as a xyz point cloud.
	sets the point cloud as this cluster's cloud and finds it's attributes.
*/
void pclCluster::open(std::string fileName)
{
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp (new pcl::PointCloud<pcl::PointXYZ>);
	
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *temp) == -1)
	{
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr Error (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointXYZ OOPS;
		Error->points.push_back(OOPS);
		Error->points[0].x=-1;
		Error->points[0].y=-1;
		Error->points[0].z=-1;
	}
		
	else
	{
		//if file was opened 
		cloud= temp;
		findSize();
	}
}


void pclCluster::save(std::string fileName)
{
	pcl::io::savePCDFileASCII(fileName, *cloud);
}

//finds the height width length and area of the cluster
void pclCluster::findSize()
{
	
	//initalizes the points
	maxX= minX = cloud->points[0].x;
	maxY= minY = cloud->points[0].y;
	maxZ= minZ = cloud->points[0].z;
	double sumX=0, sumY=0, sumZ=0;	

	for (size_t i = 0; i < cloud->points.size (); ++i)
	{
		double tempX= cloud->points[i].x;
		double tempY= cloud->points[i].y;
		double tempZ= cloud->points[i].z;
		
		//finds min/max x
		if(tempX < minX){minX=tempX;}
		if(tempX > maxX){maxX=tempX;}
		
		//finds min/max y
		if(tempY < minY){minY=tempY;}
		if(tempY > maxY){maxY=tempY;}
		
		//finds min/max z
		if(tempZ < minZ){minZ=tempZ;}
		if(tempZ > maxZ){maxZ=tempZ;}
		
		sumX+= tempX;
		sumY+= tempY;
		sumZ+= tempZ;
		
	}
	
	avgX = sumX/ cloud->points.size();
	avgY = sumY/ cloud->points.size();
	avgZ = sumZ/ cloud->points.size();

	//finds the length in each dimention
	width= maxX-minX;
	length = maxY-minY;
	height= maxZ-minZ;

	//finding the center point
	center.x= minX+(width/2.0);
	center.y= minY+(length/2.0);
	center.z= minZ+(height/2.0);
	
	//converting from m to mm
	width = width*ratio;
	length =length*ratio;
	height=height *ratio;
	

	//calculating volume
	volume = width*length*height;

}

/*
	uses a Euclidian algorithm to cluster a cloud.
	stores all clusters in a vector
	returns the vector
*/
std::vector< pclCluster > pclCluster::EuclideanCluster(int minSize, int maxSize, double tolerance)
{
	//creating the vector to be returned
	std::vector<pclCluster> clusters;

	//creating a KD search tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	
	// Setting the paramiters
	tree->setInputCloud (cloud);
	ec.setClusterTolerance (tolerance);
	ec.setMinClusterSize (minSize);
	ec.setMaxClusterSize (maxSize);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (cluster_indices);
	
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		{
			cloud_cluster->points.push_back (cloud->points[*pit]); //*
		}
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		
		pclCluster newCluster= pclCluster(cloud_cluster);

		clusters.push_back(newCluster);
	
	}
	
	return clusters;
}

/*
	resize a cluster to a certain length for every dimention
	unless zero is passed in for that peramiter
*/
void pclCluster::resize(double dimX, double dimY, double dimZ)
{
	
	
	for (size_t i = 0; i < cloud->points.size (); ++i)
	{
		if(dimX!=0)
		{
			//finds new x dimention of current point
			cloud->points[i].x= 
			(((cloud->points[i].x-minX)/(maxX-minX))*dimX)+minX;
		}
		if(dimY!=0)
		{
			//finds new y dimention of current point
			cloud->points[i].y= 
			(((cloud->points[i].y-minY)/(maxY-minY))*dimY)+minY;
		}
		if(dimZ!=0)
		{
			//finds new z dimention of current point
			cloud->points[i].z= 
			(((cloud->points[i].z-minZ)/(maxZ-minZ))*dimZ)+minZ;
		}
	}
	
}


/*
	removes all points beyond a domain of a given dimention
*/
void pclCluster::crop(std::string dim, double max, double min)
{
	//creating the cloud to be returned
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp (new pcl::PointCloud<pcl::PointXYZ>);

	//creating the pass though filter object
	pcl::PassThrough<pcl::PointXYZ> pass;
	//setting params
	pass.setInputCloud(cloud);
	pass.setFilterFieldName(dim);
	pass.setFilterLimits(min, max);
	pass.filter(*temp);
	
	//setting the new cloud as this cluster's cloud
	cloud = temp;
}


//a generic way of obtaining data from a cluster
double pclCluster::getData(int x)
{
	double data=0;
	switch(x)
	{
		case 1:
			data=0+cloud->width;
			break;
		case 2: 
			data = volume;
			break;
		case 3: 
			data = (double) center.z;
			break;
		case 4: 
			data = (double) center.y;
			break;
		case 5: 
			data = (double) center.x;
			break;
		
		
	}
	return data;
}


double pclCluster::getDimAverage(int x)
{
	double data =0;
	
	switch(x)
	{
		case 1:
			data = avgX;
			break;
		case 2:
			data = avgY;
			break;
		case 3:
			data = avgZ;
			break;
	}
	
	return data;
}

void pclCluster::translateCenter(double x, double y, double z)
{
	x=center.x-x;
	y=center.y-y;
	z=center.z-z;
	for (size_t i = 0; i < cloud->points.size (); ++i)
	{
		cloud->points[i].x=(cloud->points[i].x)-x;
		cloud->points[i].y=(cloud->points[i].y)-y;
		cloud->points[i].z=(cloud->points[i].z)-z;
	}
	findSize();	
}


void pclCluster::translate(char dim, double mv)
{
	if(dim =='X' || dim == 'x')
	{
		translateX(mv);	
	}
	if(dim =='Y' || dim == 'y')
	{
		translateY(mv);	
	}
	if(dim =='Z' || dim == 'z')
	{
		translateZ(mv);	
	}
	
	findSize();
}



void pclCluster::translateX(double mvX)
{
	mvX= minX-mvX;
	for (size_t i = 0; i < cloud->points.size (); ++i)
	{
		cloud->points[i].x=(cloud->points[i].x)-mvX;
	}
	
}

void pclCluster::translateY(double mvY)
{
	mvY= minY-mvY;
	for (size_t i = 0; i < cloud->points.size (); ++i)
	{
		cloud->points[i].y=(cloud->points[i].y)-mvY;
	}
	
}

void pclCluster::translateZ(double mvZ)
{
	mvZ= minZ-mvZ;
	for (size_t i = 0; i < cloud->points.size (); ++i)
	{
		cloud->points[i].z=(cloud->points[i].z)-mvZ;
	}
	
}


void pclCluster::reflect(char dim)
{
	
	for(int i=0; i< cloud->points.size(); i++)
	{
		switch(dim)
		{
			case 'x': case 'X':
				cloud->points[i].x=cloud->points[i].x *(-1);
				break;
			case 'y': case 'Y':
				cloud->points[i].y=cloud->points[i].y *(-1);
				break;
			case 'z': case 'Z':
				cloud->points[i].z=cloud->points[i].z *(-1);
				break;
		}
	}
}


void pclCluster::removeOutliers(int pointNumb,double stdDevMul)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp (new pcl::PointCloud<pcl::PointXYZ>);
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (pointNumb);
	sor.setStddevMulThresh (stdDevMul);
	sor.filter (*temp);
	cloud=temp;
}


void pclCluster::localizedMaxMin(char dim, double lowerLim, double upperLim, double *max, double *min)
{
	int dimVal=0;
	bool noMax=true, noMin=true;
	
	switch(dim)
	{
		case 'z': case 'Z': dimVal=0;
		break;
		case 'y': case 'Y': dimVal=1;
		break;
		case 'x': case 'X': dimVal=2;
		break;
	}
	
	//setting to mid point
	*max = *min = (getData(dimVal+3)/2) ;
	
	*max = *max-5;
	*min= *min+5;

	for(int i=0; i<cloud->points.size(); i++)
	{
		if(cloud->points[i].y> lowerLim && cloud->points[i].y <upperLim)
		{
			
			if(getPointDim(i,dimVal)>=*max){*max = getPointDim(i,dimVal); noMax=false;}
			if(getPointDim(i,dimVal)<=*min){*min = getPointDim(i,dimVal); noMin=false;}
		}
		if(cloud->points[i].y> upperLim){break;}
	}
	if(noMax){*max=0;}
	if(noMin){*min=0;}
}

double pclCluster::getPointDim(int index, int dim)
{
	double value;
	switch(dim)
	{
		case 0: value = (double) cloud->points[index].z;
		break;
		case 1: value = (double) cloud->points[index].y;
		break;
		case 2: value = (double) cloud->points[index].x;
		break;
	}
	return value;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr pclCluster::getNormalCloud(float searchRad)
{
	//creating new cloud to be returned.
	pcl::PointCloud<pcl::PointXYZ>::Ptr normalCloud(new pcl::PointCloud<pcl::PointXYZ>);
	
	//creating point normal object
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	
	cloud_normals = getNormals(cloud, searchRad);

	normalCloud->height =1;
	normalCloud->width = cloud->points.size();
	normalCloud->is_dense=true;
	normalCloud->points.resize(cloud->points.size());
	for(size_t i=0; i<normalCloud->points.size(); i++)
	{	
		normalCloud->points[i].x=  cloud_normals->points[i].normal_x;
		normalCloud->points[i].y=  cloud_normals->points[i].normal_y;
		normalCloud->points[i].z=  cloud_normals->points[i].normal_z;
	}

	return normalCloud;
}


pcl::PointCloud<pcl::Normal>::Ptr pclCluster::getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float searchRad)
{
 

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);
  ne.setViewPoint (0, 0, 10);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (searchRad);

  // Compute the features
  ne.compute (*cloud_normals);
  
  return cloud_normals;

}




void pclCluster::printCluster()
{
	cout<<"Index, Y, X, Z\n";
	for(int i=0; i<(*cloud).points.size(); i++)
	{
		cout<< i<<", "<< (*cloud).points[i].y;
		cout<<", "<< (*cloud).points[i].x;
		cout<<", "<< (*cloud).points[i].z<<endl;
	}
}


void pclCluster::cloudRound(int dec)
{
	float x,y,z;
	for(int i=0; i<cloud->size(); i++)
	{
		x=cloud->points[i].x;
		y=cloud->points[i].y;
		z=cloud->points[i].z;
				
		x= round(x*dec)/dec;
		y= round(y*dec)/dec;
		//z= round(z*dec)/dec;
		
		cloud->points[i].x=x;
		cloud->points[i].y=y;
		cloud->points[i].z=z;
	}
}


void pclCluster::cloudSmooth(float searchRad)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal> mls_points;
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	
	mls.setComputeNormals(true);
	mls.setInputCloud(cloud);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(searchRad);
	
	mls.process(mls_points);
	
	pcl::io::savePCDFile("smoothedNormals.pcd", mls_points);
}











