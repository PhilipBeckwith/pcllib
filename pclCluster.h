#pragma once
#include "downsamp.h"
#include "donFilter.h"
#include "includes.h"


class pclCluster
{
	//conversion from m to cm
	const static int ratio=100;	
	
	const static float yWeight=10000000, xWeight=10000, zWeight=1;

	public:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::PointXYZ center;

	int depth, maxDepth;
	
	double height, width, length, volume;
	double maxX, maxY, maxZ;
	double minX,minY,minZ;

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
	
	//findsAMaxand a min within a subset of the cloud
	void localizedMaxMin(char dim, double lowerLim, double upperLim, double *max, double *min);

	//removes outliers
	void removeOutliers(int pointNumb,double stdDevMul);
	
	//A way to use a variable to get a point value
	double getPointDim(int index, int dim);
	
	//used for sorting the array 
	void swap(pcl::PointXYZ* a,pcl::PointXYZ* b);
	
	/* This function takes last element as pivot, places
   the pivot element at its correct position in sorted
    array, and places all smaller (smaller than pivot)
   to left of pivot and all greater elements to right
   of pivot */
	int partition (std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > points, int low, int high);
	void quickSort(std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > points, int low, int high);
	void doQuickSort();
	void printCluster();
	
	
	//Heap Sort 
	void doHeapSort();
	void heapSort(std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > points, int n);
	void heapify(std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > points, int n, int i);

	pcl::PointCloud<pcl::PointXYZ>::Ptr extractHull(int pointsToCosider);
	pcl::PointCloud<pcl::PointXYZ>::Ptr extractSurface(int decPlaces);
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
	cout<<"opening File..."<<endl;
	
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
		
		
	}

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
			data = center.z;
			break;
		case 4: 
			data = center.y;
			break;
		case 5: 
			data = center.x;
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
	bool dimX, dimY, dimZ;
	dimX = dimY = dimZ = false;
	
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

// A utility function to swap two elements
void pclCluster::swap( pcl::PointXYZ* a,  pcl::PointXYZ* b)
{

    pcl::PointXYZ t = *a;
    *a = *b;
    *b = t;
}


/* This function takes last element as pivot, places
   the pivot element at its correct position in sorted
    array, and places all smaller (smaller than pivot)
   to left of pivot and all greater elements to right
   of pivot */
int pclCluster::partition (std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > points, int low, int high)
{
	pcl::PointXYZ p=points[high];
	double pivot, test;
	pivot = (p.y*yWeight+p.x*xWeight+p.z*zWeight);    // pivot
	int i = (low - 1);  // Index of smaller element

    for (int j = low; j <= high- 1; j++)
    {
        // If current element is smaller than or
        // equal to pivot
        
        p=points[j];
        test= (p.y*yWeight+p.x*xWeight+p.z*zWeight); 
        
        if (test <= pivot)
        {
            i++;    // increment index of smaller element
            swap(&points[i], &points[j]);
        }
    }
    swap(&points[i + 1], &points[high]);
    return (i + 1);
}


/* The main function that implements QuickSort
 arr[] --> Array to be sorted,
  low  --> Starting index,
  high  --> Ending index */
void pclCluster::quickSort(std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > points, int low, int high)
{
	depth++;
	std::cout<<depth<<endl;
    if (low < high)
    {
        /* pi is partitioning index, arr[p] is now
           at right place */
        int pi = partition(points, low, high);

        // Separately sort elements before
        // partition and after partition
        quickSort(points, low, pi - 1);
        quickSort(points, pi + 1, high);
    }
	depth--;
}


void pclCluster::doQuickSort()
{
	quickSort((*cloud).points, 0, (*cloud).points.size()-1);
	//make sure to change"is sorted" to true in the cloud
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



// C++ program for implementation of Heap Sort
// To heapify a subtree rooted with node i which is
// an index in arr[]. n is size of heap
void pclCluster::heapify(std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > points, int n, int i)
{
	depth++;
	if(maxDepth<depth){maxDepth=depth; cout<<"new Depth: "<<maxDepth<<endl;}
	
	
    int largest = i;  // Initialize largest as root
    int l = 2*i + 1;  // left = 2*i + 1
    int r = 2*i + 2;  // right = 2*i + 2

	pcl::PointXYZ p;
	double largestVal, lVal,rVal;

	p=points[largest];
	largestVal = (p.y*yWeight+p.x*xWeight+p.z*zWeight);   
	 
	 if(l<n)
	 {
		p=points[l];
		lVal = (p.y*yWeight+p.x*xWeight+p.z*zWeight);  
	  }
	  if(r<n)
	 {
		p=points[r];
		rVal = (p.y*yWeight+p.x*xWeight+p.z*zWeight);  
	  } 

	
    // If left child is larger than root
    if (l < n && lVal > largestVal)
        largest = l;
 
    // If right child is larger than largest so far
    if (r < n && rVal > largestVal)
        largest = r;
 
    // If largest is not root
    if (largest != i)
    {
    	
        swap(&points[i], &points[largest]);
 
        // Recursively heapify the affected sub-tree
        heapify(points, n, largest);
    }
    
    depth--;
}
 
// main function to do heap sort
void pclCluster::heapSort(std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > points, int n)
{
    // Build heap (rearrange array)
    for (int i = n / 2 - 1; i >= 0; i--)
        heapify(points, n, i);
 
    // One by one extract an element from heap
    for (int i=n-1; i>=0; i--)
    {
   
        // Move current root to end
        swap(&points[0], &points[i]);
 
        // call max heapify on the reduced heap
        heapify(points, i, 0);
    }
}


// Driver program
void pclCluster::doHeapSort()
{
    heapSort(cloud->points, cloud->points.size());
	cout<<"MaxDepth: "<<maxDepth<<endl;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr pclCluster::extractHull(int pointsToConsider)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr hull (new pcl::PointCloud<pcl::PointXYZ>);

	std::vector<int> indexes;	

	double yVal, zMax;
	int x, maxIndex;
	x=maxIndex=0;
	yVal=zMax=0;
	
	indexes.push_back(0);
	int stop=cloud->points.size()-pointsToConsider;
	
	for(int i=0; i<stop; i++)
	{
			
		if(yVal!= cloud->points[i].y)
		{
			
			yVal=cloud->points[i].y;

			if(indexes[indexes.size()]!= maxIndex){indexes.push_back(maxIndex);}
			
			x=0;
			zMax=cloud->points[i].z;
		}
		if(x>pointsToConsider)
		{
			
			i=i-(pointsToConsider/2);
			if(indexes[indexes.size()]!= maxIndex){indexes.push_back(maxIndex);}
			x=0;
			zMax=cloud->points[i].z;
		}

		if(zMax<cloud->points[i].z)
		{

			zMax= cloud->points[i].z;
			maxIndex=i;
		}
		
	}

	hull->width=indexes.size();
	hull->height =1;
	hull->is_dense=true;
	hull->points.resize(indexes.size());

	for (int i=0; i<indexes.size(); i++){hull->points[i]=cloud->points[indexes[i]];}
	
	return hull;
}




pcl::PointCloud<pcl::PointXYZ>::Ptr pclCluster::extractSurface(int decPlaces)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr hull (new pcl::PointCloud<pcl::PointXYZ>);

	int x, xLast;
	x=xLast=0;
	for(int i=0; i<cloud->points.size(); i++)
	{
		x = (int)(decPlaces*cloud->points[i].x);
		if(x!=xLast)
		{
			hull->points.push_back(cloud->points[i]);
			xLast=x;
		}
	}
	hull->width=hull->points.size();
	hull->height =1;
	hull->is_dense=true;
	return hull;
}	







