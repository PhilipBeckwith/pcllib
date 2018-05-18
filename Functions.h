#pragma once
#include "includes.h"

/////////////////////////////// forward declaration of functions 

//views a vector of clusters color clusters based on  size
void viewBySize(int minSize, int maxSize, std::vector<pclCluster> cluster);

//Crops all clusters in a vector and returns the resulting vector
std::vector<pclCluster> cropAll(std::string dim, double max, double min, std::vector<pclCluster>	clusters );

//returns a vector containing clusters with a specified aspect ratio
std::vector<pclCluster> aspectRatioFilter( int maxAspect, std::vector<pclCluster> clusters);

// returns a vector containing clusters within a volume
std::vector<pclCluster> volumeFilter(double minSize, double maxSize, std::vector<pclCluster> clusters);

//finds the max/min/avg number of points per cluster 
//values are returned via pointers
void getAverage(double *max, double *min, double *avg, std::vector<pclCluster> clusters, int dataType);

//creats a histogram
std::vector<int> createHistogram(int dataType, std::vector<pclCluster> clusters);
/////////////////////////////////////////END of declarations

void viewByCluster(std::vector<pclCluster> INclusters)
{
	std::vector<pclCluster> clusters(INclusters);
	
	for(int i=0; i< clusters.size(); i++)
	{
		clusters[i].translateCenter( 1, 1, clusters[i].center.z);
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3DViewer"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
	
	for(int i=0;!viewer->wasStopped(); i=(i+1)%clusters.size())
	{
		viewer->addPointCloud<pcl::PointXYZ> ( clusters[i].cloud , "Cluster");
		
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(1000000));
		
		viewer->removePointCloud("Cluster");
	}
}

//views a vector of clusters colors by size
void viewBySize(int minSize, int maxSize, std::vector<pclCluster> clusters)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3DViewer"));
	viewer->setBackgroundColor (0, 0, 0);
	
	
	int j=0;
	for(std::vector<pclCluster >::iterator i=clusters.begin(); i!= clusters.end(); i++)
	{
		//the clusters all need a unique string name
		// so they are named by what order they are added
		//converting a int to a string
		std::stringstream ss;
		ss << j;
		std::string str = ss.str();

		//setting the g value according to how close the cluster is to the max size
		int color= ((((*i).cloud)->width)-minSize)*(250/(maxSize-minSize));
		//if the green value is over the allowed level it is set to the max level
		if(color>250){color=250;}
		
		//creating a color handler for each cluster (coloring the cluster)
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color((*i).cloud, 70, color, 70);
		//adding the cluster to the viewer
		viewer->addPointCloud<pcl::PointXYZ> ((*i).cloud, single_color ,str);
		//seting params
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, str);	
		j++;
	}
	//float p1,p2,p3;
	//p1=clusters[0].cloud->points[0].x;
	//p2=clusters[0].cloud->points[0].y;
	//p3=clusters[0].cloud->points[0].z;
	//setting params
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
	//viewer->setCameraPosition(p1,p2,p3,0,0,0,0);
	
	//Opens a window and allows the user to see the clusers/ pan around
	while(!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	//suposed to close the window when exited
	//hasn't been working
	viewer->close();
}


//Crops all clusters in a vector and returns the resulting vector
std::vector<pclCluster> cropAll(std::string dim, double max, double min, std::vector<pclCluster>	clusters )
{
	std::vector<pclCluster> copyClusters=clusters;
	//calls the crop fuction on each cluster object
	for(int i=0; i<copyClusters.size(); i++)
	{
		copyClusters[i].crop(dim, max, min);
	}
	return copyClusters;
}


//returns a vector containing two clouds a ground cloud and an object cloud

std::vector<pclCluster> groundFilter(pclCluster cloud)
{
	//todo...get filter working and make method...
}




//returns a vector containing clusters with a specified aspect ratio
std::vector<pclCluster> aspectRatioFilter(std::string dim ,int maxAspect, std::vector<pclCluster> clusters)
{
	//creating a temp vector
	std::vector<pclCluster > cluster_filtered = std::vector<pclCluster>();
	
	//making string lowercase
	for (int i=0; dim[i]; i++){ dim[i] = tolower(dim[i]); }
	
	bool testx, testy, testz;
	
	testx= dim.find("x")!=std::string::npos;
	testy= dim.find("y")!=std::string::npos;
	testz= dim.find("z")!=std::string::npos;
	
	bool isRobust=true;
	
	//cycles though the input cluster vector
	for(std::vector<pclCluster >::iterator i=clusters.begin(); i!= clusters.end(); i++)
	{
		
		//testing the aspect for each dimension
		isRobust = true;

		if(testx)
		{
			if((*i).width  > ((*i).length  * maxAspect)){isRobust=false;}
			if((*i).width  > ((*i).height  * maxAspect)){isRobust=false;}
		}
		if(testy)
		{
			if((*i).length > ((*i).width   * maxAspect)){isRobust=false;}
			if((*i).length > ((*i).height  * maxAspect)){isRobust=false;}
		}
		if(testz)
		{
			if((*i).height > ((*i).width   * maxAspect)){isRobust=false;}
			if((*i).height > ((*i).length  * maxAspect)){isRobust=false;}
		}
		
		//if the cluster is within the specified aspect ratio it is added to the temp vector
		if(isRobust){cluster_filtered.push_back(*i);}
	}
	
	//the temp fector is returned.
	return cluster_filtered;
}

// returns a vector containing clusters within a volume
std::vector<pclCluster> volumeFilter(double minSize, double maxSize, std::vector<pclCluster> clusters)
{
	//creating the vector that will be returned
	std::vector<pclCluster > cluster_filtered;
	
	//looping though the input vector
	for(std::vector<pclCluster >::iterator i=clusters.begin(); i!= clusters.end(); i++)
	{
		//cheks the volume. if it's with in the specified range it is added to the vector
		if((*i).volume > minSize && (*i).volume< maxSize)
		{
			cluster_filtered.push_back(*i);	
		}
	}
	//returning the vector with only clusters with the specified volume
	return cluster_filtered;
}


/*
	Takes in a "Max", "Min", and "Average" pointers.
	finds those peramiters bassed on the specified data type.
	sets the dereferenced value of max, min and average.
*/
void getAverage(double *max, double *min, double *avg, std::vector<pclCluster> clusters, int dataType)
{
	//initalizing the max and min to the first data point
	*max = *min = clusters[0].getData(dataType);
	
	int numb=0;
	double temp;
	for(std::vector<pclCluster >::iterator i=clusters.begin(); i!= clusters.end(); i++)
	{
		
		temp=(*i).getData(dataType);
		//summing the set
		*avg=(*avg)+temp;
		//setting min
		if(temp<(*min)){*min=temp;}
		//setting max
		if(temp>(*max)){*max=temp;}
		//keeping track of enteries
		numb++;
	}
	//setting average
	*avg=(*avg)/numb;
}

/*
	creates a vector where all even numbers are some data and the odd ones are 
	it's frequency. 
	this algarithum is bassed on the assumption that the data is sorted.	
*/
std::vector<int> createHistogram(int dataType, std::vector<pclCluster> clusters)
{
	//creating the vector to-be returned
	std::vector<int> histogram;
	int index=0;
	int count =0;
	double data;
	//initalizing the first value
	data=clusters[0].getData(dataType);
	for(std::vector<pclCluster >::iterator i=clusters.begin(); i!= clusters.end(); i++)
	{
		//if the data is the same the count is incremented otherwise it's reset
		if(data==(*i).getData(dataType))
		{
			
			count++;
		}
		else
		{
			//adding the data and frequency to the vector
			histogram.push_back(data);
			histogram.push_back(count);
			//resetting the count and changing data's value
			count=1;
			data=(*i).getData(dataType);
		}
	}

	//pushing the last set onto the vector
	histogram.push_back(data);
	histogram.push_back(count);

	return histogram;
}


/*
	takes in a mean and standardDiv pointers, a dataType and a pclcluster vector
	calculates the mean and standard div of that data type
	sets derefrenced pointers to mean and standard div val
*/
void findStandardDiv(double *mean, double *stdDiv, int dataType, std::vector<pclCluster> cluster)
{
	double sum=0;
	//finding mean
	for(int i=0; i<cluster.size(); i++)
	{
		sum += cluster[i].getData(dataType);
	}
	*mean = sum/cluster.size();
	
	sum = 0;
	for(int i=0; i<cluster.size(); i++)
	{
		sum += pow (
		cluster[i].getData(dataType)-*mean, 2) ;
	}
	
	*stdDiv= sqrt(sum/ cluster.size());

}



pclCluster combineVectorCloud(std::vector<pclCluster> clusters)
{
	pclCluster newcloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<pcl::PointXYZ> points;
	
	for(int i=0; i<clusters.size(); i++)
	{
		for(int j=0; j<clusters[i].cloud->points.size(); j++)
		{
			points.push_back(clusters[i].cloud->points[j]);
		}
		
	}
	std::cout<<points.size()<<std::endl;

	cloud->width=points.size();

	cloud->height =1;
	cloud->is_dense=true;
	cloud->points.resize(points.size());
	for(int i=0; i<points.size(); i++)
	{
		cloud->points[i]=points[i];
	}

	newcloud.cloud=cloud;	
	
	return newcloud;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr getCenterPointCloud(std::vector<pclCluster> clusters)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->height =1;
	cloud->is_dense=true;
	cloud->points.resize(clusters.size());
	for(int i=0; i<clusters.size(); i++)
	{
		cloud->points[i]= clusters[i].center;
	}
	
	return cloud;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr addColor(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float color)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorized(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	colorized->height =1;
	colorized->is_dense=true;
	colorized->points.resize(cloud->points.size());
	for(size_t i=0; i<colorized->points.size(); i++)
	{	
		colorized->points[i].x= cloud->points[i].x;
		colorized->points[i].y= cloud->points[i].y;
		colorized->points[i].z= cloud->points[i].z;
		colorized->points[i].rgb = color;
	}
	return colorized;
}



































