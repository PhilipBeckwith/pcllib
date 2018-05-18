#pragma once
#include <bits/stdc++.h>
#include "includes.h"
using namespace std;
 
// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(pcl::PointXYZ p, pcl::PointXYZ q, pcl::PointXYZ r)
{
    int val = (q.z - p.z) * (r.x - q.x) -
              (q.x - p.x) * (r.z - q.z);
 
    if (val == 0) return 0;  // colinear
    return (val > 0)? 1: 2; // clock or counterclock wise
}
 
//throws an empty throw
void getSliceHull(pcl::PointCloud<pcl::PointXYZ>::Ptr points, pcl::PointCloud<pcl::PointXYZ>::Ptr hull, int start ,int stop)
{
    // There must be at least 3 points
    if (stop-start < 3) {throw;};
 
    // Find the leftmost point
	int l = start;
	for (int i = start+1; i < stop; i++)
	{
        	if (points->points[i].x < points->points[l].x){l = i;}
	}
 
    // Start from leftmost point, keep moving counterclockwise
    // until reach the start point again.  This loop runs O(h)
    // times where h is number of points in result or output.
    int p = l, q;
	cout<<"before loop\n";
    do
    {
        // Add current point to result
        hull->points.push_back(points->points[p]);
 
        // Search for a point 'q' such that orientation(p, x,
        // q) is counterclockwise for all points 'x'. The idea
        // is to keep track of last visited most counterclock-
        // wise point in q. If any point 'i' is more counterclock-
        // wise than q, then update q.
        q++;
	if(q>=stop){q=start;}
        for (int i = start; i < stop; i++)
        {
           // If i is more counterclockwise than current q, then
           // update q
           if (orientation(points->points[p], points->points[i], points->points[q]) == 2)
               {q = i; }

        }
 
        // Now q is the most counterclockwise with respect to p
        // Set p as q for next iteration, so that q is added to
        // result 'hull'
        p = q;
 
    } while (p != l);  // While we don't come to first point
 	cout<<"afterLoop\n";
    // Print Result
}
 


pcl::PointCloud<pcl::PointXYZ>::Ptr getConvexHull(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	 // Initialize Result
	pcl::PointCloud<pcl::PointXYZ>::Ptr hull (new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<int> index;

	int y,Y;
	index.push_back(0);
	for(int i=1; i<cloud->points.size(); i++)
	{
		y=(int ) (cloud->points[index.back()].y*100);
		Y=(int ) (cloud->points[i].y*100);
		if(y != Y){index.push_back(i);cout<<y<<" != "<<Y<<endl;}
	}
	index.push_back(cloud->points.size());
	
	for(int i = 0; i<index.size()-1; i++)
	{
		try
		{
			cout<<i <<": "<< index[i]<<endl;
			getSliceHull(cloud, hull, index[i], index[i+1]);
		}catch(...){}
		//cout<<"SIZE: "<< hull->points.size() <<endl;
	}
	hull->height =1;
	hull->is_dense=true;
	hull->width=hull->points.size();

	cout<<"SIZE-- HULL: "<<hull->points.size();
	cout<<"\n\tCloud: "<<cloud->points.size();
	
	return hull;
}










