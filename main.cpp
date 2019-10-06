#include <iostream>
#include "SegFSR.h"
#include <opencv2/opencv.hpp>
#include<cstdlib>
#include<ctime>
#include <stdio.h>
using namespace std;

int main(int argc, char **argv)
{	
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);	
	if (pcl::io::loadPLYFile<PointType>(argv[1], *cloud) == -1){
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	
	for(int i=0;i<cloud->points.size();i++)
	{
		cloud->points[i].r=0;
		cloud->points[i].g=255;
		cloud->points[i].b=0;
	}
	
	SegFSR alg;
	alg.Init(cloud,500);
	alg.Run();
	
	return 0;
} 


