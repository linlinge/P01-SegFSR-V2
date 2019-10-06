#include <iostream>
#include "SegFSR.h"
#include <opencv2/opencv.hpp>
#include<cstdlib>
#include<ctime>
#include <stdio.h>
using namespace std;

int main(int argc, char **argv)
{	
	
	/* cv::Mat img=cv::imread(argv[1],0);	
	FloodFill ff(img); */
	
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);	
	if (pcl::io::loadPLYFile<PointType>(argv[1], *cloud) == -1){
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	
	SegFSR alg;
	alg.Init(cloud,40);
	alg.Run();
	
	/*
	for(int i=0;i<ff.result_.size();i++)
	{
		cout<<ff.result_[i].number_<<" ";
	}
	cout<<endl; 
	*/
	
	
	return 0;
} 


