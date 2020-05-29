#include <iostream>
#include "SegFSR.h"
#include <opencv2/opencv.hpp>
#include<cstdlib>
#include<ctime>
#include <stdio.h>
#include "PCLExtend.h"
using namespace std;
vector<double> StatisticNearestDistance(pcl::PointCloud<PointType>::Ptr cloud)
{
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	vector<double> rst;
	pcl::search::KdTree<PointType> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{ 
		if (!std::isfinite((*cloud)[i].x))
		{
			continue;
		}
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		if (nres == 2)
		{			
			rst.push_back(sqr_distances[1]);
			++n_points;
		}
	}	
	
	return rst;
}
double mean(vector<double>& dat)
{
	double sum=0;
	#pragma omp parallel for reduction(+:sum)
	for(int i=0;i<dat.size();i++) sum+=dat[i];
	
	return sum/(double)dat.size();
}

int main(int argc, char **argv)
{	
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);	
	if (pcl::io::loadPLYFile<PointType>(argv[1], *cloud) == -1){
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	
 	SegFSR alg;
	alg.Init(cloud);
	alg.Run();
	pcl::io::savePLYFileBinary(argv[2],*alg.cloud_filtered_);
	
	return 0;
}