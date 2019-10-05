#pragma once
#include <iostream>
#include <string>
#include <pcl/io/ply_io.h>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <math.h>
#include "V3.hpp"
#include <vector>
using namespace std;
#ifndef PointType
#define PointType pcl::PointXYZRGBA
#endif



class BoundingBox
{
    public:
        // Variables                       
		PointType v1_,v2_,v3_,v4_; // four vertex of bounding box in x diretion
		Eigen::Vector3f cen_tf_, cen_;
		Eigen::Quaternionf bboxQ_;
		Eigen::Vector3f whd_;
		PointType pcX_;
		PointType pcY_;
		PointType pcZ_;
		PointType cp_;
		string id_;
		V3 color_;
		
        // Functions		
        BoundingBox(pcl::PointCloud<PointType>::Ptr cloud, string id="default",V3 color=V3(1.0,0.0,0.0));
		friend void DisplayBoundingBox(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<PointType>::Ptr cloud, BoundingBox bb);
};

