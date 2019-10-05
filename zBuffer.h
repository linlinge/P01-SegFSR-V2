#pragma once
#include <iostream>
#include <vector>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include "V2.hpp"
#include "V3.hpp"
#include "BasicGeometry.h"
using namespace std;


#define IMAGE_WIDTH 800.0
#ifndef PointType
#define PointType pcl::PointXYZRGBA
#endif

class PixelInfo
{
	public:
		vector<PointType> dat_;		
};

class zBuffer
{
	public:
		int image_width_, image_height_;
		float actual_width_,actual_height_;
		float wstep_,hstep_;
		vector<vector<PixelInfo>> dat_;
		cv::Mat img_;
		V2 v_left_up_,v_right_down_;
		pcl::PointCloud<PointType>::Ptr cloud_;		
		
		zBuffer(pcl::PointCloud<PointType>::Ptr cloud);
		void GetImage(int mode=0);
		void LoadMask(string mask_path);
};


