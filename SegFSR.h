#include <iostream>
#include "BoundingBox.h"
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include "V2.hpp"
#include "V3.hpp"
#include "BasicGeometry.h"
#include "PCLExtend.h"
#include <cstdlib>
#include <ctime>
#include<random>
#include<cmath>
#include<chrono>
#include "FloodFill.h"
#include <list>
#define ELAPSED(START,END)  (((END.tv_sec  - START.tv_sec) * 1000000u + END.tv_usec - START.tv_usec) / 1.e6)

using namespace std;
/*
	Image Segmentation based 2D-3D Fusion for 3D Object Filtering, Segmentation and Recognition
*/
class ZElement
{
	public:
		int depth_;
		vector<int> dat_;
		ZElement(){
			depth_=-INT_MAX;
		}
};

class ZBuffer 
{
	public:
		int rows_,cols_;
		vector<vector<ZElement>>  dat_; //
		cv::Mat img_;		
		void Init(pcl::PointCloud<PointType>::Ptr cloud, int axis);
		//void Init(pcl::PointCloud<PointType>::Ptr cloud, int axis, double gap);
		void Clear(){
			rows_=0;
			cols_=0;			
		}
};

class SegFSR
{
	public:
		vector<int> outlier_idx_;
		list<int> obj_idx_;
		vector<V3> orientations_;
		pcl::PointCloud<PointType>::Ptr cloud_;
		pcl::PointCloud<PointType>::Ptr cloud_filtered_;
		vector<ZBuffer> bufs_;  // n*1
		
		
		void Init(pcl::PointCloud<PointType>::Ptr cloud);
		void OrientationsGenerator();
		void ProjectionGenerator();
		void Viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
		void Run();
};











