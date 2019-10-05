#include <iostream>
#include "BoundingBox.h"
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include "V2.hpp"
#include "V3.hpp"
#include "BasicGeometry.h"
#include "PCLExtend.h"
#include <cstdlib>
#include <ctime>
#include "graph.h"
#include "omp.h"
#include<random>
#include<cmath>
#include<chrono>
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
		void Init(pcl::PointCloud<PointType>::Ptr cloud, int axis, double max_dist);
		void Init(pcl::PointCloud<PointType>::Ptr cloud,int axis);
		void Clear(){
			rows_=0;
			cols_=0;			
		}
};

class SegFSR
{
	public:
		vector<V3> orientations_;
		vector<int> outliers_idx_;  	// store the indices for outliers
		pcl::PointCloud<PointType>::Ptr cloud_;
		float delta_arc_;
		int n_;
		PointType p_upright_,p_forward_,p_left_,p_centre_;
		V3 v_upright_,v_forward_,v_left_;
		// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
		double mean_dist;
		vector<ZBuffer> bufs_;  // n*1
		
		
		void Init01(pcl::PointCloud<PointType>::Ptr cloud, float delta_arc);  // initial		
		void Init02(pcl::PointCloud<PointType>::Ptr cloud, int n);  // initial		
		void OrientationsGenerator01();
		void OrientationsGenerator02();
		void ProjectionGenerator();
		void Viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
		void Run();

		
		// inner function
		void UprightEstimation();
};











