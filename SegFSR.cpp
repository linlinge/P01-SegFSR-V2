#include "SegFSR.h"

// Approach 01:
// Based on mean dist
void ZBuffer::Init(pcl::PointCloud<PointType>::Ptr cloud,int axis,double mean_dist)
{
	// Generate Picture
	PointType min,max;
    pcl::getMinMax3D(*cloud,min,max);
	float border_width=(max.x-min.x)*0.01;
	min.x=min.x-border_width;
	max.x=max.x+border_width;
	min.y=min.y-border_width;
	max.y=max.y+border_width;
	
	// initial
	cols_=floor((max.x-min.x)/mean_dist)+1;
	rows_=floor((max.y-min.y)/mean_dist)+1;
	
	// Initialize ZBuffer
	vector<ZElement> tmp;
	tmp.resize(cols_);
	for(int i=0;i<rows_;i++)
		dat_.push_back(tmp);
	
	
	img_.create(rows_,cols_, CV_8UC1);
	for(int i=0;i<rows_;i++){
		for(int j=0;j<cols_;j++){
			img_.at<uchar>(i,j)=255;
		}
	}
	
	float delta_x=(max.x-min.x)/cols_;
	float delta_y=(max.y-min.y)/rows_;	
	
	for(int k=0;k<cloud->points.size();k++){
		
		int j=floor((cloud->points[k].x-min.x)/delta_x);
		int i=floor((cloud->points[k].y-min.y)/delta_y);
		
		if(dat_[i][j].depth_<cloud->points[k].z){
			dat_[i][j].depth_=cloud->points[k].z;
			img_.at<uchar>(i,j)=0;
		}
	}
	
	/* cv::imshow("3D Viewer",img_);
	cv::imwrite("1.bmp",img_);
	cv::waitKey(0); */
}

// Approach 02
// Based on specified Resolution
#define IMG_WIDTH 200
void ZBuffer::Init(pcl::PointCloud<PointType>::Ptr cloud,int axis)
{
	// Generate Picture
	PointType min,max;
    pcl::getMinMax3D(*cloud,min,max);
	float border_width=(max.x-min.x)*0.01;
	min.x=min.x-border_width;
	max.x=max.x+border_width;
	min.y=min.y-border_width;
	max.y=max.y+border_width;
	
	// calculate rows and cols
	cols_=IMG_WIDTH;
	rows_=floor((max.y-min.y)/(max.x-min.x)*IMG_WIDTH)+1;
	
	// Initialize ZBuffer
	vector<ZElement> tmp;
	tmp.resize(cols_);
	for(int i=0;i<rows_;i++)
		dat_.push_back(tmp);
	

	
	img_.create(rows_,cols_, CV_8UC1);
	for(int i=0;i<rows_;i++){
		for(int j=0;j<cols_;j++){
			img_.at<uchar>(i,j)=255;
		}
	}
	
	float delta_x=(max.x-min.x)/cols_;
	float delta_y=(max.y-min.y)/rows_;	
	
	for(int k=0;k<cloud->points.size();k++){
		
		int j=floor((cloud->points[k].x-min.x)/delta_x);
		int i=floor((cloud->points[k].y-min.y)/delta_y);
		
		// update index in buffer
		dat_[i][j].dat_.push_back(k);
		
		// update depth in buffer
		if(dat_[i][j].depth_<cloud->points[k].z){
			dat_[i][j].depth_=cloud->points[k].z;
			img_.at<uchar>(i,j)=0;
		}
	}
}


void SegFSR::Init01(pcl::PointCloud<PointType>::Ptr cloud, float delta_arc)
{
	cloud_=cloud;
	delta_arc_=delta_arc;
	
}

void SegFSR::Init02(pcl::PointCloud<PointType>::Ptr cloud, int n)
{
	cloud_=cloud;
	n_=n;
	orientations_.resize(n);
	bufs_.resize(n);
}

void SegFSR::UprightEstimation()
{
	BoundingBox bb(cloud_);
	
	p_upright_=bb.pcZ_;
	p_forward_=bb.pcX_;
	p_left_=bb.pcY_;
	p_centre_=bb.cp_;
	
	v_upright_=V3(p_upright_.x-p_centre_.x,p_upright_.y-p_centre_.y,p_upright_.z-p_centre_.z);
	v_forward_=V3(p_forward_.x-p_centre_.x,p_forward_.y-p_centre_.y, p_forward_.z-p_centre_.z);
	v_left_=V3(p_left_.x-p_centre_.x,p_left_.y-p_centre_.y,p_left_.z-p_centre_.z);
	
	v_upright_=v_upright_/v_upright_.GetLength();
	v_forward_=v_forward_/v_forward_.GetLength();
	v_left_=v_left_/v_left_.GetLength();
}

// Approach 01: Generator orientation based on UprightEstimation()
void SegFSR::OrientationsGenerator01()
{	
	// upright estimation 
	UprightEstimation();
	
	// Generate orientation
	cv::Mat mat_initial_vector = (cv::Mat_<double>(3, 1) << v_forward_.x, v_forward_.y , v_forward_.z);   // initial vector
	int n=floor(2*CV_PI/delta_arc_);
	for(int i=0;i<=n;i++)
	{
		V3 v3_rotation_vector= v_upright_*delta_arc_*i;	
		cv::Mat mat_rotation_vector = (cv::Mat_<double>(3, 1) << v3_rotation_vector.x, v3_rotation_vector.y, v3_rotation_vector.z); 		
		cv::Mat rotation_matrix;                                            // rotaiton matrix
		Rodrigues(mat_rotation_vector, rotation_matrix);
		cv::Mat des_vector = rotation_matrix * mat_initial_vector;
		
		V3 tmp=V3(des_vector.ptr  <double>(0)[0],des_vector.ptr<double>(1)[0],des_vector.ptr<double>(2)[0]);
		tmp.Normalize();
		orientations_.push_back(tmp);
	}	
}

// Approach 02: Random Generator
void SegFSR::OrientationsGenerator02()
{
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937 generator (seed);
    std::uniform_real_distribution<double> uniform01(0.0, 1.0);
	
	for (int i = 0; i < n_; i++) {
        // incorrect way
        double theta = 2 * M_PI * uniform01(generator);
        double phi = acos(1 - 2 * uniform01(generator));
        orientations_[i].x = sin(phi) * cos(theta);
        orientations_[i].y = sin(phi) * sin(theta);
        orientations_[i].z = cos(phi);
    }	
}



void SegFSR::Run()
{	
	// init
	vector<int> outlier_idx;
	
	// Generate Projection Orientations
	cout<<"1"<<endl;
	OrientationsGenerator02();
	cout<<"2"<<endl;
	ProjectionGenerator();
	
	// Outlier Removal
	cout<<"3"<<endl;
	#pragma omp parallel for
	for(int i=0;i<n_;i++){
		Graph gh;
		gh.Establish(bufs_[i].img_);
		gh.Run();
		printf(" %d ",i);
		for(int j=1;j<gh.rst_.size();j++){
			for(int k=0;k<gh.rst_[j].number_;k++){
				int tmp_i=gh.rst_[j].pLoc_[k].i_;
				int tmp_j=gh.rst_[j].pLoc_[k].j_;
				outlier_idx.insert(outlier_idx.end(),bufs_[i].dat_[tmp_i][tmp_j].dat_.begin(),bufs_[i].dat_[tmp_i][tmp_j].dat_.end());
			}
		}
	}	
	
	cout<<"4"<<endl;
	sort(outlier_idx.begin(),outlier_idx.end());
	vector<int>::iterator it=unique(outlier_idx.begin(),outlier_idx.end());
	outlier_idx.erase(it,outlier_idx.end());
	
	for(int i=0;i<cloud_->points.size();i++)
	{
		cloud_->points[i].r=0;
		cloud_->points[i].g=255;
		cloud_->points[i].b=0;
	}
	
	for(int i=0;i<outlier_idx.size();i++)
	{
		cloud_->points[outlier_idx[i]].r=255;
		cloud_->points[outlier_idx[i]].g=0;
		cloud_->points[outlier_idx[i]].b=0;
	}
	pcl::io::savePLYFileASCII("1.ply",*cloud_);
}


void SegFSR::ProjectionGenerator()
{	
	for(int i=0;i<n_;i++)
	{
		pcl::PointCloud<PointType>::Ptr tf_cloud (new pcl::PointCloud<PointType> ());	
		// define affine
		float alpha=orientations_[i].GetArcToPlane(Z_AXIS,YOZ);
		float beta=orientations_[i].GetArcToPlane(X_AXIS,XOZ);
		
		Eigen::Affine3f tf = Eigen::Affine3f::Identity();
		tf.translation()<<0,0,0;
		tf.rotate(Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitZ()));
		tf.rotate(Eigen::AngleAxisf(beta, Eigen::Vector3f::UnitX()));	
		pcl::transformPointCloud (*cloud_, *tf_cloud, tf);	
		bufs_[i].Init(tf_cloud,Z_AXIS);
	}
}
 


void SegFSR::Viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
	viewer->setBackgroundColor(1.0, 1.0, 1.0);	
	pcl::visualization::PointCloudColorHandlerRGBField<PointType> multi_color(cloud_); 	
	viewer->addPointCloud<PointType> (cloud_, multi_color, "1");  
	
/* 	// add arrow
	viewer.addArrow<PointType>(p_upright_, p_centre_, 1.0f, 0, 0, false, "X");
	viewer.addArrow<PointType>(p_forward_, p_centre_, 0.0f, 1.0f, 0, false, "Y");
	viewer.addArrow<PointType>(p_left_, p_centre_, 0.0f, 0.0f, 1.0f, false, "Z");
	
	
	for(int i=0;i< orientations_.size();i++)
	{
		string tmp="orientation"+std::to_string(i);
		PointType p_tmp;
		p_tmp.x=orientations_[i].x+p_centre_.x;
		p_tmp.y=orientations_[i].y+p_centre_.y;
		p_tmp.z=orientations_[i].z+p_centre_.z;
		viewer.addArrow<PointType>(p_tmp, p_centre_, 1.0f, 1.0f, 0.0f, false, tmp);
	} */
	
}