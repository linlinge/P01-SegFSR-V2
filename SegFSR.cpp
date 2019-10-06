#include "SegFSR.h"
#define IMG_WIDTH 400
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

void SegFSR::Init(pcl::PointCloud<PointType>::Ptr cloud, int n)
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

void SegFSR::OrientationsGenerator()
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
	// Init
	vector<int> outlier_idx;
	clock_t start,end;
	
	// Generate Projection Orientations
	start=clock();
	cout<<"[ 25%] Generate Orientation\t\t";
	OrientationsGenerator();
	end=clock();
	cout<<(float)(end-start)/CLOCKS_PER_SEC<<" (s)"<<endl;
	
	// Generate Projection Images
	cout<<"[ 50%] Generate Projection Images\t";
	start=clock();
	ProjectionGenerator();
	end=clock();
	cout<<(float)(end-start)/CLOCKS_PER_SEC<<" (s)"<<endl;
	
	// Detect Outlier
	cout<<"[ 75%] Detect Outlier\t\t\t";
	start=clock();
	for(int i=0;i<n_;i++){
		FloodFill ff(bufs_[i].img_);
		for(int j=1;j<ff.result_.size();j++){
			Vertices* ant=&ff.result_[j];
			Vertex* p=ant->head_->next;
			while(p!=NULL){
				int itmp=p->i_;
				int jtmp=p->j_;
				vector<int>& tmp=bufs_[i].dat_[itmp][jtmp].dat_;
				outlier_idx.insert(outlier_idx.end(),tmp.begin(),tmp.end());
				p=p->next;
			}
		}
	}	
	end=clock();
	cout<<(float)(end-start)/CLOCKS_PER_SEC<<" (s)"<<endl;
	
	// Outlier Removal
	cout<<"[100%] Finish!\t\t\t\t";
	start=clock();
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
	
	end=clock();
	cout<<(float)(end-start)/CLOCKS_PER_SEC<<" (s)"<<endl;
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
}