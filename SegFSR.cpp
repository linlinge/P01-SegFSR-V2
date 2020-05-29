#include "SegFSR.h"
void ZBuffer::Init(pcl::PointCloud<PointType>::Ptr cloud, int axis)
{
	//float border_width=(max.x-min.x)*0.01;
	/* double border_width=0.00001;
	min.x=min.x-border_width;
	max.x=max.x+border_width;
	min.y=min.y-border_width;
	max.y=max.y+border_width; */
	
	// calculate rows and cols
	PointType min,max;
	pcl::getMinMax3D(*cloud,min,max);
	double V=(max.x-min.x)*(max.y-min.y)*(max.z-min.z);
	double gap=0.4*pow(V/cloud->points.size(),1/3.0);
	
	cols_=floor((max.x-min.x)/gap)+1;
	rows_=floor((max.y-min.y)/(max.x-min.x)*cols_)+1;
	
	// Initialize ZBuffer
	vector<ZElement> tmp;
	tmp.resize(cols_);
	for(int i=0;i<rows_;i++)
		dat_.push_back(tmp);
	
	
	//img_.create(rows_,cols_, CV_32SC1);
	img_.create(rows_,cols_, CV_8UC1);
	for(int i=0;i<rows_;i++){
		for(int j=0;j<cols_;j++){
			img_.at<uchar>(i,j)=255;
		}
	}
	
	double delta_x=(max.x-min.x)/cols_;
	double delta_y=(max.y-min.y)/rows_;	
	
	for(int k=0;k<cloud->points.size();k++){
		
		int j=floor((cloud->points[k].x-min.x)/delta_x);
		int i=floor((cloud->points[k].y-min.y)/delta_y);
		if(i==rows_)
			i=rows_-1;
		if(j==cols_)
			j=cols_-1;
		
		// update index in buffer
		dat_[i][j].dat_.push_back(k);
		
		// update depth in buffer
		if(dat_[i][j].depth_<cloud->points[k].z){
			dat_[i][j].depth_=cloud->points[k].z;
			img_.at<uchar>(i,j)=0;
		}
	}
}


void SegFSR::Init(pcl::PointCloud<PointType>::Ptr cloud)
{
	cloud_=cloud;
	for(int i=0;i<cloud->points.size();i++)
		obj_idx_.push_back(i);
}

void SegFSR::OrientationsGenerator()
{
	orientations_.clear();
	float delta_phi=CV_PI/4.5;
	float delta_theta=CV_PI/2.25;
	
	orientations_.push_back(V3(0,0,1));
	for(float phi=delta_phi;phi<CV_PI;phi+=delta_phi){
		for(float theta=delta_theta;theta<2*CV_PI;theta+=delta_theta){		
			V3 tmp;
			tmp.x=sin(phi)*cos(theta);
			tmp.y=sin(phi)*sin(theta);
			tmp.z=cos(phi);
			orientations_.push_back(tmp);			
		}
	}
	orientations_.push_back(V3(0,0,-1));
	
	bufs_.resize(orientations_.size());
	
	/* 
		cout<<orientations_.size()<<endl;
		for(int i=0;i<orientations_.size();i++)
		{
			cout<<orientations_[i]<<endl;
		} 
	*/
	
}

void SegFSR::ProjectionGenerator()
{	
	// (0,0,1) , no need to transform
	bufs_[0].Init(cloud_, Z_AXIS);
	bufs_[bufs_.size()-1].Init(cloud_, Z_AXIS);

	/*
		Record Image
	*/
	// cv::imwrite("0.bmp",bufs_[0].img_);
	// cv::imwrite(std::to_string(bufs_.size()-1)+".bmp",bufs_[bufs_.size()-1].img_);	
	
	#pragma omp parallel for
	for(int i=1;i<orientations_.size()-1;i++){		
		pcl::PointCloud<PointType>::Ptr cloud_tf (new pcl::PointCloud<PointType>());		
		//cout<<i<<endl;
		// generate alpha
		Eigen::Vector3f v1(orientations_[i].x,orientations_[i].y,orientations_[i].z);
		float alpha=orientations_[i].GetArcToPlane(Z_AXIS,YOZ); // get angle
		Eigen::Affine3f tf1 = Eigen::Affine3f::Identity();
		tf1.translation()<<0,0,0;
		tf1.rotate(Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitZ()));
		Eigen::Vector3f v2=tf1*v1;
		// generate beta
		V3 tmp(v2(0),v2(1),v2(2));		
		float beta=tmp.GetArcToPlane(X_AXIS,XOZ);		
		// transformation
		Eigen::Affine3f tf = Eigen::Affine3f::Identity();		
		tf.translation()<<0,0,0;
		tf.rotate(Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitZ()));
		tf.rotate(Eigen::AngleAxisf(beta, Eigen::Vector3f::UnitX()));
		
		pcl::transformPointCloud(*cloud_, *cloud_tf, tf);
		bufs_[i].Init(cloud_tf, Z_AXIS);		
		// cv::imwrite(std::to_string(i)+".bmp",bufs_[i].img_);
	}
}

void SegFSR::Run()
{	
	// Init
	struct timeval start, end;
	
	// Generate Projection Orientations
	gettimeofday(&start, NULL);
	cout<<"[ 25%] Generate Orientation\t\t";
	OrientationsGenerator();
	gettimeofday(&end, NULL);
	cout<<ELAPSED(start,end)<<" (s)"<<endl;
	
	
	// Generate Projection Images
	cout<<"[ 50%] Generate Projection Images\t";
	gettimeofday(&start, NULL);
	ProjectionGenerator();
	gettimeofday(&end, NULL);
	cout<<ELAPSED(start,end)<<" (s)"<<endl; 	
	
	// Detect Outlier
	cout<<"[ 75%] Detect Outlier\t\t\t";
	
	gettimeofday(&start, NULL);
	//#pragma omp parallel for
	for(int i=0;i<orientations_.size();i++){
		FloodFill ff(bufs_[i].img_);
		
		for(int j=1;j<ff.result_.size();j++){
			Vertices* ant=&ff.result_[j];			
			Vertex* p=ant->head_->next;
			while(p!=NULL){
				int itmp=p->i_;
				int jtmp=p->j_;
				vector<int>& tmp=bufs_[i].dat_[itmp][jtmp].dat_;
				outlier_idx_.insert(outlier_idx_.end(),tmp.begin(),tmp.end());
				p=p->next;
			}
		}
	}	
	gettimeofday(&end, NULL);
	cout<<ELAPSED(start,end)<<" (s)"<<endl;
	
	// Outlier Removal 
	cout<<"[100%] Finish!\t\t\t\t";	
	gettimeofday(&start, NULL);
	sort(outlier_idx_.begin(),outlier_idx_.end());
	vector<int>::iterator it=unique(outlier_idx_.begin(),outlier_idx_.end());
	outlier_idx_.erase(it,outlier_idx_.end());
	
	
	// cloud_filtered 
	cloud_filtered_=pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);	

	//int 
	//#pragma omp parallel for
	int k=0;
	int current_idx=outlier_idx_[k];
	for(int i=0;i<cloud_->points.size();i++)
	{
		if(i!=current_idx){
			cloud_filtered_->points.push_back(cloud_->points[i]);
		}
		else{
			current_idx=outlier_idx_[++k];
		}
	}	
	
	// output
	gettimeofday(&end, NULL);
	cout<<ELAPSED(start,end)<<" (s)"<<endl;	
	cout<<"cloud size:"<<cloud_->points.size()<<endl;
	cout<<"cloud_filtered size:"<<cloud_filtered_->points.size()<<endl;	
}


void SegFSR::Viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
	viewer->setBackgroundColor(1.0, 1.0, 1.0);	
	pcl::visualization::PointCloudColorHandlerRGBField<PointType> multi_color(cloud_); 	
	viewer->addPointCloud<PointType> (cloud_, multi_color, "1"); 
}