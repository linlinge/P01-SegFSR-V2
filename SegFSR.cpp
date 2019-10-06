#include "SegFSR.h"
#define IMG_WIDTH 15
void ZBuffer::Init(pcl::PointCloud<PointType>::Ptr cloud,int axis)
{
	// Generate Picture
	PointType min,max;
    pcl::getMinMax3D(*cloud,min,max);
	//float border_width=(max.x-min.x)*0.01;
	float border_width=0.00001;
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

void SegFSR::ProjectionGenerator()
{	
	for(int i=0;i<n_;i++)
	{
		clock_t start,end;
		start=clock();
		pcl::PointCloud<PointType>::Ptr tf_cloud (new pcl::PointCloud<PointType> ());	
		// define affine
		float alpha=orientations_[i].GetArcToPlane(Z_AXIS,YOZ);
		float beta=orientations_[i].GetArcToPlane(X_AXIS,XOZ);
		
		Eigen::Affine3f tf = Eigen::Affine3f::Identity();
		tf.translation()<<0,0,0;
		tf.rotate(Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitZ()));
		tf.rotate(Eigen::AngleAxisf(beta, Eigen::Vector3f::UnitX()));
		end=clock();
		//cout<<endl<<"[ *1* ]"<<(float)(end-start)/CLOCKS_PER_SEC<<" (s)"<<endl;
		
		start=clock();
		pcl::transformPointCloud(*cloud_, *tf_cloud, tf);	
		end=clock();
		//cout<<"[ *2* ]"<<(float)(end-start)/CLOCKS_PER_SEC<<" (s)"<<endl;
		
		start=clock();
		bufs_[i].Init(tf_cloud,Z_AXIS);
		end=clock();
		//cout<<"[ *3* ]"<<(float)(end-start)/CLOCKS_PER_SEC<<" (s)"<<endl;
	}
}


/* void SegFSR::ProjectionGenerator()
{	
	Eigen::Vector4f centroid;    
    pcl::compute3DCentroid(*cloud_, centroid);
	for(int i=0;i<n_;i++)
	{
		clock_t start,end;
		pcl::PointCloud<PointType>::Ptr cloud_tf (new pcl::PointCloud<PointType> ());	
		// point set project		
		start=clock();
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
		coefficients->values.resize(4);
		coefficients->values[0] = orientations_[i].x;
		coefficients->values[1] = orientations_[i].y;
		coefficients->values[2] = orientations_[i].z;
		coefficients->values[3] = -(orientations_[i].x*centroid[0]+orientations_[i].y*centroid[1]+orientations_[i].z*centroid[2]);
		
		pcl::ProjectInliers<PointType> proj;
		proj.setModelType(pcl::SACMODEL_PLANE);
		proj.setInputCloud(cloud_);
		proj.setModelCoefficients(coefficients);
		proj.filter(*cloud_tf);
		
		end=clock();
		cout<<"[ *1* ]"<<(float)(end-start)/CLOCKS_PER_SEC<<" (s)"<<endl;
		
		// Generate Image
		start=clock();
		bufs_[i].Init(cloud_tf,Z_AXIS);
		end=clock();
		cout<<"[ *2* ]"<<(float)(end-start)/CLOCKS_PER_SEC<<" (s)"<<endl<<endl;;
	}
} */

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
	
	for(int i=0;i<orientations_.size();i++){
		printf("%f %f %f\n",orientations_[i].x,orientations_[i].y,orientations_[i].z);
	}
		
	
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
	
	for(int i=0;i<outlier_idx.size();i++)
	{
		cloud_->points[outlier_idx[i]].r=255;
		cloud_->points[outlier_idx[i]].g=0;
		cloud_->points[outlier_idx[i]].b=0;
	}
	end=clock();
	cout<<(float)(end-start)/CLOCKS_PER_SEC<<" (s)"<<endl;
	
	pcl::io::savePLYFileASCII("1.ply",*cloud_);
}



 


void SegFSR::Viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
	viewer->setBackgroundColor(1.0, 1.0, 1.0);	
	pcl::visualization::PointCloudColorHandlerRGBField<PointType> multi_color(cloud_); 	
	viewer->addPointCloud<PointType> (cloud_, multi_color, "1"); 
}