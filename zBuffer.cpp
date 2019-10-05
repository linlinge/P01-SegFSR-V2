#include "zBuffer.h"
#include <math.h>
#include <iostream>
using namespace std;

zBuffer::zBuffer(pcl::PointCloud<PointType>::Ptr cloud)
{	
	// init
	float x_min=INT_MAX;
	float y_min=INT_MAX;
	float x_max=-INT_MAX;
	float y_max=-INT_MAX;
	cloud_=cloud;
	for(auto pt: cloud_->points)
	{
		x_min=x_min < pt.x ? x_min:pt.x;
		x_max=x_max > pt.x ? x_max:pt.x;
		y_min=y_min < pt.y ? y_min:pt.y;
		y_max=y_max > pt.y ? y_max:pt.y;
	}
	v_left_up_.x=x_min;
	v_left_up_.y=y_max;
	v_right_down_.x=x_max;
	v_right_down_.y=y_min;
	
	// calculate paramters
	actual_width_=x_max-x_min;
	actual_height_=y_max-y_min;

	
	float scale_width_height= actual_height_/actual_width_;
	image_width_=IMAGE_WIDTH;
	image_height_=ceil(IMAGE_WIDTH*scale_width_height);
	wstep_=actual_width_/image_width_;
	hstep_=actual_height_/image_height_;
	
	// create image
	vector<PixelInfo> tmp;
	tmp.resize(image_width_);
	for(int i=0;i<image_height_;i++)
	{
		dat_.push_back(tmp);
	}
	img_.create(image_height_,image_width_,CV_8UC3);
}

void zBuffer::GetImage(int mode)
{
	if(mode==0)
	{
		for(int k=0;k < cloud_->points.size(); k++){
			int i=floor(abs(cloud_->points[k].y-v_left_up_.y)/hstep_);
			int j=floor(abs(cloud_->points[k].x-v_left_up_.x)/wstep_);
			i=i < image_height_ ? i:image_height_-1;
			j=j < image_width_ ? j: image_width_-1;	
			dat_[i][j].dat_.push_back(cloud_->points[k]);
		}
		
 		for(int i=0;i<dat_.size();i++){
			for(int j=0;j<dat_[0].size();j++){						
				if(dat_[i][j].dat_.size()!=0){											
					vector<PointType>& tmp=dat_[i][j].dat_;				
					sort(tmp.begin(),tmp.end(),[](PointType& e1, PointType& e2){return e1.z<e2.z;});
				}
			}
		}
		
		// image
		for(int i=0;i<image_height_;i++){
			for(int j=0;j<image_width_;j++){			
				if(dat_[i][j].dat_.size()!=0){
					img_.at<cv::Vec3b>(i,j)[0]=dat_[i][j].dat_[0].b;
					img_.at<cv::Vec3b>(i,j)[1]=dat_[i][j].dat_[0].g;
					img_.at<cv::Vec3b>(i,j)[2]=dat_[i][j].dat_[0].r;
				}
				else{
					img_.at<cv::Vec3b>(i,j)[0]=255;
					img_.at<cv::Vec3b>(i,j)[1]=255;
					img_.at<cv::Vec3b>(i,j)[2]=255;
				}				
			}
		}
	}
}

void zBuffer::LoadMask(string mask_path)
{
	pcl::PointCloud<PointType>::Ptr p_tmp(new pcl::PointCloud<PointType>);
	mask_path="1-mask.png";
	cv::Mat img=cv::imread(mask_path,0);
	for(int i=0;i<img.rows;i++)
	{
		for(int j=0;j<img.cols;j++)
		{
			if(img.ptr<uchar>(i)[j]==255)
			{
				for(int k=0;k<dat_[i][j].dat_.size();k++)
				{				
					p_tmp->points.push_back(dat_[i][j].dat_[k]);
				}
			}
		}
	}			
	pcl::io::savePLYFileASCII("filter01.ply",*p_tmp);
}