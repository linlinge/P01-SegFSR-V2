#include <iostream>
/*
#include "BasicGeometry.h"
#include "zBuffer.h"
#include "V3.hpp"
#include <pcl/registration/transformation_estimation_3point.h> 
*/
#include "SegFSR.h"

using namespace std;

int main(int argc, char **argv)
{	
	
	/* cv::Mat v=(cv::Mat_<uchar>(4,5)<<1,0,0,1,0,1,0,0,1,1,1,0,0,1,1,0,1,1,1,1);
	cout<<v<<endl; */
	
	/* cv::Mat v=cv::imread(argv[1],0);
	Graph gh;
	gh.Establish(v);
	gh.Run();
	gh.Print(); */

	// load point cloud
	
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	if (pcl::io::loadPLYFile<PointType>(argv[1], *cloud) == -1){
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	SegFSR alg;
	alg.Init02(cloud,100);
	alg.Run();
	
	
	/* alg.Viewer(viewer);
	viewer->addCoordinateSystem(1.0f);
	while(!viewer->wasStopped()){
		viewer->spin();
		boost::this_thread::sleep (boost::posix_time::microseconds (10));
	} */
	
	
	
	
	
	
	/*
	Mat vec01=(Mat_<float>(3,1) <<-1,-1,-1);
	Mat R=GetRotationMatrixToAxis(V3(vec01),Y_AXIS);		
	Mat vec02=R*vec01;
	
	//Mat vec02=R01*vec01;
	V3 v1=V3(vec01);
	V3 v2=V3(vec02);
	
	
	pcl::visualization::PCLVisualizer viewer;
	viewer.setBackgroundColor(1.0, 1.0, 1.0);
	viewer.addCoordinateSystem(1.0f);
	// add arrow
	viewer.addArrow<pcl::PointXYZ>(pcl::PointXYZ(v1.x,v1.y,v1.z), pcl::PointXYZ(0,0,0), 1.0f, 0.0f, 0, false, "X");
	viewer.addArrow<pcl::PointXYZ>(pcl::PointXYZ(v2.x,v2.y,v2.z), pcl::PointXYZ(0,0,0), 1.0f, 1.0f, 0, false, "Y");
	
	while(!viewer.wasStopped()){	
		viewer.spin();
		boost::this_thread::sleep (boost::posix_time::microseconds (10));
	}  
	*/
	
	
	  
	/* // get bounding box
	BoundingBox bb(cloud,"cloud");	
	
	// projection
	Line line2(V3(bb.v1_.x,bb.v1_.y,bb.v1_.z),V3(bb.v2_.x,bb.v2_.y,bb.v2_.z),PP);
	V3 x2_prime=line2.TransformTo(XY);	
	Line line3(V3(bb.v1_.x,bb.v1_.y,bb.v1_.z),V3(bb.v3_.x,bb.v3_.y,bb.v3_.z),PP);
	V3 x3_prime=line3.TransformTo(XY);	
	Line line4(V3(bb.v1_.x,bb.v1_.y,bb.v1_.z),V3(bb.v4_.x,bb.v4_.y,bb.v4_.z),PP);
	V3 x4_prime=line4.TransformTo(XY);
	
	
	// get rigid transformation
	pcl::registration::TransformationEstimation3Point<PointXYZ, PointXYZ> te;
    pcl::registration::TransformationEstimation3Point<PointXYZ, PointXYZ>::Matrix4 R;
    pcl::PointCloud<PointXYZ> src, tgt;
    src.resize(3); tgt.resize(3);
    src.points[0].getVector3fMap() << bb.v2_.x, bb.v2_.y, bb.v2_.z;
    src.points[1].getVector3fMap() << bb.v3_.x, bb.v3_.y, bb.v3_.z;
    src.points[2].getVector3fMap() << bb.v4_.x, bb.v4_.y, bb.v4_.z;
    // std::cout << "source cloud is " << endl << src.getMatrixXfMap(3, 4, 0) << endl;
    tgt.points[0].getVector3fMap() << x2_prime.x,x2_prime.y,x2_prime.z;
    tgt.points[1].getVector3fMap() << x3_prime.x,x3_prime.y,x3_prime.z;
    tgt.points[2].getVector3fMap() << x4_prime.x,x4_prime.y,x4_prime.z;
    // std::cout << "target cloud is " << endl << tgt.getMatrixXfMap(3, 4, 0) << endl;
    te.estimateRigidTransformation(src, tgt, R);
    // std::cout << "computed transformation is \n" << R << endl;	
	
	// transform point cloud
	pcl::PointCloud<PointType>::Ptr transformedCloud(new pcl::PointCloud<PointType>());
	pcl::transformPointCloud(*cloud, *transformedCloud, R);
			
	
	// project to image plane	
	zBuffer buf(transformedCloud);	
	buf.GetImage(0);
	imwrite(string(argv[2]),buf.img_);
	buf.LoadMask("1-mask.png");  */
	
	
	// display point cloud
	
	
	
	
	
	
	/* pcl::visualization::PointCloudColorHandlerRGBField<PointType> multi_color(cloud);  //输入的初始点云相关
	viewer.addPointCloud(cloud, multi_color, "cloud");	
	DisplayBoundingBox(viewer,cloud,BoundingBox(cloud,"cloud",V3(0.0,1.0,1.0)));  */

	/* pcl::visualization::PointCloudColorHandlerRGBField<PointType> multi_color2(transformedCloud);  //输入的初始点云相关
	viewer.addPointCloud(transformedCloud, multi_color2, "cloud2");	
	DisplayBoundingBox(viewer,transformedCloud,BoundingBox(transformedCloud,"transformedCloud",V3(1.0,0.0,0.0)));  */
																
	// display 	
	/* viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox");
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "bbox"); */	
	//viewer.addCoordinateSystem(0.5f);	
	
	
	
	
	

	
	// projection
	/* 
		Mat img(IMG_SCALE, IMG_SCALE*ratio_width_height,CV_8UC3,Scalar(100,100,0));
		imshow("Image Viewer",img);
		waitKey(0); 
	*/
	
	return 0;
} 


