#include "BoundingBox.h"
/*
	BoundingBox
*/
BoundingBox::BoundingBox(pcl::PointCloud<PointType>::Ptr cloud,string id,V3 color)
{
	// Definition
	pcl::PointCloud<PointType>::Ptr transformedCloud(new pcl::PointCloud<PointType>());	
	PointType pmin_tf, pmax_tf;	
	Eigen::Vector4f centroid;
	Eigen::Matrix3f covariance;
	Eigen::Matrix4f tm;
	Eigen::Matrix4f tm_inv;

	// Init
	id_=id;
	color_=color;

    // calculate eigenvalue, eigenvector and centroid
	pcl::compute3DCentroid(*cloud, centroid);
	pcl::computeCovarianceMatrixNormalized(*cloud, centroid, covariance);	
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eig_vec = eigen_solver.eigenvectors();	
	Eigen::Vector3f eig_val = eigen_solver.eigenvalues();
	
	eig_vec.col(2) = eig_vec.col(0).cross(eig_vec.col(1));
	eig_vec.col(0) = eig_vec.col(1).cross(eig_vec.col(2));
	eig_vec.col(1) = eig_vec.col(2).cross(eig_vec.col(0));

    // transformation matrix
    tm = Eigen::Matrix4f::Identity();
    tm_inv = Eigen::Matrix4f::Identity();	
    // get 4x4 matrix [R | t]
	tm.block<3, 3>(0, 0) = eig_vec.transpose();   //R.
	tm.block<3, 1>(0, 3) = -1.0f * (eig_vec.transpose()) *(centroid.head<3>());//  -R*t	
	tm_inv = tm.inverse();
	
	// convert current position to origin position
	pcl::transformPointCloud(*cloud, *transformedCloud, tm);
	pcl::getMinMax3D(*transformedCloud, pmin_tf, pmax_tf);
	cen_tf_ = 0.5f*(pmin_tf.getVector3fMap() + pmax_tf.getVector3fMap());
	pcl::transformPoint(cen_tf_, cen_, Eigen::Affine3f(tm_inv));
	whd_ = pmax_tf.getVector3fMap() - pmin_tf.getVector3fMap();	
	float sc1 = (whd_(0) + whd_(1) + whd_(2)) / 3;  //点云平均尺度，用于设置主方向箭头大小
	bboxQ_=tm_inv.block<3, 3>(0, 0);
	
	// calculate axis direction
	cp_.x = centroid(0);
	cp_.y = centroid(1);
	cp_.z = centroid(2);	
	pcX_.x = sc1 * eig_vec(0, 0) + cp_.x; // x axis
	pcX_.y = sc1 * eig_vec(1, 0) + cp_.y;
	pcX_.z = sc1 * eig_vec(2, 0) + cp_.z;
	pcY_.x = sc1 * eig_vec(0, 1) + cp_.x; // y axis
	pcY_.y = sc1 * eig_vec(1, 1) + cp_.y;
	pcY_.z = sc1 * eig_vec(2, 1) + cp_.z;
	pcZ_.x = sc1 * eig_vec(0, 2) + cp_.x; // z axis
	pcZ_.y = sc1 * eig_vec(1, 2) + cp_.y;
	pcZ_.z = sc1 * eig_vec(2, 2) + cp_.z;
	
	// calculate vertex	
	//v1
	v1_.x=cen_tf_(0)+whd_(0)/2.0;
	v1_.y=cen_tf_(1)+whd_(1)/2.0;
	v1_.z=cen_tf_(2)+whd_(2)/2.0;
	v1_=pcl::transformPoint(v1_,Eigen::Affine3f(tm_inv));	
	//v2
	v2_.x=cen_tf_(0)+whd_(0)/2.0;
	v2_.y=cen_tf_(1)-whd_(1)/2.0;
	v2_.z=cen_tf_(2)+whd_(2)/2.0;
	v2_=pcl::transformPoint(v2_,Eigen::Affine3f(tm_inv));	
	//v3
	v3_.x=cen_tf_(0)+whd_(0)/2.0;
	v3_.y=cen_tf_(1)-whd_(1)/2.0;
	v3_.z=cen_tf_(2)-whd_(2)/2.0;
	v3_=pcl::transformPoint(v3_,Eigen::Affine3f(tm_inv));
	//v4
	v4_.x=cen_tf_(0)+whd_(0)/2.0;
	v4_.y=cen_tf_(1)+whd_(1)/2.0;
	v4_.z=cen_tf_(2)-whd_(2)/2.0;
	v4_=pcl::transformPoint(v4_,Eigen::Affine3f(tm_inv));	
}

void DisplayBoundingBox(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<PointType>::Ptr cloud, BoundingBox bb)
{
	/* 
		//show vertex
		viewer.addSphere(bb.v1_,0.01,0.0,0.0,1.0,"sphere1");
		viewer.addSphere(bb.v2_,0.01,0.0,0.0,1.0,"sphere2");
		viewer.addSphere(bb.v3_,0.01,0.0,0.0,1.0,"sphere3");
		viewer.addSphere(bb.v4_,0.01,1.0,1.0,0.0,"sphere4");
	*/
	
	viewer.addCube(bb.cen_, bb.bboxQ_, bb.whd_(0), bb.whd_(1), bb.whd_(2) , bb.id_);
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, bb.id_);
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, bb.color_.x, bb.color_.y, bb.color_.z, bb.id_);
	
	// viewer.addSphere(cp_,0.1,1.0,1.0,0);
	viewer.addArrow(bb.pcX_, bb.cp_, 1.0, 0.0, 0.0, false, bb.id_+"arrow_x");
	viewer.addArrow(bb.pcY_, bb.cp_, 0.0, 1.0, 0.0, false, bb.id_+"arrow_y");
	viewer.addArrow(bb.pcZ_, bb.cp_, 0.0, 0.0, 1.0, false, bb.id_+"arrow_z");		
}