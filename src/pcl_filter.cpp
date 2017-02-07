#include "iwasaki_pclfilter/pcl_filter.hpp"

RyoFilter::RyoFilter(){

}
void RyoFilter::Init(const sensor_msgs::PointCloud2ConstPtr& arg_input){

	pcl::fromROSMsg (*arg_input, *cloud);
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indicies);

}
void RyoFilter::DownSample(){
	sor1.setInputCloud (cloud);
 	sor1.setLeafSize (0.02, 0.02, 0.02);
	sor1.filter (*cloud);

}
void RyoFilter::Sor(){

	sor.setInputCloud (cloud);
	sor.setMeanK (50);
	sor.setStddevMulThresh (1.0);
	sor.setNegative (false);
	sor.filter (*cloud);
	
}
void RyoFilter::Smooth(){
	
	mls.setComputeNormals (true);//法線の計算を行うかどうか
		// 各パラメーターの設定
	mls.setInputCloud (cloud);
	mls.setPolynomialFit (true);
	mls.setSearchMethod (tree);
	mls.setSearchRadius (0.04);
}
void RyoFilter::PlaneExtract(){

	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_LMEDS);
	seg.setMaxIterations (100);//最大繰り返し回数設定
	seg.setDistanceThreshold (0.04);//モデルの閾値までの距離設定
	seg.setInputCloud (cloud);//インプットデータセットの提供
	seg.segment (*inliers, *coefficients);//セグメンテーションのための基本メソッド
	std::cout<<"inliers"<<inliers->indices.size ()<<std::endl;//いらない文章

}
void RyoFilter::PlaneRem(){

	extract.setInputCloud (cloud);//cloud.makeShared()はクラウドのコピーへのポインタを返す
	extract.setIndices (inliers); 
	extract.setNegative (false);
	extract.filter (*cloud_plane);
	extract.setNegative (true);  
	extract.filter (*cloud);
}

void RyoFilter::Ece(){

	tree->setInputCloud (cloud);  
	ec.setClusterTolerance (0.02); // 2cm 
	ec.setMinClusterSize (100);//もともと100
	ec.setMaxClusterSize (25000);  
	ec.setSearchMethod (tree);  
	ec.setInputCloud(cloud);  
	ec.extract (cluster_indices);
	std::cout<<"vectorクリア前"<<cluster_indices.size()<< " "
						<<cluster_indices.capacity()<< " "
						<<std::endl;
}

void RyoFilter::Ror(){
				
	outrem.setInputCloud(cloud);
	outrem.setRadiusSearch(0.04);//元は０．０４
	outrem.setMinNeighborsInRadius (10);
	outrem.filter (*cloud);
	
}

void RyoFilter::Exception(){
	
	if(cloud->points.size()<20){
		cloud->width  =1;
		cloud->height = 1;
		cloud->points.resize (cloud->width * cloud->height);
		cloud->points[0].x=100;
		cloud->points[0].y=100;
		cloud->points[0].z=100;

	}
}
