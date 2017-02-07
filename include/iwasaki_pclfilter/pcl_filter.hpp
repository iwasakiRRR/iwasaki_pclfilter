#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>  
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <stdio.h>
#include <time.h>
#include <pcl/filters/passthrough.h>

#include <iostream>
#include <math.h>
#include <fstream>
#include <string>
#include <limits>

class RyoFilter{
	public:
		RyoFilter();
		ros::Publisher point_pub;
		ros::Publisher pub;
		ros::Subscriber point_sub;
		ros::NodeHandle nh;
		clock_t start,end;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane;
		pcl::ModelCoefficients::Ptr coefficients;
		pcl::PointIndices::Ptr inliers;
		pcl::VoxelGrid<pcl::PointXYZ> sor1;
		sensor_msgs::PointCloud2 kinect2_points;
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
		pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		std::vector<pcl::PointIndices> cluster_indices;
		std::vector<pcl::PointIndices> hoge;
  		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;  
		pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
		sensor_msgs::PointCloud2 mls_cloud;
		sensor_msgs::PointCloud2 cluster;
		std::vector<int> indicies;
		
		void Init(const sensor_msgs::PointCloud2ConstPtr& _input);
		void DownSample();
		void Sor();
		void Smooth();
		void PlaneExtract();
		void PlaneRem();
		void Ece();
		void Ror();
		void Exception();
		void OfsFunc(double ryo);
		

		void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& input){
		
			cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
			//cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZ>);
			cloud_plane.reset(new pcl::PointCloud<pcl::PointXYZ>);
			coefficients.reset(new pcl::ModelCoefficients);
			inliers.reset(new pcl::PointIndices);
			tree.reset(new pcl::search::KdTree<pcl::PointXYZ>);
																															
			start=clock()/1000;
			Init(input);//ポインタを送る
			DownSample();
			Sor();
			Smooth();
			PlaneExtract();
			if (inliers->indices.size () == 0)  {  
				PCL_ERROR ("Could not estimate a planar model for the given dataset.");   
			}  
			else{
				PlaneRem();
			}
			Ror();
			Exception();
			Ece();
			pcl::toROSMsg (*cloud, mls_cloud);
			point_pub.publish (mls_cloud);
			end=clock()/1000;
			if(cloud->points.size()>1){
				std::cerr<<"pointnum="<<cloud->points.size()<<"  "<<"time:"<<end-start<<" ms"<<std::endl;
			}
			else{
				std::cerr<<"pointnum=0 "<<"  "<<"time:"<<end-start<<" ms"<<std::endl;
			}
			//Ece();
			
			cluster_indices.clear();
			cluster_indices.swap(hoge);
			//std::cout<<"vectorクリア後"<<cluster_indices.size()<< "　"<<cluster_indices.capacity()<<std::endl;	
		}		
};


