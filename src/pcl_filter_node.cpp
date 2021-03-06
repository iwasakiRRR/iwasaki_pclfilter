#include "iwasaki_pclfilter/pcl_filter.hpp"

int main (int argc, char** argv){
	ros::init (argc, argv, "iwasaki_pclfilter");
	RyoFilter rf;
	/*コールバック関数が主体ですヘッダーを参照してください*/
	rf.point_sub = rf.nh.subscribe("/camera/depth/points",1, &RyoFilter::cloud_callback,&rf);
	rf.point_pub = rf.nh.advertise<sensor_msgs::PointCloud2> ("mls_cloud", 1);
	
	ros::spin ();
}
