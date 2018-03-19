#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <fstream>
#include <iostream>
#include <string>
#include <math.h>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
	ros::init(argc,argv,"imgSender");
	ros::NodeHandle n;
	ros::Publisher pub_img = n.advertise<sensor_msgs::Image>(("calibImg"),1000);
	string package_path = ros::package::getPath("calib_intrinsic");
	cout << package_path <<std::endl;
	ros::Rate loop_rate(5);
	for(int i = 1; i <=5000;i++)
	{
		string filename = package_path+"/leftCamData/left_"+to_string(i%8)+".jpg";
		Mat Img1 = imread(filename);
		cvtColor(Img1,Img1,CV_RGB2GRAY);
		sensor_msgs::Image img_msg;

		cv_bridge::CvImage cvi;
		cvi.header.stamp = ros::Time::now();
		cvi.header.frame_id = "image0";
		cvi.encoding = "mono8";
		cvi.image = Img1;
		cvi.toImageMsg(img_msg);
		imshow("11",Img1);
		waitKey(20);
		pub_img.publish(img_msg);
		ros::spinOnce();  
		loop_rate.sleep();
	}
	return 0;
	
}
