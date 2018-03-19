#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <fstream>

#include "ros/ros.h"

using  namespace cv;

using namespace std;

#define PROJ_IMG_SIZE 376

#define GRID_SIZE 24


int main(int argc, char **argv)
{
	ros::init(argc, argv, "ImgSender");
	ros::NodeHandle n;
	Mat Img = imread("/home/cyy/catkin_ws/src/mytest/pictures/circle/I (1).png",IMREAD_GRAYSCALE);

	if(Img.empty())
	{
	cout<< "load image failed!"<<endl;
	return 0;
	}
	ROS_INFO("%s", "load success");
	ros::Publisher pub_img = n.advertise<sensor_msgs::Image>(("img"),1000);
	
	{
		sensor_msgs::Image img_msg;
		cv_bridge::CvImage cvi;
		cvi.header.stamp = ros::Time::now();
		cvi.header.frame_id = "image0";
		cvi.encoding = "mono8";
		cvi.image = Img;
		cvi.toImageMsg(img_msg);
		pub_img.publish(img_msg);
	}
	
	ros::Rate loop_rate(60);

	
	char filename[] ="/home/cyy/catkin_ws/src/mytest/pictures/circle/I (1000).png";
	for(int i = 1; i <=100;i++)
	{
		snprintf(filename, sizeof(filename), "/home/cyy/catkin_ws/src/mytest/pictures/circle/I (%d).png", i);
		Mat Img1 = imread(filename,IMREAD_GRAYSCALE);
		sensor_msgs::Image img_msg;

		cv_bridge::CvImage cvi;
		cvi.header.stamp = ros::Time::now();
		cvi.header.frame_id = "image0";
		cvi.encoding = "mono8";
		cvi.image = Img1;
		cvi.toImageMsg(img_msg);	
		pub_img.publish(img_msg);
		ros::spinOnce();  
		loop_rate.sleep();

	}
	
	for(int i = 1; i <= 60;i++)
	{
		snprintf(filename, sizeof(filename), "/home/cyy/catkin_ws/src/mytest/pictures/line/I (%d).png", i);
		Mat Img1 = imread(filename,IMREAD_GRAYSCALE);
		sensor_msgs::Image img_msg;
		cv_bridge::CvImage cvi;
		cvi.header.stamp = ros::Time::now();
		cvi.header.frame_id = "image1";
		cvi.encoding = "mono8";
		cvi.image = Img1;
		cvi.toImageMsg(img_msg);
		pub_img.publish(img_msg);
		ros::spinOnce();  
		loop_rate.sleep();
	}
	{
		sensor_msgs::Image img_msg;
		snprintf(filename, sizeof(filename), "/home/cyy/catkin_ws/src/mytest/pictures/line/I (%d).png", 61);
		Mat Img1 = imread(filename,IMREAD_GRAYSCALE);
		cv_bridge::CvImage cvi;
		cvi.header.stamp = ros::Time::now();
		cvi.header.frame_id = "end_image";
		cvi.encoding = "mono8";
		cvi.image = Img1;
		cvi.toImageMsg(img_msg);
		pub_img.publish(img_msg);
		ros::spinOnce();  
		loop_rate.sleep();
	}
	return 0;
}

