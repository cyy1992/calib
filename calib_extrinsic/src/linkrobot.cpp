#include <ros/ros.h>
#include <sstream>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>
#include "basemovement.h"
using namespace std;
using namespace cv;

bool tempFlag = false;
bool endFlag = false;
baseMovement* pMv;
ros::Publisher pub_img;
bool startCalib = true;
void callbackImg(const sensor_msgs::ImageConstPtr &img_msg)
{
	bool flag;baseMovement::MOVE_STATE mstate;
	pMv->getState(flag,mstate);
	ros::Rate loop_rate(60);
	if(flag){
		cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(img_msg);
		Mat Img = ptr->image;
		cv_bridge::CvImage cvi;
		cvi.header.stamp = ptr->header.stamp;
		if(mstate == baseMovement::CIRCLE)
			cvi.header.frame_id = "image0";
		else if(mstate == baseMovement::LINE)
			cvi.header.frame_id = "image1";

		cvi.encoding = ptr->encoding;
		cvi.image = Img;
		sensor_msgs::Image new_msg;
		cvi.toImageMsg(new_msg);
		pub_img.publish(new_msg);
		ros::spinOnce();
		loop_rate.sleep();
		imshow("img",Img);
		waitKey(10);		
	}
	else
	{
		if(startCalib){
			if(mstate == baseMovement::ENDMOVE)
			{
				cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(img_msg);
				startCalib = false;
				Mat Img = ptr->image;
				cv_bridge::CvImage cvi;
				cvi.header.stamp = ptr->header.stamp;
				cvi.header.frame_id = "end_image";
				cvi.encoding = ptr->encoding;
				cvi.image = Img;
				sensor_msgs::Image new_msg;
				cvi.toImageMsg(new_msg);
				pub_img.publish(new_msg);
				ros::spinOnce();
				loop_rate.sleep();
			}
		}
	}
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "linkRobotNode");
	ros::NodeHandle n;
	pub_img = n.advertise<sensor_msgs::Image>(("img"),1000);
	//ros::Subscriber sub_img = n.subscribe("/jzhw/jzcamera", 100, callbackImg);
	pMv = new baseMovement(n);
	//pMv->startMove();
// 	
// 	ros::Publisher Calib_Movement_pub = n.advertise<geometry_msgs::Twist>("/ros_diff_controller/cmd_vel",100);
// 	geometry_msgs::Twist mv_circle,mv_line,setZero;
// 	mv_circle.linear.x = 0.0;mv_circle.linear.y = 0.0;mv_circle.linear.z = 0.0;
// 	mv_circle.angular.x = 0.0; mv_circle.angular.y = 0.0; mv_circle.angular.z = 0.2;
// 	
// 	mv_line.linear.x = -0.05; mv_line.linear.y = 0.0;  mv_line.linear.z = 0.0;
// 	mv_line.angular.x = 0.0; mv_line.angular.y = 0.0; mv_line.angular.z = 0.0;
// 	
// 	setZero.linear.x = 0.0;setZero.linear.y = 0.0;setZero.linear.z = 0.0;
// 	setZero.angular.x = 0.0; setZero.angular.y = 0.0; setZero.angular.z = 0.0;
// 	
// 	ros::Rate loop_rate(50);
// 	for(int i = 0; i < 1500; i++)
// 	{
// 		Calib_Movement_pub.publish(mv_circle);
// 		tempFlag = true;
// 		ros::spinOnce();
// 		loop_rate.sleep();
// 	}
// 	for(int i = 0; i < 10; i++)
// 	{
// 		Calib_Movement_pub.publish(setZero);
// 		tempFlag = true;
// 		ros::spinOnce();
// 		loop_rate.sleep();
// 	}
// 	ros::Duration(2).sleep();
// 	for(int i = 0; i < 150; i++)
// 	{
// 		Calib_Movement_pub.publish(mv_line);
// 		tempFlag = true;
// 		ros::spinOnce();
// 		loop_rate.sleep();
// 	}
// 	
// 	ros::Duration(2).sleep();
// 	for(int i = 0; i < 300; i++)
// 	{
// 		mv_line.linear.x = 0.05;
// 		Calib_Movement_pub.publish(mv_line);
// 		tempFlag = true;
// 		ros::spinOnce();
// 		loop_rate.sleep();
// 	}
// 	for(int i = 0; i < 300; i++)
// 	{
// 		Calib_Movement_pub.publish(mv_circle);
// 		tempFlag = false;
// 		ros::spinOnce();
// 		loop_rate.sleep();
// 	}
	ros::spin();
	return 0;
}