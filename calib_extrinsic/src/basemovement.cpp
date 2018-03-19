
#include "basemovement.h"

baseMovement::baseMovement(ros::NodeHandle &n)
{
	Calib_Movement_pub = n.advertise<geometry_msgs::Twist>("/ros_diff_controller/cmd_vel",1000);
	
	mv_circle.linear.x = 0.0;mv_circle.linear.y = 0.0;mv_circle.linear.z = 0.0;
	mv_circle.angular.x = 0.0; mv_circle.angular.y = 0.0; mv_circle.angular.z = 0.1;
	
	mv_line.linear.x = -0.02; mv_line.linear.y = 0.0;  mv_line.linear.z = 0.0;
	mv_line.angular.x = 0.0; mv_line.angular.y = 0.0; mv_line.angular.z = 0.0;
	
	setZero.linear.x = 0.0;setZero.linear.y = 0.0;setZero.linear.z = 0.0;
	setZero.angular.x = 0.0; setZero.angular.y = 0.0; setZero.angular.z = 0.0;
	
	isReceiveImg = false;
	mv_State = RESTING;
	std::thread mthread(&baseMovement::startMove,this);
	cout<< "starting join thread";
	mthread.detach();
}
baseMovement::~baseMovement()
{

}


void baseMovement::startMove()
{
	ros::Rate loop_rate(100);
	for(int i = 0; i < 6100; i++)
	{
		Calib_Movement_pub.publish(mv_circle);
		if(i>200&&i<5900)
		{
			isReceiveImg = true;
			mv_State = CIRCLE;
		}
		loop_rate.sleep();
	}
	ros::Duration(3).sleep();

	for(int i = 0; i < 600; i++)
	{
		Calib_Movement_pub.publish(mv_line);
		isReceiveImg = false;
		mv_State = RESTING;
		loop_rate.sleep();
	}
	ros::Duration(3).sleep();
	for(int i = 0; i < 1200; i++)
	{
		mv_line.linear.x = 0.02;
		Calib_Movement_pub.publish(mv_line);
		
		if(i>200&&i<1000)
		{
			isReceiveImg = true;
			mv_State = LINE;
		}
		else
		{
			isReceiveImg = false;
			mv_State = LINE;
		}

		if( i > 1001)
		{
			mv_State = ENDMOVE;
		}
		loop_rate.sleep();
	}
}

void baseMovement::getState(bool& _isReceiveImg, baseMovement::MOVE_STATE& _mvState)
{
	_isReceiveImg = isReceiveImg;
	_mvState = mv_State;
}
