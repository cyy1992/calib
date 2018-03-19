#ifndef BASEMOVEMENT_H
#define BASEMOVEMENT_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <thread>
#include <pthread.h>
using namespace std;
class baseMovement
{
public:
	enum MOVE_STATE{
		RESTING,
		CIRCLE,
		LINE,
		ENDMOVE
	};
	baseMovement(ros::NodeHandle &n);
	~baseMovement();
	void startMove();
	void getState(bool &isReceiveImg, MOVE_STATE &mv_State);
	
	
private:
	geometry_msgs::Twist mv_circle,mv_line,setZero;
	ros::Publisher Calib_Movement_pub;
	bool isReceiveImg;
	MOVE_STATE mv_State;
	pthread_t mThread;
	
};

#endif // BASEMOVEMENT_H
