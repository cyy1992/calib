#ifndef CALIBRATEBASECAMERA_H
#define CALIBRATEBASECAMERA_H

#include <sstream>
#include <string>
#include <fstream>
#include <iostream>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <time.h>

#include "ros/ros.h"

#include "DataMatrixCreator.h"
#include "DataMatrixDetector.h"
#include "ImageProjector.h"

#include "basemovement.h"
//#include "DataMatrixDetector.h"

//#define IBVS_CONSTRAINED_SHOW_IMAGE
using  namespace cv;
using namespace std;


#define PROJ_IMG_SIZE 376
#define GRID_SIZE 24
#define REPROJECT_ERROR_THRESHOLD 3
#define SHOW_SINGLE_RESULR false
#define SHOW_FINAL_RESULT true

#define DM_CODE_LINE_NUM  5
#define DM_CODE_NUM       (DM_CODE_LINE_NUM*DM_CODE_LINE_NUM)
#define DM_CODE_SIZE      18.
#define DM_GRID_SIZE      24.

#define DM_CODE_NEEDED    8

class calibrateBaseCamera
{
public:
	calibrateBaseCamera(ros::NodeHandle &n,const std::string &path);
	~calibrateBaseCamera();

	void img_callback(const sensor_msgs::ImageConstPtr &img_msg);
	void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg);
	
	
	vector<Mat> worldInCamera_rvecs[2];
	vector<Mat> worldInCamera_tvecs[2];
	vector<Point3f> worldsInCamera[2];
	
	Mat base2camera_rot_result,base2camera_t_result;
	baseMovement *pMv;
	
	void test();
	void loadParams(std::string filename);
private:
	void paramSet();
	bool checkCollinearity(const vector<Point2f> &pt_obj_fitline);
	bool checkReprojectError(const vector<Point2f> &pt_img, const vector<Point3f> &pt_obj,Mat &rvec,Mat &tvec);
	
	void cvFitPlane(const CvMat* points, float* plane);
// 	void showSingleImgResult(Mat srcImg,const Mat &rvec,const Mat &tvec,map<string, vector<Point2f> > pts_img);
	void showFinalResult();
	
	bool ptsDetector(ImageProjector &ip, DataMatrixDetector &dmd,const Mat &srcImg,vector<Point2f> &pt_obj_fitline, vector<Point2f> &pt_img, 
				 vector<Point3f> &pt_obj);
	void firstDetector(ImageProjector &ip, DataMatrixDetector &dmd, const Mat &srcImg, vector<Point2f> &pt_obj_fitline, vector<Point2f> &pt_img,vector<Point3f> &pt_obj);
	
	bool get_RT(cv::Mat &srcImg,cv::Mat &rvec,cv::Mat &tvec);
	bool tagToPose(const Mat& srcImg, Mat& tvec, Mat& rvec,
                               double& reprojectError);
	void computeCalibratedResult();
	
	Mat base2camera_rot_calib,base2camera_t_calib,intrinsic_,distortion_,extrinsicR_,extrinsicT_;
	int width_,height_,projWidth_,projHeight_;
	double objWidth_,objHeight_;
	bool first_Img_flag;
	int camera_toward_;
	Mat showImg,showImg_rvec,showImg_tvec;
	DataMatrixDetector::DetectParams dp;
	
	
	ImageProjector* p_ip_ ;
	bool isFirstFrame;
	
	ros::Subscriber sub_img;
	ros::Subscriber sub_odom;
	bool startCalibFlag;
	
	struct DmInfo
	{
		cv::Point2f pt_distort_img;
		cv::Point2d pt_project_img;
		cv::Point2f pt_obj_fitline;
		cv::Point3f pt_obj;
		double img_angle = 0;
	};
	struct ThreadInfo
	{
		int id;
		char info;
		cv::Point2i start_loc;
	};
	int DM_MAX_THREADS;

	
	double resolution_width_inv_; //proj_pixel / obj_size
	double resolution_height_inv_;
	int part_size_;
	int part_size_2_;
	int part_size_4_;
	DataMatrixDetector::DetectParams dp_part_;
	
	double mfx,mfy,mu0,mv0,mk1,mk2;
	std::string camera_topic;
	baseMovement::MOVE_STATE mvState;
	
	struct odomData{
		double timeStamp;
		double xPos;
		double yPos;
		double qx,qy,qz,qw;
		double tpx,tpy,tpz,tax,tay,taz;
	};
	//save files
	vector<vector<Point2f>> saveImgPoints_circle;
	vector<vector<Point2f>> saveImgPoints_line;
	vector<vector<Point3f>> saveWorldPoints_circle;
	vector<vector<Point3f>> saveWorldPoints_line;
	vector<double> timeStamp_circle;
	vector<double> timeStamp_line;
	vector<odomData> odom_circle;
	vector<odomData> odom_line;
	double tmpTimeStamp_camera;
	odomData modom, setOdom;	

};

#endif // CALIBRATEBASECAMERA_H
