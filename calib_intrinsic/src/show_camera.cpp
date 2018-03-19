#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <iostream>
#include <string>
#include <math.h>

using namespace std;
using namespace cv;
#define RATIO_THRES	0.7

bool g_x_min_flag = false;
bool g_x_max_flag = false;
bool g_y_min_flag = false;
bool g_y_max_flag = false;
bool g_z_flag = false;
bool showTextFlag = false;
bool isFirstImg = true;
int reCalibFlag = 1;
double g_ratio;
Size imageSize;
Size checkerSize;
Mat covered_area;
string camera_topic;

double timeRemaining;
int imgNumber;

int blackScreenNum;
vector<Point2f> globalCoor;

void checkImg(const Mat &rvec,const Mat &tvec, vector<Point2f> &ImgCoor)
{
	Mat RotateM;
	Rodrigues(rvec, RotateM);

	Mat cameraLoc = -RotateM.inv()*tvec;
	Mat axis_Z = (Mat_<double>(3, 1) << 0, 0, 1); 
	Mat camera_axis_Z = RotateM.inv()*(axis_Z-tvec);
	Mat camera_dir = camera_axis_Z-cameraLoc;
	camera_dir = camera_dir/norm(camera_dir);

	string msg = format("(x y z): %.2lf %.2lf %.2lf", camera_dir.at<double>(0), camera_dir.at<double>(1), camera_dir.at<double>(2));

	if(camera_dir.at<double>(0) < -0.3)
		g_x_min_flag = true;
	else if(camera_dir.at<double>(0) > 0.3)
		g_x_max_flag = true;

	if(camera_dir.at<double>(1) < -0.3)
		g_y_min_flag = true;
	else if(camera_dir.at<double>(1) > 0.3)
		g_y_max_flag = true;
	if(camera_dir.at<double>(2) < 0.8)
		g_z_flag = true;
}
void img_callback(const sensor_msgs::ImageConstPtr& img_msg)
{
	cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(img_msg);
	Mat img = ptr->image;
	if(isFirstImg)
	{
		imageSize = img.size();
		covered_area = Mat::zeros(imageSize, CV_8UC1);
		isFirstImg = false;
	}
	if(ptr->header.frame_id !="justShowImage")
	{
		Mat mcor = Mat(globalCoor);
		cvtColor(img, img, COLOR_GRAY2BGR);
		drawChessboardCorners(img, checkerSize, Mat(mcor), 1);
	}
	if(blackScreenNum> 0)
		img = Mat::zeros(imageSize, CV_8UC1);
	blackScreenNum = blackScreenNum-1;
	if(showTextFlag)
	{
		if(!g_x_min_flag)
			putText( img, "x < -0.3 unsatisfied!", Point(50, 150), 1, 1.5, Scalar(0, 0, 255), 2);
		if(!g_x_max_flag)
			putText( img, "x > 0.3 unsatisfied!", Point(50, 200), 1, 1.5, Scalar(0,0,255), 2);

		if(!g_y_min_flag)
			putText( img, "y < -0.3 unsatisfied!", Point(50, 250), 1, 1.5, Scalar(0,0,255), 2);
		if(!g_y_max_flag)
			putText( img, "y > 0.3 unsatisfied!", Point(50, 300), 1, 1.5, Scalar(0,0,255), 2);

		if(!g_z_flag)
			putText( img, "z > 0.8 unsatisfied!", Point(50, 350), 1, 1.5, Scalar(0,0,255), 2);
	}
	string textMsg0 = format("Coverage not unsatisfied, it is %.2lf now ", g_ratio);
	if(g_ratio < RATIO_THRES)
			putText( img, textMsg0, Point(50, 100), 1, 1.5, Scalar(0,0,255), 2);
	string textMsg1 = format("nextImg will be received in %.2lf cputime", timeRemaining);
	putText( img, textMsg1, Point(50, 400), 1, 1.5, Scalar(0,0,255), 2);
		
	string textMsg2 = format("There ara %2d images received", imgNumber);
	putText( img, textMsg2, Point(50, 50), 1, 1.5, Scalar(0,0,255), 2);
	imshow("careCalibFlaglibImg",img);
	int k = waitKey(2);
// 	if(k == 's'||k == 'S')
// 		imwrite(std::to_string(ros::Time::now().toNSec())+".bmp",img);
}
void ptsData_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
 	vector <double> tmp = msg->data;
	vector<double>::iterator it = tmp.begin();
	reCalibFlag = tmp[0];
	double fx = tmp[1];
	double fy = tmp[2];
	double u0 = tmp[3];
	double v0 = tmp[4];
	double kx = tmp[5];
	double ky = tmp[6];
	checkerSize.width = tmp[7]-1;
	checkerSize.height = tmp[8]-1;
	int tmpFlag = tmp[9];
	if(tmpFlag)
		blackScreenNum = 2;
	vector<Point3f> ObjCoor;
	vector<Point2f> ImgCoor;
	int num = (tmp.size()-8)/4;
	for(int i = 0; i< num; i++)
	{
		ImgCoor.push_back(Point2f(tmp[i*2+10],tmp[i*2+11]));
		ObjCoor.push_back(Point3f(tmp[i*2+10+2*num],tmp[i*2+11+2*num],0));
	}
	globalCoor = ImgCoor;
	if (fx != 0)
	{
		Mat camera_matrix = (cv::Mat_<double>(3,3) << fx,0,u0, 0, fy, v0, 0, 0,1);
		Mat distortion_coefficients = (cv::Mat_<double>(1,5) << kx,ky,0,0,0);

		Mat rvec;
		Mat tvec;
		solvePnP(ObjCoor,ImgCoor,camera_matrix,distortion_coefficients,rvec,tvec,false,CV_EPNP);
		showTextFlag = true;
		checkImg(rvec,tvec,ImgCoor);
	}
	
	if(!isFirstImg)
	{
		Rect b_rect = boundingRect(ImgCoor);
		Mat temp_coverd = Mat::zeros(imageSize, CV_8UC1);
		rectangle(temp_coverd, b_rect, Scalar(255), CV_FILLED);
		covered_area = covered_area+temp_coverd;
		Mat tmp_show;
		resize(covered_area, tmp_show, Size(188,120));
		//imshow("coverd_area",tmp_show);waitKey(2);
		int cnt_nonZero = countNonZero(covered_area);
		g_ratio = cnt_nonZero/(float)(covered_area.rows*covered_area.cols);
	}
	vector<double>().swap(tmp);
}

void timeNumber_callback(const std_msgs::Float64MultiArray::ConstPtr & msg)
{
	vector <double> tmp = msg->data;
	timeRemaining = tmp[0];
	imgNumber = tmp[1];
	if(!g_x_max_flag)
		g_x_min_flag = tmp[2];
	if(!g_x_min_flag)
		g_x_max_flag = tmp[3];
	if(!g_y_max_flag)
		g_y_min_flag = tmp[4];
	if(!g_y_min_flag)
		g_y_max_flag = tmp[5];
	if(!g_z_flag)
		g_z_flag = tmp[6];
	if(reCalibFlag)
	{
		g_x_min_flag = tmp[2];
		g_x_max_flag = tmp[3];
		g_y_min_flag = tmp[4];
		g_y_max_flag = tmp[5];
	}
}
int main(int argc, char **argv)
{	
// 	string config_file = argv[1];
// 	cout <<"config : " <<config_file<<endl;
	ros::init(argc,argv,"showCamera");

// 	loadParams(config_file);
	ros::NodeHandle n;
	ros::Subscriber sub_img;
	ros::Subscriber sub_ptsData;
	ros::Subscriber sub_timeNumber;
	sub_img = n.subscribe("/matlabImg", 3, img_callback);
	sub_ptsData = n.subscribe("/ptsData_matlab", 3, ptsData_callback);
	sub_timeNumber = n.subscribe("/remainingTimeAndImgNumber",3,timeNumber_callback);
	ros::spin();
	return 0;
	
}
