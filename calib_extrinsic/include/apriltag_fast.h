#ifndef IBVS_CONSTRAINED_H_
#define IBVS_CONSTRAINED_H_

#include <iostream>
#include <fstream>
#include <time.h>

#include <opencv2/opencv.hpp>

#include "ImageProjector.h"
//#include "DataMatrixDetector.h"

//#define IBVS_CONSTRAINED_SHOW_IMAGE

#define DM_CODE_LINE_NUM  5
#define DM_CODE_NUM       (DM_CODE_LINE_NUM*DM_CODE_LINE_NUM)
#define DM_CODE_SIZE      18.
#define DM_GRID_SIZE      24.

#define DM_MAX_THREADS    10
#define DM_CODE_NEEDED    10

class apriltag_fast
{
public:
  apriltag_fast(Mat intrinsic, Mat distortion,Mat extrinsicR,Mat extrinsicT,int width, int height, int projWidth, int projHeight, double  objWidth,
                             double objHeight);
  ~apriltag_fast();
  bool tagToPose(const cv::Mat& srcImg, cv::Mat& tvec, cv::Mat& rvec, double& reprojectError);
private:
  

//   enum Action
//   {
//     IDLE,
//     DIRECTION_ADJUSTING,
//     POSITION_ADJUSTING,
//     FINAL_DIRECTION_ADJUSTING,
//     PREEMPTED,
//     SUCCEED,
//     TEACHING
//   };

  struct DmInfo
  {
    cv::Point2f pt_distort_img;
    cv::Point2d pt_project_img;
    cv::Point2f pt_obj_fitline;
    cv::Point3f pt_obj;
    //double img_angle;
  };

//   std::string action_name_;
//   visual_servo_msgs::IbvsConstrainedFeedback feedback_;
//   visual_servo_msgs::IbvsConstrainedResult result_;
// 
//   //  double task_loc_lambda_zero_;
//   //  double task_loc_lambda_infinite_;
//   //  double task_loc_lambda_slope_;
//   double task_loc_lambda_;
// //  double task_loc_mu_;
//   double task_loc_error_ratio_;
//   //  double task_rot_lambda_zero_;
//   //  double task_rot_lambda_infinite_;
//   //  double task_rot_lambda_slope_;
//   double task_rot_lambda_;
// //  double task_rot_mu_;
//   double task_rot_error_ratio_;
// //  double min_vx_;
// //  double min_wz_;
//   double max_vx_;
//   double max_wz_;
//   double max_vel_acc_;
//   double max_ang_acc_;
  cv::Mat intrinsic_;
  cv::Mat distortion_;
  cv::Mat extrinsicR_,extrinsicT_;
  int width_, height_, projWidth_, projHeight_;
  double objWidth_,objHeight_;
//   cv::Point3f targetPointInWorld_;
//   cv::Mat targetPointInWorld_mat_;
//   cv::Mat targetDirectionPointInWorld_mat_;
//   cv::Point3f targetPointInBaseDesired_;
//   cv::Mat targetPointInBaseDesired_mat_;
// 
  ImageProjector* p_ip_ ;
// 
//   double resolution_width_inv_distortion; //proj_pixel / obj_size
//   double resolution_height_inv_;
//   int part_size_;
//   int part_size_2_;
//   int part_size_4_;
// 
//   Action action_;
//   uint8_t status_;
//   double vx_;
//   double wz_;
//   double error_x_;
//   double error_y_;
//   double error_theta_;
//   double error_value_;
//   double targetToBase_distance_; // mm
//   double targetToBase_angle_; // rad
//   double base2world_x_;
//   double base2world_y_;
//   double base2world_theta_;
// 
//   bool save_initial_map_;
//   bool log_initial_pose_;
//   std::ofstream initial_log_;
//   ros::Time time_start_;
//   int cnt_;
  
  bool isFirstFrame;
#ifdef IBVS_CONSTRAINED_SHOW_IMAGE
  vpPlot graph_;
  //int cnt_;
#endif
};

#endif
