#ifndef IBVS_CONSTRAINED_H_
#define IBVS_CONSTRAINED_H_

#include <iostream>
#include <fstream>
#include <time.h>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureDepth.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/vs/vpServo.h>
#include <visp3/robot/vpSimulatorPioneer.h>
#include <visp3/core/vpVelocityTwistMatrix.h>

#include <opencv2/opencv.hpp>

#include "ImageProjector.h"
#include "DataMatrixDetector.h"

#include <visual_servo_msgs/IbvsConstrainedAction.h>
#include <visual_servo_msgs/VisualServoTeaching.h>

//#define IBVS_CONSTRAINED_SHOW_IMAGE

#define DM_CODE_LINE_NUM  5
#define DM_CODE_NUM       (DM_CODE_LINE_NUM*DM_CODE_LINE_NUM)
#define DM_CODE_SIZE      18.
#define DM_GRID_SIZE      24.

#define DM_MAX_THREADS    10
#define DM_CODE_NEEDED    8

class IbvsConstrained
{
public:
  IbvsConstrained(ros::NodeHandle nh, const std::string& name);
  ~IbvsConstrained();

private:
  bool
  teachingServer(::visual_servo_msgs::VisualServoTeaching::Request& request,
                 ::visual_servo_msgs::VisualServoTeaching::Response& response);
  void asCB(const visual_servo_msgs::IbvsConstrainedGoalConstPtr& goal);
  void imgCb(const sensor_msgs::ImageConstPtr& img_msg);
  bool dmToPose(const cv::Mat& srcImg, cv::Mat& tvec, cv::Mat& rvec,
                double& reproject_factorError);

  enum Action
  {
    IDLE,
    DIRECTION_ADJUSTING,
    POSITION_ADJUSTING,
    FINAL_DIRECTION_ADJUSTING,
    PREEMPTED,
    SUCCEED,
    TEACHING
  };

  struct DmInfo
  {
    cv::Point2f pt_distort_img;
    cv::Point2d pt_project_img;
    cv::Point2f pt_obj_fitline;
    cv::Point3f pt_obj;
    double img_angle;
  };

  struct ThreadInfo
  {
    int id;
    char info;
    cv::Point2i start_loc;
  };

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<visual_servo_msgs::IbvsConstrainedAction>
      as_; // NodeHandle instance must be created before this line. Otherwise
           // strange error occurs.
  ros::Subscriber img_sub_;
  ros::Publisher vel_pub_;
  ros::ServiceServer teaching_srv_;

  std::string action_name_;
  visual_servo_msgs::IbvsConstrainedFeedback feedback_;
  visual_servo_msgs::IbvsConstrainedResult result_;

  //  double task_loc_lambda_zero_;
  //  double task_loc_lambda_infinite_;
  //  double task_loc_lambda_slope_;
  double task_loc_lambda_;
//  double task_loc_mu_;
  double task_loc_error_ratio_;
  //  double task_rot_lambda_zero_;
  //  double task_rot_lambda_infinite_;
  //  double task_rot_lambda_slope_;
  double task_rot_lambda_;
//  double task_rot_mu_;
  double task_rot_error_ratio_;
//  double min_vx_;
//  double min_wz_;
  double max_vx_;
  double max_wz_;
  double max_vel_acc_;
  double max_ang_acc_;
  cv::Mat intrinsic_;
  cv::Mat distortion_;
  cv::Mat extrinsicR_;
  cv::Mat extrinsicT_;
  cv::Mat base2camera_rot_;
  cv::Mat base2camera_t_;
  vpHomogeneousMatrix cMe_; // Camera frame to mobile platform frame
  vpHomogeneousMatrix eMc_;
  vpVelocityTwistMatrix cVe_loc_;
  vpVelocityTwistMatrix cVe_rot_;
  vpMatrix eJe_loc_; // Robot jacobian
  vpMatrix eJe_rot_; // Robot jacobian
  vpPoint point_x_loc_;
  vpPoint point_xd_loc_;
  vpPoint point_x_rot_;
  vpPoint point_xd_rot_;
  vpFeaturePoint s_x_loc_;
  vpFeaturePointPolar s_x_rot_;
  vpFeaturePoint s_xd_loc_;
  vpFeaturePointPolar s_xd_rot_;
  vpServo* p_task_loc_;
  vpServo* p_task_init_rot_;
  vpServo* p_task_final_rot_;

  cv::Point3f targetPointInWorld_;
  cv::Mat targetPointInWorld_mat_;
  cv::Mat targetDirectionPointInWorld_mat_;
  cv::Point3f targetPointInBaseDesired_;
  cv::Mat targetPointInBaseDesired_mat_;

  ImageProjector* p_ip_;
  DataMatrixDetector::DetectParams dp_;
  DataMatrixDetector::DetectParams dp_part_;
  DataMatrixDetector* p_dmd_;
  DataMatrixDetector* p_dmd_part_[DM_MAX_THREADS];
  double resolution_width_inv_; //proj_pixel / obj_size
  double resolution_height_inv_;
  int part_size_;
  int part_size_2_;
  int part_size_4_;

  Action action_;
  uint8_t status_;
  double vx_;
  double wz_;
  double error_x_;
  double error_y_;
  double error_theta_;
  double error_value_;
  double targetToBase_distance_; // mm
  double targetToBase_angle_; // rad
  double base2world_x_;
  double base2world_y_;
  double base2world_theta_;

  bool save_initial_map_;
  bool log_initial_pose_;
  std::ofstream initial_log_;
  std::mutex mtx_;
  ros::Time time_start_;
  int cnt_;

#ifdef IBVS_CONSTRAINED_SHOW_IMAGE
  vpPlot graph_;
  int cnt_;
#endif
};

#endif
