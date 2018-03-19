#include "apriltag_fast.h"
#include "apriltag_opencv.h"
#include "apriltag_family.h"

#include <fstream>
#include <ros/param.h>

using namespace std;
using namespace cv;

//#define GRID_SIZE 29.76
//#define GRID_SIZE 30.5

apriltag_fast::apriltag_fast(Mat intrinsic, Mat distortion,Mat extrinsicR,Mat extrinsicT,int width, int height, int projWidth, int projHeight, double  objWidth,
                             double objHeight):intrinsic_(intrinsic),distortion_(distortion),extrinsicR_(extrinsicR),extrinsicT_(extrinsicT),
                             width_(width),height_(height),projWidth_(projWidth),projHeight_(projHeight),objWidth_(objWidth),objHeight_(objHeight)
{
	p_ip_ = new ImageProjector(intrinsic_, distortion_, extrinsicR_, extrinsicT_,
                             width, height, projWidth, projHeight, objWidth,
                             objHeight);
	isFirstFrame = true;
}

apriltag_fast::~apriltag_fast()
{
  delete p_ip_; 
}

bool apriltag_fast::tagToPose(const Mat& srcImg, Mat& tvec, Mat& rvec,
                               double& reprojectError)
{
  ROS_INFO("Detecting!");

  Mat projImg;
  if(isFirstFrame)
	p_ip_->projectImage(srcImg, projImg);
  
//  imshow("projImg", projImg);
//  waitKey(2);
  const char *famname = "tag36h11"; //Tag family to use
  apriltag_family_t *tf = apriltag_family_create(famname);

  if (!tf) {
    printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
    exit(-1);
  }

  tf->black_border = 1;       // Set tag family border size

  apriltag_detector_t *td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);

  bool useContours = true;    // speed up
  if (useContours) apriltag_detector_enable_quad_contours(td, 1);

  td->quad_decimate = 1.0;    // Decimate input image by this factor
  td->quad_sigma = 0.0;       // Apply low-pass blur to input
  td->nthreads = 4;           // Use this many CPU threads
  td->debug = 0;          // Enable debugging output (slow)
  td->refine_edges = 1;    // Spend more time trying to align edges of tags
  td->refine_decode = 0;  // one way to increase the number of detected tags
  td->refine_pose = 0;    // another way to increases the number of detected tags

  int quiet = 0;          // Reduce output
  int nogui = 1;           // Suppress GUI output from OpenCV
  int benchmark = 1;      // Benchmark mode
  int maxiters = 1;           // Repeat processing on input set this many times

  if (benchmark) nogui = 1;

  int total_detections = 0;
  uint64_t total_time = 0;

  const int hamm_hist_max = 10;

  map<int, DmInfo> dmInfo;

  for (int iter = 0; iter < maxiters; iter++) {

    if (maxiters > 1 && !benchmark) {
      printf("iter %d / %d\n", iter + 1, maxiters);
    }

    int hamm_hist[hamm_hist_max];
    memset(hamm_hist, 0, sizeof(hamm_hist));

    Mat8uc1 gray;

    if (projImg.channels() == 3) {
      cv::cvtColor(projImg, gray, cv::COLOR_RGB2GRAY);
    } else {
      projImg.copyTo(gray);
    }

    image_u8_t* im8 = cv2im8_copy(gray);

    if (gray.empty()) {
      fprintf(stderr, "error loading image!\n");
      continue;
    }

//    ros::Time start, end;
//    start = ros::Time::now();

    zarray_t *detections = apriltag_detector_detect(td, im8);

//    end = ros::Time::now();

    total_detections += zarray_size(detections);

    if(!total_detections){
      ROS_INFO_THROTTLE(1, "No tags, detection failed!");
      return false;
    }

//    ROS_INFO("It tooks %.4lf ms to detect %d tags!",
//             (end - start).toSec()*1000, zarray_size(detections));

    cv::Mat display;

    if (!nogui) {
      display = detectionsImage(detections, projImg.size(), projImg.type());
    }

    printf("id_used:");

    for (int i = 0; i < zarray_size(detections); i++) {

      apriltag_detection_t *det;  // c[2], p[4][2]
      zarray_get(detections, i, &det);

      int id = det->id;
      int row = id / DM_CODE_LINE_NUM;
      int col = id % DM_CODE_LINE_NUM;

//      if(det->c[0] < 0.2*srcImg.cols || det->c[0] > 0.8*srcImg.cols ||
//         det->c[1] < 0.2*srcImg.rows || det->c[1] > 0.8*srcImg.rows){
//        continue;
//      }

      DmInfo info;
      Point2d distort_point, project_point;
      project_point =p_ip_firstFrame_ Point2d(det->p[0][0], det->p[0][1]);
//      project_point = Point2d(
//            (det->p[0][0]+det->p[1][0]+det->p[2][0]+det->p[3][0])/4,
//            (det->p[0][1]+det->p[1][1]+det->p[2][1]+det->p[3][1])/4);
      p_ip_->project2distortPoint(project_point, &distort_point);
      info.pt_distort_img = distort_point;
      info.pt_project_img = project_point;
      info.pt_obj = Point3f(col * DM_GRID_SIZE,
                            row * DM_GRID_SIZE, 0);
      info.pt_obj_fitline = Point2f(col * DM_GRID_SIZE,
                                    row * DM_GRID_SIZE);

      DmInfo info1;
      Point2d distort_point1, project_point1;
      project_point1 = Point2d(det->p[1][0], det->p[1][1]);
      p_ip_->project2distortPoint(project_point1, &distort_point1);
      info1.pt_distort_img = distort_point1;
      info1.pt_project_img = project_point1;
      info1.pt_obj = Point3f(col * DM_GRID_SIZE + DM_CODE_SIZE,
                            row * DM_GRID_SIZE, 0);
      info1.pt_obj_fitline = Point2f(col * DM_GRID_SIZE + DM_CODE_SIZE,
                                    row * DM_GRID_SIZE);

      DmInfo info2;
      Point2d distort_point2, project_point2;
      project_point2 = Point2d(det->p[2][0], det->p[2][1]);
      p_ip_->project2distortPoint(project_point2, &distort_point2);
      info2.pt_distort_img = distort_point2;
      info2.pt_project_img = project_point2;
      info2.pt_obj = Point3f(col * DM_GRID_SIZE + DM_CODE_SIZE,
                            row * DM_GRID_SIZE + DM_CODE_SIZE, 0);
      info2.pt_obj_fitline = Point2f(col * DM_GRID_SIZE + DM_CODE_SIZE,
                                    row * DM_GRID_SIZE + DM_CODE_SIZE);

      DmInfo info3;
      Point2d distort_point3, project_point3;
      project_point3 = Point2d(det->p[3][0], det->p[3][1]);
      p_ip_->project2distortPoint(project_point3, &distort_point3);
      info3.pt_distort_img = distort_point3;
      info3.pt_project_img = project_point3;
      info3.pt_obj = Point3f(col * DM_GRID_SIZE,
                            row * DM_GRID_SIZE + DM_CODE_SIZE, 0);
      info3.pt_obj_fitline = Point2f(col * DM_GRID_SIZE,
                                    row * DM_GRID_SIZE + DM_CODE_SIZE);

//      DmInfo info4;
//      Point2d distort_point4, project_point4;
//      project_point4 = Point2d(det->c[0], det->c[1]);
//      p_ip_->project2distortPoint(project_point4, &distort_point4);
//      info4.pt_distort_img = distort_point4;
//      info4.pt_project_img = project_point4;
//      info4.pt_obj = Point3f(col * DM_GRID_SIZE + DM_CODE_SIZE / 2,
//                            row * DM_GRID_SIZE + DM_CODE_SIZE / 2, 0);
//      info4.pt_obj_fitline = Point2f(col * DM_GRID_SIZE + DM_CODE_SIZE / 2,
//                                    row * DM_GRID_SIZE + DM_CODE_SIZE / 2);

      dmInfo.insert(make_pair(5 * id, info));
      dmInfo.insert(make_pair(5 * id + 1, info1));
      dmInfo.insert(make_pair(5 * id + 2, info2));
      dmInfo.insert(make_pair(5 * id + 3, info3));
//      dmInfo.insert(make_pair(5 * id + 4, info4));

      if (benchmark) {
        printf(" %d", det->id);
      } else if (!quiet) {
        printf("detection %2d: id (%2dx%2d)-%-2d, hamming %d,"
               "goodness %5.3f, margin %6.3f, center:(%3.0f, %3.0f),\n"
               "p[0]: (%3.0f, %3.0f), p[1]: (%3.0f, %3.0f),"
               "p[2]: (%3.0f, %3.0f), p[3]: (%3.0f, %3.0f)\n",
               i, det->family->d*det->family->d, det->family->h,
               det->id, det->hamming, det->goodness, det->decision_margin, det->c[0], det->c[1],
               det->p[0][0], det->p[0][1], det->p[1][0], det->p[1][1],
               det->p[2][0], det->p[2][1], det->p[3][0], det->p[3][1]);
      }

      hamm_hist[det->hamming]++;

    }

    apriltag_detections_destroy(detections);

    if (!benchmark) {

      if (!quiet) {
        timeprofile_display(td->tp);
        printf("nedges: %d, nsegments: %d, nquads: %d\n",
               td->nedges, td->nsegments, td->nquads);
      }

      if (!quiet)
        printf("Hamming histogram: ");

      for (int i = 0; i < hamm_hist_max; i++)
        printf("%5d", hamm_hist[i]);

      if (quiet) {
        printf("%12.3f", timeprofile_total_utime(td->tp) / 1.0E3);
      }

    }

    printf("\n");

    if (!nogui) {
      display = 0.5*display + 0.5*projImg;
      cv::imshow("foo", projImg);
      cv::waitKey(0);
      cv::imshow("foo", display);
      cv::waitKey(0);
    }

    image_u8_destroy(im8);
    total_time += timeprofile_total_utime(td->tp);

  }

  if (benchmark) {
    int nin = maxiters;
    fprintf(stderr, "%d detections over %d images in %.3f ms (%.3f ms per frame)\n",
            total_detections, nin,
            (total_time*1e-3), (total_time*1e-3)/nin);
  }

  // don't deallocate contents of inputs; those are the argv
  apriltag_detector_destroy(td);

  apriltag_family_destroy(tf);

  int code_detected_num = dmInfo.size();
  cout << "point_detected_num: " << code_detected_num << endl;
  vector<Point2f> pt_img(code_detected_num), pt_obj_fitline(code_detected_num);
  vector<Point3f> pt_obj(code_detected_num);
  int cnt = 0;
  for (map<int, DmInfo>::iterator info_iter = dmInfo.begin();
       info_iter != dmInfo.end(); info_iter++)
  {
    pt_img[cnt] = info_iter->second.pt_distort_img;
    pt_obj_fitline[cnt] = info_iter->second.pt_obj_fitline;
    pt_obj[cnt] = info_iter->second.pt_obj;

    // printf("%d: (%.2f, %.2f), (%.2f, %.2f, %.2f)\n", info_iter->first,
    // pt_img[cnt].x, pt_img[cnt].y, pt_obj[cnt].x, pt_obj[cnt].y,
    // pt_obj[cnt].z);

    cnt++;
  }

  // check collineation
  bool is_collineation = true;
  float vx, vy, x0, y0, A, B, C, thres_dis, cur_dis;
  Mat line;
  fitLine(pt_obj_fitline, line, CV_DIST_L2, 0, 0.01, 0.01);
  vx = line.at<float>(0, 0);
  vy = line.at<float>(1, 0);
  x0 = line.at<float>(2, 0);
  y0 = line.at<float>(3, 0);

  A = vy;
  B = -vx;
  C = vx * y0 - vy * x0;

  thres_dis = sqrt(A * A + B * B);
  for (int i = 0; i < pt_obj_fitline.size(); i++)
  {
    cur_dis = abs(A * pt_obj_fitline.at(i).x + B * pt_obj_fitline.at(i).y + C);
    if (cur_dis > thres_dis)
    {
      is_collineation = false;
      break;
    }
  }

  if (is_collineation)
  {
    ROS_INFO_THROTTLE(1, "Loc failed:Collineation!");
    return false;
  }
  //

  // world in camera
  solvePnP(pt_obj, pt_img, intrinsic_, distortion_, rvec, tvec);

  // check reprojectError
  Mat colorImg;
  cvtColor(srcImg, colorImg, CV_GRAY2BGR);

  reprojectError = 0;
  vector<Point2f> reprojectPixeles;
  projectPoints(pt_obj, rvec, tvec, intrinsic_, distortion_, reprojectPixeles);
  for (int i = 0; i < reprojectPixeles.size(); i++)
  {
    cv::circle(colorImg,  cv::Point(reprojectPixeles[i].x, reprojectPixeles[i].y ),
               3, cv::Scalar(0, 255, 0), 2, 8, 0); // gt
    cv::circle(colorImg,  cv::Point(pt_img[i].x, pt_img[i].y),
               3, cv::Scalar(0, 0, 255), 2, 8, 0); // detected
    float dx = reprojectPixeles[i].x - pt_img[i].x;
    float dy = reprojectPixeles[i].y - pt_img[i].y;
    reprojectError += sqrt(dx * dx + dy * dy);
  }
  reprojectError /= reprojectPixeles.size();

  ROS_INFO("ReprojectError: %.2lf", reprojectError);
  putText(colorImg, "reprojectError: " + std::to_string(reprojectError),
          Point(30, 30), 1, 1, Scalar(0, 255, 0), 1);

//  imshow("reproject debug", colorImg);
//  waitKey(2);

  if (reprojectError > 3.)
  {
    ROS_INFO_THROTTLE(1, "Loc failed:ReprojectError %.2lf!", reprojectError);
    return false;
  }
	if(isFirstFrame)
	{
		delete p_ip_;
		Mat tmp_R;
		Rodrigues(rvec,tmp_R);
		extrinsicR_ = tmp_R; extrinsicT_ = tvec;
		p_ip_ = new ImageProjector(intrinsic_, distortion_, extrinsicR_, extrinsicT_,
                             width_, height_, projWidth_, projHeight_, objWidth_,
                             objHeight_);
		isFirstFrame = false;
	}
  //  Mat r_mat;
  //  Rodrigues(rvec, r_mat);
  //  cout<<"R:\n"<<r_mat;
  //  cout<<"t:\n"<<tvec;

  return true;
}
