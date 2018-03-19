#include "calibratebasecamera.h"
#include "apriltag_opencv.h"
#include "apriltag_family.h"
#define SHOW_MESSAGE
calibrateBaseCamera::calibrateBaseCamera(ros::NodeHandle &n,const std::string &path)
{
	std::string tmp = path;
	loadParams(tmp);
	paramSet();
	p_ip_ = new ImageProjector(intrinsic_, distortion_, extrinsicR_, extrinsicT_,
                             width_, height_, projWidth_, projHeight_, objWidth_,
                             objHeight_);
	isFirstFrame = true;
// 	aptag_ = new apriltag_fast(intrinsic,distortion,extrinsicR,extrinsicT,width,height,projWidth,projHeight,objWidth,objHeight);
// 	test();
// 	sub_img = n.subscribe("/jzhw/jzcamera", 20, &calibrateBaseCamera::img_callback,this);
	sub_img = n.subscribe(camera_topic, 3, &calibrateBaseCamera::img_callback,this);
	sub_odom = n.subscribe("/ros_diff_controller/odom",1,&calibrateBaseCamera::odom_callback,this);
	pMv = new baseMovement(n);
}

calibrateBaseCamera::~calibrateBaseCamera()
{
	delete pMv;
	pMv = NULL;
}

void calibrateBaseCamera::loadParams(string filename)
{
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened()) {
        cout<<"loadParams falied. 'camera.yml' does not exist\n"<<endl;
        return;
    }

    fs["fx"] >> mfx;
	fs["fy"] >> mfy;
	fs["u0"] >> mu0;
	fs["v0"] >> mv0;
	fs["k1"] >> mk1;
	fs["k2"] >> mk2;

	fs["camera_topic"]>>camera_topic;
	
	fs["ImgWidth"] >>width_;
	fs["ImgHeight"] >>height_;
	fs["projWidth"] >>projWidth_ ;
	fs["projHeight"] >>projHeight_;
	fs["objWidth"] >>objWidth_;
	fs["objHeight"] >>objHeight_;
	fs["camera_toward"] >>camera_toward_;
}

void calibrateBaseCamera::paramSet()
{
	
// 	base2camera_rot_calib = (Mat_<double>(3, 3) << 0.01523793929760663, -0.9997761453086123, -0.01467870893550333,
// 	0.9998649314191976, 0.01514559198291459, 0.006381962268719785,
// 	-0.006158256575141575, -0.01477397425676979, 0.9998718948253259);
// 	base2camera_t_calib = (Mat_<double>(3, 1) << -2.351849067683608,
// 	-1.664574254866626,
// 	78.83827322899126);

	//内参
// 	intrinsic =
// 	(Mat_<double>(3, 3) << 442.906545801087, 0, 360.065765342203,
// 	0, 441.057140585713, 226.568184093991,
// 	0, 0, 1);
	intrinsic_ =
	(Mat_<double>(3, 3) << mfx, 0, mu0,
	0, mfy, mv0,
	0, 0, 1);
	//畸变参数
// 	distortion =
// 		(Mat_<double>(4, 1) << -0.354603627826886,	0.117671119240506, 0, 0);
	distortion_ =
		(Mat_<double>(4, 1) << mk1,mk2, 0, 0);
	//外参旋转矩阵
// 	extrinsicR = (Mat_<double>(3, 3) << -0.9994161062659113, 0.03271154280822035, 0.009869219999107975,
// 	-0.03264944972317968, -0.9994464471597955, 0.006388481230757017,
// 	0.01007273394160629, 0.006062526434428159, 0.9999308905140265);
// 	//外参平移向量
// 	extrinsicT = (Mat_<double>(3, 1) << 52.27045717100402,
// 	40.56957808786657,
// 	79.11038615682448);
		
	extrinsicR_ = (Mat_<double>(3, 3) << 0.002261131713937293, -0.9999761785877515, 0.006521467657630404,
0.9998876674348485,0.002357463684322738, 0.01480185382561659,
-0.01481687534772592, 0.006487266143329704, 0.9998691792344214);
	//外参平移向量
	extrinsicT_ = (Mat_<double>(3, 1) << 40.55757680365112,-43.84534412904078,85.2556156403148);
// 	extrinsicR_ = (Mat_<double>(3, 3) << -0.9998513788237653,-0.01653749106770473,0.004871514487391196,
// 0.0166936448376695,-0.9992831259722685,0.03397876350200197,
// 0.004306098727273459,0.03405503687087737,0.9994106823410859);
// 	//外参平移向量
// 	extrinsicT_ = (Mat_<double>(3, 1) << 68.01678559744146,55.58865875767152,78.76610338826991);
	//图像宽度
// 	width_ = 752;
// 	//图像高度
// 	height_ = 480;
// 	projWidth_ = 520;
// 	projHeight_ = 332;
// 	objWidth_ = 166.02;
// 	objHeight_ = 106.;
	
	dp.imgWidth = projWidth_;
	dp.imgHeight = projHeight_;
	//dp.imgWidth = 588;// PROJ_IMG_SIZE;			//图像宽度
	//dp.imgHeight = PROJ_IMG_SIZE;// PROJ_IMG_SIZE;			//图像高度
	dp.edgeMin = 17;			//二维码边缘长度极小值（ICARRIER下方42；IAGV 17；ICARRIER上方32）
	dp.edgeMax = 200;			//二维码边缘长度极大值（IAGV 200；ICARRIER 52）
	dp.edgeThresh = 77;			//种子点边缘梯度阈值（77）
	dp.edgeStopThresh = 50;		//边缘终点梯度阈值（50）
	dp.solidContiAngRng = 15;	//连续运动时，实线边的霍夫变换的角度正负搜索范围（15）
	dp.dashedContiAngRng = 2;	//连续运动时，虚线边的霍夫变换的角度正负搜索范围（2）
	dp.type = DataMatrixDetector::DmtxSymbolSquareAuto;	//二维码类型（IAGV DmtxSymbolSquareAuto；ICARRIER DmtxSymbol10x10）
	dp.isCoarse2Fine = 0;		//检测输入是否为粗定位后的二维码图像（0 不是，非0 是）
	
	first_Img_flag = true;
	startCalibFlag = true;
	
	
	resolution_width_inv_ = projWidth_ / objWidth_;
	resolution_height_inv_ = projHeight_ / objHeight_;
	
	//new
	Point2d edgeSizeWorld, edgeSizeProject;
	edgeSizeWorld = Point2d(DM_CODE_SIZE, DM_CODE_SIZE);
	edgeSizeProject.x = edgeSizeWorld.x * resolution_width_inv_;
	edgeSizeProject.y = edgeSizeWorld.y * resolution_height_inv_;
	part_size_ = (int)(sqrt(edgeSizeProject.x * edgeSizeProject.x +
						edgeSizeProject.y * edgeSizeProject.y) *
						1.2 +
					0.5);
	part_size_ = (part_size_ + 4) / 4 * 4;
	part_size_2_ = part_size_ / 2;
	part_size_4_ = part_size_ / 4;
	dp_part_.imgWidth = part_size_;  //图像宽度
	dp_part_.imgHeight = part_size_; //图像高度
	dp_part_.edgeMin =
		17;                 //二维码边缘长度极小值（ICARRIER下方42；IAGV 17；ICARRIER上方32）
	dp_part_.edgeMax = 200; //二维码边缘长度极大值（IAGV 200；ICARRIER 52）
	dp_part_.edgeThresh = 77;     //种子点边缘梯度阈值（77）
	dp_part_.edgeStopThresh = 50; //边缘终点梯度阈值（50）
	dp_part_.solidContiAngRng =
		15; //连续运动时，实线边的霍夫变换的角度正负搜索范围（15）
	dp_part_.dashedContiAngRng =
		2; //连续运动时，虚线边的霍夫变换的角度正负搜索范围（2）
	dp_part_.type = DataMatrixDetector::DmtxSymbolSquareAuto; //二维码类型（IAGV
	// DmtxSymbolSquareAuto；ICARRIER
	// DmtxSymbol10x10）
	dp_part_.isCoarse2Fine =1; 
	

}

bool calibrateBaseCamera::checkCollinearity(const vector< Point2f >& pt_obj_fitline)
{
	bool valid = false;
	float vx, vy, x0, y0, A, B, C, thres_dis, cur_dis;
	Mat line;
	fitLine(pt_obj_fitline, line, CV_DIST_L2, 0, 0.01, 0.01);
	vx = line.at<float>(0, 0);
	vy = line.at<float>(1, 0);
	x0 = line.at<float>(2, 0);
	y0 = line.at<float>(3, 0);

	A = vy;
	B = -vx;
	C = vx*y0 - vy*x0;

	thres_dis = sqrt(A*A + B*B);
	for (unsigned int i = 0; i < pt_obj_fitline.size(); i++)
	{
		cur_dis = abs(A*pt_obj_fitline.at(i).x + B*pt_obj_fitline.at(i).y + C);
		if (cur_dis > thres_dis)
		{
			valid = true;
			break;
		}
	}
	return valid;
}

bool calibrateBaseCamera::checkReprojectError(const vector< Point2f >& pt_img, const vector< Point3f >& pt_obj, Mat& rvec, Mat& tvec)
{
	double reproject_factorError = 0;
	solvePnP(pt_obj, pt_img, intrinsic_, distortion_, rvec, tvec);
	vector<Point2f> reproject_factorPixeles;
	projectPoints(pt_obj, rvec, tvec, intrinsic_, distortion_, reproject_factorPixeles);

	for (unsigned int i = 0; i < reproject_factorPixeles.size(); i++)
	{
		float dx = reproject_factorPixeles[i].x - pt_img[i].x;
		float dy = reproject_factorPixeles[i].y - pt_img[i].y;
		reproject_factorError += sqrt(dx*dx + dy*dy);
	}
	reproject_factorError /= reproject_factorPixeles.size();
	cout<< "reproject_factorError: " << reproject_factorError<<endl;
	if(reproject_factorError < REPROJECT_ERROR_THRESHOLD)
		return true;
	else
		return false;
}

void calibrateBaseCamera::cvFitPlane(const CvMat* points, float* plane)
{
	int nrows = points->rows;
	int ncols = points->cols;
	int type = points->type;
	CvMat* centroid = cvCreateMat(1, ncols, type);
	cvSet(centroid, cvScalar(0));
	for (int c = 0; c<ncols; c++){
		for (int r = 0; r < nrows; r++)
		{
			centroid->data.fl[c] += points->data.fl[ncols*r + c];
		}
		centroid->data.fl[c] /= nrows;
	}
	// Subtract geometric centroid from each point.  
	CvMat* points2 = cvCreateMat(nrows, ncols, type);
	for (int r = 0; r<nrows; r++)
	for (int c = 0; c<ncols; c++)
		points2->data.fl[ncols*r + c] = points->data.fl[ncols*r + c] - centroid->data.fl[c];
	// Evaluate SVD of covariance matrix.  
	CvMat* A = cvCreateMat(ncols, ncols, type);
	CvMat* W = cvCreateMat(ncols, ncols, type);
	CvMat* V = cvCreateMat(ncols, ncols, type);
	cvGEMM(points2, points, 1, NULL, 0, A, CV_GEMM_A_T);
	cvSVD(A, W, NULL, V, CV_SVD_V_T);
	// Assign plane coefficients by singular vector corresponding to smallest singular value.  
	plane[ncols] = 0;
	for (int c = 0; c<ncols; c++){
		plane[c] = V->data.fl[ncols*(ncols - 1) + c];
		plane[ncols] += plane[c] * centroid->data.fl[c];
	}
	// Release allocated resources.  
	cvReleaseMat(&centroid);
	cvReleaseMat(&points2);
	cvReleaseMat(&A);
	cvReleaseMat(&W);
	cvReleaseMat(&V);
}

void calibrateBaseCamera::showFinalResult()
{
	Mat srcImg_color;
	cvtColor(showImg, srcImg_color, CV_GRAY2BGR);
	vector<Point3f> pointInWorld_vec;
	for(unsigned int moveType = 0; moveType<2; moveType++)
	{
		for(unsigned int i = 0;i<worldInCamera_rvecs[moveType].size();i++)
		{
			Mat rvec_, tvec_, rot_mat_, rvec ,tvec;
			rvec = worldInCamera_rvecs[moveType][i];
			tvec = worldInCamera_tvecs[moveType][i];
			
			Rodrigues(rvec,rot_mat_);
			rot_mat_ = rot_mat_.t();
			Rodrigues(rot_mat_, rvec_);
			tvec_ = -rot_mat_*tvec;
			Mat base2world_rot = rot_mat_*base2camera_rot_result;
			Mat base2world_t = rot_mat_*base2camera_t_result + tvec_;
			Mat pointInbase = Mat::zeros(3, 1, CV_64FC1);
			pointInbase.at<double>(2) = -base2world_t.at<double>(2) / base2world_rot.at<double>(2, 2);
			Mat pointInWorld = base2world_rot*pointInbase + base2world_t;
			
			pointInWorld_vec.push_back(Point3f(pointInWorld.at<double>(0), pointInWorld.at<double>(1), pointInWorld.at<double>(2)));
			double x_dir = 10;
			pointInbase.at<double>(0) = x_dir;
			pointInbase.at<double>(1) = 0;
			pointInbase.at<double>(2) = -(base2world_t.at<double>(2) + x_dir*base2world_rot.at<double>(2, 0)) / base2world_rot.at<double>(2, 2);
			pointInWorld = base2world_rot*pointInbase + base2world_t;
			pointInWorld_vec.push_back(Point3f(pointInWorld.at<double>(0), pointInWorld.at<double>(1), pointInWorld.at<double>(2)));
		}
	}
	
	vector<Point2f> pointInPixel;
	projectPoints(pointInWorld_vec, showImg_rvec, showImg_tvec, intrinsic_, distortion_, pointInPixel);
	
	cout << "total circle points' number is " << worldInCamera_rvecs[0].size()<<endl;
	cout << "total line points' number is " << worldInCamera_rvecs[1].size()<<endl;

	for(unsigned int i = 0;i < worldInCamera_rvecs[0].size();i++)
		circle(srcImg_color,Point(pointInPixel[2*i].x, pointInPixel[2*i].y),2, cv::Scalar(0, 0, 255));
	for(unsigned int i = (pointInWorld_vec.size()/2);i > worldInCamera_rvecs[0].size();i--)
		arrowedLine(srcImg_color, Point(pointInPixel[2*i].x, pointInPixel[2*i].y), Point(pointInPixel[2*i+1].x, pointInPixel[2*i+1].y), Scalar(0, 0, 255), 1);
	//imshow("Final Result", srcImg_color);
	imwrite("FinalResult.png", srcImg_color);
	//waitKey(50);
	/*pointInWorld_vec[0].x = 295.342;
	pointInWorld_vec[0].y = 232.474;*/

	//draw direction
}

bool calibrateBaseCamera::ptsDetector(ImageProjector &ip, DataMatrixDetector &dmd,const Mat &srcImg,vector<Point2f> &pt_obj_fitline, vector<Point2f> &pt_img, 
				 vector<Point3f> &pt_obj)
{
	vector<string> msg;
	Mat projImg;
	ip.projectImage(srcImg, projImg);
	if(!dmd.detect(projImg, true))
	{
		ROS_WARN("The first thread is busy, so we abort!");
		return false;
	}
	
	DataMatrixDetector::notify_all();

	map<int, DmInfo>  dmInfo;
	vector<DataMatrixDetector::DataMatrixInfo> dmInfoTmp;
	while (dmd.getDetectInfo(dmInfoTmp) == DataMatrixDetector::DETECTING)
	{
		if (dmInfoTmp.size() > 0)
		{
			for (uint32_t j = 0; j < dmInfoTmp.size(); j++)
			{
				// ROS_INFO("%s found!", dmInfoTmp[i].message);

				if (dmInfoTmp[j].message[1] != '\0')
				continue;

				int id;
				id = dmInfoTmp[j].message[0] - '0';
				if (id < 0 || id > 9)
				id = dmInfoTmp[j].message[0] - 'A' + 10;
				if (id > -1 && id < DM_CODE_NUM)
				{
					dmd.abort();

					int row = id / DM_CODE_LINE_NUM;
					int col = id % DM_CODE_LINE_NUM;

					DmInfo info;
					Point2d distort_point, project_point;
					project_point = Point2d(dmInfoTmp[j].x, dmInfoTmp[j].y);
					ip.project2distortPoint(project_point, &distort_point);
					info.pt_distort_img = distort_point;
					info.pt_project_img = project_point;
					info.pt_obj = Point3f(col * DM_GRID_SIZE, row * DM_GRID_SIZE, 0);
					info.pt_obj_fitline = Point2f(col * DM_GRID_SIZE, row * DM_GRID_SIZE);
					info.img_angle = dmInfoTmp[j].angle;
					dmInfo.insert(make_pair(id, info));

					break;
				}
			}
			dmInfoTmp.clear();

			if (dmInfo.size() > 0)
				break;
		}
	}
	if (dmInfo.size() == 0)
	{
		ROS_INFO_THROTTLE(1, "No Valid DM Codes Exist!");
		return false;
	}

	int seed_id = dmInfo.begin()->first;
	DmInfo seed_info = dmInfo.begin()->second;

	double res_angle = seed_info.img_angle;
	Point3d deltaWorldX, deltaWorldY, centerBiasWorld;
	deltaWorldX.x = DM_GRID_SIZE * cos(res_angle);
	deltaWorldX.y = DM_GRID_SIZE * sin(res_angle);
	deltaWorldX.z = 0;

	deltaWorldY.x = -DM_GRID_SIZE * sin(res_angle);
	deltaWorldY.y = DM_GRID_SIZE * cos(res_angle);
	deltaWorldY.z = 0;

	centerBiasWorld.x = DM_CODE_SIZE / 2 * (cos(res_angle) + sin(res_angle));
	centerBiasWorld.y = DM_CODE_SIZE / 2 * (sin(res_angle) - cos(res_angle));
	centerBiasWorld.z = 0;
// 	cout<< "centerBiasWorld:  "<<centerBiasWorld<<endl;

	Point2d deltaProjectX, deltaProjectY, centerBiasProject;
	deltaProjectX.x = deltaWorldX.x * resolution_width_inv_;
	deltaProjectX.y = deltaWorldX.y * resolution_height_inv_;
	deltaProjectY.x = deltaWorldY.x * resolution_width_inv_;
	deltaProjectY.y = deltaWorldY.y * resolution_height_inv_;
	centerBiasProject.x = centerBiasWorld.x * resolution_width_inv_;
	centerBiasProject.y = centerBiasWorld.y * resolution_height_inv_;

	
	int seed_row = seed_id / DM_CODE_LINE_NUM;
	int seed_col = seed_id % DM_CODE_LINE_NUM;

	
	Point2d loc0;
	loc0 = seed_info.pt_project_img - seed_col * deltaProjectX -
			seed_row * deltaProjectY;
	loc0 = loc0 + centerBiasProject;

	//cout<< "loc0:  "<<loc0<<endl;

	vector<char> preIdx;
	vector<Point2i> partStartLoc;
	vector<int> distVec;
	int projWidth = dp.imgWidth;
	int projHeight = dp.imgHeight;
	int projCenterX = projWidth / 2;
	int projCenterY = projHeight / 2;
	// for (uint32_t i = 0; i < DM_CODE_NUM; i++)
	for (int i = 0; i < DM_CODE_NUM; i++)
	{
		if (i == seed_id)
			continue;

		int distX, distY;
		Point2d tmpLoc;
		Point2i startLoc;
		int tmpRow = i / DM_CODE_LINE_NUM;
		int tmpCol = i % DM_CODE_LINE_NUM;
		tmpLoc = loc0 + tmpCol * deltaProjectX + tmpRow * deltaProjectY;

		//    if (tmpLoc.x < part_size_4_ || tmpLoc.x > projWidth - part_size_4_ ||
		//        tmpLoc.y < part_size_4_ || tmpLoc.y > projHeight - part_size_4_)
		//      continue;

		if (tmpLoc.x < part_size_2_ || tmpLoc.x > projWidth - part_size_2_ ||
			tmpLoc.y < part_size_2_ || tmpLoc.y > projHeight - part_size_2_)
			continue;

		if (tmpLoc.x < part_size_2_)
			startLoc.x = 0;
		else if (tmpLoc.x > projWidth - part_size_2_)
			startLoc.x = projWidth - part_size_;
		else
			startLoc.x = (int)(tmpLoc.x - part_size_2_ + 0.5);

		if (tmpLoc.y < part_size_2_)
			startLoc.y = 0;
		else if (tmpLoc.y > projHeight - part_size_2_)
			startLoc.y = projHeight - part_size_;
		else
			startLoc.y = (int)(tmpLoc.y - part_size_2_ + 0.5);

		partStartLoc.push_back(startLoc);

		distX = startLoc.x + part_size_2_ - projCenterX;
		distY = startLoc.y + part_size_2_ - projCenterY;
		distVec.push_back(distX * distX + distY * distY);

		if (i < 10)
			preIdx.push_back('0' + i);
		else
			preIdx.push_back('A' + i - 10);
	}
	DM_MAX_THREADS = preIdx.size();
	if(DM_MAX_THREADS < DM_CODE_NEEDED)
	{
		cout << "DM_MAX_THREADS size too small"<<endl;
		return false;
	}
	DataMatrixDetector* p_dmd_part_[DM_MAX_THREADS];
	for (int i = 0; i < DM_MAX_THREADS; i++)
		p_dmd_part_[i] = new DataMatrixDetector(dp_part_);

	vector<int> distVec_sort;
	int distThresh;
	distVec_sort = distVec;
	sort(distVec_sort.begin(), distVec_sort.end());
	distThresh = distVec_sort[DM_MAX_THREADS - 1];

	vector<ThreadInfo> threadsInfo;
	for (int i = 0; i < (int)preIdx.size(); i++)
	{
		if (distVec[i] > distThresh)
			continue;

		Mat part_projImg = projImg(Rect(partStartLoc[i].x, partStartLoc[i].y, part_size_, part_size_));

		int next_thread_id = threadsInfo.size();
		
		if (p_dmd_part_[next_thread_id]->detect(part_projImg, false))
		{
			ThreadInfo tmp;
			tmp.id = next_thread_id;
			tmp.info = preIdx[i];
			tmp.start_loc = partStartLoc[i];
			threadsInfo.push_back(tmp);

			if (next_thread_id == DM_MAX_THREADS - 1)
			break;
		}
		else
		{
			p_dmd_part_[next_thread_id]->abort();
			ROS_ERROR("This thread should be free!");
		}

	}
	
	DataMatrixDetector::notify_all();

	unsigned long time_start = clock();
	bool isTimeOut = false;
	if(threadsInfo.size() < DM_CODE_NEEDED)
	{
		cout<< "threadsInfo size too small"<<endl;
		return false;
	}
	while (threadsInfo.size() > 0)
	{
		for (int i = 0; i < (int)threadsInfo.size(); i++)
		{
			int thread_idx = threadsInfo[i].id;
			bool found = false;
			vector<DataMatrixDetector::DataMatrixInfo> dmInfoPartTmp;
			DataMatrixDetector::ErrorCode err =
				p_dmd_part_[thread_idx]->getDetectInfo(dmInfoPartTmp);

			if (dmInfoPartTmp.size() > 0)
			{
				for (int j = 0; j < (int)dmInfoPartTmp.size(); j++)
				{
					char ch = threadsInfo[i].info;
					if (dmInfoPartTmp[j].message[0] == ch &&
						dmInfoPartTmp[j].message[1] == '\0')
					{
						dmInfoPartTmp[j].x += threadsInfo[i].start_loc.x;
						dmInfoPartTmp[j].y += threadsInfo[i].start_loc.y;

						int id;
						id = ch - '0';
						if (id < 0 || id > 9)
							id = ch - 'A' + 10;
						int row = id / DM_CODE_LINE_NUM;
						int col = id % DM_CODE_LINE_NUM;

						DmInfo info;
						Point2d distort_point;
						ip.project2distortPoint(
							Point2d(dmInfoPartTmp[j].x, dmInfoPartTmp[j].y),
							&distort_point);
						info.pt_distort_img = distort_point;
						info.pt_project_img =
							Point2d(dmInfoPartTmp[j].x, dmInfoPartTmp[j].y);
						info.pt_obj = Point3d(col * DM_GRID_SIZE, row * DM_GRID_SIZE, 0);
						info.pt_obj_fitline =
							Point2f(col * DM_GRID_SIZE, row * DM_GRID_SIZE);
						info.img_angle = dmInfoPartTmp[j].angle;
						dmInfo.insert(make_pair(id, info));

						found = true;
						break;
					}
				}
			}
			
			if (err == DataMatrixDetector::DETECTED || found)
			{
				p_dmd_part_[thread_idx]->abort();
				threadsInfo.erase(threadsInfo.begin() + i);
				i--;
			}

		}		
		if ((dmInfo.size() > threadsInfo.size() -1))//
		{
			for (int i = 0; i < (int)threadsInfo.size(); i++)
			{
				p_dmd_part_[threadsInfo[i].id]->abort();
			}
			break;
		}
		unsigned long time_end = clock();
		if(dmInfo.size()>(DM_CODE_NEEDED -1)&&((time_end - time_start)>80*1000))
		{

			for (int i = 0; i < (int)threadsInfo.size(); i++)
			{
				p_dmd_part_[threadsInfo[i].id]->abort();
			}
			break;
		}
		if((time_end - time_start)>200*1000)
		{

			for (int i = 0; i < (int)threadsInfo.size(); i++)
			{
				p_dmd_part_[threadsInfo[i].id]->abort();
			}
			isTimeOut = true;
			break;
		}
	}
	
// 	if (dmInfo.size() < DM_CODE_NEEDED)
// 	{
// 		ROS_WARN("Only msg.size() %lu detected in %d threads, so we abort!",dmInfo.size(), workingThreadsNum);
// 		return false;
// 	}
	
	for (int kk = 0; kk < DM_MAX_THREADS; kk++)
	{
		delete p_dmd_part_[kk];
	}
	if(dmInfo.size() < DM_CODE_NEEDED - 2)
	{
		cout << "dmInfo size too small!"<<endl;
		return false;
	}
	if(isTimeOut)
	{
		cout << "Detect timeout! "<<endl;
		return false;
	}
	for (map<int, DmInfo>::iterator info_iter = dmInfo.begin();
		info_iter != dmInfo.end(); info_iter++)
	{
		pt_img.push_back(info_iter->second.pt_distort_img);
		pt_obj_fitline.push_back(info_iter->second.pt_obj_fitline);
		pt_obj.push_back(info_iter->second.pt_obj);
	}
	return true;
}

void calibrateBaseCamera::firstDetector(ImageProjector &ip, DataMatrixDetector &dmd, const Mat &srcImg, vector<Point2f> &pt_obj_fitline, vector<Point2f> &pt_img,vector<Point3f> &pt_obj)
{
	  //old points detector of DM
	vector<string> msg;
	Mat projImg;

	ip.projectImage(srcImg, projImg);
	while(!dmd.detect(projImg, true))
	{}
	DataMatrixDetector::notify_all();

	vector<DataMatrixDetector::DataMatrixInfo> dmInfoTmp;
	RNG rng;
	while (dmd.getDetectInfo(dmInfoTmp) == DataMatrixDetector::DETECTING)
	{
		if (dmInfoTmp.size() > 0)
		{
			for (int j = 0; j < (int)dmInfoTmp.size(); j++)
			{
				bool found = false;
				for (unsigned int k = 0; k < msg.size(); k++)
				{
					if (msg[k] == dmInfoTmp[j].message)
					{
						found = true;	;

						break;
					}
				}
				if (found)
					continue;

				if (dmInfoTmp[j].message[1] != '\0')
					continue;

				int id;
				id = dmInfoTmp[j].message[0] - '0';
				if (id < 0 || id > 9)
					id = dmInfoTmp[j].message[0] - 'A' + 10;
				if (id > -1 && id < 25)
				{
					int row = id / 5;
					int col = id % 5;

					Point2d distort_point;
					ip.project2distortPoint(Point2d(dmInfoTmp[j].x, dmInfoTmp[j].y), &distort_point);
					pt_img.push_back(distort_point);
					pt_obj.push_back(Point3d(col*GRID_SIZE, row*GRID_SIZE, 0));
					pt_obj_fitline.push_back(Point2f(col*GRID_SIZE, row*GRID_SIZE));
					msg.push_back(dmInfoTmp[j].message);
				}
			}

			dmInfoTmp.clear();
		}

	}
}

bool calibrateBaseCamera::get_RT(Mat& srcImg, Mat& rvec, Mat& tvec)
{
	ImageProjector ip(intrinsic_, distortion_, extrinsicR_, extrinsicT_, width_, height_, projWidth_, projHeight_, objWidth_, objHeight_);
	DataMatrixDetector dmd(dp);

	//vector<DataMatrixDetector::DataMatrixInfo> dmInfo;
	vector<Point2f> pt_obj_fitline;
	vector<Point2f> imgPoints;
	vector<Point3f> objPoints;
	//map<string, Point3f> pts_obj;
	//map<string, Scalar> colors;
	if(first_Img_flag)
		firstDetector(ip,dmd,srcImg, pt_obj_fitline, imgPoints,objPoints);
	else
	{
		if(!ptsDetector(ip,dmd,srcImg, pt_obj_fitline, imgPoints,objPoints))
			return false;
	}
	cout<<"The number of found DM is:   "<<pt_obj_fitline.size()<<endl;
	if(first_Img_flag)
	{
		if (imgPoints.size() < 7){
			cout << "ExParam Calibration Failed!" << endl;
			return false;
		}

	}
	
	if(pt_obj_fitline.size() == 0)
		return false;
	
	if(checkCollinearity(pt_obj_fitline))
	{
		if(!checkReprojectError(imgPoints,objPoints,rvec,tvec))
		{
			cout<<"reproject_factor error too big!"<<endl;
			return false;
		}
		else
		{
			Mat tmpR;
			Rodrigues(rvec,tmpR);
			extrinsicR_ = tmpR;
			extrinsicT_ = tvec;
		}
	}
	//save files
// 	{
// 		if(mvState == baseMovement::CIRCLE)
// 		{
// 			timeStamp_circle.push_back(tmpTimeStamp_camera);
// 			saveImgPoints_circle.push_back(imgPoints);
// 			saveWorldPoints_circle.push_back(objPoints);
// 		}
// 		if(mvState == baseMovement::LINE)
// 		{
// 			timeStamp_line.push_back(tmpTimeStamp_camera);
// 			saveImgPoints_line.push_back(imgPoints);
// 			saveWorldPoints_line.push_back(objPoints);
// 		}
// 	}
	return true;
}


/*****************************************new method*********************************************/
bool calibrateBaseCamera::tagToPose(const Mat& srcImg, Mat& tvec, Mat& rvec,
                               double& reprojectError)
{
  Mat projImg;
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

//   int quiet = 0;          // Reduce output
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
    } 
    else 
	{
      projImg.copyTo(gray);
    }
    
    if (gray.empty()) {
      fprintf(stderr, "error loading image!\n");
      continue;
    }
    
    image_u8_t* im8 = cv2im8_copy(gray);
    zarray_t *detections = apriltag_detector_detect(td, im8);
	image_u8_destroy(im8);
    total_detections += zarray_size(detections);

    if(!total_detections){
		ROS_INFO_THROTTLE(1, "No tags, detection failed!");
		apriltag_detector_destroy(td);
		apriltag_family_destroy(tf);
		return false;
    }

//    ROS_INFO("It tooks %.4lf ms to detect %d tags!",
//             (end - start).toSec()*1000, zarray_size(detections));

    cv::Mat display;

    if (!nogui) 
	{
      display = detectionsImage(detections, projImg.size(), projImg.type());
    }


    for (int i = 0; i < zarray_size(detections); i++) {

      apriltag_detection_t *det;  // c[2], p[4][2]
      zarray_get(detections, i, &det);

      int id = det->id;
      int row = id / DM_CODE_LINE_NUM;
      int col = id % DM_CODE_LINE_NUM;

     if(det->p[0][0] < 0.2*projImg.cols || det->p[0][0] > 0.8*projImg.cols ||
        det->p[0][1] < 0.2*projImg.rows || det->p[0][1] > 0.8*projImg.rows ||
		det->p[1][0] < 0.2*projImg.cols || det->p[1][0] > 0.8*projImg.cols ||
        det->p[1][1] < 0.2*projImg.rows || det->p[1][1] > 0.8*projImg.rows ||
        det->p[2][0] < 0.2*projImg.cols || det->p[2][0] > 0.8*projImg.cols ||
        det->p[2][1] < 0.2*projImg.rows || det->p[2][1] > 0.8*projImg.rows ||
        det->p[3][0] < 0.2*projImg.cols || det->p[3][0] > 0.8*projImg.cols ||
        det->p[3][1] < 0.2*projImg.rows || det->p[3][1] > 0.8*projImg.rows){
       continue;
     }

      DmInfo info;
      Point2d distort_point, project_point;
      project_point =Point2d(det->p[0][0], det->p[0][1]);
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

     DmInfo info4;
     Point2d distort_point4, project_point4;
     project_point4 = Point2d(det->c[0], det->c[1]);
     p_ip_->project2distortPoint(project_point4, &distort_point4);
     info4.pt_distort_img = distort_point4;
     info4.pt_project_img = project_point4;
     info4.pt_obj = Point3f(col * DM_GRID_SIZE + DM_CODE_SIZE / 2,
                           row * DM_GRID_SIZE + DM_CODE_SIZE / 2, 0);
     info4.pt_obj_fitline = Point2f(col * DM_GRID_SIZE + DM_CODE_SIZE / 2,
                                   row * DM_GRID_SIZE + DM_CODE_SIZE / 2);

      dmInfo.insert(make_pair(5 * id, info));
      dmInfo.insert(make_pair(5 * id + 1, info1));
      dmInfo.insert(make_pair(5 * id + 2, info2));
      dmInfo.insert(make_pair(5 * id + 3, info3));
//      dmInfo.insert(make_pair(5 * id + 4, info4));

      hamm_hist[det->hamming]++;

    }

    apriltag_detections_destroy(detections);

    total_time += timeprofile_total_utime(td->tp);

  }
  int nin = maxiters;
#ifdef SHOW_MESSAGE
	printf("id_used:");
	fprintf(stderr, "%d detections over %d images in %.3f ms (%.3f ms per frame)\n",
		total_detections, nin,
		(total_time*1e-3), (total_time*1e-3)/nin);
#endif
    
    

  // don't deallocate contents of inputs; those are the argv
  apriltag_detector_destroy(td);
  apriltag_family_destroy(tf);
  
  int code_detected_num = dmInfo.size();
	if(code_detected_num < 5)
	{
		cout << "point_detected_num: " << code_detected_num << endl;
		return false;
	}
  
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
  for (unsigned int i = 0; i < pt_obj_fitline.size(); i++)
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
  for (unsigned int i = 0; i < reprojectPixeles.size(); i++)
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

#ifdef SHOW_MESSAGE
  ROS_INFO("ReprojectError: %.2lf", reprojectError);
#endif
  putText(colorImg, "reprojectError: " + std::to_string(reprojectError),
          Point(30, 30), 1, 1, Scalar(0, 255, 0), 1);

//  imshow("reproject debug", colorImg);
//  waitKey(2);

  if (reprojectError > 3.)
  {
    ROS_INFO_THROTTLE(1, "Loc failed:ReprojectError %.2lf!", reprojectError);
    return false;
  }

  return true;
}
void calibrateBaseCamera::computeCalibratedResult()
{
	vector<Point3f> worldsInCamera_total;
	worldsInCamera_total.insert(worldsInCamera_total.end(), worldsInCamera[0].begin(), worldsInCamera[0].end());
	worldsInCamera_total.insert(worldsInCamera_total.end(), worldsInCamera[1].begin(), worldsInCamera[1].end());

	//fit plane, this plane should be parallel with the ground
	//定义用来存储需要拟合点的矩阵 
	CvMat* points_mat = cvCreateMat(worldsInCamera_total.size(), 3, CV_32FC1);
	for (unsigned int i = 0; i < worldsInCamera_total.size(); ++i)
	{
		//矩阵的值进行初始化   X的坐标值
		points_mat->data.fl[i * 3 + 0] = worldsInCamera_total[i].x;
		//  Y的坐标值
		points_mat->data.fl[i * 3 + 1] = worldsInCamera_total[i].y;
		points_mat->data.fl[i * 3 + 2] = worldsInCamera_total[i].z;
	}
	//Ax+by+cz=D  
	//定义用来储存平面参数的数组 
	float plane12[4] = { 0 };
	//调用方程
	cvFitPlane(points_mat, plane12); 

	float A = plane12[0];
	float B = plane12[1];
	float C = plane12[2];
	float D = plane12[3];
	float sum_squar = A * A + B * B + C * C;

// 	ofstream fout("worldsInCamera.txt");
// 	fout << A << ", " << B << ", " << C << ", " << D << endl;
// 	// debug info (TODO: remove debug message which is not necessary anymore)
// 	if (1)
// 	{
// 		//float sqrt_sum_squar = sqrt(sum_squar);
// 		fout << "worldsInCamera[0]:" << endl;
// 		for (unsigned int i = 0; i < worldsInCamera[0].size(); ++i)
// 			fout << worldsInCamera[0][i].x << '\t' << worldsInCamera[0][i].y << '\t' << worldsInCamera[0][i].z << ";" << endl;
// 		fout << "worldsInCamera[1]:" << endl;
// 		for (unsigned int i = 0; i < worldsInCamera[1].size(); ++i)
// 			fout << worldsInCamera[1][i].x << '\t' << worldsInCamera[1][i].y << '\t' << worldsInCamera[1][i].z << ";" << endl;
// 	}

	// creat a new coordinate system which is in the fitted plane
	double sum;
	Point3d p0(0, 0, D / C), p1(10, 10, (D - 10 * A - 10 * B) / C);
	Point3d vx = p1 - p0;
	sum = sqrt(vx.x*vx.x + vx.y*vx.y + vx.z*vx.z);
	vx = vx * (1. / sum);
	Point3d vz(A, B, C);
	sum = sqrt(vz.x*vz.x + vz.y*vz.y + vz.z*vz.z);
	if(camera_toward_ == 0)
		vz = vz * (1. / sum);
	else
		vz = -vz * (1. / sum);
	Point3d vy;
	vy = vz.cross(vx);
	sum = sqrt(vy.x*vy.x + vy.y*vy.y + vy.z*vy.z);
	vy = vy * (1. / sum);

	Mat new2camera_rot = (Mat_<double>(3, 3) <<
		vx.x, vy.x, vz.x,
		vx.y, vy.y, vz.y,
		vx.z, vy.z, vz.z);
	Mat new2camera_t = (Mat_<double>(3, 1) << p0.x, p0.y, p0.z);

	Mat camera2new_rot = new2camera_rot.t();
	Mat camera2new_t = -camera2new_rot*new2camera_t;

	//project origin points of world frames into the fitted plane, and these points is expressed in camera frame and new frame
	vector<Point2f> fit_pts[2];
	for (int img_type = 0; img_type < 2; img_type++)
	{
		for (unsigned int i = 0; i < worldsInCamera[img_type].size(); ++i)
		{
			float t = (A * worldsInCamera[img_type][i].x + B * worldsInCamera[img_type][i].y + C * worldsInCamera[img_type][i].z - D) / sum_squar;
			float x = worldsInCamera[img_type][i].x - A * t;
			float y = worldsInCamera[img_type][i].y - B * t;
			float z = worldsInCamera[img_type][i].z - C * t;

			if (A * x + B * y + C * z - D > 1e-4)
				cout << "Warning: A * x + B * y + C * z - D = " << A * x + B * y + C * z - D << std::endl;

			//point in camera
			Mat cameraPoint = (Mat_<double>(3, 1) << x, y, z);
			//point in new
			Mat newPoint = camera2new_rot*cameraPoint + camera2new_t;

			if (newPoint.at<double>(2) > 1e-4)
				cout << "Warning: newPoint.at<double>(2) = " << newPoint.at<double>(2) << std::endl;

			fit_pts[img_type].push_back(Point2f(newPoint.at<double>(0), newPoint.at<double>(1)));

			worldInCamera_tvecs[img_type][i] = cameraPoint;
		}
	}

	//fit ellipse with image_type "circle"
	RotatedRect rect = fitEllipse(fit_pts[0]);
	Mat centerNewPoint = (Mat_<double>(3, 1) << rect.center.x, rect.center.y, 0);
	Mat centerCameraPoint = new2camera_rot*centerNewPoint + new2camera_t;

	//debug info
	if (1)
	{
		//get X,Y in camera of rotate axis intersect with world plane
		double k = -centerCameraPoint.at<double>(2) / C;
		double X__ = centerCameraPoint.at<double>(0) + k*A;
		double Y__ = centerCameraPoint.at<double>(1) + k*B;
		cout << "X__: " << X__ << ", Y__:" << Y__ << endl;
	}
	
	//fit line	with image_type "circle"
	Mat line;
	float vx_, vy_;//, x0, y0;
	fitLine(fit_pts[1], line, CV_DIST_L2, 0, 0.01, 0.01);
	vx_ = line.at<float>(0, 0);
	vy_ = line.at<float>(1, 0);
	//x0 = line.at<float>(2, 0);
	//y0 = line.at<float>(3, 0);

	// calc average rvec and tvec in camera (need to be careful with outliers)
	Mat avg_rvec = Mat::zeros(3, 1, CV_64FC1);
	Mat avg_tvec = Mat::zeros(3, 1, CV_64FC1);
	for (unsigned int i = 0; i < worldsInCamera[1].size(); i++)
	{
		avg_rvec = avg_rvec + worldInCamera_rvecs[1][i];
		avg_tvec = avg_tvec + worldInCamera_tvecs[1][i];
	}
	avg_rvec = avg_rvec / worldsInCamera[1].size();
	avg_tvec = avg_tvec / worldsInCamera[1].size();
	Mat avg_rot;
	Rodrigues(avg_rvec, avg_rot);

	Mat basePointInCamera = centerCameraPoint;

	float theta = atan2(vy_, vx_);
	float theta_t = atan2(fit_pts[1].back().y - fit_pts[1][0].y, fit_pts[1].back().x - fit_pts[1][0].x);
	cout<< "abs(theta_t - theta)" << abs(theta_t - theta) << endl;
	if (abs(theta_t - theta) < CV_PI / 2)
	{
		if (theta > 0)
			theta -= CV_PI;
		else
			theta += CV_PI;
	}

	p0.x = basePointInCamera.at<double>(0);
	p0.y = basePointInCamera.at<double>(1);
	p0.z = basePointInCamera.at<double>(2);
	Mat baseXPointInNew = centerNewPoint;
	baseXPointInNew.at<double>(0) += cos(theta);
	baseXPointInNew.at<double>(1) += sin(theta);
	Mat baseXPointInCamera = new2camera_rot*baseXPointInNew + new2camera_t;
	p1.x = baseXPointInCamera.at<double>(0);
	p1.y = baseXPointInCamera.at<double>(1);
	p1.z = baseXPointInCamera.at<double>(2);
	vx = p1 - p0;
	sum = sqrt(vx.x*vx.x + vx.y*vx.y + vx.z*vx.z);
	vx = vx * (1. / sum);
	vy = vz.cross(vx);
	sum = sqrt(vy.x*vy.x + vy.y*vy.y + vy.z*vy.z);
	vy = vy * (1. / sum);

	Mat base2camera_rot = (Mat_<double>(3, 3) <<
		vx.x, vy.x, vz.x,
		vx.y, vy.y, vz.y,
		vx.z, vy.z, vz.z);
	Mat base2camera_t = (Mat_<double>(3, 1) << p0.x, p0.y, p0.z);
	
	base2camera_rot_result = base2camera_rot;
	base2camera_t_result = base2camera_t;

	{
		Mat tmp_R;
		Rodrigues(showImg_rvec,tmp_R);
		cout << "showImg_rvec :\n" << tmp_R<<endl;
		cout << "showImg_tvec :\n" <<showImg_tvec<<endl;

		std::cout << "base2camera_rot:\n" << base2camera_rot << std::endl;
		std::cout << "base2camera_t:\n" << base2camera_t << std::endl;
		
		ofstream fout("worldsInCamera.txt");
		fout << "showImg_rvec :"<<endl;
		fout<< tmp_R <<endl;
		fout << "showImg_tvec :"<<endl;
		fout<< showImg_tvec <<endl;
		fout << "base2camera_rot :"<<endl;
		fout<< base2camera_rot <<endl;
		fout << "base2camera_t :"<<endl;
		fout<< base2camera_t <<endl;
		fout.close();
	}	
	
	//save files
// 	{
// 		ofstream imgPts_circle("imgPts_circle.txt"),imgPts_line("imgPts_line.txt"),worldPts_circle("worldPts_circle.txt"),worldPts_line("worldPts_line.txt");
// 		ofstream times_circle("times_circle.txt"),times_line("times_line.txt"),odoms_circle("odoms_circle.txt"),odoms_line("odoms_line.txt");
// 
// 		for(unsigned int i =0; i<timeStamp_circle.size();i++)
// 		{
// 			vector<Point2f> tmpImgPts = saveImgPoints_circle[i];
// 			vector<Point3f> tmpWorldPts = saveWorldPoints_circle[i];
// 			times_circle <<fixed<<setprecision(10)<< timeStamp_circle[i]<<'\t'<<tmpImgPts.size() <<endl;
// 			for(unsigned int j = 0; j< tmpImgPts.size();j++)
// 			{
// 				imgPts_circle <<fixed<<setprecision(10)<< tmpImgPts[j].x <<'\t' << tmpImgPts[j].y<<endl;
// 				worldPts_circle <<fixed<<setprecision(10)<< tmpWorldPts[j].x <<'\t' << tmpWorldPts[j].y<<'\t' << tmpWorldPts[j].z<<endl;
// 			}
// 			imgPts_circle <<endl;
// 			worldPts_circle<<endl;
// 		}
// 		for(unsigned int i = 0; i<odom_circle.size(); i++ )
// 			odoms_circle<<fixed<<setprecision(10)<< odom_circle[i].timeStamp << '\t'<< odom_circle[i].xPos << '\t'<< odom_circle[i].yPos << '\t'<< odom_circle[i].qx << '\t'
// 			<< odom_circle[i].qy << '\t'<< odom_circle[i].qz << '\t'<< odom_circle[i].qw << '\t'<< odom_circle[i].tpx << '\t'
// 			<< odom_circle[i].tpy << '\t'<< odom_circle[i].tpz << '\t'<< odom_circle[i].tax << '\t'<< odom_circle[i].tay << '\t'<< odom_circle[i].taz << endl;
// 		
// 		for(unsigned int i =0; i<timeStamp_line.size();i++)
// 		{
// 			vector<Point2f> tmpImgPts = saveImgPoints_line[i];
// 			vector<Point3f> tmpWorldPts = saveWorldPoints_line[i];
// 			times_line <<fixed<<setprecision(10)<< timeStamp_line[i]<<'\t';times_line<<tmpImgPts.size()<<endl;
// 			for(unsigned int j = 0; j< tmpImgPts.size();j++)
// 			{
// 				imgPts_line <<fixed<<setprecision(10)<< tmpImgPts[j].x <<'\t' << tmpImgPts[j].y<<endl;
// 				worldPts_line <<fixed<<setprecision(10)<< tmpWorldPts[j].x <<'\t' << tmpWorldPts[j].y <<'\t' << tmpWorldPts[j].z<<endl;
// 			}
// 			
// 			imgPts_line <<endl;
// 			worldPts_line<<endl;
// 		}
// 		imgPts_circle.close();imgPts_line.close();worldPts_circle.close();worldPts_line.close();times_circle.close();times_line.close();odoms_circle.close();odoms_line.close();
// 		cout << "save all done"<<endl;
// 	}
// 	
}


void calibrateBaseCamera::img_callback(const sensor_msgs::ImageConstPtr& img_msg)
{
	bool isReveive;
	pMv->getState(isReveive,mvState);
	if(isReveive)
	{
		cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(img_msg);
		Mat img = ptr->image;
		
		tmpTimeStamp_camera = ptr->header.stamp.toSec();
// 		string type = ptr->header.frame_id;
// 		map<string, vector<Point2f>> pts_img;

		Mat rvec,tvec;
		
// 		old method 
// 		if(get_RT(img,rvec,tvec))  //如要使用请将该行取消注释，将下列两行注释掉
		double reprojectError;
		if(tagToPose(img,tvec,rvec,reprojectError))
		{
			if(first_Img_flag)
			{
				if(rvec.data)
				{
					
					first_Img_flag = false;
					
					showImg = img;
					showImg_rvec = rvec;
					showImg_tvec = tvec;
					return;
				}
			}
			
			if(rvec.data)
			{
				int img_type;
				if(mvState != baseMovement::RESTING)
				{
					if(mvState == baseMovement::CIRCLE)
						img_type = 0;
					else 
						img_type = 1;
					worldsInCamera[img_type].push_back(Point3f(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)));
					worldInCamera_rvecs[img_type].push_back(rvec);
					worldInCamera_tvecs[img_type].push_back(tvec);
				}
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
		}
// 		imshow("single Img",img);
// 		waitKey(2);
	}


	if(startCalibFlag)
	{

		if(mvState == baseMovement::ENDMOVE)
		{
			startCalibFlag =false;
			cout<< "starting calibrating base and camera" << endl;
			//save files
			{
				ofstream f1("worldsInCamera_circle.txt");
				ofstream f2("worldInCamera_rvecs_circle.txt");
				ofstream f3("worldInCamera_tvecs_circle.txt");
				for(unsigned int i = 0; i < worldsInCamera[0].size(); i++)
				{
					f1 << worldsInCamera[0][i].x <<'\t' <<worldsInCamera[0][i].y <<'\t' <<worldsInCamera[0][i].z <<'\t' <<endl; 
					f2 << worldInCamera_rvecs[0][i].at<double>(0) <<'\t' <<worldInCamera_rvecs[0][i].at<double>(1) <<'\t' <<worldInCamera_rvecs[0][i].at<double>(2) <<'\t' <<endl; 
					f3 << worldInCamera_tvecs[0][i].at<double>(0) <<'\t' <<worldInCamera_tvecs[0][i].at<double>(1) <<'\t' <<worldInCamera_tvecs[0][i].at<double>(2) <<'\t' <<endl; 
				}
				f1.close();f2.close();f3.close();
				ofstream f4("worldsInCamera_line.txt");
				ofstream f5("worldInCamera_rvecs_line.txt");
				ofstream f6("worldInCamera_tvecs_line.txt");
				for(unsigned int i = 0; i < worldsInCamera[1].size(); i++)
				{
					f4 << worldsInCamera[1][i].x <<'\t' <<worldsInCamera[1][i].y <<'\t' <<worldsInCamera[1][i].z <<'\t' <<endl; 
					f5 << worldInCamera_rvecs[1][i].at<double>(0) <<'\t' <<worldInCamera_rvecs[1][i].at<double>(1) <<'\t' <<worldInCamera_rvecs[1][i].at<double>(2) <<'\t' <<endl; 
					f6 << worldInCamera_tvecs[1][i].at<double>(0) <<'\t' <<worldInCamera_tvecs[1][i].at<double>(1) <<'\t' <<worldInCamera_tvecs[1][i].at<double>(2) <<'\t' <<endl; 
				}
				f4.close();f5.close();f6.close();
			}
			
			
			computeCalibratedResult();
			if(SHOW_FINAL_RESULT)
				showFinalResult();
		}
	}
}

void calibrateBaseCamera::odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
		

	modom.timeStamp = odom_msg->header.stamp.toSec();
	modom.xPos = odom_msg->pose.pose.position.x;
	modom.yPos = odom_msg->pose.pose.position.y;
	
	modom.qx = odom_msg->pose.pose.orientation.x;
	modom.qy = odom_msg->pose.pose.orientation.y;
	modom.qz = odom_msg->pose.pose.orientation.z;
	modom.qw = odom_msg->pose.pose.orientation.w;
	
	modom.tpx = odom_msg->twist.twist.linear.x;
	modom.tpy = odom_msg->twist.twist.linear.y;
	modom.tpz = odom_msg->twist.twist.linear.z;
	
	modom.tax = odom_msg->twist.twist.angular.x;
	modom.tay = odom_msg->twist.twist.angular.y;
	modom.taz = odom_msg->twist.twist.angular.z;
// 	bool isReveive;
// 	pMv->getState(isReveive,mvState);
// 	if(mvState == baseMovement::CIRCLE)
// 	odom_circle.push_back(modom);
// 	if(mvState == baseMovement::LINE)
// 		odom_line.push_back(modom);
		
		
}
void calibrateBaseCamera::test()
{
	char filename[] ="/home/cyy/catkin_ws/src/mytest/pictures/circle/I (1000).png";
	first_Img_flag = false;
	for(int i = 1; i <=100;i++ )
	{
		snprintf(filename, sizeof(filename), "/home/cyy/catkin_ws/src/mytest/pictures/circle/I (%d).png", i);
		Mat img = imread(filename,IMREAD_GRAYSCALE);
		//cout <<"reveive image success!"<<endl;
		Mat rvec,tvec;
		//map<string, vector<Point2f>> pts_img;
		
		get_RT(img,rvec,tvec);
		imshow("single Img",img);
		waitKey(20);
	}
	cout << "ending "<<endl;
}


