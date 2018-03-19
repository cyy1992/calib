#include <iostream>
#include "mclmcr.h"  
#include "matrix.h"
#include <string>
#include <stdlib.h>
#include <fstream>
#include "libmatlabCalib.h"

using namespace std;
int main(int argc, char **argv) {
	if( !libmatlabCalibInitialize())  
	{  
		std::cout << "Could not initialize libmyFunc!" << std::endl;  
		return -1;
	}
	std::cout << "Matlab library initialize success!" << std::endl;
	string camera_topic = argv[1];
	int max_img_number = atoi(argv[2]);
	int min_img_number = atoi(argv[3]);
	int checkerboard_size = atoi(argv[4]);
	int img_wigth = atoi(argv[5]);
	int img_higth = atoi(argv[6]);
	double time_interval = atof(argv[7]);
	double tolerable_reproject_error = atof(argv[8]);
	int checker_wigth = atoi(argv[9]);
	int checker_higth = atoi(argv[10]);
	const char *subTopicName_str = camera_topic.c_str();
	
	//input parameters
	mwArray input_param(9,1,mxDOUBLE_CLASS);
	cout << "number : "<<camera_topic<<endl;
	//max image numbers, min image numbers, size of checkerboard, image wigth, image higth, time interval, max Mean reproject Error 
	double mparam[9];
	mparam[0] = max_img_number;
	mparam[1] = min_img_number;
	mparam[2] = checkerboard_size;
	mparam[3] = img_wigth;
	mparam[4] = img_higth;
	mparam[5] = time_interval;
	mparam[6] = tolerable_reproject_error;
	mparam[7] = checker_wigth;
	mparam[8] = checker_higth;
	input_param.SetData(mparam,9);
	//char subname[] = "/calibImg";
	// topic name
	mwArray subTopicName(subTopicName_str);
	
	//output parameters
	mwArray mwIntrinsic(3, 3, mxDOUBLE_CLASS);  
	mwArray mwdistort(1, 2, mxDOUBLE_CLASS); 
	
	matlabCalib(2, mwIntrinsic,mwdistort,input_param,subTopicName); 
	
	//output result
	double intrinsic[4];
	intrinsic[0] = mwIntrinsic(1, 1);  intrinsic[1] = mwIntrinsic(2, 2);intrinsic[2] = mwIntrinsic(1, 3);intrinsic[3] = mwIntrinsic(2, 3);
	double distort[2];
	distort[0] = mwdistort(1,1); distort[1] = mwdistort(1,2);
	
	ofstream fout("intrinsic_parameters.txt");
	fout << "intrinsic: " << endl;
	fout << mwIntrinsic(1, 1) <<'\t'<< mwIntrinsic(1, 2) <<'\t'<< mwIntrinsic(1, 3) <<endl;
	fout << mwIntrinsic(2, 1) <<'\t'<< mwIntrinsic(2, 2) <<'\t'<< mwIntrinsic(2, 3) <<endl;
	fout << mwIntrinsic(3, 1) <<'\t'<< mwIntrinsic(3, 2) <<'\t'<< mwIntrinsic(3, 3) <<";"<<endl;
	fout << "distortion:" << endl;
	fout << mwdistort(1,1) << '\t' << mwdistort(1,2) << '\t' << 0<<'\t' << 0<< ";" << endl;
	fout.close();
	
	std::cout<<"The intrinsic(fx,fy,u0,v0) is: "<<intrinsic[0]<<", "<<intrinsic[1]<<", "<<intrinsic[2]<<", "<<intrinsic[3]<<endl;
	cout<<"The distortion is: "<<distort[0]<<", "<<distort[1]<<endl;
	libmatlabCalibTerminate();
	
	mclTerminateApplication();  
    return 0;
}
