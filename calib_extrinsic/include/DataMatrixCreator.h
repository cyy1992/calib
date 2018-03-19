#ifndef DATAMATRIXCREATOR_H
#define DATAMATRIXCREATOR_H

#include <string>
#include <opencv2/opencv.hpp>

/* 生成二维码图像
 * dmImage		OUT	生成的二维码图像
 * str			IN	二维码信息
 * moduleSize	IN	单格像素尺寸*/
void createDataMatrix(cv::Mat& dmImage, const std::string& str, const int& moduleSize);


#endif