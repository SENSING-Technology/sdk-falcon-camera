#ifndef __SMEAR_FILTER_H_
#define __SMEAR_FILTER_H_

#include<stdio.h>
//#ifdef _DEPTHEVAL
#include "opencv2/opencv.hpp"

void smearfilter(const cv::Mat src, cv::Mat& dst, int radius, float threshold, int p);
//#endif
#endif // !__SMEAR_FILTER_H_
