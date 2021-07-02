#pragma once
#ifndef __DEPTH_REGISTRATION_H__
#define __DEPTH_REGISTRATION_H__

//#include <vector>

#include <opencv2/opencv.hpp>

class DepthRegistrationOpenCL
{
private:
	struct OCLData;

	OCLData *data;
	
	cv::Mat cameraMatrixRegistered, cameraMatrixDepth, rotation, translation, mapX, mapY;
	cv::Size sizeRegistered, sizeDepth;
	float zNear, zFar;

	
public:
	DepthRegistrationOpenCL();
	
	~DepthRegistrationOpenCL();

	bool init(const int deviceId);
	bool init(const cv::Mat &cameraMatrixRegistered, const cv::Size &sizeRegistered, const cv::Mat &cameraMatrixDepth, const cv::Size &sizeDepth,
			  const cv::Mat &distortionDepth, const cv::Mat &rotation, const cv::Mat &translation,
			  const float zNear = 0.5f, const float zFar = 12.0f, const int deviceId = -1);

	bool registerDepth(const cv::Mat &depth, cv::Mat &registered);

private:
	void generateOptions(std::string &options) const;

	bool readProgram(std::string &source) const;
		

	
};

#endif
