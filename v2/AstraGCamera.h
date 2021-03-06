#pragma once

#include "pch.h"
#include "BLAstraGCamera.h"
#include <fstream>

using namespace openni;

class CLASS_DECLSPEC AstraGCamera : public BLAstraCamrea
{


protected:
	// 获取相机内参
	void getCameraParams(openni::Device& Device);
	static void* cameraThread(void *);


public:
	AstraGCamera();
	~AstraGCamera();

	// 开启相机
	int start(bool getColor = true, bool getDepth = true);

	// 获取图像
	cv::Mat getColor();
	cv::Mat getDepth();

	// 获取相机参数
	cv::Mat getRgb2Ir();
	cv::Mat getRgbParamMat();
	cv::Mat getIrDistCoeffs();
	cv::Mat getRgbDistCoeffs();
	cv::Mat getIrParamMat();

	// 设置按键回调
	void setkeyCallback(std::function<void(cv::Mat, cv::Mat)>);

	// 获取像素点的相机坐标
	cv::Point3f piexl2cam(cv::Point2d);

	void setShowMode(int mode);
	void join();

	std::vector<cv::Point3d> getCamPoints();

	void clearCamPoints();

	std::vector<cv::Point2d> getPiexlPoints();

	void clearPiexlPoints();

	void setColorCamCode(int);

	// 关闭相机
	void close();

	bool isConnect();


};

