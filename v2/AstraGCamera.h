#pragma once

#include "pch.h"
#include "BLAstraGCamera.h"

using namespace openni;

class CLASS_DECLSPEC AstraGCamera : public BLAstraCamrea
{


protected:
	// ��ȡ����ڲ�
	void  getCameraParams(openni::Device& Device);

public:
	AstraGCamera();
	~AstraGCamera();

	// �������
	int start(bool getColor = true, bool getDepth = true);

	// ��ȡͼ��
	cv::Mat getColor();
	cv::Mat getDepth();

	// ��ȡ�������
	cv::Mat getRgb2Ir();
	cv::Mat getRgbParamMat();
	cv::Mat getIrDistCoeffs();
	cv::Mat getRgbDistCoeffs();
	cv::Mat getIrParamMat();

	// ���ð����ص�
	void setkeyCallback(std::function<void(cv::Mat, cv::Mat)>);

	// ��ȡ���ص���������
	cv::Point3f piexl2cam(cv::Point2d);

	void setShowMode(int mode);

	// �ر����
	void close();


};

