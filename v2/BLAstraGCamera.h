#pragma once
#include "pch.h"
#define CLASS_DECLSPEC __declspec(dllexport)//表示这里要把类导出//

class CLASS_DECLSPEC BLAstraCamrea
{
protected:
	/*继续标志*/
	bool shouldContinue = true;

	/*相机运行线程*/
	std::thread camthread;
public:
	/*Ir相机内参*/
	cv::Mat IrParamMat;
	/*rgb相机内参*/
	cv::Mat RgbParamMat;

	/*Ir相机畸变因子*/
	cv::Mat IrDistCoeffs;
	/*Rgb相机畸变因子*/
	cv::Mat RgbDistCoeffs;
	/*rgb2ir的转换矩阵*/
	cv::Mat H_rgb2ir;

protected:
	/*获取相机内参*/
	virtual void  getCameraParams(openni::Device& Device) = 0;

public:
	BLAstraCamrea() {};
	~BLAstraCamrea() {};

public:
	/*
	开启相机
	getColor: 打开颜色流 默认打开
	getDepth: 打开深度流	默认打开
	注意：
		开启后程序会打开线程，持续不断的读取流数据
	*/
	virtual int start(bool getColor = true, bool getDepth = true) = 0;

	/*
	获取颜色图
	return:当前帧的颜色图
	*/
	virtual cv::Mat getColor() = 0;

	/*
	获取深度图
	return: 当前帧的深度图，格式为CV_16SC1
	*/
	virtual cv::Mat getDepth() = 0;

	/*
	设置回调函数
	func: 回调函数，两个参数分别为颜色图和深度图
	*/
	virtual void setkeyCallback(std::function<void(cv::Mat, cv::Mat)>) = 0;

	/*
	获取Ir相机内参
	return:系统烧录的红外相机内参
	*/
	virtual cv::Mat getIrParamMat() = 0;

	/*
	获取Rgb相机内参
	return:系统烧录的彩色相机内参
	*/
	virtual cv::Mat getRgbParamMat() = 0;

	/*
	获取Ir相机畸变
	return:系统烧录的深度相机畸变
	*/
	virtual cv::Mat getIrDistCoeffs() = 0;

	/*
	获取Rgb相机畸变
	return:系统烧录的彩色相机畸变
	*/
	virtual cv::Mat getRgbDistCoeffs() = 0;

	/*
	获取深度到颜色的转变矩阵
	注：此处可能有错，SDK中变量表示是r2l,但我看论坛介绍里说这是l2r，存在争议
	*/
	virtual cv::Mat getRgb2Ir() = 0;

	/*
	将像素坐标转为相机坐标
	piexlPoint: 要转换的像素坐标
	return: 对应的相机坐标
	*/
	virtual cv::Point3f piexl2cam(cv::Point2d) = 0;

	/*
	设置显示模式
	0: 不显示 默认
	1：显示颜色
	2：显示深度
	3: 显示颜色和深度
	注：设置显示模式会打开相应数据流
	*/
	virtual void setShowMode(int mode) = 0;


	/*
	设置相机join
	主进程等待相机结束
	注：手动关闭相机，线程结束，否则一直运行
	*/
	virtual void join() = 0;

	/*关闭相机*/
	virtual void close() = 0;

	/*
	获取的相机点集
	return: 鼠标点击事件的点
	*/
	virtual	std::vector<cv::Point3d> getCamPoints() = 0;

	/*
	清空相机点集
	*/
	virtual void clearCamPoints() = 0;

};

