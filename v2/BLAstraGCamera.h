#pragma once
#include "pch.h"
#define CLASS_DECLSPEC __declspec(dllexport)//��ʾ����Ҫ���ർ��//

class CLASS_DECLSPEC BLAstraCamrea
{
protected:
	/*������־*/
	bool shouldContinue = true;

	/*��������߳�*/
	std::thread camthread;
public:
	/*Ir����ڲ�*/
	cv::Mat IrParamMat;
	/*rgb����ڲ�*/
	cv::Mat RgbParamMat;

	/*Ir�����������*/
	cv::Mat IrDistCoeffs;
	/*Rgb�����������*/
	cv::Mat RgbDistCoeffs;
	/*rgb2ir��ת������*/
	cv::Mat H_rgb2ir;

protected:
	/*��ȡ����ڲ�*/
	virtual void  getCameraParams(openni::Device& Device) = 0;

public:
	BLAstraCamrea() {};
	~BLAstraCamrea() {};

public:
	/*
	�������
	getColor: ����ɫ�� Ĭ�ϴ�
	getDepth: �������	Ĭ�ϴ�
	ע�⣺
		������������̣߳��������ϵĶ�ȡ������
	*/
	virtual int start(bool getColor = true, bool getDepth = true) = 0;

	/*
	��ȡ��ɫͼ
	return:��ǰ֡����ɫͼ
	*/
	virtual cv::Mat getColor() = 0;

	/*
	��ȡ���ͼ
	return: ��ǰ֡�����ͼ����ʽΪCV_16SC1
	*/
	virtual cv::Mat getDepth() = 0;

	/*
	���ûص�����
	func: �ص����������������ֱ�Ϊ��ɫͼ�����ͼ
	*/
	virtual void setkeyCallback(std::function<void(cv::Mat, cv::Mat)>) = 0;

	/*
	��ȡIr����ڲ�
	return:ϵͳ��¼�ĺ�������ڲ�
	*/
	virtual cv::Mat getIrParamMat() = 0;

	/*
	��ȡRgb����ڲ�
	return:ϵͳ��¼�Ĳ�ɫ����ڲ�
	*/
	virtual cv::Mat getRgbParamMat() = 0;

	/*
	��ȡIr�������
	return:ϵͳ��¼������������
	*/
	virtual cv::Mat getIrDistCoeffs() = 0;

	/*
	��ȡRgb�������
	return:ϵͳ��¼�Ĳ�ɫ�������
	*/
	virtual cv::Mat getRgbDistCoeffs() = 0;

	/*
	��ȡ��ȵ���ɫ��ת�����
	ע���˴������д�SDK�б�����ʾ��r2l,���ҿ���̳������˵����l2r����������
	*/
	virtual cv::Mat getRgb2Ir() = 0;

	/*
	����������תΪ�������
	piexlPoint: Ҫת������������
	return: ��Ӧ���������
	*/
	virtual cv::Point3f piexl2cam(cv::Point2d) = 0;

	/*
	������ʾģʽ
	0: ����ʾ Ĭ��
	1����ʾ��ɫ
	2����ʾ���
	3: ��ʾ��ɫ�����
	ע��������ʾģʽ�����Ӧ������
	*/
	virtual void setShowMode(int mode) = 0;


	/*
	�������join
	�����̵ȴ��������
	ע���ֶ��ر�������߳̽���������һֱ����
	*/
	virtual void join() = 0;

	/*�ر����*/
	virtual void close() = 0;

	/*
	��ȡ������㼯
	return: ������¼��ĵ�
	*/
	virtual	std::vector<cv::Point3d> getCamPoints() = 0;

	/*
	�������㼯
	*/
	virtual void clearCamPoints() = 0;

};

