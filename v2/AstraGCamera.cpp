#include "pch.h"
#include "AstraGCamera.h"

void piexl2Cam(cv::Point2d piexl, cv::Point3d& camPoint, double zc, cv::Mat cameraMatrix);

AstraGCamera::AstraGCamera()
{
	IrParamMat = cv::Mat::zeros(cv::Size(3, 3), CV_32FC1);
	RgbParamMat = cv::Mat::zeros(cv::Size(3, 3), CV_32FC1);
	IrDistCoeffs = cv::Mat::zeros(cv::Size(1, 5), CV_32FC1);
	RgbDistCoeffs = cv::Mat::zeros(cv::Size(1, 5), CV_32FC1);

}

// ��������ṻ��
typedef struct OBCameraParams2 {
	float l_intr_p[4]; //���[fx,fy,cx,cy] 
	float r_intr_p[4]; //��ɫ[fx,fy,cx,cy] 
	float r2l_r[9]; //��ȵ���ɫ[r00,r01,r02;r10,r11,r12;r20,r21,r22]
	float r2l_t[3]; //[t1,t2,t3]
	float l_k[5]; //[k1,k2,k3,p1,p2]
	float r_k[5];
	int is_mirror;
}OBCameraParams2;

cv::Mat cameraParamsMat;

// �ص���־
bool keyCallbackFlog = false;
// �ص�����
std::function<void(cv::Mat, cv::Mat)> keyCallback;

/*
��ʾģʽ
0: ����ʾ
1����ʾ��ɫ
2����ʾ���
3: ��ʾ��ɫ�����

*/
int showMod = 0;

// ����ɫ����־
bool openColor = true;
// ���������־
bool openDepth = true;

// ʵʱ���ͼ
cv::Mat depth_img;
// ʵʱ��ɫͼ
cv::Mat color_img;
// ������ʾ�Ĳ�ɫͼ
cv::Mat color_img_show;

// �������
int errCode = 0;
// Y��Ĳ���
static const int Y_ERR = 20;
// Z��Ĳ���
static const int Z_ERR = 5;

// ��ɫ����豸��
int color_cam_code = 0;

// �豸�Ƿ�����
bool cam_is_run = 1;

// �����������ȡ����ά��
std::vector<cv::Point3d> camPs;
// �����������ȡ�Ķ�ά��
std::vector<cv::Point2d> piexlPs;

// color �������¼�
void MouseEvent(int event, int x, int y, int flags, void* data)
{
	cv::Mat posMat = cv::Mat(40, 640, CV_8UC3, cv::Scalar(255,255,255));
	static cv::Point pre_pt = cv::Point(-1, -1);//��ʼ����
	static cv::Point cur_pt = cv::Point(-1, -1);//ʵʱ����
	cv::Point3d camP(0, 0, 0);
	char temp[1024];

	cur_pt = cv::Point(x, y);
	cv::Point2d p1(x, y + Y_ERR);
	cv::Mat depth_t = depth_img.clone();
	float zc = (float)depth_t.at<uint16_t>(p1.y, p1.x);

	piexl2Cam(cur_pt, camP, zc, cameraParamsMat);

	if (event == cv::EVENT_LBUTTONDOWN)//������£���ȡ��ʼ���꣬����ͼ���ϸõ㴦��Բ
	{
		//color_img.copyTo(img);//��ԭʼͼƬ���Ƶ�img��
		camPs.push_back(camP);
		piexlPs.push_back(cur_pt);
	
	}
	else if (event == cv::EVENT_RBUTTONDOWN) {
		if (camPs.size() != 0)
		{
			camPs.pop_back();
			piexlPs.pop_back();
		}
		else
		{
			std::cout << "��ǰ�Ѿ���գ�\n";
		}
	}
	else if (event == cv::EVENT_MOUSEMOVE) {
		sprintf_s(temp, "(%d,%d) cam:%f,%f,%f count:%d", x, y,camP.x,camP.y,camP.z,camPs.size());
		pre_pt = cv::Point(0, 20);
		cv::putText(posMat, temp, pre_pt, cv::FONT_HERSHEY_SIMPLEX, 0.6f, cv::Scalar(0, 0, 255), 1, 8);//�ڴ�������ʾ����
	//	cv::circle(color_img_show, cur_pt, 2, cv::Scalar(255, 0, 0), cv::FILLED, 8, 0);//��Բ
		cv::imshow("pos", posMat);

		// -----------------����������У׼���ͼ
		
		cv::Mat depth_temp = depth_img.clone();
		cv::normalize(depth_temp, depth_temp, 255, 1, cv::NORM_INF);
		depth_temp.convertTo(depth_temp, CV_8UC1);
		//// ��ת���ͼ

		cv::putText(depth_temp, temp, pre_pt, cv::FONT_HERSHEY_SIMPLEX, 0.6f, cv::Scalar(0, 0, 255), 1, 8);//�ڴ�������ʾ����
		cv::circle(depth_temp, p1, 2, cv::Scalar(255, 0, 0), cv::FILLED, 8, 0);//��Բ
		cv::imshow("imgd", depth_temp);	
		
	}
}



// ���ͼ��ļ�����
class DepthCallback : public VideoStream::NewFrameListener
{
public:
	// ÿ����һ֡�ͻ�ִ�����º���
	void onNewFrame(VideoStream& stream)
	{

		if (!stream.isValid())
		{
			cam_is_run = 0; 
			return;
		}

		// ��ȡ��ɫ��
		if (openColor)
		{
			colorStream >> color_img;
			color_img_show = color_img.clone();
			//if (!color_img.empty())	// �ж��Ƿ�Ϊ��
			//	cv::flip(color_img, color_img, 1);
			//cv::imshow("color", color_img);
			int width = color_img.cols;
			int height = color_img.rows;
			if (width != lastWidth_ || height != lastHeight_)
			{
				lastWidth_ = width;
				lastHeight_ = height;
			}
		}

		stream.readFrame(&m_frame);
	//	depth_img = cv::Mat(m_frame.getHeight(), m_frame.getWidth(), CV_16SC1);
		get_depth_img(m_frame, depth_img);
		// ��ת���ͼ
		cv::flip(depth_img, depth_img, 1);
		
		if (depth_img.empty())
		{
			return;
		}
		

		switch (showMod)
		{

		case 0:
			break;
		case 1:
		{
			cv::namedWindow("color");
			cv::setMouseCallback("color", MouseEvent);
			cv::imshow("color", color_img_show);
			break;
		}
		case 2:
		{
			cv::imshow("depth", depth_img_show);
			break;
		}
		case 3:
		{
			cv::namedWindow("color");
			cv::setMouseCallback("color", MouseEvent);
			cv::imshow("color", color_img_show);
			cv::imshow("depth", depth_img_show);

		}
		default:
			break;
		}

		if (keyCallbackFlog)
			keyCallback(color_img, depth_img);
		else
			cv::waitKey(10);

	}

	void get_depth_img(const VideoFrameRef& frame, cv::Mat& depth_img)
	{
		if (frame.isValid())
		{
			DepthPixel* pDepth;
			int height_ = 400;
			int width_ = frame.getWidth();

			const int byteLength = lastWidth_ * lastHeight_ * 3;

			//determine if buffer needs to be reallocated
			if (width_ != lastWidth_depth_ || height_ != lastHeight_depth_) {
				buffer_depth_ = buffer_depth_ptr(new int16_t[byteLength/3]);
				lastWidth_depth_ = width_;
				lastHeight_depth_ = height_;

			}

			const float x_scale = float(lastWidth_) / lastWidth_depth_;
			const float y_scale = float(lastHeight_) / lastHeight_depth_;

			
		//	std::cout << "x:" << x_scale << " y:" << y_scale << std::endl;
		//	std::cout << "lastw:" << lastWidth_ << " h:" << lastHeight_ << std::endl;
		//	std::cout << "lastw2:" << lastWidth_depth_ << " h:" << lastHeight_depth_ << std::endl;

			switch (frame.getVideoMode().getPixelFormat())
			{
			case PIXEL_FORMAT_DEPTH_1_MM:
			case PIXEL_FORMAT_DEPTH_100_UM:
			{
				pDepth = (DepthPixel*)frame.getData();
				// �Զ�����  ==================================
				depth_img = cv::Mat(frame.getHeight(), frame.getWidth(), CV_16SC1, (void*)frame.getData());


				// �ֶ�����  ======================================================
				//�ֶ������ڴ�
				//BufferPtr depthdisplayBuffer_ = BufferPtr(new uint8_t[byteLength]);
				//std::fill(&depthdisplayBuffer_[0], &depthdisplayBuffer_[0] + byteLength, 0);
				//	std::cout << depth_img.at<uint16_t>(300, 220);	

				//for (int y = 0; y < lastHeight_; ++y)
				//{
				//	for (int x = 0; x < lastWidth_; ++x)
				//	{
				//		const int index_color = y * lastWidth_ + x;
				//		const int index_depth = int(y / y_scale + 0.5) * lastWidth_depth_ + int(x / x_scale + 0.5);  //���ùٷ�sample���������.
				//		const int rgbaOffset = index_color * 3;
				//		depthdisplayBuffer_[rgbaOffset] = uint8_t(pDepth[index_depth]);   //����Ӧλ�õ����ͼȡ��
				//		depthdisplayBuffer_[rgbaOffset + 1] = uint8_t(pDepth[index_depth]);
				//		depthdisplayBuffer_[rgbaOffset + 2] = uint8_t(pDepth[index_depth]);
				//		buffer_depth_[index_color] = pDepth[index_depth];	
				//	}
				//}
				//depth_img = cv::Mat(lastHeight_, lastWidth_, CV_16SC1,buffer_depth_.get());

			//	depth_img_show = cv::Mat(height_, width_, CV_8UC3, depthdisplayBuffer_.get()).clone();
				depth_img_show = depth_img.clone();
				cv::normalize(depth_img_show, depth_img_show, 255, 1, cv::NORM_INF);
				depth_img_show.convertTo(depth_img_show, CV_8UC1);
				//// ��ת���ͼ
				cv::flip(depth_img_show, depth_img_show, 1);
				//cv::imshow("depth", depth_img_show);
				break;

			}

			default:
				printf("Unknown format\n");
			}

		}


	}

private:
	VideoFrameRef m_frame;
public:
	using BufferPtr = std::unique_ptr<uint8_t[]>;
	using buffer_depth_ptr = std::unique_ptr<int16_t[]>;
	buffer_depth_ptr buffer_depth_;
	cv::VideoCapture colorStream;
	bool openColor = false;
	cv::Mat depth_img_show;
	int lastWidth_{ 640 };
	int lastHeight_{ 480 };
	int lastWidth_depth_{ 0 };
	int lastHeight_depth_{ 0 };

};

// ��������
DepthCallback depthPrinter;

/*��ȡ�������*/
void AstraGCamera::getCameraParams(openni::Device& Device)
{
	OBCameraParams2 cameraParam;
	//OBCameraParams2 cameraParam;
	int dataSize = sizeof(cameraParam);
	memset(&cameraParam, 0, sizeof(cameraParam));
	openni::Status rc = Device.getProperty(openni::OBEXTENSION_ID_CAM_PARAMS, (uint8_t*)&cameraParam, &dataSize);
	if (rc != openni::STATUS_OK)
	{
		std::cout << "Error:" << openni::OpenNI::getExtendedError() << std::endl;
		return;
	}

	this->IrParamMat.at<float>(0, 0) = cameraParam.l_intr_p[0];
	this->IrParamMat.at<float>(1, 1) = cameraParam.l_intr_p[1];
	this->IrParamMat.at<float>(0, 2) = cameraParam.l_intr_p[2];
	this->IrParamMat.at<float>(1, 2) = cameraParam.l_intr_p[3];
	this->IrParamMat.at<float>(2, 2) = 1.0;

	this->RgbParamMat.at<float>(0, 0) = cameraParam.r_intr_p[0];
	this->RgbParamMat.at<float>(1, 1) = cameraParam.r_intr_p[1];
	this->RgbParamMat.at<float>(0, 2) = cameraParam.r_intr_p[2];
	this->RgbParamMat.at<float>(1, 2) = cameraParam.r_intr_p[3];
	this->RgbParamMat.at<float>(2, 2) = 1.0;
	//	std::cout << "IrParmMat:\n" << this->IrParamMat << std::endl;
	//	std::cout << "RgbParmMat:\n" << this->RgbParamMat << std::endl;
	cameraParamsMat = RgbParamMat.clone();

	this->IrDistCoeffs = cv::Mat(1, 5, CV_32FC1, cameraParam.l_k).clone();
	this->RgbDistCoeffs = cv::Mat(1, 5, CV_32FC1, cameraParam.r_k).clone();

	cv::Mat R = cv::Mat(3, 3, CV_32FC1, cameraParam.r2l_r);
	cv::Mat T = cv::Mat(3, 1, CV_32FC1, cameraParam.r2l_t);

	cv::Mat_<float> R1 = (cv::Mat_<float>(4, 3) <<
		R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2),
		R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2),
		R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2),
		0, 0, 0);
	cv::Mat_<float> T1 = (cv::Mat_<float>(4, 1) <<
		T.at<float>(0, 0),
		T.at<float>(1, 0),
		T.at<float>(2, 0),
		1);
	cv::hconcat(R1, T1, H_rgb2ir);		//����ƴ��

//	this->H_rgb2ir = MyCalibration::R_T2HomogeneousMatrix(R,T).clone();
//	std::cout << "H:\n" << this->H_rgb2ir << std::endl;

}


/*
	�������
	getColor: ����ɫ��
	getDepth: �������

	ע�⣺
		�������������ѭ�����������ϵĶ�ȡ������
*/
int AstraGCamera::start(bool get_Color, bool get_Depth)
{


	openColor = get_Color;
	openDepth = get_Depth;
	if (showMod == 1)
	{
		openColor = true;
	}
	else if (showMod == 2)
	{
		openDepth = true;
	}
	else if (showMod == 3)
	{
		openColor = true;
		openDepth = true;
	}

	camthread = std::thread(cameraThread, (void*)this);
//	std::thread(cameraThread, (void*)this);
	return 0;
}

/*
	����߳� 
*/
void * AstraGCamera::cameraThread(void* __this)
{

	AstraGCamera * _this = (AstraGCamera*)__this;

	//��ʼ�����
	Status rc = OpenNI::initialize();
	if (rc != STATUS_OK)
	{
		printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
		errCode = 1;
		return &errCode;
	}
	Device device;
	
	//��ȡDevice����
	rc = device.open(ANY_DEVICE);
	if (rc != STATUS_OK)
	{
		printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
		errCode = 2;
		return &errCode;
	}

	_this->getCameraParams(device);

	// ������ɫ��
	if (openColor)
	{
		depthPrinter.colorStream.open(color_cam_code);
		if (!depthPrinter.colorStream.isOpened())    // �ж��Ƿ�򿪳ɹ�
		{
			std::cout << "open camera failed. " << std::endl;
			errCode = 3;
			return &errCode;
		}

		depthPrinter.openColor = true;


	}


	VideoStream depth;

	// ���������
	if (openDepth)
	{
		if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
		{
			rc = depth.create(device, SENSOR_DEPTH);
			if (rc != STATUS_OK)
			{
				printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
				errCode = 4;
				return &errCode;
			}
		}
		//start depth stream
		rc = depth.start();
		if (rc != STATUS_OK)
		{
			printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
			errCode = 5;
			return &errCode;
		}


		// Register frame listener
		depth.addNewFrameListener(&depthPrinter);


	}

	// ����Ӳ��D2C
	if (device.isImageRegistrationModeSupported(
		IMAGE_REGISTRATION_DEPTH_TO_COLOR))
	{
		device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	}

	std::cout << "camera running ..\n";
	do
	{
		if (!device.isValid()) {
			cam_is_run=0;
			std::cout << "over...";
			break;
		}
		if (!openDepth)
		{
			depthPrinter.colorStream >> color_img;
			if (!color_img.empty())	// �ж��Ƿ�Ϊ��
			{
				cv::flip(color_img, color_img, 1);
			}
			if (showMod == 1)
				cv::imshow("color", color_img);

			if (keyCallbackFlog)
			{
				keyCallback(color_img, cv::Mat());
			}
		}
		Sleep(10);

	} while (_this->shouldContinue);

	if (openDepth)
	{
		depth.removeNewFrameListener(&depthPrinter);
		//stop depth stream
		depth.stop();
		//destroy depth stream
		depth.destroy();
	}
	if (openColor)
	{
		depthPrinter.colorStream.release();

	}
	//close device
	device.close();
	//shutdown OpenNI
	OpenNI::shutdown();
	cv::destroyAllWindows();
	std::cout << "over...";
}

/*
	��ȡ��ɫͼ
	return: ��ǰ֡����ɫͼ
*/
cv::Mat AstraGCamera::getColor()
{
	return color_img.clone();
}

/*
	��ȡ���ͼ
	return: ��ǰ֡�����ͼ����ʽΪCV_16SC1
*/
cv::Mat AstraGCamera::getDepth()
{
	return depth_img.clone();
}

/*
	���ûص�����
	func: �ص����������������ֱ�Ϊ��ɫͼ�����ͼ
*/
void AstraGCamera::setkeyCallback(std::function<void(cv::Mat, cv::Mat)> func)
{
	keyCallback = func;
	keyCallbackFlog = true;
}

/*��ȡIr����ڲ�*/
cv::Mat AstraGCamera::getIrParamMat()
{
	return IrParamMat.clone();
}

/*��ȡRgb����ڲ�*/
cv::Mat AstraGCamera::getRgbParamMat()
{
	return RgbParamMat.clone();
}

/*��ȡIr�������*/
cv::Mat AstraGCamera::getIrDistCoeffs()
{
	return IrDistCoeffs.clone();
}

/*��ȡRgb�������*/
cv::Mat AstraGCamera::getRgbDistCoeffs()
{
	return RgbDistCoeffs.clone();
}

/*
������
ע��
*/
cv::Mat AstraGCamera::getRgb2Ir()
{
	return H_rgb2ir.clone();
}


/*
����������תΪ�������
piexlPoint: Ҫת������������
return: ��Ӧ���������
*/
cv::Point3f AstraGCamera::piexl2cam(cv::Point2d piexlPoint)
{

	if (depth_img.empty()) {
		std::cout << "������δ��!";
		return cv::Point3f(0, 0, 0);
	}
	float zc = (float)depth_img.at<uint16_t>(piexlPoint.y+Y_ERR, piexlPoint.x);
	if(zc!=0)
		zc += Z_ERR;
	float fx = RgbParamMat.at<float>(0, 0);
	float fy = RgbParamMat.at<float>(1, 1);
	float cx = RgbParamMat.at<float>(0, 2);
	float cy = RgbParamMat.at<float>(1, 2);

	cv::Point3f camPoint;
	camPoint.x = zc * (piexlPoint.x - cx) / fx;
	camPoint.y = zc * (piexlPoint.y - cy) / fy;
	camPoint.z = zc;

//	piexl2Cam(piexlPoint, camPoint, zc, RgbParamMat);


	return camPoint;

}


void piexl2Cam(cv::Point2d piexl, cv::Point3d& camPoint, double zc, cv::Mat cameraMatrix) {

	if (zc != 0)	zc += Z_ERR;
	double fx = cameraMatrix.at<float>(0, 0);
	double fy = cameraMatrix.at<float>(1, 1);
	double cx = cameraMatrix.at<float>(0, 2);
	double cy = cameraMatrix.at<float>(1, 2);

	camPoint.x = zc * (piexl.x - cx) / fx;
	camPoint.y = zc * (piexl.y - cy) / fy;
	camPoint.z = zc;

}



void AstraGCamera::setShowMode(int mode = 0)
{
	showMod = mode;

}


void AstraGCamera::join() 
{
	camthread.join();
}


std::vector<cv::Point3d> AstraGCamera::getCamPoints()
{
	return camPs;
}


void AstraGCamera::clearCamPoints() 
{
	camPs.clear();
}


std::vector<cv::Point2d> AstraGCamera::getPiexlPoints()
{
	return piexlPs;
}


void AstraGCamera::clearPiexlPoints()
{
	piexlPs.clear();
}

void AstraGCamera::setColorCamCode(int code)
{
	color_cam_code = code;
}

bool AstraGCamera::isConnect()
{
	return cam_is_run;
}


/*�ر����*/
void AstraGCamera::close()
{
	shouldContinue = false;
	
}

AstraGCamera::~AstraGCamera()
{
	shouldContinue = false;
}
