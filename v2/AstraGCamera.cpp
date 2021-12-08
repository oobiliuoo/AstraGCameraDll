#include "pch.h"
#include "AstraGCamera.h"

AstraGCamera::AstraGCamera()
{
	IrParamMat = cv::Mat::zeros(cv::Size(3, 3), CV_32FC1);
	RgbParamMat = cv::Mat::zeros(cv::Size(3, 3), CV_32FC1);
	IrDistCoeffs = cv::Mat::zeros(cv::Size(1, 5), CV_32FC1);
	RgbDistCoeffs = cv::Mat::zeros(cv::Size(1, 5), CV_32FC1);

}

// 相机参数结够体
typedef struct OBCameraParams2 {
	float l_intr_p[4]; //深度[fx,fy,cx,cy] 
	float r_intr_p[4]; //颜色[fx,fy,cx,cy] 
	float r2l_r[9]; //深度到颜色[r00,r01,r02;r10,r11,r12;r20,r21,r22]
	float r2l_t[3]; //[t1,t2,t3]
	float l_k[5]; //[k1,k2,k3,p1,p2]
	float r_k[5];
	int is_mirror;
}OBCameraParams2;

// 回调标志
bool keyCallbackFlog = false;
// 回调函数
std::function<void(cv::Mat, cv::Mat)> keyCallback;

/*
显示模式
0: 不显示
1：显示颜色
2：显示深度
3: 显示颜色和深度

*/
int showMod = 0;

bool openColor = true;
bool openDepth = true;

cv::Mat depth_img;
cv::Mat color_img;


// 深度图像的监听类
class DepthCallback : public VideoStream::NewFrameListener
{
public:
	// 每读到一帧就会执行以下函数
	void onNewFrame(VideoStream& stream)
	{
		stream.readFrame(&m_frame);
		depth_img = cv::Mat(m_frame.getHeight(), m_frame.getWidth(), CV_16SC1);
		get_depth_img(m_frame, depth_img);
		// 获取颜色流
		if (openColor)
		{
			colorStream >> color_img;
			if (!color_img.empty())	// 判断是否为空
				cv::flip(color_img, color_img, 1);
			//cv::imshow("color", color_img);

		}


		switch (showMod)
		{

		case 0:
			break;
		case 1:
		{
			cv::imshow("color", color_img);
			break;
		}
		case 2:
		{
			cv::imshow("depth", depth_img_show);
			break;
		}
		case 3:
		{
			cv::imshow("color", color_img);
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
			int height_ = frame.getHeight();
			int width_ = frame.getWidth();
			switch (frame.getVideoMode().getPixelFormat())
			{
			case PIXEL_FORMAT_DEPTH_1_MM:
			case PIXEL_FORMAT_DEPTH_100_UM:
			{
				pDepth = (DepthPixel*)frame.getData();
				depth_img = cv::Mat(height_, width_, CV_16SC1, (void*)frame.getData());
				//手动分配内存
				const int byteLength = width_ * height_ * 3;
				BufferPtr depthdisplayBuffer_ = BufferPtr(new uint8_t[byteLength]);
				std::fill(&depthdisplayBuffer_[0], &depthdisplayBuffer_[0] + byteLength, 0);
				//	std::cout << depth_img.at<uint16_t>(300, 220);			
				for (int row = 0; row < height_; ++row)
				{
					for (int col = 0; col < width_; ++col)
					{
						const int index_color = row * width_ + col;
						const int rgbaOffset = index_color * 3;
						depthdisplayBuffer_[rgbaOffset] = uint8_t(pDepth[index_color]);   //将对应位置的深度图取出
						depthdisplayBuffer_[rgbaOffset + 1] = uint8_t(pDepth[index_color]);
						depthdisplayBuffer_[rgbaOffset + 2] = uint8_t(pDepth[index_color]);

					}
				}
				depth_img_show = cv::Mat(height_, width_, CV_8UC3, depthdisplayBuffer_.get()).clone();
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
	cv::VideoCapture colorStream;
	bool openColor = false;

	cv::Mat depth_img_show;

};

// 监听对象
DepthCallback depthPrinter;

/*获取相机参数*/
void AstraGCamera::getCameraParams(openni::Device& Device)
{
	OBCameraParams2 cameraParam;
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
	cv::hconcat(R1, T1, H_rgb2ir);		//矩阵拼接

//	this->H_rgb2ir = MyCalibration::R_T2HomogeneousMatrix(R,T).clone();
//	std::cout << "H:\n" << this->H_rgb2ir << std::endl;

}

/*
	开启相机
	getColor: 打开颜色流
	getDepth: 打开深度流

	注意：
		开启后程序会进入循环，持续不断的读取流数据
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

	//初始化相机
	Status rc = OpenNI::initialize();
	if (rc != STATUS_OK)
	{
		printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
		return 1;
	}
	Device device;
	//获取Device对象
	rc = device.open(ANY_DEVICE);
	if (rc != STATUS_OK)
	{
		printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
		return 2;
	}

	getCameraParams(device);

	// 开启颜色流
	if (openColor)
	{
		depthPrinter.colorStream.open(0);
		if (!depthPrinter.colorStream.isOpened())    // 判断是否打开成功
		{
			std::cout << "open camera failed. " << std::endl;
			return -1;
		}

		depthPrinter.openColor = true;


	}


	VideoStream depth;

	// 开启深度流
	if (openDepth)
	{
		if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
		{
			rc = depth.create(device, SENSOR_DEPTH);
			if (rc != STATUS_OK)
			{
				printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
				return 3;
			}
		}
		//start depth stream
		rc = depth.start();
		if (rc != STATUS_OK)
		{
			printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
			return 4;
		}


		// Register frame listener
		depth.addNewFrameListener(&depthPrinter);


	}

	// 设置硬件D2C
	if (device.isImageRegistrationModeSupported(
		IMAGE_REGISTRATION_DEPTH_TO_COLOR))
	{
		device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	}

	std::cout << "camera running ..\n";
	do
	{
		if (!openDepth)
		{
			depthPrinter.colorStream >> color_img;
			if (!color_img.empty())	// 判断是否为空
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

	} while (shouldContinue);

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

}

/*
	获取颜色图
	return: 当前帧的颜色图
*/
cv::Mat AstraGCamera::getColor()
{
	return color_img.clone();
}

/*
	获取深度图
	return: 当前帧的深度图，格式为CV_16SC1
*/
cv::Mat AstraGCamera::getDepth()
{
	return depth_img.clone();
}

/*
	设置回调函数
	func: 回调函数，两个参数分别为颜色图和深度图
*/
void AstraGCamera::setkeyCallback(std::function<void(cv::Mat, cv::Mat)> func)
{
	keyCallback = func;
	keyCallbackFlog = true;
}

/*获取Ir相机内参*/
cv::Mat AstraGCamera::getIrParamMat()
{
	return IrParamMat.clone();
}

/*获取Rgb相机内参*/
cv::Mat AstraGCamera::getRgbParamMat()
{
	return RgbParamMat.clone();
}

/*获取Ir相机畸变*/
cv::Mat AstraGCamera::getIrDistCoeffs()
{
	return IrDistCoeffs.clone();
}

/*获取Rgb相机畸变*/
cv::Mat AstraGCamera::getRgbDistCoeffs()
{
	return RgbDistCoeffs.clone();
}

/*
相机外参
注：
*/
cv::Mat AstraGCamera::getRgb2Ir()
{
	return H_rgb2ir.clone();
}


/*
将像素坐标转为相机坐标
piexlPoint: 要转换的像素坐标
return: 对应的相机坐标
*/
cv::Point3f AstraGCamera::piexl2cam(cv::Point2d piexlPoint)
{

	if (depth_img.empty()) {
		std::cout << "深度相机未打开!";
		return cv::Point3f(0, 0, 0);
	}
	float zc = (float)depth_img.at<uint16_t>(piexlPoint.x, piexlPoint.y);

	float fx = RgbParamMat.at<float>(0, 0);
	float fy = RgbParamMat.at<float>(1, 1);
	float cx = RgbParamMat.at<float>(0, 2);
	float cy = RgbParamMat.at<float>(1, 2);

	cv::Point3f camPoint;
	camPoint.x = zc * (piexlPoint.x - cx) / fx;
	camPoint.y = zc * (piexlPoint.y - cy) / fy;
	camPoint.z = zc;
	return camPoint;

}


void AstraGCamera::setShowMode(int mode = 0)
{
	showMod = mode;

}


/*关闭相机*/
void AstraGCamera::close()
{
	shouldContinue = false;
}

AstraGCamera::~AstraGCamera()
{
	shouldContinue = false;
}
