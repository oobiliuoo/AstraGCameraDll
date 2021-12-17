#include "test.h"
#include <string>


BLAstraCamrea* cam;

// 可在此处实时处理图像
void callback(cv::Mat color, cv::Mat depth)
{
	int key = cv::waitKey(30) & 0xFF;

	if (key == 65) {
		cam->close();
	
	}
	else if (key == 66) 
	{
		std::vector<cv::Point3d> camPs = cam->getCamPoints();
		std::cout << camPs[0];

	}


}

void test1()
{

	// 构建工厂类
	BLfactory b;
	// 获取相机对象
    cam = b.createBLAstraCamera();
	// 设置回调函数，可实时处理图像
//	cam->setkeyCallback(callback);
	// 设置窗口显示
	cam->setShowMode(3);
	// 打开相机
	cam->start();

	// 在这里做一些其它操作
	while (true) 
	{
		std::string cmd;
		std::cin >> cmd;
		if (cmd == "0") {
			// 关闭相机
			cam->close();
		
		}
		if (cmd == "1") {
			// 获取信息
			
			std::cout << "prams:" << cam->getRgbParamMat();
			std::cout << "K:" << cam->getRgbDistCoeffs();
				
			// 获取彩色图
			cv::Mat color = cam->getColor();
			cv::imshow("color2", color);
			cv::waitKey(100);

		}
		if (cmd == "2") {
			cam->setShowMode(0);
		//	cv::destroyAllWindows();
		}
		if(cmd=="q")
		{
			cam->close();
			break;
		}
	
	}



	cam->join();
	cam->close();
	b.deleteObject(cam);

}