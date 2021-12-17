#include "test.h"
#include <string>


BLAstraCamrea* cam;

// ���ڴ˴�ʵʱ����ͼ��
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

	// ����������
	BLfactory b;
	// ��ȡ�������
    cam = b.createBLAstraCamera();
	// ���ûص���������ʵʱ����ͼ��
//	cam->setkeyCallback(callback);
	// ���ô�����ʾ
	cam->setShowMode(3);
	// �����
	cam->start();

	// ��������һЩ��������
	while (true) 
	{
		std::string cmd;
		std::cin >> cmd;
		if (cmd == "0") {
			// �ر����
			cam->close();
		
		}
		if (cmd == "1") {
			// ��ȡ��Ϣ
			
			std::cout << "prams:" << cam->getRgbParamMat();
			std::cout << "K:" << cam->getRgbDistCoeffs();
				
			// ��ȡ��ɫͼ
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