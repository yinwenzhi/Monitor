#include "CameraDevice.h"
#include <iostream>

int main()
{

    Device::CameraDevice camera;
    if(camera.ConnectDevice() == -1){
        std::cout << "camera Device connect failed.please check camera device" <<std::endl;
        return 0;
    }
    cv::Mat edges; //定义一个Mat变量，用于存储每一帧的图像
    
    // //循环显示图像
    while(true)
    {
        // std::cout << "showimage in main:############"<<std::endl;
        // clone image 到edges，有内存拷贝操作，后续可以改进
        camera.GetImage(edges);
        imshow("Device",edges);

        // Check if ESC was pressed
		if (cv::waitKey(30) == 27) {
		    break;
		}
        
    }

    return 0;
}