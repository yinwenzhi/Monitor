

#include "../../include/Monitor/System.h"

// #include "Converter.h"
#include <thread>
// #include <pangolin/pangolin.h>
// #include <iomanip>

namespace Monitor
{

System::System(const std::string &strSettingsFile)
{
    // Output welcome message
    std::cout << std::endl <<
    "Monitor software " << std::endl;
    std::cout<<std::endl;
    std::cout<<std::endl;
    // 设置配置类 配置文件路径
    Monitor::Config::setParameterFile ( strSettingsFile );
    mpConfig = Config::getInstance();
    
    //构建相机对象，同时从配置文件中读取相机内参（在构造函数中实现）
    //myslam::Camera::Ptr camera ( new myslam::Camera );
    mpCamera = new Monitor::Camera(mpConfig);
    mpTracker = new  Monitor::Tracking(mpConfig);

}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    std::cout << " system::TrackMonocular " << std::endl;
    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);
    // cv::Mat Tcw = mpTracker->AddFrame(im,timestamp);

    return Tcw;
}

void System::Shutdown()
{

    // Wait until all thread have effectively stopped
    // while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    // {
    //     usleep(5000);
    // }

    // if(mpViewer)
    //     pangolin::BindToContext("ORB-SLAM2: Map Viewer");
    std::cout << "Monitor closed!" << std::endl;
}

} //namespace Monitor
