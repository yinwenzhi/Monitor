

// #include "../../include/Monitor/System.h"
# include "System.h"

// #include "Converter.h"
// #include <thread>
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
    Monitor::Config::setParameterFile( strSettingsFile );
    mpConfig = Config::getInstance();
    
    //构建相机对象，同时从配置文件中读取相机内参（在构造函数中实现）
    //myslam::Camera::Ptr camera ( new myslam::Camera );
    Monitor::Camera::Ptr Camera ( new Monitor::Camera(mpConfig));
    Monitor::Tracking::Ptr Tracker ( new Monitor::Tracking(mpConfig));
    mpCamera = Camera;
    mpTracker = Tracker;
    std::cout << "mpTracker in System initial" << mpTracker << std::endl;


    
    mpTracker->setCameraModel(mpCamera);
    std::cout << "setcameraModel "<< mpCamera->fx_ << std::endl;
    std::cout << "setcameraModel mcamera_"<< mpTracker -> mcamera_->fx_ << std::endl;
    // mpTracker->setViewer(mpViewer);

}

void System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    #ifdef DEBUG
        std::cout << "system::TrackMonocular " << std::endl;
    #endif
    std::cout << "system::TrackMonocular " << std::endl;
    std::cout << "mpTracker in trackMonocular " << mpTracker<<std::endl;
    // mpTracker->GrabImageMonocular(im,timestamp);

    #ifdef DEBUG
        std::cout << "get a image" << std::endl; 
    #endif
    Monitor::Frame::Ptr pFrame = Monitor::Frame::createFrame();// 局部变量
    // pFrame = Frame;   // 添加一次引用才能保持生命
    // std::cout << "pFrame :" << pFrame << std::endl ;
    // std::cout << "seting frame" << std::endl; 
    // std::cout << "mcamera_ " << &(mpTracker->mcamera_);
    // std::cout << "mcamera_fx" << mpTracker->mcamera_->fx_;
    pFrame->camera_ = mpTracker->mcamera_;
    // std::cout << "seting frame" << std::endl; 
    pFrame->color_ = im;
    // pFrame->depth_ = depth;
    // pFrame->id_ = k++;
    // std::cout << "seting frame" << std::endl; 
    // pFrame->id_ = Tracking::mindex++;
    // std::cout << "seting frame" << std::endl; 
    pFrame->time_stamp_ = timestamp;


    mpTracker->AddFrame(pFrame);
    // AddFrame(mCurrentFrame);
}

Monitor::Tracking::Ptr System::getTracker(){
    return mpTracker ;
}

Vec3f System::getcameraangle(){
    return mpTracker->angle_;      // 欧拉角
}
Vector3d System::getcameratransition(){
    return mpTracker->translation_;
}

vector<cv::KeyPoint> System::getCurKeyPoints(){
    return mpTracker-> keypoints_curr_;
}

vector<cv::KeyPoint> System::getRefKeyPoints(){
    return mpTracker-> keypoints_ref_;
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
