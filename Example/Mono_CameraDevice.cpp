

#include <iostream>
#include <fstream>
#include <algorithm>
#include <cassert>
#include <opencv2/opencv.hpp>
// #include <glog>
#include "../include/CameraDevice/CameraDevice.h"
#include "../include/Monitor/System.h"

using namespace cv;
// using namespace Monitor;

bool flag = true;
void exit_while(int sig)
{
  flag = false;
}

class ImageGrabber {
 public:
  // ImageGrabber(){}
  explicit ImageGrabber(Monitor::System* pSLAM):mpSLAM(pSLAM){}
  
  // 获得img图像时将调用 pSLAM-> Tracker
  void GrabMono(const cv::Mat& img, double timeStamp);

  void View(cv::Mat image,bool & isstop);

  Monitor::System* mpSLAM;
  bool do_rectify;
  bool use_camera_device;
  cv::Mat M1, M2;
};

void ImageGrabber::GrabMono(const cv::Mat& img, double timeStamp){
  if (do_rectify) {
    cv::Mat _img;

    cv::remap(
      img,                  // 输入图像
      _img,                 // 输出图像
      M1,                   // 两种可能  1 xy 的地一个映射  2 表示CV_16SC2 ..
      M2,                   // 两种可能   1 对上M1的第一种时 无任何  2  CV_16UC1 ..
      cv::INTER_LINEAR      // 插值方式 
      );

    std::cout << " 矫正后 跟踪" << std::endl;
    mpSLAM->TrackMonocular(_img, timeStamp);
  } else {
    std::cout << " 直接 跟踪" << std::endl;

    mpSLAM->TrackMonocular(img, timeStamp);
  }
}

void ImageGrabber::View(cv::Mat image,bool& isstop){
  Monitor::Tracking::Ptr tracker = mpSLAM->getTracker();
  Vec3f angle = mpSLAM->getcameraangle();
  Vector3d trans = mpSLAM->getcameratransition();
  // cout << "trans in Mono: " << trans;
  vector<cv::KeyPoint> curkeypts = mpSLAM->getCurKeyPoints();
  // vector<cv::KeyPoint> refkeypts = mpSLAM->getRefKeyPoints();
  
// #define INITIAL
#ifdef INITIAL
  // 显示参考帧
  namedWindow("refframe",0);
  cv::resizeWindow("refframe",640,480);
  // 3  5  7  9
  vector<bool> parity = {1,0};
  vector<int>  model=  {3,5,7,9 };
  
  for(int j = 0; j< parity.size(); j++){
      for(int k =0; k<model.size(); k++){
        cv::Mat refimage = tracker->mref_->color_.clone();
        bool waitnext = true;
        std::cout << "new keypoint index !" << std::endl;
        for(int n = 0; n < tracker->keypoints_all_ref_.size(); n++)
        {
            //初始化随机种子
            cv::RNG rng(cvGetTickCount());
            CvScalar color = cv::Scalar(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));
            if(bool(n%2) == parity[j] && (n%model[k]) == 0 ) {
              circle(refimage, tracker->keypoints_all_ref_[n].pt, 1, color, 2, 8, 0);
              cv::putText(refimage,std::to_string(n), tracker->keypoints_all_ref_[n].pt, cv::FONT_HERSHEY_SIMPLEX,0.60, color,1,8);

            }
            // circle(edges, vo->keypoints_curr_[n].pt, 1, Scalar(255, 0, 255), 2, 8, 0);
        }
        circle(refimage, CvPoint(20,1000), 5, CV_RGB(255,0,255), 2, 8, 0);
        // Mat dst;  
        // transpose(refimage, dst);
        // flip(refimage, dst, 1);
        imshow("refframe",refimage);
        while(waitnext){
          std::cout << "waiting for next: press n !" << std::endl;
          char n = cvWaitKey(30);
          // 显示下一帧
          if(n==110) {
            waitnext = !waitnext;
            break;
          }
          // std::cout << "get :" << (int)(n) << std::endl;
          sleep(1);
        }
      }
  }
#endif

  // 显示当前帧
  int a1 = 20,b1 = 300;
  //         ( 待绘制图像，文字，，字体，尺寸因子，线条颜色，线宽，线型 )
  cv::putText(image,"R: ", cv::Point2f(a1,200), cv::FONT_HERSHEY_SIMPLEX,0.60, CV_RGB(0, 0, 0),1,8);
  cv::putText(image,"t: ", cv::Point2f(a1,230), cv::FONT_HERSHEY_SIMPLEX,0.60, CV_RGB(0, 0, 0),1,8);
  for(int m =0;m<3;m++)
  {
    
    double a= angle[m];
    double b= trans[m];
    cout << "b:" << b;
    cv::Point2f num1 (a1+(m+1)*150,200);
    cv::Point2f num2 (a1+(m+1)*150,230);
    cv::putText(image,std::to_string(a*180/3.1415926), num1, cv::FONT_HERSHEY_SIMPLEX,0.60, CV_RGB(0, 0, 0),1,8);
    cv::putText(image,std::to_string(b), num2, cv::FONT_HERSHEY_SIMPLEX,0.60, CV_RGB(0, 0, 0),1,8);
  }
  
  std::cout <<  " curkeypts.size()" << curkeypts.size() << std::endl;
  for(size_t n = 0; n < curkeypts.size(); n++)
  {
    circle(image, curkeypts[n].pt, 1, Scalar(255, 0, 255), 2, 8, 0);
  }
  namedWindow("Mono_CameraDevice_currframe",0);
  cv::resizeWindow("Mono_CameraDevice_currframe",640,480);
  imshow("Mono_CameraDevice_currframe",image);

  // 显示匹配关系图
  namedWindow("img_goodmatch",0);
  cv::resizeWindow("img_goodmatch",640,480);
  cv::Mat img_goodmatch;
  drawMatches ( 
      tracker->mref_->color_,             // 参考帧图像
      tracker->keypoints_all_ref_,        // 因为匹配使用描述子是所有特征点的描述子
      // tracker->keypoints_ref_,        // 因为匹配使用描述子是所有只有3d的描述子
      image,                              // 当前帧图像
      tracker->keypoints_curr_, 
      tracker->feature_matches_, 
      img_goodmatch 
       );
  imshow("img_goodmatch",img_goodmatch);


  char c = cvWaitKey(33); // space
  if(c == 27) {       //esc
    isstop = true;
    return ;
  }else if (c==112){
    waitKey(0);
  }else if( c == 32){ // 空格键暂停功能
    cout<<"System Paused, press again to continue......"<<endl;
    bool ispause = true;
    while(ispause){
      char p = cvWaitKey(33);
      if( p == 32){
        ispause = false;
      }else   if(p == 27) { //esc
        isstop = true;
        return ;
      }

      sleep(1);
    }
    sleep(1);
  }
}

int main(int argc, char** argv) {
  // glog_init _(argc, argv);

  if (argc != 4) {
    std::cout << std::endl << "Usage: ./Monitor path_to_setting(string) do_rectify(bool) use_camera_device(bool)" << std::endl;
    return 1;
  }


  std::cout << "--args: " << std::endl
    << "  path_to_vocabulary: " << argv[1] << std::endl 
    << "  path_to_setting: " << argv[1] << std::endl
    << "  do_rectify(ture | false): " << argv[2] << std::endl
    << "  use_camera_device (true | false): " << argv[3] << std::endl;

  // Initialize Google's logging library
  // google::InitGoogleLogging("test");
  // google::SetLogDestination(google::INFO, "./");
  // LOG(INFO) << "Found"  << endl;

  Monitor::System SLAM(argv[1]);
  ImageGrabber igb(&SLAM);

  std::string images_pre_path; 
  std::string images_list_file_path; //

  std::stringstream ssrectify(argv[2]);
  ssrectify >>  std::boolalpha >> igb.do_rectify;
  std::cout << "do_rectify: " << igb.do_rectify << std::endl;

  std::stringstream ssdevice(argv[3]);
  ssdevice >>  std::boolalpha >> igb.use_camera_device;
  std::cout << "use_camera_device: " << igb.use_camera_device << std::endl;

  if(igb.do_rectify || !igb.use_camera_device ){

    cv::FileStorage fsSetting(argv[1], cv::FileStorage::READ);
    if (!fsSetting.isOpened()) {
      std::cerr << "error: wrong path to setting" << std::endl;
      return -1;
    }

    if (igb.do_rectify) {
      std::cout << "loading rectify settings..." << std::endl;

      cv::Mat K, P, R, D;
      fsSetting["K"] >> K;

      fsSetting["P"] >> P;

      fsSetting["R"] >> R;

      fsSetting["D"] >> D;

      int rows = fsSetting["height"];
      int cols = fsSetting["width"];

      if(K.empty()  || P.empty() || R.empty()  ||  D.empty() ||
                  rows==0 || cols ==0)	{
        std::cerr << "error: calibration parameters to rectify image are missing!" << std::endl;
        return -1;
      }
  
      cv::initUndistortRectifyMap(
          K,                                // K 相机内参 包括 fx fy cx cy 
          D,                                // D 畸变系数 有 4,5,8,12,或14 个参数，如果是空的，则认为是零畸变系数
          R,                                // R 可选的修正变换矩阵，是3×3 矩阵， 可以是通过 stereoRectify计算的来的R1 R2  如果是空的，就假设为单位矩阵。在cvInitUndistortMap中被认为是单位矩阵
          P.rowRange(0,3).colRange(0,3),    // P 新的相机矩阵
          cv::Size(cols,rows),              // 未畸变的图像尺寸
          CV_32F,                           // 第一个输出的映射的类型， 可以是 CV_32FC1，CV_32FC@, CV_16SC2
          igb.M1,                           // 第一个输出映射
          igb.M2);                          // 第二个输出映射

      std::cout << "loading rectify settings success!" << std::endl;
    }

    if (!igb.use_camera_device){
      std::cout << "loading dataset settings..." << std::endl;

      fsSetting["dataset_images_list"] >> images_list_file_path;
      fsSetting["dataset_images"] >> images_pre_path;

      std::cout << "images_list_file_path:   " << images_list_file_path <<std::endl;
      std::cout << "dataset_images:   " << images_pre_path <<std::endl;
    }

    fsSetting.release();
  }


  if(igb.use_camera_device) { // usecameradevice 

    // config camera device
    Device::CameraDevice camera;
    if(camera.ConnectDevice() == -1){
        // std::cout << "camera Device connect failed.please check camera device" <<std::endl;
        return 0;
    }

    bool isstop = false;
    
    while (flag) {
      cv::Mat image_data; // 创建图像对象 用于存储每一帧的图像
      camera.GetImage(image_data); 

      if (image_data.channels() >= 3) {
        // cv::cvtColor(image_data,image_data,cv::COLOR_RGB2GRAY);
      }

      igb.GrabMono(image_data,0.00001f);
      
      // // 绘制并显示图像
      // imshow("Device",image_data);
      // 绘制并显示图像  
      igb.View(image_data,isstop);

      if(isstop) break;
      // Check if ESC was pressed
      if (cv::waitKey(30) == 27) {
          break;
      }
    }
  }else{  // use dataset
    
    int k = 0;
    std::string framName;
    bool isstop = false;
    // load dataset settings
    ifstream listFramesFile(images_list_file_path); // fream to read images list

    cout<<"******"<<"导航开始"<<"*******"<<endl;
    while ( getline(listFramesFile, framName) ) {
      std::string framPath = images_pre_path + framName;
      cout << "framPath:" << framPath<< endl;
      cout << "当前为第" << " " << k++<<" "<< "帧" <<endl;

      Mat image_data = imread(framPath, 1);
      if ( image_data.data==nullptr ){
        cout << "framPath is nullptr:" << framPath<<  endl; 
        break;
      }

      // cv::Mat reverse;
      // transpose(image_data, reverse);
      // image_data = reverse.clone();

      // 传送图像数据到Monitor系统
      igb.GrabMono(image_data,0.00001f);
      
      // 绘制并显示图像  
      igb.View(image_data,isstop);

      if(isstop) break;
      /////
      sleep(1);
    }
    cout<<"******"<<"导航结束"<<"*******"<<endl;
  }

  SLAM.Shutdown();
  // api->Stop(Source::VIDEO_STREAMING);
  // std::cout << "save camera trajectory..." << std::endl;
  // SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");
  std::cout << "save camera trajectory complete." << std::endl;

  return 0;
}
