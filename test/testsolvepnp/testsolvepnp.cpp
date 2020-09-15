#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d.hpp>
using namespace cv;
using namespace std;
int main(){

        Mat K = ( Mat_<double> ( 3,3 ) << 5059.9, 0, 599.9, 0, 5055.1, 396.508, 0, 0, 1 );
        vector<Point3f> pts_3d;
        vector<Point2f> pts_2d;

        //将控制点在世界坐标系的坐标压入容器
        Mat objM;
        pts_3d.clear();
        pts_3d.push_back(Point3f(0.698117, 55.9588, 0));
        pts_3d.push_back(Point3f(-71.369, -31.8313, 0));
        pts_3d.push_back(Point3f(0.0537013, 44.7778, 0));
        pts_3d.push_back(Point3f(-34.154, -6.68724, 0));
        // Mat(pts_3d).convertTo(objM, CV_32F);
        
        pts_2d.push_back(Point2f(589.732, 935.609));
        pts_2d.push_back(Point2f(268.739, 510.603));
        pts_2d.push_back(Point2f(591.225, 863.547));
        pts_2d.push_back(Point2f(426.399, 627.057));
   

        
        if(pts_3d.size()<4) {
            cout<<"3d-2d pairs : "<<pts_3d.size() << " < 4 , skip this frame." << endl;
            return false;
        }else{
            cout << "3d-2d pairs : " << pts_3d.size()  << endl;
        }

        Mat R;
        Mat t;
        // solvePnP ( pts_3d, pts_2d, K, Mat(), R, t, false, SOLVEPNP_EPNP ); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
        solvePnP ( pts_3d, pts_2d, K, Mat(), R, t, false, CV_ITERATIVE ); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
        std::cout << "旋转向量: " << std::endl << R << std::endl;
        cv::Rodrigues ( R, R ); // r为旋转向量形式，用Rodrigues公式转换为矩阵

        // R = ( cv::Mat_<double> ( 3,3 ) << 1, 0, 0,
        //       0, 1, 0,
        //       0, 0, 1
        // );
        // t = ( cv::Mat_<double> ( 3,1 ) << 0, 0 , -1);
        // t << 0, 0 , -1;
        cout<<"beforeBA R=" << R <<endl;
        cout<<"beforeBA t="<< endl << t <<endl;
        return 0;
}