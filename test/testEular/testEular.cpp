#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;
//将旋转矩阵转换为欧拉角 欧拉角顺序为 
Vec3f rotationMatrixToEulerAnglesXaYbZc(Mat &R)
{
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
    bool singular = sy < 1e-6; // If
    float a,b,c;
    if (!singular) {
        a = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        b = atan2(-R.at<double>(2,0), sy);
        c = atan2(R.at<double>(1,0), R.at<double>(0,0));
    } else {
        a = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        b = atan2(-R.at<double>(2,0), sy);
        c = 0;
    }
    a = a*(180/3.1415926);
    b = b*(180/3.1415926);
    c = c*(180/3.1415926);
    return Vec3f(a, b, c);
}
cv::Vec3f  rotationMatrixToEulerAnglesZaYbXc(Mat &R){
    // float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
    float sy = sqrt(R.at<double>(2,1) * R.at<double>(2,1) +  R.at<double>(2,2) * R.at<double>(2,2) );
    bool singular = sy < 1e-6; // If
    float a, b, c;
    if (!singular) {
        a = atan2(R.at<double>(1,0) , R.at<double>(0,0));
        b = atan2(-R.at<double>(2,0), sy);
        c = atan2(R.at<double>(2,1), R.at<double>(2,2));
    } else {
        a = atan2(R.at<double>(1,0) , R.at<double>(0,0));
        b = 0;
        c = atan2(R.at<double>(2,1), R.at<double>(2,2));;
    }
    a = a*(180/3.1415926);
    b = b*(180/3.1415926);
    c = c*(180/3.1415926);
    return Vec3f(a, b, c);
}
cv::Vec3f  rotationMatrixToEulerAnglesZaYbZc(Mat & R){
    float S2 = sqrt(R.at<double>(2,0) * R.at<double>(2,0) +  R.at<double>(2,1) * R.at<double>(2,1) );
    bool ismin = S2 < 1e-6; // If
    float a, b, c;
    if (!ismin) { // S2不接近0
        b = atan2(S2,R.at<double>(2,2));
        a = atan2(R.at<double>(1,2) , R.at<double>(0,2));
        c = atan2(R.at<double>(2,1), -R.at<double>(2,0));
    } 
    a = a*(180/3.1415926);
    b = b*(180/3.1415926);
    c = c*(180/3.1415926);
    return Vec3f(a, b, c);
}
int main(){
    
    Mat R  = Mat::eye(3,3,CV_64F);
    //
    R.at<double> ( 0,0 ) = -0.9496;
    R.at<double> ( 0,1 ) = -0.2016;
    R.at<double> ( 0,2 ) = -0.23965;
    R.at<double> ( 1,0 ) = -0.21430;
    R.at<double> ( 1,1 ) = -0.13966;
    R.at<double> ( 1,2 ) = 0.96673;
    R.at<double> ( 2,0 ) = -0.22839;
    R.at<double> ( 2,1 ) = 0.969;
    R.at<double> ( 2,2 ) = 0.08942;
    // 
    // R.at<double> ( 0,0 ) = 0.9021;
    // R.at<double> ( 0,1 ) = -0.3836;
    // R.at<double> ( 0,2 ) = 0.1977;
    // R.at<double> ( 1,0 ) = 0.3875;
    // R.at<double> ( 1,1 ) = 0.9216;
    // R.at<double> ( 1,2 ) = 0.0198;
    // R.at<double> ( 2,0 ) = -0.1898;
    // R.at<double> ( 2,1 ) = 0.0587;
    // R.at<double> ( 2,2 ) = 0.9801;
    // R.at<double> ( 0,0 ) = 0.866;
    // 一验证
    // R.at<double> ( 0,1 ) = 0;
    // R.at<double> ( 0,2 ) = 0.5;
    // R.at<double> ( 1,0 ) = 0.433;
    // R.at<double> ( 1,1 ) = 0.5;
    // R.at<double> ( 1,2 ) = -0.75;
    // R.at<double> ( 2,0 ) = -0.25;
    // R.at<double> ( 2,1 ) = 0.866;
    // R.at<double> ( 2,2 ) = 0.433;
    cv::Vec3f XYZ;
    XYZ = rotationMatrixToEulerAnglesXaYbZc(R);
    cout << " X-Y-Z:" << XYZ[0] << " " <<XYZ[1]  << " "  <<XYZ[2] << endl;
    XYZ = rotationMatrixToEulerAnglesZaYbXc(R);
    cout << " Z-Y-X: " << XYZ[0] << " " <<XYZ[1]  << " "  <<XYZ[2] << endl;
    XYZ = rotationMatrixToEulerAnglesZaYbZc(R);
    cout << " Z-Y-Z: " << XYZ[0] << " " <<XYZ[1]  << " "  <<XYZ[2] << endl;;
}