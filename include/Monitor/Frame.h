//
// Created by lizechuan on 20-7-13.
//

#ifndef FRAME_H
#define FRAME_H
// forward declare
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "common_include.h"
#include "Camera.h"
namespace Monitor
{
class MapPoint;
class Frame
{
public:
    typedef std::shared_ptr<Frame> Ptr;
    unsigned long                  id_;             // id of this frame
    double                         time_stamp_;     // when it is recorded
    // SE3                            T_c_w_;       // transform from world to camera
    Camera::Ptr                    camera_;         // Pinhole RGBD Camera model
    Mat                            color_;          // color and depth image
    // vector< Point3f >              pts_3d_ref_;
    map<int,Point3f>               pts_3d_ref_;

    vector<cv::DMatch>      feature_matches_;   // feature matches
    vector<cv::KeyPoint>    keypoints_;         // keypoints in  frame
    cv::Mat                 descriptors_;       // descriptor in  frame
    vector<cv::Point2f>     pts_2d_;                 // 参考帧中3d点对应的当前帧2d像素坐标
    // Eigen::Isometry3d       T_esti;                  // 当前帧相机欧式变换矩阵 BA优化后
    Eigen::Vector3d         translation_;            // 当前帧相机在世界坐标系中的平移向量
    cv::Vec3f               angle_;                  // 当前帧欧拉角
    double                  distance_;


    // std::vector<cv::KeyPoint>      keypoints_;  // key points in image
    // std::vector<MapPoint*>         map_points_; // associated map points
    // bool                           is_key_frame_;  // whether a key-frame

public: // data members
    Frame();
    // Frame( long id, double time_stamp=0, SE3 T_c_w=SE3(), Camera::Ptr camera=nullptr, Mat color=Mat() );
    Frame( long id);
    ~Frame();

    static Frame::Ptr createFrame();


    // Get Camera Center
    // Vector3d getCamCenter() const;

    // void setPose( const SE3& T_c_w );

};

}// namespace Monitor
#endif //FRAME_H
