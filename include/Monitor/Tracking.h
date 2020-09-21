//
// Created by lizechuan on 20-7-13.
//
#ifndef TRACKING_H
#define TRACKING_H
#include "Frame.h"
#include "Feature.h"
#include "Camera.h"
#include "Config.h"
#include "ORBmatcher.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

using Eigen::Vector2d;
using Eigen::Vector3d;

namespace Monitor
{
    typedef struct Point{
        KeyPoint p1;
        Mat p2;
        Point3f p3;
    }pointx;

    class Tracking
    {
    public:
        Camera::Ptr mcamera_;  
        Frame::Ptr mpCurrentFrame;  
        string mbase_img_path;
        bool mbbase_initialed;
        size_t mindex;   // use to set frame id 
        
        cv::FlannBasedMatcher   matcher_flann_;       // flann matcher
        vector<cv::DMatch>      feature_matches_;     // feature matches
        // 参考帧
        vector<cv::KeyPoint>    keypoints_all_ref_;   // keypoints (all) in ref frame
        vector<cv::KeyPoint>    keypoints_ref_;       // keypoints (just 3d point) in ref frame
        cv::Mat                 descriptors_all_ref_; // descriptor in reference frame for all keypoint
        cv::Mat                 descriptors_ref_;     // descriptor in reference frame for 3d keypoint
        map<int,cv::Point3f>    pts_3d_map_;   // 3d点在参考帧特征点中的序号以及坐标
        vector<cv::Point3f>     pts_3d_;       // 3d点坐标
        
        // last
        Eigen::Vector3d         lasttranslation_;

        // 当前帧
        vector<cv::KeyPoint>    keypoints_curr_;      // keypoints in current frame
        cv::Mat                 descriptors_curr_;    // descriptor in current frame
        vector<cv::Point2f>     pts_2d_;       // 参考帧中3d点对应的当前帧2d像素坐标
        Eigen::Isometry3d       T_esti;        // 当前帧相机欧式变换矩阵 BA优化后
        Eigen::Vector3d         translation_;  // 当前帧相机在世界坐标系中的平移向量
        cv::Vec3f               angle_;        // 当前帧欧拉角


        // others not used
        typedef shared_ptr<Tracking> Ptr;
        enum VOState {
            INITIALIZING=-1,
            OK=0,
            LOST
        };

        VOState     state_;
        cv::Ptr<cv::ORB> orb_;                        // orb detector and computer
        // vector<cv::Point3f>     pts_3d_ref_;          // 3d points in reference frame
        // unordered_map<int,cv::Point3f>    pts_3d_ref_;
        Frame * firstf_;  
        Frame::Ptr mref_;                             // reference key-frame
        Frame::Ptr mlast_;
        Frame::Ptr mcurr_;                            // current frame
        Frame::Ptr mprefFrame;  
          
        // vector< Point3f >              pts_3d_;       // Tracking->pts_3d_=pF_ini->pts_3d_ref_;
    public:
        // Tracking(Monitor::Config * ConfigInstance);
        Tracking(std::shared_ptr<Monitor::Config> ConfigInstance);
        void setCameraModel(Camera::Ptr mpCamera);
        bool InitFrame(string imgPath);
        void GrabImageMonocular(const cv::Mat &im,double timestamp);
        // bool AddFrame( Frame::Ptr frame,double timestamp );      // add a new frame
        bool AddFrame( Frame::Ptr frame );      // add a new frame
        void computeDescriptors();
        void featureMatching();
        bool pose_estimation_3d2d ( std::vector< DMatch > matches,Mat& R, Mat& t );
        bool bundleAdjustment (
                const vector< Point3f > points_3d,
                const vector< Point2f > points_2d,
                const Mat& K,
                Mat& R, Mat& t );
        double reprojection_error(const Eigen::Matrix3d R , const Eigen::Vector3d t);

    };
} //namespace Monitor
#endif //TRACKING_H
