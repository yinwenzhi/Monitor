//
// Created by lizechuan on 20-7-13.
//
// #include "common_include.h"
#include "Frame.h"
#include "Feature.h"
#include "Camera.h"
#include "Config.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#ifndef TRACKING_H
#define TRACKING_H

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
        typedef shared_ptr<Tracking> Ptr;
        enum VOState {
            INITIALIZING=-1,
            OK=0,
            LOST
        };

        VOState     state_;
        cv::Ptr<cv::ORB> orb_;                      // orb detector and computer
        vector<cv::Point3f>     pts_3d_ref_;        // 3d points in reference frame
        vector<cv::KeyPoint>    keypoints_curr_;    // keypoints in current frame
        Mat                     descriptors_curr_;  // descriptor in current frame
        Mat                     descriptors_ref_;   // descriptor in reference frame
        vector<cv::DMatch>      feature_matches_;   // feature matches
        cv::FlannBasedMatcher   matcher_flann_;     // flann matcher
        Frame * firstf_;
        Frame::Ptr mref_;                           // reference key-frame
        Frame::Ptr mcurr_;                          // current frame
        Frame::Ptr mpCurrentFrame;
        Frame::Ptr mprefFrame;
        
        vector< Point3f >              pts_3d_;     // Tracking->pts_3d_=pF_ini->pts_3d_ref_;
        Camera::Ptr                    mcamera_;
        Eigen::Isometry3d              T_esti;      // 欧式变换矩阵
        cv::Vec3f                      angle_;      // 欧拉角
        Eigen::Vector3d                translation_;
        string mbase_img_path;
        bool mbbase_initialed;
        size_t mindex;   // use to set frame id 
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
        void bundleAdjustment (
                const vector< Point3f > points_3d,
                const vector< Point2f > points_2d,
                const Mat& K,
                Mat& R, Mat& t );

    };
} //namespace Monitor
#endif //TRACKING_H
