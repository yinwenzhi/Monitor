//
// Created by lizechuan on 20-7-13.
//
#include "../../include/Monitor/Tracking.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <boost/timer.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d.hpp>
// #include <opencv2/core/eigen.hpp>
// #include <Eigen/Core>
// #include <Eigen/Geometry>
// #include <g2o/core/base_vertex.h>
// #include <g2o/core/base_unary_edge.h>
// #include <g2o/core/block_solver.h>
// #include <g2o/core/optimization_algorithm_levenberg.h>
// #include <g2o/solvers/csparse/linear_solver_csparse.h>
// #include <g2o/types/sba/types_six_dof_expmap.h>
using namespace std;
using namespace cv;
namespace Monitor
{
    //将旋转矩阵转换为欧拉角
    Vec3f rotationMatrixToEulerAngles(Mat &R)
    {

        float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );


        bool singular = sy < 1e-6; // If


        float x, y, z;
        if (!singular) {
            x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
            y = atan2(-R.at<double>(2,0), sy);
            z = atan2(R.at<double>(1,0), R.at<double>(0,0));
        } else {
            x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
            y = atan2(-R.at<double>(2,0), sy);
            z = 0;
        }
        return Vec3f(x, y, z);
    }
    
    // Tracking::Tracking(std::shared_ptr<Monitor::Config>  ConfigInstance){
    Tracking::Tracking(Config::Ptr ConfigInstance){
        //初始化基准帧图片信息
        mbase_img_path = ConfigInstance->get<string> ("dataset_first");
        // std::cout << "base frame image path： "<<  mbase_img_path << std::endl;
        mbbase_initialed = Tracking::InitFrame(mbase_img_path);
        Tracking::mindex = 0;   // use to set frame id 
    }

    void Tracking::setCameraModel(Monitor::Camera::Ptr pCamera){
        Tracking::mcamera_ = pCamera;
    }

    bool Tracking::InitFrame(string imgPath){

        Mat img1 = imread(imgPath, 1);
        // 提取基准帧特征点
        // step1 提取特征点
        Monitor::Feature f;
        // f.GetCrossPoint(img1);
        f.GetORBPoint(img1);
        vector<Point2f> corners1=f._corners;

        int idxnum1=corners1.size();
        cout<<"num of first img's corners is :"<<idxnum1<<endl;
        // //类型转换
        vector<KeyPoint> keypoints1(idxnum1);
        KeyPoint::convert(corners1, keypoints1);

        // step2计算描述子

        Mat descriptors_1;
        cv::Ptr<DescriptorExtractor> descriptor = ORB::create();
        //Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
        descriptor->compute (img1, keypoints1, descriptors_1 );
        cout<<"descriptors_1 :"<<descriptors_1.size()<<endl;
        
        // step3创建基准帧
        Monitor::Frame::Ptr pF_ini = Monitor::Frame::createFrame();

        pF_ini->color_ = img1;//读取图片
        pF_ini->time_stamp_ = 0;//时间戳
        pF_ini->id_=0;
        // //  读入基准帧的3D点坐标（夹板坐标系下）
        vector<KeyPoint> keypoints;//和3d点对应的特征点
        Mat descriptors;//和3d点对应的描述子
        // //从文档中读取3D坐标
        string str_3d = Monitor::Config::get<string> ( "dataset_3d" );
        cout<<"dataset: "<<str_3d<<endl;
        size_t num;
        int x,y,z;
        for ( size_t i=0; i<keypoints1.size(); i++ )
        {
            ifstream myfile(str_3d);
            if (!myfile.is_open()) {
                cout << "can not open this file" << endl;
            }
            while(myfile>>num>>x>>y>>z)
            {
                if(i==num)
                {
                    Point3f p_world;
                    p_world.x=x;
                    p_world.y=y;
                    p_world.z=z;
                    pF_ini->pts_3d_ref_.push_back(p_world);
                    keypoints.push_back(keypoints1[i]);
                    descriptors.push_back(descriptors_1.row(i).clone());
                }
                else
                {
                    continue;
                }
            }
        }

        // 状态信息输出
        cout<<"keypoints :"<<keypoints.size()<<endl;
        cout<<"3d point :"<<pF_ini->pts_3d_ref_.size()<<endl;
        cout<<"descriptors :"<<descriptors.size()<<endl;

        // 设置Tracking 属性
        Tracking::pts_3d_=pF_ini->pts_3d_ref_;
        Tracking::descriptors_ref_=descriptors;
        Tracking::mref_=pF_ini;
        return true;
    }

    // bool Tracking::AddFrame ( Frame::Ptr frame ,double time=0)//计算了每一帧的位姿
    bool Tracking::AddFrame (Monitor::Frame::Ptr  frame )//计算了每一帧的位姿
    {
        #ifdef DEBUG
            cout<<"AddFrame"<<endl;
        #endif
        cout<<"AddFramebeging"<<endl;
        // mcurr_ = frame;
        cout<<"featurebeging"<<endl;
        Feature f;
        cout<<"getCrossbeging"<<endl;
        f.GetKeyPoint(mcurr_->color_);  //提取交点
        KeyPoint::convert(f._corners, keypoints_curr_);//类型转换
        cout<<"keypoints_curr_.size()"<<keypoints_curr_.size()<<endl;

        boost::timer timer;

        vector<DMatch> match;
        cv::Ptr<DescriptorExtractor> descriptor = ORB::create();

        //当前帧提取描述子
        descriptor->compute ( mcurr_->color_, keypoints_curr_, descriptors_curr_ );
        cv::Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
        matcher->match ( descriptors_ref_, descriptors_curr_, match );
        cout<<"match.size()= "<<match.size()<<endl;

        double min_dist=10000, max_dist=0;
        //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
        for ( int i = 0; i < descriptors_ref_.rows; i++ )
            // for ( int i = 0; i < descriptors_1.rows; i++ )
        {
            double dist = match[i].distance;
            if ( dist < min_dist ) min_dist = dist;
            if ( dist > max_dist ) max_dist = dist;
        }

        printf ( "-- Max dist : %f \n", max_dist );
        printf ( "-- Min dist : %f \n", min_dist );

        //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
        for ( int i = 0; i < descriptors_ref_.rows; i++ )
        {
            if ( match[i].distance <= max ( 2*min_dist, 30.0 ) )
            {
                feature_matches_.push_back ( match[i] );
            }
        }
        cout<<"matches.size()= "<<feature_matches_.size()<<endl;
    //    featureMatching();

        //计算当前帧位姿
        Mat R,t;
        if(!pose_estimation_3d2d(feature_matches_,R, t)){
            return false;
        }
        //筛选匹配到的当前帧的特征点和描述子，
        // Mat desp_tem;
        // vector<cv::KeyPoint> kp_temp;
        // vector< Point3f >  temp_3d=pts_3d_;
        // pts_3d_.clear();
        // for ( size_t i=0; i<keypoints_curr_.size(); i++ )
        // {
        //     for (size_t m = 0; m < feature_matches_.size(); m++)
        //     {
        //         if(i==feature_matches_[m].trainIdx)
        //         {
        //             cv::Point3f pp1;
        //             pp1=temp_3d[feature_matches_[m].queryIdx];
        //             // cout<<"pp1.p3= "<<pp1.p3<<endl;
        //             pts_3d_.push_back(pp1);
        //             // desp_tem.push_back(descriptors_curr_.row(i).clone());
        //             kp_temp.push_back(keypoints_curr_[i]);
        //         }
        //         else
        //         {
        //             continue;
        //         }
        //     }
        // }
        feature_matches_.clear();
        descriptors_curr_.release();
        // keypoints_curr_.clear();
        // descriptors_ref_=desp_tem; // 注释原因：不更新参考帧 所以也不更新描述子 

        // 并将当前帧设为参考帧  
        // mref_ = mcurr_;
        // desp_tem.release(); // 注释原因：取消更新参考帧，因为定位Logo 的相机移动并不大。

        cout<<"match cost time: "<<timer.elapsed()<<endl;
        return true;
    }

    void Tracking::computeDescriptors()
    {
        cv::Ptr<DescriptorExtractor> descriptor = ORB::create();
        cv:: Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
        descriptor->compute ( mcurr_->color_, keypoints_curr_, descriptors_curr_ );
        cout<<123<<endl;
        boost::timer timer;
        cout<<123<<endl;
        orb_->compute ( mcurr_->color_, keypoints_curr_, descriptors_curr_ );
        cout<<"descriptor computation cost time: "<<timer.elapsed()<<endl;
    }

    void Tracking::featureMatching()
    {
        boost::timer timer;
        vector<cv::DMatch> matches;
        matcher_flann_.match( descriptors_ref_, descriptors_curr_, matches );
        // select the best matches
        cout<<123<<endl;
        float min_dis = std::min_element (
                matches.begin(), matches.end(),
                [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
                {
                    return m1.distance < m2.distance;
                } )->distance;

        feature_matches_.clear();
        for ( cv::DMatch& m : matches )
        {
            if ( m.distance < max<float> ( min_dis*2, 30.0 ) )
            {
                feature_matches_.push_back(m);
            }
        }
        cout<<"good matches: "<<feature_matches_.size()<<endl;
        cout<<"match cost time: "<<timer.elapsed()<<endl;
    }

    bool Tracking:: pose_estimation_3d2d (std::vector< DMatch > matches,Mat& R, Mat& t )
    {
        cout<<"ref_->pts_3d_ref_.size:"<<pts_3d_.size()<<endl;
        //Mat K = f_cur.camera_->K;
        Mat K = ( Mat_<double> ( 3,3 ) << mcamera_->fx_, 0, mcamera_->fy_, 0, mcamera_->cx_, mcamera_->fy_, 0, 0, 1 );
        vector<Point3f> pts_3d;
        vector<Point2f> pts_2d;
        for ( DMatch m:matches )
        {
            Point3f p1 = pts_3d_[m.queryIdx];
            pts_3d.push_back (p1);
            pts_2d.push_back ( keypoints_curr_[m.trainIdx].pt );
            cout<<"m.trainIdx= "<<m.trainIdx<<endl; // ref
            cout<<"m.queryIdx= "<<m.queryIdx<<endl; // cur
            cout<<"3d= "<<p1<<endl;
            cout<<"2d= "<<keypoints_curr_[m.trainIdx].pt<<endl;
        }

        
        if(pts_3d.size()<4) {
            return false;
            cout<<"3d-2d pairs  "<<pts_3d.size() << " < 4 , skip this frame." << endl;
        }
        // solvePnP ( pts_3d, pts_2d, K, Mat(), R, t, false, SOLVEPNP_EPNP ); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
        solvePnP ( pts_3d, pts_2d, K, Mat(), R, t, false, CV_ITERATIVE ); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法

        cv::Rodrigues ( R, R ); // r为旋转向量形式，用Rodrigues公式转换为矩阵

        // cout<<"R="<<endl<<R<<endl;
        // cout<<"t="<<endl<<t<<endl;
        bundleAdjustment ( pts_3d, pts_2d, K, R, t );
        std::cout << "R: " << std::endl << R << std::endl;
        return true;
    }

    void  Tracking::bundleAdjustment (
            const vector< Point3f > points_3d,
            const vector< Point2f > points_2d,
            const Mat& K,
            Mat& R, Mat& t )
    {
        // 初始化g2o
        typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose 维度为 6, landmark 维度为 3
        // Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>(); // 线性方程求解器
        // g2o接口更改
        std::unique_ptr<Block::LinearSolverType> linearSolver ( new g2o::LinearSolverCSparse<Block::PoseMatrixType>());

        // Block* solver_ptr = new Block ( linearSolver );     // 矩阵块求解器
        // g2o接口更改
        std::unique_ptr<Block> solver_ptr ( new Block ( std::move(linearSolver)));     // 矩阵块求解器

        // g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
        // g2o接口更改
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::move(solver_ptr));
        
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm ( solver );

        // vertex
        g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
        Eigen::Matrix3d R_mat;
        R_mat <<
              R.at<double> ( 0,0 ), R.at<double> ( 0,1 ), R.at<double> ( 0,2 ),
                R.at<double> ( 1,0 ), R.at<double> ( 1,1 ), R.at<double> ( 1,2 ),
                R.at<double> ( 2,0 ), R.at<double> ( 2,1 ), R.at<double> ( 2,2 );
        pose->setId ( 0 );
        pose->setEstimate ( g2o::SE3Quat (
                R_mat,
                Eigen::Vector3d ( t.at<double> ( 0,0 ), t.at<double> ( 1,0 ), t.at<double> ( 2,0 ) )
        ) );
        optimizer.addVertex ( pose );

        int index = 1;
        for ( const Point3f p:points_3d )   // landmarks
        {
            g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
            point->setId ( index++ );
            point->setEstimate ( Eigen::Vector3d ( p.x, p.y, p.z ) );
            point->setMarginalized ( true ); // g2o 中必须设置 marg 参见第十讲内容
            optimizer.addVertex ( point );
        }

        // parameter: camera intrinsics
        g2o::CameraParameters* camera = new g2o::CameraParameters (
                K.at<double> ( 0,0 ), Eigen::Vector2d ( K.at<double> ( 0,2 ), K.at<double> ( 1,2 ) ), 0
        );
        camera->setId ( 0 );
        optimizer.addParameter ( camera );

        // edges
        index = 1;
        for ( const Point2f p:points_2d )
        {
            g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
            edge->setId ( index );
            edge->setVertex ( 0, dynamic_cast<g2o::VertexSBAPointXYZ*> ( optimizer.vertex ( index ) ) );
            edge->setVertex ( 1, pose );
            edge->setMeasurement ( Eigen::Vector2d ( p.x, p.y ) );
            edge->setParameterId ( 0,0 );
            edge->setInformation ( Eigen::Matrix2d::Identity() );
            optimizer.addEdge ( edge );
            index++;
        }

        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        // optimizer.setVerbose ( true );
        optimizer.initializeOptimization();
        optimizer.optimize ( 100 );
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2-t1 );
        cout<<"optimization costs time: "<<time_used.count() <<" seconds."<<endl;
        
        T_esti=Eigen::Isometry3d ( pose->estimate() ).matrix();
        Eigen::Matrix4d tt=T_esti.matrix();
        // std::cout << "tt: " <<std::endl << tt<< std::endl;
        Eigen::Matrix3d rotation=tt.block(0,0,3,3);
        Eigen::Vector3d trans=tt.block(0,3,3,1);
        // Eigen::Vector3d trans=tt.block(0,3,3,0);
        // Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> trans=tt.block(0,3,3,1);
        // std::cout << "rotation: " << std::endl << rotation << std::endl;
        // std::cout << "trans: " << std::endl << trans<< std::endl;
        Mat rotation1;
        cv::eigen2cv(rotation,rotation1);
        angle_= rotationMatrixToEulerAngles(rotation1);
        translation_=trans;
        
        cout<<"angle: "<<angle_<<endl;
        // cout << "size of translation_: " << translation_.size()<< endl;
        cout<<"translation_ : "<<translation_(0) << " " << translation_(1) << " "<<  translation_(2)<<endl;
        // cout<<endl<<"after optimization:"<<endl;
        // cout<<"T="<<endl<<Eigen::Isometry3d ( pose->estimate() ).matrix() <<endl;
    }
}//namespace Monitor