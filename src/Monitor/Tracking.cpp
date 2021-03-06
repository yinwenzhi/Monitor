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
    //将旋转矩阵转换为欧拉角 欧拉角顺序为 Z Y X 
    Vec3f rotationMatrixToEulerAngles(Mat &R)
    {
        // float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
        float Cy = sqrt(R.at<double>(2,1) * R.at<double>(2,1) +  R.at<double>(2,2) * R.at<double>(2,2) );


        bool singular = Cy < 1e-6; // If


        float x, y, z;
        if (!singular) {
            x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
            y = atan2(-R.at<double>(2,0), Cy);
            z = atan2(R.at<double>(1,0), R.at<double>(0,0));
        } else {
            x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
            y = atan2(-R.at<double>(2,0), Cy);
            z = 0;
        }
        // x = x*(180/3.1415926);
        // y = y*(180/3.1415926);
        // z = z*(180/3.1415926);
        return Vec3f(z, y, x);
    }

    cv::Vec3f  rotationMatrixToEulerAnglesZYZ(Mat & R){
        float S2 = sqrt(R.at<double>(2,0) * R.at<double>(2,0) +  R.at<double>(2,1) * R.at<double>(2,1) );
        bool ismin = S2 < 1e-6; // If
        float a, b, c;
        if (!ismin) { // S2不接近0
            a = atan2(R.at<double>(1,2) , R.at<double>(0,2));
            b = atan2(S2,R.at<double>(2,2));
            c = atan2(R.at<double>(2,1), -R.at<double>(2,0));
        } else
        {
            a=0;
            b = 0;
            c = atan2(R.at<double>(2,1), -R.at<double>(2,0));
        }
        // a = a*(180/3.1415926);
        // b = b*(180/3.1415926);
        // c = c*(180/3.1415926);
        return Vec3f(a, b, c);
    }


    double Tracking::reprojection_error(const Eigen::Matrix3d R , const Eigen::Vector3d t)
    {
        double sum = 0.0;
        int number_of_correspondences = pts_3d_.size();
        for(int i = 0; i < number_of_correspondences; i++) {
            Eigen::Vector3d pw,pc ;
            pw << pts_3d_[i].x , pts_3d_[i].y, pts_3d_[i].z ;
            pc = R*pw + t;
            double Xc = pc(0);
            double Yc = pc(1);
            double inv_Zc = 1.0 / pc(2);
            double ue = mcamera_->cx_ + mcamera_->fx_ * Xc * inv_Zc;
            double ve = mcamera_->cy_ + mcamera_->fy_ * Yc * inv_Zc;
            double u = pts_2d_[i].x, v = pts_2d_[i].y;
            cout << "pts_2d_[i].x, v = pts_2d_[i].y;"<< pts_2d_[i].x<< pts_2d_[i].y << endl;
            int cursum = sqrt( (u - ue) * (u - ue) + (v - ve) * (v - ve) );
            sum += cursum;
            // for(int i=0 ;i<pw.size(); i++) cout << pw(i) << " " << endl;
            // for(int i=0 ;i<pc.size(); i++) cout << pc(i) << " " << endl;
            cout << "pw: " << pw << endl;
            cout << "uv: " << ue <<"  "<< ve << endl; 
            cout << "error: " << u-ue << "   " << v-ve << endl;
            cout << "cursum: " << cursum << endl;
            cout << "--------------" << endl;
        }
        cout << ">>>  reprojection_error : " << sum << endl;
        return sum / number_of_correspondences;
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
        // Mat reverse ;
        // transpose(img1, reverse);
        // img1 = reverse.clone();
        // 提取基准帧特征点
        // step1 提取特征点
        Monitor::Feature f;
        // f.GetCrossPoint(img1);
        // f.GetORBPoint(img1);
        f.GetKeyPoint(img1);  //提取特征点，使用GetKeyPoint来保证当前真和参考帧相统一
        vector<Point2f> corners1=f._corners;

        int idxnum1=corners1.size();
        cout<<"num of first img's corners is :"<<idxnum1<<endl;
        // 类型转换
        vector<KeyPoint> keypoints_all(idxnum1);
        KeyPoint::convert(corners1, keypoints_all);

        // step2计算描述子
        Mat descriptors_all;
        cv::Ptr<DescriptorExtractor> descriptor = ORB::create();
        //Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
        descriptor->compute (img1, keypoints_all, descriptors_all );
        cout<<"descriptors_all :"<<descriptors_all.size()<<endl;
        
        // step3创建基准帧
        Monitor::Frame::Ptr pF_ini = Monitor::Frame::createFrame();

        pF_ini->color_ = img1;//读取图片
        pF_ini->time_stamp_ = 0;//时间戳
        pF_ini->id_=0;
        // 读入基准帧的3D点坐标（夹板坐标系下）
        vector<KeyPoint> keypoints;//和3d点对应的特征点
        Mat descriptors;//和3d点对应的描述子
        // 从文档中读取3D坐标
        string str_3d = Monitor::Config::get<string> ( "dataset_3d" );
        cout<<"dataset: "<<str_3d<<endl;
        int num;
        int x,y,z;
        
        for ( int i=0; i<keypoints_all.size(); i++ )
        {
            ifstream myfile(str_3d);
            if (!myfile.is_open()) {
                cout << "can not open this file" << endl;
            }
            int width , height;
            int pixX , pixY;
            myfile>> width >> height;
            myfile>> pixX >> pixY;
            while(myfile>>num>>x>>y>>z)
            {
                if(z==-1) continue; // ！z = -1 的点是无效的点
                if(i==num)
                {
                    Point3f p_world;
                    // p_world.x= (x- (pixX%2) == 0 ? (pixX/2) : (pixX/2 +1))*(1.0)*width/pixX ;
                    // p_world.y= (y- (pixY%2) == 0 ? (pixY/2) : (pixX/2 +1))*(1.0)*height/pixY ;
                    p_world.x= (x*(1.0)- pixX/2)*width/pixX*(1.0) ;
                    p_world.y= (y*(1.0)- pixY/2)*height/pixY*(1.0) ;
                    p_world.z= 0 ;
                    cout << "x: " << x<< " y: "<< y << " z: "<< z <<" p_world " << p_world << endl;
                    
                    
                    //pF_ini->pts_3d_ref_.push_back(p_world);
                    pF_ini->pts_3d_ref_[num] = p_world; // 改为map 容易根据序号提取3d点
                    keypoints.push_back(keypoints_all[i]);
                    descriptors.push_back(descriptors_all.row(i).clone());
                    break;
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
        Tracking::pts_3d_map_=pF_ini->pts_3d_ref_;
        Tracking::keypoints_ref_ = keypoints;
        Tracking::keypoints_all_ref_ = keypoints_all;
        Tracking::descriptors_ref_ = descriptors;           // ref 3d keypoint descriptors
        Tracking::descriptors_all_ref_ = descriptors_all;   // ref all keyp
        Tracking::mref_ = pF_ini;

        mcurr_ =  mref_;

        return true;
    }

    // bool Tracking::AddFrame ( Frame::Ptr frame ,double time=0)//计算了每一帧的位姿
    bool Tracking::AddFrame(Monitor::Frame::Ptr  frame )//计算了每一帧的位姿
    {
        // mpTracker->mlast_ = mpTracker-> mcurr_;
        // cout << "**********mpTracker->mlast_->translation_*********" << mpTracker->mlast_->translation_ << endl;
        // mpTracker->mcurr_ = pFrame;

        // mlast_ = Frame::Ptr(&(*mcurr_));
        mlast_ = mcurr_;
        mcurr_ = frame;
        mcurr_->translation_  = mlast_ -> translation_; // 先复制上一帧的平移，防止这一帧计算过程跳出。
        mcurr_->distance_ = mlast_ -> distance_;
        cout << "mlast_ : " << mlast_ << "  mcurr_: " << mcurr_ << endl;
        cout <<"mlast_->distance_: " << mlast_->distance_ << endl;


        // Monitor::Frame::Ptr pF_cur = Monitor::Frame::createFrame();
        #ifdef DEBUG
            cout<<"AddFrame"<<endl;
        #endif
        Feature f;
        f.GetKeyPoint(mcurr_->color_);  //提取特征点
        KeyPoint::convert(f._corners, keypoints_curr_); //类型转换
        mcurr_ -> keypoints_ = keypoints_curr_;
        cout<<"keypoints_curr_.size()"<<keypoints_curr_.size()<<endl;

        boost::timer timer;

        vector<DMatch> match;
        cv::Ptr<DescriptorExtractor> descriptor = ORB::create();

        //当前帧提取描述子
        descriptor->compute ( mcurr_->color_, keypoints_curr_, descriptors_curr_ );
        mcurr_ -> descriptors_ = descriptors_curr_;

        cv::Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
        // 第一步 使用 keypoints_all_ref_ 匹配
        matcher->match ( descriptors_all_ref_, descriptors_curr_, match );
        // 使用 keypoints_all_ref_ 匹配
        // matcher->match ( descriptors_ref_, descriptors_curr_, match );
        cout<<"match.size()= "<< match.size()<<endl;

        double min_dist=10000, max_dist=0;
        //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
        for ( int i = 0; i < match.size(); i++ ) // 应该使用match的数量 而不是descriptors_res_ 的大小
            // for ( int i = 0; i < descriptors_1.rows; i++ )
        {
            double dist = match[i].distance;
            if ( dist < min_dist ) min_dist = dist;
            if ( dist > max_dist ) max_dist = dist;
        }

        printf( "-- Max dist : %f \n", max_dist );
        printf( "-- Min dist : %f \n", min_dist );

        feature_matches_.clear(); //featurematch 信息要用来绘制匹配图
        //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
        // for ( int i = 0; i < descriptors_ref_.rows; i++ )
        for ( int i = 0; i < match.size(); i++ )
        {
            // feature_matches_.push_back ( match[i] );
            if ( match[i].distance <= max ( 2*min_dist, 30.0 ) )
            {
                feature_matches_.push_back ( match[i] );
            }
        }
        cout<<"matches.size()= "<<feature_matches_.size()<<endl;

        // 第二步：使用主方向进行删除匹配点对
        ORBmatcher rotmatcher;

        try{
            rotmatcher.DisBlogeByRot( keypoints_all_ref_, keypoints_curr_,  feature_matches_);
        }catch(exception e){
            cout << "DisBlogeByRot this frame failed,continue.";
            return false;
        }
        cout<<"matches.size()= "<<feature_matches_.size()<<endl;

        // 第三步：  使用ransac
        vector<Point2f> srcPoints(feature_matches_.size()),dstPoints(feature_matches_.size());
        //保存从关键点中提取到的匹配点对的坐标
        for(int i=0;i<feature_matches_.size();i++)
        {
            srcPoints[i]=keypoints_all_ref_[feature_matches_[i].queryIdx].pt;
            dstPoints[i]=keypoints_curr_[feature_matches_[i].trainIdx].pt;
        }
        //保存计算的单应性矩阵
        Mat homography;
        //保存点对是否保留的标志
        vector<unsigned char> inliersMask1(srcPoints.size());
        //匹配点对进行RANSAC过滤
        homography = findHomography(srcPoints,dstPoints,CV_RANSAC,5,inliersMask1);
        //RANSAC过滤后的点对匹配信息
        vector<DMatch> matches_ransac;
        //手动的保留RANSAC过滤后的匹配点对
        for(int i=0;i<inliersMask1.size();i++)
        {
            if(inliersMask1[i])
            {
                matches_ransac.push_back(feature_matches_[i]);
                //cout<<"第"<<i<<"对匹配："<<endl;
                // cout<<"queryIdx:"<<feature_matches_[i].queryIdx<<"\ttrainIdx:"<<feature_matches_[i].trainIdx<<endl;
                //cout<<"imgIdx:"<<matches[i].imgIdx<<"\tdistance:"<<matches[i].distance<<endl;
            }
        }
        feature_matches_ = matches_ransac;
        cout<<"matches.size() after ransac= "<<feature_matches_.size()<<endl;
        // ransanc 结束

        namedWindow("add_frame_match",0);
        cv::resizeWindow("add_frame_match",1280,480);
        cv::Mat add_frame_match;
        drawMatches( 
            mref_->color_,                  // 参考帧图像
            keypoints_all_ref_,           // 因为匹配使用描述子是所有特征点的描述子
            mcurr_->color_,                             // 当前帧图像
            keypoints_curr_, 
            feature_matches_, 
            add_frame_match
        );
        imshow("add_frame_match",add_frame_match);


        // 使用已知3D点的序号来进行提取feature_matches_
        vector<cv::DMatch> good_matchs;
        std::map<int,cv::Point3f>::iterator it=Tracking::pts_3d_map_.begin();

        for(;it!=Tracking::pts_3d_map_.end();it++)
        {
            cout << " pF_ini->pts_3d_ref_ id : " << it->first << " point: "<< it->second << endl;
        }
        it=Tracking::pts_3d_map_.begin();

        int matchindex = 0; 
        // 对每一个匹配对判断 是不是 3d点序列的一个序号
        while(matchindex<feature_matches_.size() && it!=Tracking::pts_3d_map_.end()){
            // cout <<"queryIdx:"<<feature_matches_[matchindex].queryIdx<<"    cur3d->first: " << it->first  << endl;
            if(feature_matches_[matchindex].queryIdx < it->first) matchindex++;
            else if (feature_matches_[matchindex].queryIdx > it->first) it++;
            else {
                good_matchs.push_back(feature_matches_[matchindex]);
                matchindex++;
                it++;
            }
        }
        feature_matches_ = good_matchs;
        cout<<"matches.size() after 3d= "<<feature_matches_.size()<<endl;
        mcurr_-> feature_matches_ = feature_matches_;
        //计算当前帧位姿
        Mat R,t;
        if(!pose_estimation_3d2d(feature_matches_,R, t)){
            state_ = LOST;
            return false;
        }
        //筛选匹配到的当前帧的特征点和描述子，
        // Mat desp_tem;
        // vector<cv::KeyPoint> kp_temp;
        // vector< Point3f >  temp_3d=pts_3d_;          // 参考帧3D点
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
        // feature_matches_.clear(); featurematch 信息要用来绘制匹配图所以应该在最开始的地方清空
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

    bool Tracking:: pose_estimation_3d2d (std::vector<DMatch> matches,Mat& R, Mat& t )
    {
        cout<<"ref_->pts_3d_ref_.size:"<<pts_3d_map_.size()<<endl;
        //Mat K = f_cur.camera_->K;
        Mat K = ( Mat_<double> ( 3,3 ) << mcamera_->fx_, 0, mcamera_->cx_, 0, mcamera_->fy_, mcamera_->cy_, 0, 0, 1 );
        Mat D = ( Mat_<float> ( 5,1 ) << mcamera_->k1_, mcamera_->k2_, 0, mcamera_->p1_, mcamera_->p2_ );
        vector<Point3f> pts_3d;
        vector<Point2f> pts_2d;
        for ( DMatch m:matches )
        {
            // 这里使用匹配点在参考帧上的序号来获得3D点的序号，所以 pts_3d_map_ 保存为map ，通过序号获得坐标值
            Point3f p1 = pts_3d_map_[m.queryIdx]; 
            pts_3d.push_back (p1);
            pts_2d.push_back ( keypoints_curr_[m.trainIdx].pt );
            std::cout << "3d， id： " << m.queryIdx << " point: " << p1 <<  "  2d.point: "<< keypoints_curr_[m.trainIdx].pt << endl;
            // cout<<"m.trainIdx= "<<m.trainIdx<<endl; //  cur
            // cout<<"m.queryIdx= "<<m.queryIdx<<endl; //  ref
            // cout<<"3d= "<<p1<<endl;
            // cout<<"2d= "<<keypoints_curr_[m.trainIdx].pt<<endl;
        }

        int minp3dSize = 5;
        if( pts_3d.size() < minp3dSize) {
            cout<<"3d-2d pairs : "<<pts_3d.size() << " < " << minp3dSize <<" , skip this frame." << endl;
            Tracking::pts_3d_.clear();
            Tracking::pts_2d_.clear();
            return false;
        }else{
            cout << "3d-2d pairs : " << pts_3d.size()  << endl;
            Tracking::pts_3d_ = pts_3d;
            Tracking::pts_2d_ = pts_2d;
            // cv::imwrite("matches.png", image_show);
        }

        solvePnP( pts_3d, pts_2d, K, D, R, t, false, SOLVEPNP_EPNP ); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
        // solvePnP(pts_3d, pts_2d, K, D, R, t, false, CV_ITERATIVE ); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
        // solvePnP(pts_3d, pts_2d, K, Mat(), R, t, false, CV_ITERATIVE )
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
        if( !bundleAdjustment ( pts_3d, pts_2d, K, R, t ) ) return false; 


        cout << "afterBA R="<< endl << R <<endl;
        // cout<<"afterBA t="<< endl << t <<endl;
        // Mat rotation1;
        // cv::eigen2cv(rotation,rotation1);
 
        // Tracking::angle_ = rotationMatrixToEulerAngles(rotation1);
        Tracking::angle_ = rotationMatrixToEulerAngles(R);
        cout << "EulerAngles1_: "<<angle_<<endl;
        cout << "EulerAngles2_: "<<angle_*52.7<<endl;
        

        

        return true;
    }

    bool  Tracking::bundleAdjustment (
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
        // optimizer.setVerbose ( true ); // 打开优化输出
        optimizer.initializeOptimization();
        optimizer.optimize ( 100 );
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2-t1 );
        cout<<"optimization costs time: "<<time_used.count() <<" seconds."<<endl;
        
        T_esti=Eigen::Isometry3d ( pose->estimate() ).matrix();

        Eigen::Matrix4d tt=T_esti.matrix();
        // std::cout << "tt: " <<std::endl << tt<< std::endl;
        Eigen::Matrix3d rotation = tt.block(0,0,3,3);
        Eigen::Vector3d trans = tt.block(0,3,3,1);
        
        //重投影误差
        // reprojection_error(rotation,trans);
        if (reprojection_error(rotation,trans)>20.0) {
            cout << "\033[1;33m>>> reprojection_error too large, continue.\033[0m"  << endl;
            return false;
        }

        // Eigen::Vector3d trans=tt.block(0,3,3,0);
        // Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> trans=tt.block(0,3,3,1);
        // Eigen::Matrix3d temp =  rotation.inverse();
        // rotation = temp;
        cout<<"translation_ before: "<<trans(0) << " " << trans(1) << " "<<  trans(2) ;
        trans = (-1)*(rotation.inverse()*trans); // 坐标系变换  
        Tracking::translation_ = trans; // 更新tracking 后续将删除，只保留frame中的
        mcurr_ -> translation_ = trans; // 更新frame
        cout <<" -->  translation_ after: "<<translation_(0) << " " << translation_(1) << " "<<  translation_(2)<<endl;

        // 平滑
        double s1 = 0.2, s2 = 0.8;
        // translation_(0) = trans(0)*s1 + mlast_->translation_(0)*s2;
        // translation_(1) = trans(1)*s1 + mlast_->translation_(1)*s1;
        // translation_(2) = trans(2)*s1 + mlast_->translation_(2)*s1;
        // cout <<"mcurr_->translation_: " << mcurr_->translation_ << endl;
        // cout <<"mlast_->translation_: " << mlast_->translation_ << endl;
        double distance = sqrt(translation_(0)*translation_(0)+translation_(1)*translation_(1)+translation_(2)*translation_(2));
        cout <<"distance: filter before: "<< distance <<endl;
        distance = distance*s1 + mlast_->distance_*s2;
        cout <<"distance: filter after : "<< distance <<endl;

        mcurr_-> distance_ = distance;
        cv::eigen2cv(rotation,R);
        cv::eigen2cv(trans,t);
        // cout<<endl<<"after optimization:"<<endl;
        // cout<<"T="<<endl<<Eigen::Isometry3d ( pose->estimate() ).matrix() <<endl;
        return true;
 
    }


}//namespace Monitor