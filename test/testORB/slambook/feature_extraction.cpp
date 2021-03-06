#include "ORBmatcher.h"

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;
using namespace Monitor;

int main ( int argc, char** argv )
{
    if ( argc != 3 )
    {
        cout<<"usage: feature_extraction img1 img2"<<endl;
        return 1;
    }
    //-- 读取图像
    Mat img_1 = imread ( argv[1], CV_LOAD_IMAGE_COLOR );
    Mat img_2 = imread ( argv[2], CV_LOAD_IMAGE_COLOR );

    //-- 初始化
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // Ptr<FeatureDetector> detector = FeatureDetector::create(detector_name);
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create(descriptor_name);
    //定义特征描述子匹配器
    //参数MatcherType：匹配器类型，这里使用MatcherType::BRUTEFORCE（暴力匹配算法）
    //注意：对于与ORB特征描述子类似的二进制描述子而言，在匹配时采用NORM_HANMING来作为度量距离；
    //而对于类似SIFT、SURF之类的非二进制描述子而言，一般常用NORM_L2作为距离度量，也可以选用NORM_L1。
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );

    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    Mat outimg1;
    drawKeypoints( img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    imshow("ORB特征点",outimg1);

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    //通过描述子匹配器，对两幅图像的描述子进行匹配，也就是将两幅图像中的对应特征点进行匹配；
    //输出的是一个DMatch结构体向量，其每一个DMatch结构体包含一组对应特征点的信息。
    vector<DMatch> matches;
    
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, matches );
    // 可以改为快速最近邻逼近搜索函数库(Fast Approximate Nearest Neighbor Search Library)  的高效匹配来进行实验
    //-- Step 3: Matching descriptor vectors using FLANN matcher
    // FlannBasedMatcher matcher;
    // std::vector< DMatch > matches;
    // matcher.match( descriptors_1, descriptors_2, matches );
    
    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = matches[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }
    
    // 仅供娱乐的写法
    min_dist = min_element( matches.begin(), matches.end(), [](const DMatch& m1, const DMatch& m2) {return m1.distance<m2.distance;} )->distance;
    max_dist = max_element( matches.begin(), matches.end(), [](const DMatch& m1, const DMatch& m2) {return m1.distance<m2.distance;} )->distance;

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    std::vector< DMatch > good_matches;
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( matches[i].distance <= max ( 1.5*min_dist, 30.0 ) )
        {
            good_matches.push_back ( matches[i] );
            std::cout << "**********************************" << std::endl;
            std::cout << "keypoints_1[matches[i].queryIdx].angle:" << keypoints_1[matches[i].queryIdx].angle << endl;
            std::cout << "**********************************" << std::endl;
            std::cout << "keypoints_2[matches[i].trainIdx].angle:" << keypoints_2[matches[i].queryIdx].angle << endl;
            std::cout << "**********************************" << std::endl;
            std::cout << descriptors_1 << std::endl;
        }
    }

    ORBmatcher rotmatcher;
    rotmatcher.DisBlogeByRot( keypoints_1, keypoints_2,  good_matches);

    //-- 第五步:绘制匹配结果
    Mat img_match;
    Mat img_goodmatch;
    drawMatches ( img_1, keypoints_1, img_2, keypoints_2, matches, img_match );
    drawMatches ( img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch );
    namedWindow("优化后匹配点对",0);
    namedWindow("所有匹配点对",0);
    cv::resizeWindow("优化后匹配点对",640,480);
    cv::resizeWindow("所有匹配点对",640,480);
    imshow ( "所有匹配点对", img_match );
    imshow ( "优化后匹配点对", img_goodmatch );
    waitKey(0);

    return 0;
}
