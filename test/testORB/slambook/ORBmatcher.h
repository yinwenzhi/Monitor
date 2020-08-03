/*
 * @Author: your name
 * @Date: 2020-08-03 17:26:20
 * @LastEditTime: 2020-08-03 19:08:45
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /Monitor/test/testORB/slambook/ORBmatcher.h
 */
#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

// #include "MapPoint.h"
// #include "KeyFrame.h"
// #include "Frame.h"
using namespace cv;
namespace Monitor
{
    
class ORBmatcher
{
private:
    /* data */
public:

    // 要用到的一些阈值
    static const int TH_LOW;            ///< 判断描述子距离时比较低的那个阈值,主要用于基于词袋模型加速的匹配过程，可能是感觉使用词袋模型的时候对匹配的效果要更加严格一些
    static const int TH_HIGH;           ///< 判断描述子距离时比较高的那个阈值,用于计算投影后能够匹配上的特征点的数目；如果匹配的函数中没有提供阈值的话，默认就使用这个阈值
    static const int HISTO_LENGTH;      ///< 判断特征点旋转用直方图的长度

public:

    ORBmatcher();
    
    // 寻找三个最大的主方向 
    void ComputeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);

    //匹配后再使用主方向进行一次剔除 方向不是主方向周围的匹配对
    void DisBlogeByRot(std::vector<KeyPoint> & _keypoints_1, std::vector<KeyPoint> & _keypoints_2,std::vector< DMatch >  & _matches);

    ~ORBmatcher();
};

}
#endif