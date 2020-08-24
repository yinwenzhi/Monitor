#include<stdio.h>
#include<math.h>

#define pi 3.1415926
/*
@brief:输入预定义关键点像素坐标；
       输出关键点在气柱坐标系中的坐标;
       坐标原点为图像中心；
       x轴与气柱相切,平行于logo_height；
       y轴指向气柱轴心；
       z轴沿气柱纵向轴心方向，平行于logo_width；
*/
vector<float> ComputeCoordinatesFromPicture(float logo_height,float logo_width,float pillars_radius,float key_point_pixel_x,float key_point_pixel_y){
    vector<float> key_point_coordinates;
    //关键点在气柱坐标系的物理坐标
    float key_point_x = logo_height / 2916 * key_point_pixel_y;
    float key_point_y = logo_width / 5363 * key_point_pixel_x;
    //弧长
    float arc_length = key_point_x; 
    //圆心角
    float central_angle = arc_length / pillars_radius; 
    //关键点与原点连线
    float hypotenuse = 2 * pillars_radius * sin(central_angle / 2);
    //关键点与原点连线与x轴夹角
    float alpha = 90 - (180 - central_angle) / 2;
    //关键点在气柱坐标系坐标
    float x = hypotenuse * sin(alpha);
    float y = hypotenuse * cos(alpha);
    float z = key_point_y;

    key_point_coordinates.push_back(x);
    key_point_coordinates.push_back(y);
    key_point_coordinates.push_back(z);
    
    return key_point_coordinates;
}


/*
    由3D空间点计算相机成像平面上成像的像素坐标
    输入： 
        空间3D点位置
        相机参数
    输出： 
        3D点在相机成像平面的坐标
/*
vector<float> ComputeCoordinatesFromPicture(){

}