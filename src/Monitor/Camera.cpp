
#include "../../include/Monitor/Camera.h"


namespace Monitor
{

    // Camera::Camera()
    // {
    //     fx_ = Config::get<float>("Camera.fx");
    //     fy_ = Config::get<float>("Camera.fy");
    //     cx_ = Config::get<float>("Camera.cx");
    //     cy_ = Config::get<float>("Camera.cy");
    //     K = ( cv::Mat_<double> ( 3,3 ) << fx_, 0, cx_,
    //             0, fy_, cy_,
    //             0,0,1
    //     );
    //     cout<<"camera parameters is load"<<endl;
    // }
    Camera::Camera(std::shared_ptr<Monitor::Config>  ConfigInstance){
        fx_ = ConfigInstance->get<float>("Camera.fx");
        // fx_ = ConfigInstance->get<float> ("dataset_first");
        fy_ = ConfigInstance->get<float>("Camera.fy");
        cx_ = ConfigInstance->get<float>("Camera.cx");
        cy_ = ConfigInstance->get<float>("Camera.cy");
        K = ( cv::Mat_<double> ( 3,3 ) << fx_, 0, cx_,
                0, fy_, cy_,
                0,0,1
        );
        cout<<"camera parameters is load"<<endl;        
    }
    
    /**
     * @brief 
     * 
     * @param[in] p_w 
     * @param[in] T_c_w 
     * @return Vector3d 
     */
    // Vector3d Camera::world2camera ( const Vector3d& p_w, const SE3& T_c_w )
    // {
    //     return T_c_w*p_w;
    // }

    // Vector3d Camera::camera2world ( const Vector3d& p_c, const SE3& T_c_w )
    // {
    //     return T_c_w.inverse() *p_c;
    // }

    // Vector2d Camera::camera2pixel ( const Vector3d& p_c )
    // {
    //     return Vector2d (
    //             fx_ * p_c ( 0,0 ) / p_c ( 2,0 ) + cx_,
    //             fy_ * p_c ( 1,0 ) / p_c ( 2,0 ) + cy_
    //     );
    // }

    // Vector3d Camera::pixel2camera ( const Vector2d& p_p, double depth )
    // {
    //     return Vector3d (
    //             ( p_p ( 0,0 )-cx_ ) *depth/fx_,
    //             ( p_p ( 1,0 )-cy_ ) *depth/fy_,
    //             depth
    //     );
    // }

    // Vector2d Camera::world2pixel ( const Vector3d& p_w, const SE3& T_c_w )
    // {
    //     return camera2pixel ( world2camera(p_w, T_c_w) );
    // }

    // Vector3d Camera::pixel2world ( const Vector2d& p_p, const SE3& T_c_w, double depth )
    // {
    //     return camera2world ( pixel2camera ( p_p, depth ), T_c_w );
    // }


}//namespace Monitor
