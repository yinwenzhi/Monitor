#include "CameraApi.h" //相机SDK的API头文件

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>  


namespace Monitor{

    class CameraDevice
    {
    private:
        int                     iCameraCounts = 1;
        int                     iStatus=-1;
        tSdkCameraDevInfo       tCameraEnumList;
        int                     hCamera;
        tSdkCameraCapbility     tCapability;      //设备描述信息
        tSdkFrameHead           sFrameInfo;
        BYTE*			        pbyBuffer;
        int                     iDisplayFrames = 10000;
        IplImage *iplImage = NULL;
        int                     channel=3;

        unsigned char           * g_pRgbBuffer;     //处理后数据缓存区
    public:
        CameraDevice();
        GetImage();
        ~CameraDevice();
        
    }
}