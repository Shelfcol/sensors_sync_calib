#include "camera_camera_sync/camera_camera_sync.hpp"

int main()
{
    // cv::initModule_nonfree();
    CameraCameraSync camera_camera_sync_;

    std::string oriDirs = "/home/gxf/multi-sensor-fusion/calib_ws/data/practice_1_1_multi_camera_sync/camera_front_left_60";
    std::string dstDirs = "/home/gxf/multi-sensor-fusion/calib_ws/data/practice_1_1_multi_camera_sync/camera_front_right_60";
    camera_camera_sync_.getImageTimeStamp(oriDirs, dstDirs);
    std::vector<std::pair<std::string, std::string> > syncImageLists;
    int number = camera_camera_sync_.getImageNumber();
    if (number > 0)
    {
        syncImageLists = camera_camera_sync_.imageTimeStampSyncFuncion();
    }
    std::cout<<" sync image size = "<<syncImageLists.size()<<std::endl;
    for(auto syncPair : syncImageLists)
    {
        cv::Mat image1 = cv::imread(syncPair.first, cv::IMREAD_GRAYSCALE);
        cv::Mat image2 = cv::imread(syncPair.second, cv::IMREAD_GRAYSCALE);
        if( !image1.data || !image2.data )
        { 
            std::cout<< " --(!) Error reading images " << std::endl; 
            return -1;
        }
        
<<<<<<< HEAD
        // camera_camera_sync_.spatialSynchronizationWithSURF(image1, image2);
        camera_camera_sync_.synchronizePitchRollWithCeres(image1, image2);
=======
        camera_camera_sync_.spatialSynchronizationWithSURF(image1, image2);
>>>>>>> e85acb3439abcdd9dece6cd7feb04f57baff235e
    }

    // cv::Mat image1 = cv::imread("/disk3/sensor_fusion/datasets/src/rosbag_dump/2021-07-13-10-34-45/camera_c5_front_left_60/1625566154.370427.png", cv::IMREAD_GRAYSCALE);
    // cv::Mat image2 = cv::imread("/disk3/sensor_fusion/datasets/src/rosbag_dump/2021-07-13-10-34-45/camera_c6_front_right_60/1625566154.370326.png", cv::IMREAD_GRAYSCALE);
    // if( !image1.data || !image2.data )
    // { 
    //     std::cout<< " --(!) Error reading images " << std::endl; 
    //     return -1;
    // }
    // camera_camera_sync_.spatialSynchronizationWithORB(image1, image2);

}
