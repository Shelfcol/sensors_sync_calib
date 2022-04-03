#include "camera_camera_sync/camera_camera_sync.hpp"
#include <dirent.h>
#include <unistd.h>
#include <ros/ros.h>
#include <cstdlib>
#include <vector>
#include <algorithm>


using namespace std;

void CameraCameraSync::getFiles(string path, vector<string>& files)
{
    DIR *dir;
    struct dirent *ptr;
    char base[1000];

    char* basePath = const_cast<char*>(path.c_str()); 


    if ((dir=opendir(basePath)) == NULL)
    {
        perror("Open dir error...");
        exit(1);

    }

    while ((ptr=readdir(dir)) != NULL)
    {
        // current dir 
        if(strcmp(ptr->d_name, ".")==0 || strcmp(ptr->d_name, "..")==0)
            continue;
        else if(ptr->d_type == 8) // file
            sprintf(base, "%s/%s", basePath, ptr->d_name);
        //puts(base);
        files.push_back(std::string(base));
    }
}

void CameraCameraSync::getImageTimeStamp(std::string oriDirName, std::string dstDirName)
{
    //采用该函数遍历获得得队列不是顺序的，正好适合采用时间距离最近法来匹配
    getFiles(oriDirName, oriImageLists_);
    getFiles(dstDirName, dstImageLists_);
    if(oriImageLists_.size() != dstImageLists_.size())
    {
        std::cout << "the two image lists not equal!" << std::endl;
        ROS_ERROR_STREAM("the two image lists not equal!");
        return;
    }
}

int CameraCameraSync::getImageNumber()
{
    // if(oriImageLists_.size() != dstImageLists_.size())
    // {
    //     std::cout << "the two image lists not equal!" << std::endl;
    //     ROS_ERROR_STREAM("the two image lists not equal!");
    //     return -1;
    // }
    return oriImageLists_.size();
}

double CameraCameraSync::getbaseTime(std::string pngfilenames, std::string patt)
{
    size_t pattern = pngfilenames.find(patt);
    std::string baseFile = pngfilenames.substr(pattern-18, 17); // 仅针对该项目所提供的文件适用
    double baseImageTime = atof(baseFile.c_str());  
    return baseImageTime;
}

std::vector<std::pair<std::string, std::string> > CameraCameraSync::imageTimeStampSyncFuncion()
{
    std::vector<std::pair<std::string, std::string> > syncPairLists;

    double timeDifference;
    for(auto baseFileNames : oriImageLists_)
    {
        double maxSSIM = 0;
        std::string anchorFilenames;
        double baseImageTime = getbaseTime(baseFileNames, "png"); // 这一帧图像的时间戳

        for(auto candidateFileNames : dstImageLists_)
        {
            double candidateImageTime = getbaseTime(candidateFileNames, "png"); // 候选帧的时间戳
            timeDifference = std::abs(baseImageTime - candidateImageTime);
            if(timeDifference <= 0.1)
            {
                cv::Mat orgImage = cv::imread(baseFileNames, cv::IMREAD_GRAYSCALE);
                cv::Mat dstImage = cv::imread(candidateFileNames, cv::IMREAD_GRAYSCALE);
                if( !orgImage.data || !dstImage.data )
                { 
                    std::cout<< " --(!) Error reading images " << std::endl; 
                    break;
                }
                double ssim = evaluateImageTimeStampSync(orgImage, dstImage); // 两帧图像的相似度
                if (ssim > maxSSIM) // 找出最项近的图片
                {
                    maxSSIM = ssim;
                    anchorFilenames = candidateFileNames;
                }
            }
        }
        std::pair<std::string, std::string> syncPair(std::make_pair(baseFileNames, anchorFilenames));
        syncPairLists.push_back(syncPair);
        // std::cout << " Get the "<< baseFileNames << " time sync file is " << anchorFilenames << " and ssim is " << maxSSIM << std::endl;
    }

    return syncPairLists;
}

// 度量图像
// 可以用于清洗图像，将相邻的相似度高的图像删除，比如车停止采集的数据
double CameraCameraSync::evaluateImageTimeStampSync(cv::Mat orgImage, cv::Mat dstImage)
{
    //这里采用SSIM结构相似性来作为图像相似性评判
    double C1 = 6.5025, C2 = 58.5225;
    int width = orgImage.cols;
    int height = orgImage.rows;
    
    int width2 = dstImage.cols;
    int height2 = dstImage.rows;

    double mean_x = 0;
    double mean_y = 0;
    double sigma_x = 0;
    double sigma_y = 0;
    double sigma_xy = 0;
    for (int v = 0; v < height; v++)
    {
        for (int u = 0; u < width; u++)
        {
            mean_x += orgImage.at<uchar>(v, u);
            mean_y += dstImage.at<uchar>(v, u);

        }
    }
    mean_x = mean_x / width / height;
    mean_y = mean_y / width / height;
    for (int v = 0; v < height; v++)
    {
        for (int u = 0; u < width; u++)
        {
            sigma_x += (orgImage.at<uchar>(v, u) - mean_x)* (orgImage.at<uchar>(v, u) - mean_x);
            sigma_y += (dstImage.at<uchar>(v, u) - mean_y)* (dstImage.at<uchar>(v, u) - mean_y);
            sigma_xy += std::abs((orgImage.at<uchar>(v, u) - mean_x)* (dstImage.at<uchar>(v, u) - mean_y));
        }
    }
    sigma_x = sigma_x / (width*height - 1);
    sigma_y = sigma_y / (width*height - 1);
    sigma_xy = sigma_xy / (width*height - 1);
    double molecule = (2 * mean_x*mean_y + C1) * (2 * sigma_xy + C2);
    double denominator = (mean_x*mean_x + mean_y * mean_y + C1) * (sigma_x + sigma_y + C2);
    double ssim = molecule / denominator;
    return ssim;
}

void CameraCameraSync::spatialSynchronizationWithORB(cv::Mat srcImage1, cv::Mat srcImage2)
{
    // 提取特征点    
    //使用SURF算子检测关键点
	// int minHessian = 400;//SURF算法中的hessian阈值
    // std::vector<cv::KeyPoint> keypoints_object, keypoints_scene;//vector模板类，存放任意类型的动态数组
    // cv::Mat descriptors_object, descriptors_scene;
	// cv::Ptr<cv::FeatureDetector> detector = cv::Feature2D::SurfFeatureDetector::create(minHessian);
	
    // cv::Ptr <cv::Feature2D::SURF> extractor = cv::Feature2D::SURF::create(minHessian);
    
	// //调用detect函数检测出SURF特征关键点，保存在vector容器中
	// detector->detect(srcImage1, keypoints_object);
	// detector->detect(srcImage2, keypoints_scene);

    // //特征点描述，为下边的特征点匹配做准备  
    // cv::Mat matshow1, matshow2, kp1, kp2; 
    // extractor->compute(srcImage1, keypoints_object, descriptors_object);
	// extractor->compute(srcImage2, keypoints_scene, descriptors_scene);

    std::vector<cv::KeyPoint> keypoints_object, keypoints_scene;//vector模板类，存放任意类型的动态数组
    cv::Mat descriptors_object, descriptors_scene;
    // cv::Ptr<cv::ORB> orb = cv::ORB_create(500,1.2f,8,31,0,2,cv::ORB::HARRIS_SCORE,31,20);
    // cv::Ptr<cv::ORB> orb =  cv::ORB::create(); //
    cv::Ptr<cv::Feature2D> orb =  cv::ORB::create(); //
   //特征点描述，为下边的特征点匹配做准备  

    // 检测特征点
    orb->detect(srcImage1, keypoints_object);
    orb->detect(srcImage2, keypoints_scene);

    // 计算描述子
    cv::Mat matshow1, matshow2, kp1, kp2; 
    orb->compute(srcImage1, keypoints_object, descriptors_object);
	orb->compute(srcImage2, keypoints_scene, descriptors_scene);

    cv::Mat outImg;
    cv::drawKeypoints(srcImage1,keypoints_object,outImg,cv::Scalar::all(-1),cv::DrawMatchesFlags::DEFAULT);
    // imshow("ORB descriptor",outImg);
    // cv::waitKey(0);

    //使用FLANN匹配算子进行匹配
    // cv::FlannBasedMatcher matcher;
    std::vector<cv::DMatch> matchePoints;
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    matcher.match(descriptors_object, descriptors_scene, matchePoints);
    std::cout<<"descriptors_object.rows = "<<descriptors_object.rows<<std::endl;
    //最小距离和最大距离
    int max_dist = 0; 
    int min_dist = 1000;

	//计算出关键点之间距离的最大值和最小值
	for (int i = 0; i < descriptors_object.rows; i++)
	{
		int dist = matchePoints[i].distance;
        // std::cout<<"dist = "<<dist<<std::endl;
		if (dist < min_dist) {min_dist = dist;}
		if (dist > max_dist) max_dist = dist;
	}

	printf(">Max dist 最大距离 : %d \n", max_dist);
	printf(">Min dist 最小距离 : %d \n", min_dist);

	//匹配距离小于3*min_dist的点对
	std::vector< cv::DMatch > goodMatches;
 
	for (int i = 0; i < descriptors_object.rows; i++)
	{
		if (matchePoints[i].distance <  std::max(2 * min_dist,30)) // 有时候距离太小，所以需要设置一个下界
		{
			goodMatches.push_back(matchePoints[i]);
		}
	}
    
    if(goodMatches.size()==0){
        std::cout<<"no good matches"<<std::endl;
        return;
    }

	//绘制出匹配到的关键点
	cv::Mat imgMatches;
	cv::drawMatches(srcImage1, keypoints_object, srcImage2, keypoints_scene,
		goodMatches, imgMatches, cv::Scalar::all(-1), cv::Scalar::all(-1),
		std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    // imshow("ORB descriptor",imgMatches);
    // cv::waitKey(0);
	//定义两个局部变量
	std::vector<cv::Point2f> obj;
	std::vector<cv::Point2f> scene;
 
	//从匹配成功的匹配对中获取关键点
	for (unsigned int i = 0; i < goodMatches.size(); i++)
	{
		obj.push_back(keypoints_object[goodMatches[i].queryIdx].pt); // 对应的二维像素点
		scene.push_back(keypoints_scene[goodMatches[i].trainIdx].pt);
	}
 
	cv::Mat H = cv::findHomography(obj, scene, cv::RANSAC);//计算透视变换 
 
    // 从H矩阵恢复旋转平移
    // cv::recoverPose(H,obj,scene,cv::RANSAC,3,cv::noArray(),2000,0.99);

	//从待测图片中获取角点
	std::vector<cv::Point2f> obj_corners(4);
	// obj_corners[0] = cv::Point(0, 0);
	// obj_corners[1] = cv::Point(srcImage1.cols, 0);
	// obj_corners[2] = cv::Point(srcImage1.cols, srcImage1.rows);
	// obj_corners[3] = cv::Point(0, srcImage1.rows);

	obj_corners[0] = cv::Point(srcImage1.cols/3, srcImage1.rows/3);
	obj_corners[1] = cv::Point(srcImage1.cols*2/3, srcImage1.rows/3);
	obj_corners[2] = cv::Point(srcImage1.cols*2/3, srcImage1.rows*2/3);
	obj_corners[3] = cv::Point(srcImage1.cols/3, srcImage1.rows*2/3);

	std::vector<cv::Point2f> scene_corners(4);
 
	//进行透视变换
	cv::perspectiveTransform(obj_corners, scene_corners, H);

    
    cv::line(imgMatches, obj_corners[0], obj_corners[1], cv::Scalar(33, 33, 133), 3);
    cv::line(imgMatches, obj_corners[1], obj_corners[2], cv::Scalar(33, 33, 133), 3);
    cv::line(imgMatches, obj_corners[2], obj_corners[3], cv::Scalar(33, 33, 133), 3);
    cv::line(imgMatches, obj_corners[3], obj_corners[0], cv::Scalar(33, 33, 133), 3);

    // 可视化scene_corners
    scene_corners[0]=cv::Point(scene_corners[0].x+srcImage1.cols,scene_corners[0].y);
    scene_corners[1]=cv::Point(scene_corners[1].x+srcImage1.cols,scene_corners[1].y);
    scene_corners[2]=cv::Point(scene_corners[2].x+srcImage1.cols,scene_corners[2].y);
    scene_corners[3]=cv::Point(scene_corners[3].x+srcImage1.cols,scene_corners[3].y);
    cv::line(imgMatches, scene_corners[0], scene_corners[1], cv::Scalar(33, 33, 133), 3);
    cv::line(imgMatches, scene_corners[1], scene_corners[2], cv::Scalar(33, 33, 133), 3);
    cv::line(imgMatches, scene_corners[2], scene_corners[3], cv::Scalar(33, 33, 133), 3);
    cv::line(imgMatches, scene_corners[3], scene_corners[0], cv::Scalar(33, 33, 133), 3);


	// 显示最终结果
	// imshow("效果图", imgMatches);
    time_t timep;
    time(&timep);
    
    char name[1024];
    sprintf(name, "效果_%d.jpg", timep);
    // cv::namedWindow("ORB descriptor", 0);    
    // cv::resizeWindow("ORB descriptor", 700, 900);   // 自己设定窗口图片的大小
    // imshow("ORB descriptor",imgMatches);
    // cv::waitKey(0);
    
    cv::imwrite(name,imgMatches);

}


void CameraCameraSync::spatialSynchronizationWithSURF(cv::Mat srcImage1, cv::Mat srcImage2)
{
    // 提取特征点    
    //使用SURF算子检测关键点
	int minHessian = 400;//SURF算法中的hessian阈值
    std::vector<cv::KeyPoint> keypoints_object, keypoints_scene;//vector模板类，存放任意类型的动态数组
    cv::Mat descriptors_object, descriptors_scene;
	cv::Ptr<cv::FeatureDetector> detector = cv::xfeatures2d::SurfFeatureDetector::create(minHessian);
	
    cv::Ptr<cv::xfeatures2d::SURF> extractor = cv::xfeatures2d::SURF::create(minHessian);
    
	//调用detect函数检测出SURF特征关键点，保存在vector容器中
	detector->detect(srcImage1, keypoints_object);
	detector->detect(srcImage2, keypoints_scene);

    //特征点描述，为下边的特征点匹配做准备  
    cv::Mat matshow1, matshow2, kp1, kp2; 
    extractor->compute(srcImage1, keypoints_object, descriptors_object);
	extractor->compute(srcImage2, keypoints_scene, descriptors_scene);

    //使用FLANN匹配算子进行匹配
    cv::FlannBasedMatcher matcher;
    std::vector<cv::DMatch> matchePoints;
    matcher.match(descriptors_object, descriptors_scene, matchePoints);

    //最小距离和最大距离
    double max_dist = 0; 
    double min_dist = 100;

	//计算出关键点之间距离的最大值和最小值
	for (int i = 0; i < descriptors_object.rows; i++)
	{
		double dist = matchePoints[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}

	printf(">Max dist 最大距离 : %f \n", max_dist);
	printf(">Min dist 最小距离 : %f \n", min_dist);

	//匹配距离小于3*min_dist的点对
	std::vector< cv::DMatch > goodMatches;
 
	for (int i = 0; i < descriptors_object.rows; i++)
	{
		if (matchePoints[i].distance < 2.5 * min_dist)
		{
			goodMatches.push_back(matchePoints[i]);
		}
	}
    
	//绘制出匹配到的关键点
	cv::Mat imgMatches;
	cv::drawMatches(srcImage1, keypoints_object, srcImage2, keypoints_scene,
		goodMatches, imgMatches, cv::Scalar::all(-1), cv::Scalar::all(-1),
		std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
 
	//定义两个局部变量
	std::vector<cv::Point2f> obj;
	std::vector<cv::Point2f> scene;
 
	//从匹配成功的匹配对中获取关键点
	for (unsigned int i = 0; i < goodMatches.size(); i++)
	{
		obj.push_back(keypoints_object[goodMatches[i].queryIdx].pt);
		scene.push_back(keypoints_scene[goodMatches[i].trainIdx].pt);
	}
 
	cv::Mat H = cv::findHomography(obj, scene, cv::RANSAC);//计算透视变换 
 
	//从待测图片中获取角点
	std::vector<cv::Point2f> obj_corners(4);
	obj_corners[0] = cv::Point(0, 0);
	obj_corners[1] = cv::Point(srcImage1.cols, 0);
	obj_corners[2] = cv::Point(srcImage1.cols, srcImage1.rows);
	obj_corners[3] = cv::Point(0, srcImage1.rows);
	std::vector<cv::Point2f> scene_corners(4);
 
	//进行透视变换
	cv::perspectiveTransform(obj_corners, scene_corners, H);
 
    //! 没有内参矩阵，无法恢复出旋转平移
	// //显示最终结果
    time_t timep;
    time(&timep);
    
    char name[1024];
    sprintf(name, "效果_%d.jpg", timep);
    
    cv::imwrite(name,imgMatches);
	imshow("效果图", imgMatches);
    cv::waitKey(0);
}


bool CameraCameraSync::synchronizePitchRoll(cv::Mat img_left, cv::Mat img_right)
{
    if(!img_left.data || !img_right.data )
    {
        ROS_ERROR_STREAM("no image data!");
        return false;
    }

    std::vector<cv::Point2f> left_pts, right_pts;
    printf("finding match points\n");
    findMatchPoints(img_left, img_right, left_pts, right_pts);
    std::cout << "find match points:size: left:" << left_pts.size() << " right: " << right_pts.size() << std::endl;

    // solve pitch and roll between cameras
    vector<vector<Point2f> > data = {left_pts, right_pts};
    // 待优化变量
    Eigen::VectorXd x(2);
    x << 0., 0.;
    // 定义一个functor
    MeanFunctor functor(data);
    // 定义数值微分求解方法，设定收敛阈值
    Eigen::NumericalDiff<MeanFunctor> num_diff(functor, 1e-6);
    // 定义LM求解其
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<MeanFunctor>, double> lm(num_diff);
    // 调用LM算法
    int info = lm.minimize(x);

    std::cout << "current result: pitch & roll: " << x[0]/PI*180 << " " << x[1]/PI*180 << endl;

    pitch_cache_.push_back(x[0]);
    roll_cache_.push_back(x[1]);

    return true;
}

bool CameraCameraSync::synchronizePitchRollWithCeres(cv::Mat img_left, cv::Mat img_right)
{
    if(!img_left.data || !img_right.data )
    {
        ROS_ERROR_STREAM("no image data!");
        return false;
    }

    std::vector<cv::Point2f> left_pts, right_pts;
    printf("finding match points\n");
    findMatchPoints(img_left, img_right, left_pts, right_pts);
    std::cout << "find match points:size: left:" << left_pts.size() << " right: " << right_pts.size() << std::endl;

    // 构建优化问题
    double x[]={0.0,0.0};
    ceres::Problem problem;
    for(int i=0;i<left_pts.size();++i){
        ceres::CostFunction* cost_func = CamPitchRollFactor::Create(left_pts[i],right_pts[i]);
        problem.AddResidualBlock(cost_func,nullptr,x);
    }
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options,&problem,&summary);
    std::cout<<summary.BriefReport()<<std::endl;

    std::cout << "current result: pitch & roll: " << x[0]/PI*180 << " " << x[1]/PI*180 << endl;
    // current result: pitch & roll: 0.0358881 0.783933
    pitch_cache_.push_back(x[0]);
    roll_cache_.push_back(x[1]);

    return true;
}


void CameraCameraSync::findMatchPoints(const Mat srcImage1, const Mat srcImage2, vector<Point2f>& pts1, vector<Point2f>& pts2)
{
 std::vector<cv::KeyPoint> keypoints_object, keypoints_scene;//vector模板类，存放任意类型的动态数组
    cv::Mat descriptors_object, descriptors_scene;
    // cv::Ptr<cv::ORB> orb = cv::ORB_create(500,1.2f,8,31,0,2,cv::ORB::HARRIS_SCORE,31,20);
    // cv::Ptr<cv::ORB> orb =  cv::ORB::create(); //
    cv::Ptr<cv::Feature2D> orb =  cv::ORB::create(); //
   //特征点描述，为下边的特征点匹配做准备  

    // 检测特征点
    orb->detect(srcImage1, keypoints_object);
    orb->detect(srcImage2, keypoints_scene);

    // 计算描述子
    cv::Mat matshow1, matshow2, kp1, kp2; 
    orb->compute(srcImage1, keypoints_object, descriptors_object);
	orb->compute(srcImage2, keypoints_scene, descriptors_scene);

    cv::Mat outImg;
    cv::drawKeypoints(srcImage1,keypoints_object,outImg,cv::Scalar::all(-1),cv::DrawMatchesFlags::DEFAULT);
    // imshow("ORB descriptor",outImg);
    // cv::waitKey(0);

    //使用FLANN匹配算子进行匹配
    // cv::FlannBasedMatcher matcher;
    std::vector<cv::DMatch> matchePoints;
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    matcher.match(descriptors_object, descriptors_scene, matchePoints);
    std::cout<<"descriptors_object.rows = "<<descriptors_object.rows<<std::endl;
    //最小距离和最大距离
    int max_dist = 0; 
    int min_dist = 1000;

	//计算出关键点之间距离的最大值和最小值
	for (int i = 0; i < descriptors_object.rows; i++)
	{
		int dist = matchePoints[i].distance;
        // std::cout<<"dist = "<<dist<<std::endl;
		if (dist < min_dist) {min_dist = dist;}
		if (dist > max_dist) max_dist = dist;
	}

	// printf(">Max dist 最大距离 : %d \n", max_dist);
	// printf(">Min dist 最小距离 : %d \n", min_dist);

	//匹配距离小于3*min_dist的点对
	std::vector< cv::DMatch > goodMatches;
 
	for (int i = 0; i < descriptors_object.rows; i++)
	{
		if (matchePoints[i].distance <  std::max(2 * min_dist,30)) // 有时候距离太小，所以需要设置一个下界
		{
			goodMatches.push_back(matchePoints[i]);
		}
	}
    
    if(goodMatches.size()==0){
        std::cout<<"no good matches"<<std::endl;
        return;
    }

	//绘制出匹配到的关键点
	cv::Mat imgMatches;
	cv::drawMatches(srcImage1, keypoints_object, srcImage2, keypoints_scene,
		goodMatches, imgMatches, cv::Scalar::all(-1), cv::Scalar::all(-1),
		std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    // imshow("ORB descriptor",imgMatches);
    // cv::waitKey(0);
	//定义两个局部变量

	//从匹配成功的匹配对中获取关键点
	for (unsigned int i = 0; i < goodMatches.size(); i++)
	{
		pts1.push_back(keypoints_object[goodMatches[i].queryIdx].pt); // 对应的二维像素点
		pts2.push_back(keypoints_scene[goodMatches[i].trainIdx].pt);
	}
 
}