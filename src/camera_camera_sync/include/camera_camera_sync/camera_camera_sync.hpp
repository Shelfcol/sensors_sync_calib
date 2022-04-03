#ifndef _CAMERA_CAMERA_SYNC_HPP_
#define _CAMERA_CAMERA_SYNC_HPP_

#include <string>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
<<<<<<< HEAD
// #include <opencv2/nonfree/nonfree.hpp>
=======
>>>>>>> e85acb3439abcdd9dece6cd7feb04f57baff235e
// #include </home/gxf/opencv_build/opencv_contrib/modules/xfeatures2d/include/opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <time.h>
#include <Eigen/Dense>
#define PRINTLOG std::cout << __LINE__ << " in " << __FILE__ << std::endl;

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
#include <cmath>
#include <ceres/ceres.h>

using namespace std;
using namespace cv;
#define PI 3.14159265
const double UTILS_MATCH_MIN_DIST = 25.0;
const double UTILS_FEATURE_MATCH_RATIO_THRESH = 0.5;

#define PRINTLOG std::cout << __LINE__ << " in " << __FILE__ << std::endl;
struct CamParams
{
    // left: cam1, right: cam1
    Mat K1, K2, D1, D2, R, T;
    Size imgSize;

    CamParams()
    {
        // camera1:intrinsic
        double arrK1[9] = {1.9457000000000000e+03, 0., 8.9667539999999997e+02, 0.,1.9433000000000000e+03, 5.0516239999999999e+02, 0., 0., 1.};
        // camera2:intrinsic
        double arrK2[9] = {1.9492000000000000e+03, 0., 9.2153930000000003e+02, 0.,1.9472000000000000e+03, 5.6912049999999999e+02, 0., 0., 1.};

        double arrR[9] = {1.0,0.,0.,0.,1.0,0.,0.,0.,1.0};
        double arrT[3] = {-1.1,0,0}; // 两个相机的相对平移，不是投影矩阵里面的Tx Ty
        // camera1:distortion coefficient  矩阵D是失真系数，包括(k1, k2, t1, t2, k3)
        double arrD1[5] = {5.7289999999999996e-01, 3.10500e-01,3.50001e-03, 6.8995000005e-04,-3.3500000000000002e-02};
        // camera2:distortion coefficient
        double arrD2[5] = {-5.8879999999999999e-01, 3.0020000000000002e-01,2.0999999999999999e-03, -2.2568999999999999e-04,2.1190000000000001e-01};

        K1 = Mat(3,3, CV_64F, arrK1).clone(); // cam1 内参
        K2 = Mat(3,3, CV_64F, arrK2).clone(); // cam2 内参

        R = Mat(3,3, CV_64F,arrR).clone();
        T = Mat(3,1, CV_64F,arrT).clone();

        D1 = Mat(1,5,CV_64F,arrD1).clone();
        D2 = Mat(1,5,CV_64F,arrD2).clone();

        imgSize.width = 1920;
        imgSize.height = 1080;
    }

    ~CamParams(){}

};

void findGoodMatch(std::vector<DMatch> matches, std::vector<DMatch> &good_matches);
void findMatchPoints(const Mat img_left, const Mat img_right, vector<Point2f>& pts1, vector<Point2f>& pts2);

template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor
{
    typedef _Scalar Scalar;
    enum {InputsAtCompileTime = NX, ValuesAtCompileTime = NY};
    typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

    const int m_inputs, m_values;

    Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
    Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

    int inputs() const { return m_inputs;}
    int values() const { return m_values;}
};

class MeanFunctor : public Functor<double> 
{
public:
    MeanFunctor(vector<vector<Point2f> > data):
        Functor<double>(2, data[0].size()), // 1:number of parameters to optimize
        _data(data),
        _cam(CamParams()) {}

    // pitch用y值优化
    int operator()(const Eigen::VectorXd& x, Eigen::VectorXd& fvec) const //输入：优化变量，残差
    {
        // x =(pitch,roll)
        // pitch 在相机坐标系下 ,向前为z，向右为x，向下为y，roll绕z轴，pitch绕x轴，yaw绕y轴。
        // 而坐标轴的旋转与坐标点的旋转刚好角度相反。
        double arrDeltaRPitch[9] = {1., 0., 0.,
                                    0.,cos(x[0]),-sin(x[0]),
                                    0.,sin(x[0]),cos(x[0])};
        Mat detlaRPitch = Mat(3,3,CV_64F, arrDeltaRPitch);
        // roll
        double arrDetlaRRoll[9] = {cos(x[1]),-sin(x[1]), 0.,
                                    sin(x[1]),cos(x[1]),0.,
                                    0.,0.,1.};
        Mat detlaRRoll = Mat(3,3,CV_64F, arrDetlaRRoll);
        
        // add disturb
        double distPitch = 3.14159265/180 * 2;
        double arrDistPitch[9] = {1.,0.,0.,
                                    0.,cos(distPitch), -sin(distPitch),
                                    0.,sin(distPitch), cos(distPitch)};
        
        Mat distRPitch = Mat(3,3,CV_64F, arrDistPitch);

        Mat optimR = _cam.R * detlaRPitch * detlaRRoll * distRPitch; // 优化后的R与x产生联系,依次右乘 Rotx,Roty,Rotz
        // Mat optimR = _cam.R * detlaRPitch * detlaRRoll; // 优化后的R与x产生联系
        //stereo rectify 为每个摄像头计算立体校正后的映射矩阵。所以其运行结果并不是直接将图片进行立体矫正，而是得出进行立体矫正所需要的映射矩阵。
        Mat R1, R2, P1, P2, Q;
        stereoRectify(_cam.K1, _cam.D1, _cam.K2, _cam.D2, _cam.imgSize, optimR, _cam.T, R1, R2, P1, P2, Q); //只优化旋转optimR
        // cameraMatrix1-第一个摄像机的摄像机矩阵
        // distCoeffs1-第一个摄像机的畸变向量
        // cameraMatrix2-第二个摄像机的摄像机矩阵
        // distCoeffs1-第二个摄像机的畸变向量
        // imageSize-图像大小
        // R- stereoCalibrate() 求得的R矩阵
        // T- stereoCalibrate() 求得的T矩阵
        // R1-输出矩阵，第一个摄像机的校正变换矩阵（旋转变换）
        // R2-输出矩阵，第二个摄像机的校正变换矩阵（旋转矩阵）
        // P1-输出矩阵，第一个摄像机在新坐标系下的投影矩阵
        // P2-输出矩阵，第二个摄像机在想坐标系下的投影矩阵
        // Q-4*4的深度差异映射矩阵
        // 由两个相机内参K和畸变矩阵，图像大小，两个相机相对旋转平移，求解两个相机的校正变换矩阵R和新坐标系下的投影矩阵P，以及深度差异映射矩阵

        // 鱼眼相机 cv::fisheye::undistortPoints
        //  points rectify
        vector<Point2f> rectpts1, rectpts2;
        undistortPoints(_data[0], rectpts1, _cam.K1, _cam.D1, R1, P1); //输入点,去畸变后的点,相机内参,畸变系数;3x3旋转矩阵,使双目左右极线平行;投影矩阵
        undistortPoints(_data[1], rectpts2, _cam.K2, _cam.D2, R2, P2);

        // cost :L1/L2
        for(size_t i =0; i< rectpts1.size(); i++)
        {
            fvec[i] = abs(rectpts1[i].y - rectpts2[i].y); //残差，两个相机在新坐标系下的投映点坐标差为误差
        }
        return 0;
    }

private:
    vector<vector<Point2f> > _data;
    CamParams _cam;
};


class CamPitchRollFactor{
public:
    CamPitchRollFactor(const Point2f& p1, const Point2f& p2):_cam(CamParams()) {
        p1_.x=p1.x;
        p1_.y=p1.y;
        p2_.x=p2.x;
        p2_.y=p2.y;
    }
    bool operator()(const double* const pitch_roll, double* residual) const{
        double pitch = pitch_roll[0];
        double roll  = pitch_roll[1];

        double arrDeltaRPitch[9] = {double(1.), double(0.), double(0.),
                                    double(0.),cos(pitch),-sin(pitch),
                                    double(0.),sin(pitch),cos(pitch)};
        Mat deltaRPitch = Mat(3,3,CV_64F, arrDeltaRPitch);
        // roll
        double arrDetlaRRoll[9] = { cos(roll),-sin(roll), double(0.),
                                    sin(roll),cos(roll),double(0.),
                                    double(0.),double(0.),double(1.)   };
        Mat deltaRRoll = Mat(3,3,CV_64F, arrDetlaRRoll);
        double distPitch = 3.14159265/180 * 2;
        double arrDistPitch[9] = {1.,0.,0.,
                                    0.,cos(distPitch), -sin(distPitch),
                                    0.,sin(distPitch), cos(distPitch)};
        
        Mat distRPitch = Mat(3,3,CV_64F, arrDistPitch);
        Mat optimR = _cam.R*deltaRPitch*deltaRRoll*distRPitch;
        Mat R1, R2, P1, P2, Q;
        stereoRectify(_cam.K1, _cam.D1, _cam.K2, _cam.D2, _cam.imgSize, optimR, _cam.T, R1, R2, P1, P2, Q); //只优化旋转optimR
        vector<Point2d> rectpts1, rectpts2;
        undistortPoints(vector<Point2d>{p1_}, rectpts1, _cam.K1, _cam.D1, R1, P1); //输入点,去畸变后的点,相机内参,畸变系数;3x3旋转矩阵,使双目左右极线平行;投影矩阵
        undistortPoints(vector<Point2d>{p2_}, rectpts2, _cam.K2, _cam.D2, R2, P2);

        residual[0]=double(rectpts1[0].y)-double(rectpts2[0].y);
        return true;
    }
    static ceres::CostFunction* Create(const Point2f& p1, const Point2f& p2){
        return new ceres::NumericDiffCostFunction<CamPitchRollFactor,ceres::FORWARD,1,2>(new CamPitchRollFactor(p1,p2));
    }

private:
    CamParams _cam;
    Point2d p1_;
    Point2d p2_;
};


class CameraCameraSync
{
public:
    // 说明：获取图像的时间戳，这里的时间戳就是文件名，所以可以理解为直接获取文件名
    // 然后将获取的文件列表保存在两个队列中，方便后续找到对应的时间戳
    void getImageTimeStamp(std::string oriDirName, std::string dstDirName);
    void findMatchPoints(const Mat srcImage1, const Mat srcImage2, vector<Point2f>& pts1, vector<Point2f>& pts2);

    int getImageNumber();

    // 说明：返回两个图像时间最接近的图像
    std::vector<std::pair<std::string, std::string> > imageTimeStampSyncFuncion();

    // 说明：评估两个图像的时间是否最接近的方法
    // 假设已经完成了时间硬件同步且两者曝光时间、帧率相同，内参一致，那么两个相机帧之间不会相差太多,
    // 如果完全同步，则两者的图像距离最接近，所以采用距离信息进行评价.
    // 假设 队列A中的元素n应该与队列B中的元素n距离最近，仅仅与B中的元素n-1，n+1进行比较，如果相差太多，那么认为时间硬件有问题
    double evaluateImageTimeStampSync(cv::Mat orgImage, cv::Mat dstImage);

    // 空间同步
    void spatialSynchronizationWithORB(cv::Mat image1, cv::Mat image2);
    void spatialSynchronizationWithSURF(cv::Mat image1, cv::Mat image2);
<<<<<<< HEAD
    bool synchronizePitchRoll(cv::Mat img_left, cv::Mat img_right);
    bool synchronizePitchRollWithCeres(cv::Mat img_left, cv::Mat img_right);
=======

>>>>>>> e85acb3439abcdd9dece6cd7feb04f57baff235e

private:
    std::vector<std::string> oriImageLists_;
    std::vector<std::string> dstImageLists_;

    float timeThreshold_;

    void getFiles(std::string path, std::vector<std::string>& files);
    double getbaseTime(std::string pngfilenames, std::string patt);

    std::vector<double> pitch_cache_;
    std::vector<double> roll_cache_;
};

#endif
