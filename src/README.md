1. camera camera snyc:本质上是两张图片做对极几何，求出H或者E，然后恢复位姿。但是需要内参矩阵。
	E恢复位姿：cv::revoverPose(E,pts1,pts2,R,t,focal_length,principal_point)
	H矩阵恢复位姿：参考ORB-SLAM2的代码和相关的论文
