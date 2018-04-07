#include <fstream>
#include <vector>
#include <Eigen/Dense> 

using namespace std;
using namespace Eigen;
using Eigen::MatrixXd;

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>


#include <Eigen/Core>
#include <Eigen/Geometry>

// 类型定义
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 相机内参结构
struct CAMERA_INTRINSIC_PARAMETERS 
{ 
    double cx, cy, fx, fy, scale;
};

struct FRAME{
	cv::Mat rgb, depth;
	cv::Mat descriptor;
	vector<cv::KeyPoint> keypoint;
};

struct PnP{
	cv::Mat rvec, tvec;
	int inliers;
};

class Tem{
	
public:
	Eigen::Isometry3d T;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	
};


// image2PonitCloud 将rgb图转换为点云
PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera );

// point2dTo3d 将单个点从图像坐标转换为空间坐标
cv::Point3f point2dTo3d( cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera );

//特征点检测
void computeKeypoint(FRAME& frame);
//PnP 相机姿态计算
PnP estimateMap( FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& C );
//矩阵变换
Eigen::Isometry3d cvMat2Eigen(cv::Mat& rvec, cv::Mat& tvec);
//读取文件程序
FRAME ReadImage(string filename, string fileExtect, string filenum);
//运动距离估算
double normofTransform( cv::Mat rvec, cv::Mat tvec );
//合并点云图
PointCloud::Ptr joinPointCloud( PointCloud::Ptr original,int min_inliers,double max_norm,FRAME& oldFrame, FRAME& newFrame, CAMERA_INTRINSIC_PARAMETERS camera );