#include<iostream>
#include<fstream>
#include<string>
#include<sstream>
#include "KeyFunction.h"

using namespace std;

// OpenCV 特征检测模块
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/io/png_io.h>
#include <pcl/range_image/range_image.h>

typedef pcl::PointXYZRGB PointType;



int main(int argc, char** argv)
{
	
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>& point_cloud = *cloud;
	if (pcl::io::loadPCDFile<PointType>("pointcloud.pcd", *cloud) == -1)
	{
		PCL_ERROR("Could not read the file\n ");
		return -1;
	}
	system("pause");

	float angularResolution = (float)(0.1f * (M_PI / 180.0f));  //   1.0 degree in radians
	float maxAngleWidth = (float)(60.0f * (M_PI / 180.0f));  // 360.0 degree in radians
	float maxAngleHeight = (float)(40.0f * (M_PI / 180.0f));  // 180.0 degree in radians
	//Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
	Eigen::Affine3f sensorPose(Eigen::Affine3f::Identity());
	sensorPose = Eigen::Affine3f(Eigen::Translation3f(point_cloud.sensor_origin_[0],point_cloud.sensor_origin_[1],
		point_cloud.sensor_origin_[2])) *
		Eigen::Affine3f(point_cloud.sensor_orientation_);
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	
	float noiseLevel = 0.00;
	float minRange = 0.0f;
	int borderSize = 0;

	boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
	pcl::RangeImage& range_image = *range_image_ptr;

	range_image.createFromPointCloud(point_cloud, angularResolution, maxAngleWidth, maxAngleHeight,
		sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

	float* ranges = range_image.getRangesArray();
	unsigned char* rgb_image = pcl::visualization::FloatImageUtils::getVisualImage(ranges, range_image.width, range_image.height);
	pcl::io::saveRgbPNGFile("saveRangeImageRGB.png", rgb_image, range_image.width, range_image.height);

	/*double **array;
	array = new double*[10000];

	ofstream out("XYZ.txt");//打开文件。
	ofstream outrgb("rgb.txt");

	for (size_t m = 0; m < cloud->points.size(); ++m) {
		array[m] = new double[5];
		
		//int frgb=0;          
		//fscanf(fp,"%lf %lf %lf %lf %lf %lf",&array[m][0],&array[m][1],&array[m][2],&array[m][3],&array[m][4],&array[m][5]);//后面三列的输入类型也要为lf。
		array[m][0] = cloud->points[m].x;
		array[m][2] = -(cloud->points[m].y);
		array[m][1] = cloud->points[m].z;
		array[m][3] = (double)cloud->points[m].r;
		array[m][4] = (double)cloud->points[m].g;
		array[m][5] = (double)cloud->points[m].b;

		cout << "round finish" << endl;
		//system("pause");

		for (int j = 0; j <= 2; j++)
		{
			out << array[m][j] << ' ';//将每个元素写入文件
		}
		out << endl;//每行输出结束，添加换行。

		for (int j =3; j <= 5; j++)
		{
			outrgb << array[m][j] << ' ';//将每个元素写入文件
		}
		outrgb << endl;//每行输出结束，添加换行。
		
		//frgb=((int)array[m][3]<<16|(int)array[m][4]<<8|(int)array[m][5]);
		//cloud->points[m].rgb=*reinterpret_cast<float*>(&frgb);//frgb在这由int型转换成了float型。//reinterpret_cast()为指针类型转换。
	}

	/*for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		std::cout
			<< "    " << cloud->points[i].x
			<< " " << cloud->points[i].y
			<< " " << cloud->points[i].z 
			<< " " << (double)cloud->points[i].r
			<< " " << (double)cloud->points[i].g
			<< " " << (double)cloud->points[i].b
			<< std::endl;
		system("pause");
	}
	*/
	cout << "finish" << endl;


	/*for (size_t n = 0; n < cloud->points.size(); n++) {
		for (int j = 0; j <= 2; j++)
		{
			out << array[n][j] << ' ';//将每个元素写入文件
		}
		out << endl;//每行输出结束，添加换行。
	}*/
	system("pause");

	return (0);
}
