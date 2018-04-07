#include<iostream>
#include<fstream>
#include<string>
#include<sstream>
#include "KeyFunction.h"

using namespace std;

// OpenCV �������ģ��
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/range_image/range_image.h>


int main( int argc, char** argv )
{
    
	//������� �������õ���ͼƬ����������
	CAMERA_INTRINSIC_PARAMETERS C;
	/*C.cx = 325.5;
	C.cy = 253.5;
	C.fx = 518.0;
	C.fy = 519.0;
	C.scale = 1000.0;*/
    /*C.cx = 319.5;
    C.cy = 239.5;
    C.fx = 525.0;
    C.fy = 525.0;
    C.scale = 5000.0;*/

	C.cx = 330.8;
	C.cy = 229.5;
	C.fx = 616.0;
	C.fy = 621.5;
    C.scale =1000.0;
	
	
	string filepath;
	string filetype;

	filepath = "E:/ZJU/data/pic";
	filetype = ".png";
	/*cout<<"Please input the file path (which contains rgb and depth in it):"<<endl;
	cin>>filepath;

	cout<<"Please input the file type(.jpg or .png):"<<endl;
	cin>>filetype;*/

	int startRead = 16;
	int endRead = 20;

	string lastFramename;
	strstream temp;
	temp<<startRead;
	temp>>lastFramename;
	FRAME lastFrame = ReadImage(filepath,filetype, lastFramename );
	computeKeypoint(lastFrame);
	PointCloud::Ptr cloud = image2PointCloud( lastFrame.rgb, lastFrame.depth, C );
 
	 pcl::visualization::CloudViewer viewer("viewer");
	 //PointCloud::Ptr cloudnew = image2PointCloud( lastFrame.rgb, lastFrame.depth, C );
	 PointCloud::Ptr test(new PointCloud());
	 PointCloud::Ptr tempcloud(new PointCloud());


	for( int currReadnum = startRead+1;currReadnum<=endRead;currReadnum++){
	   string newRead;
	   strstream tempname;
	   tempname<<currReadnum;
	   tempname>>newRead;

	   cout<<newRead<<endl;
	   FRAME newFrame = ReadImage(filepath,filetype, newRead );


	   tempcloud = joinPointCloud(cloud,5,0.3,lastFrame,newFrame,C);//5 Ϊinliers��С��ֵ��0.3Ϊ����������ֵ

	   
	   //�������Ե���û�а취���д����������һ��ͼƬ�Ķ�ȡ
	   if(tempcloud==test){
	   viewer.showCloud(cloud );
	   lastFrame = newFrame; 
	    continue;
	   }
	   cloud = tempcloud;
	   viewer.showCloud(cloud );  
	   lastFrame = newFrame;  

   }

	

   pcl::io::savePCDFile("result.pcd",*cloud);
   
   system("pause");

   double **array_xyz;
   double **array_rgb;
   array_xyz = new double*[cloud->points.size()];
   array_rgb = new double *[cloud->points.size()];

   ofstream out("XYZ.txt");//���ļ���
   ofstream outrgb("rgb.txt");

   for (size_t m = 0; m < cloud->points.size(); ++m) {
	   array_xyz[m] = new double[2];
	   array_rgb[m] = new double[2];

	   //int frgb=0;          
	   //fscanf(fp,"%lf %lf %lf %lf %lf %lf",&array[m][0],&array[m][1],&array[m][2],&array[m][3],&array[m][4],&array[m][5]);//�������е���������ҲҪΪlf��
	   array_xyz[m][0] = cloud->points[m].x;
	   array_xyz[m][2] = -(cloud->points[m].y);
	   array_xyz[m][1] = cloud->points[m].z;
	   array_rgb[m][0] = (double)cloud->points[m].r;
	   array_rgb[m][1] = (double)cloud->points[m].g;
	   array_rgb[m][2] = (double)cloud->points[m].b;

	   //cout << "round finish" << endl;
	   //system("pause");

	   for (int j = 0; j <= 2; j++)
	   {
		   out << array_xyz[m][j] << ' ';//��ÿ��Ԫ��д���ļ�
	   }
	   out << endl;//ÿ�������������ӻ��С�

	   for (int j = 0; j <= 2; j++)
	   {
		   outrgb << array_rgb[m][j] << ' ';//��ÿ��Ԫ��д���ļ�
	   }
	   outrgb << endl;//ÿ�������������ӻ��С�

					  //frgb=((int)array[m][3]<<16|(int)array[m][4]<<8|(int)array[m][5]);
					  //cloud->points[m].rgb=*reinterpret_cast<float*>(&frgb);//frgb������int��ת������float�͡�//reinterpret_cast()Ϊָ������ת����
	  // system("pause");
   }


     
   /*while( !viewer.wasStopped() )
    {
        
    }*/

   /*float angularResolution = (float)(1.0f * (M_PI / 180.0f));
   float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));
   float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));
   Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
   pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
   float noiseLevel = 0.00;    //noise_level��ȡ���ͼ�����ʱ�����ڵ�Բ�ѯ�����ֵ��Ӱ��ˮƽ
   float minRange = 0.0f;     //min_range������С�Ļ�ȡ���룬С����С��ȡ�����λ��Ϊ��������ä��
   int borderSize = 1;        //border_size������ͼ��ı�Ե�Ŀ��

   pcl::RangeImage rangeImage;
   rangeImage.createFromPointCloud(*cloud, angularResolution, maxAngleWidth, maxAngleHeight,
	   sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
   */
   system("pause");
    return 0;
}


