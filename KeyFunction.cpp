#include "KeyFunction.h"

void computeKeypoint(FRAME& frame){
// 声明特征提取器与描述子提取器
    cv::Ptr<cv::FeatureDetector> _detector;
    cv::Ptr<cv::DescriptorExtractor> _descriptor;

    cv::initModule_nonfree();
    _detector = cv::FeatureDetector::create("GridSURF");
    _descriptor = cv::DescriptorExtractor::create("SURF");

	
	_detector -> detect(frame.rgb,frame.keypoint );  //提取关键点
	_descriptor -> compute(frame.rgb,frame.keypoint,frame.descriptor );
	
	cv::Mat imgShow;
	cv::drawKeypoints( frame.rgb, frame.keypoint, imgShow, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
	//cv::imshow( "keypoints", imgShow );
    //cv::imwrite( "keypoints.png", imgShow );
	return;
}

cv::Point3f point2dTo3d( cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    cv::Point3f p; // 3D 点

	//重要！！！不要将坐标系标乱！要和相机的一样！调了一整个下午的教训
    p.z = double( point.z ) / camera.scale;
    p.x = ( point.x - camera.cx) * p.z / camera.fx;
    p.y = ( point.y - camera.cy) * p.z / camera.fy;
    return p;
}

PnP estimateMap(FRAME& frame1, FRAME& frame2,CAMERA_INTRINSIC_PARAMETERS& C){
	
	vector< cv::DMatch > matches; 
    cv::FlannBasedMatcher matcher;
	matcher.match( frame1.descriptor, frame2.descriptor, matches );
	cout<<"Find total "<<matches.size()<<" matches."<<endl;
	
	cv::Mat imgMatch;
	cv::drawMatches(frame1.rgb,frame1.keypoint, frame2.rgb,frame2.keypoint,matches,imgMatch);
	//cv::imshow("matches", imgMatch);
	//system("pause");
	//cv::imwrite("matches.png", imgMatch);


	vector< cv::DMatch > goodMatches; 
	double minDis = 9999;
    for ( size_t i=0; i<matches.size(); i++ )
    {
        if ( matches[i].distance < minDis )
            minDis = matches[i].distance;
    }

    for ( size_t i=0; i<matches.size(); i++ )
    {
        if (matches[i].distance < 4*minDis)
            goodMatches.push_back( matches[i]);
    }

    // 显示 good matches
    cout<<"good matches="<<goodMatches.size()<<endl;
	
	cv::Mat goodMatch;
	cv::drawMatches(frame1.rgb,frame1.keypoint, frame2.rgb,frame2.keypoint,goodMatches,goodMatch);
	//cv::imshow("matches", goodMatch);
	
	cv::imwrite("goodmatches.png", goodMatch);
	system("pause");
    vector<cv::Point3f> pts_obj;
    
    vector< cv::Point2f > pts_img;
 

    for (size_t i=0; i<goodMatches.size(); i++)
    {
		cv::Point2f p = frame1.keypoint[goodMatches[i].queryIdx].pt;
        // x是向右的，y是向下的，y是行，x是列
		ushort d = frame1.depth.ptr<ushort>( int(p.y) )[ int(p.x) ];
        if (d == 0)
            continue;
		pts_img.push_back( cv::Point2f( frame2.keypoint[goodMatches[i].trainIdx].pt ) );

        // 将(u,v,d)转成(x,y,z)
        cv::Point3f pt ( p.x, p.y, d );
        cv::Point3f pd = point2dTo3d( pt, C );
        pts_obj.push_back( pd );
    }

    double camera_matrix_data[3][3] = {
        {C.fx, 0, C.cx},
        {0, C.fy, C.cy},
        {0, 0, 1}
    };

    // 构建相机矩阵
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    cv::Mat rvec, tvec, inliers;
    // 求解pnp
    cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 100, inliers );
	cout<<"solve PNP"<<endl;
    PnP result;
	result.rvec = rvec;
	result.tvec = tvec;
	result.inliers = inliers.rows;

	cout<<"R = "<<rvec<<endl;
	cout<<"T = "<<tvec<<endl;

	return result;
	}

Eigen::Isometry3d cvMat2Eigen(cv::Mat& rvec, cv::Mat& tvec){
	cv::Mat R;
	cv::Rodrigues(rvec,R);
	Eigen::Matrix3d S;
	cv::cv2eigen(R,S);

	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
	Eigen::AngleAxisd angle(S);
   // Eigen::Translation<double,3> trans(tvec.at<double>(0,0), tvec.at<double>(0,1), tvec.at<double>(0,2));
    T = angle;
    T(0,3) = tvec.at<double>(0,0); 
    T(1,3) = tvec.at<double>(1,0); 
    T(2,3) = tvec.at<double>(2,0);
    return T;

}

//计算运动大小 |min(r,2π-r)|+|Δt|
double normofTransform( cv::Mat rvec, cv::Mat tvec ){
     return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}

PointCloud::Ptr joinPointCloud( PointCloud::Ptr original,int min_inliers,double max_norm,FRAME& oldFrame, FRAME& newFrame, CAMERA_INTRINSIC_PARAMETERS camera ){
	//PointCloud::Ptr cloud = image2PointCloud( oldFrame.rgb, oldFrame.depth, camera );
	PointCloud::Ptr newCloud = image2PointCloud( newFrame.rgb, newFrame.depth, camera );
	computeKeypoint(oldFrame);
	computeKeypoint(newFrame);
	PointCloud::Ptr flag( new PointCloud() );
        
	// 比较currFrame 和 lastFrame
         PnP result = estimateMap( oldFrame, newFrame, camera );
		 cout<<" inliers = "<<result.inliers<<endl;
		 if ( result.inliers < min_inliers ) //inliers不够，放弃该帧
             return flag;
         
	// 计算运动范围是否太大
		 else if(result.inliers >= min_inliers){
		 double norm = normofTransform(result.rvec, result.tvec);
         cout<<"norm = "<<norm<<endl;
        
		 if ( norm >= max_norm )
             return flag;
		
		 else if(norm< max_norm){
		 Eigen::Isometry3d T = cvMat2Eigen( result.rvec, result.tvec );
		 cout<<"T = "<<T.matrix()<<endl;
		
 
     // 合并点云
     PointCloud::Ptr output (new PointCloud());
     pcl::transformPointCloud( *original, *output,T.matrix() );
     *newCloud += *output;
 
     //滤波降采样
     static pcl::VoxelGrid<PointT> voxel;
     double gridsize = 0.05; //采样率，越小重建结果越精细
     voxel.setLeafSize( gridsize, gridsize, gridsize );
     voxel.setInputCloud( newCloud );
     PointCloud::Ptr tmp( new PointCloud() );
     voxel.filter( *tmp );
     //return tmp;
	 return newCloud;
		 }
	}
}


PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& C )
{
    PointCloud::Ptr cloud ( new PointCloud );

	for (int m = 0; m < depth.rows; m++)
		for (int n = 0; n < depth.cols; n++)
		{
			if(depth.ptr<ushort>(m)[n] >=5000){
				depth.ptr<ushort>(m)[n]=0;
			}
		}
		

    for (int m = 0; m < depth.rows; m++)
        for (int n=0; n < depth.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            // d 没有值，则跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointT p;

            // 计算这个点的空间坐标
            p.z = double(d) / C.scale;
            p.x = (n - C.cx) * p.z / C.fx;
            p.y = (m - C.cy) * p.z / C.fy;
            
            // 从rgb图像中获取它的颜色
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            // 把p加入到点云中
            cloud->points.push_back( p );
        }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    return cloud;
}

FRAME ReadImage(string filename, string fileExtect, string filenum){
	FRAME frame;
	string  fileFolder1 = filename+"/rgb/"+filenum+"彩色图640480.jpg";
	string  fileFolder2 = filename+"/depth/"+filenum+"为16位与彩色图对齐的深度图640480.png";
	frame.rgb= cv::imread(fileFolder1);
	frame.depth = cv::imread(fileFolder2,-1);
	cout<<"done"<<endl;
	return frame; 
}
