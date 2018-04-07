#include "KeyFunction.h"

void computeKeypoint(FRAME& frame){
// ����������ȡ������������ȡ��
    cv::Ptr<cv::FeatureDetector> _detector;
    cv::Ptr<cv::DescriptorExtractor> _descriptor;

    cv::initModule_nonfree();
    _detector = cv::FeatureDetector::create("GridSURF");
    _descriptor = cv::DescriptorExtractor::create("SURF");

	
	_detector -> detect(frame.rgb,frame.keypoint );  //��ȡ�ؼ���
	_descriptor -> compute(frame.rgb,frame.keypoint,frame.descriptor );
	
	cv::Mat imgShow;
	cv::drawKeypoints( frame.rgb, frame.keypoint, imgShow, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
	//cv::imshow( "keypoints", imgShow );
    //cv::imwrite( "keypoints.png", imgShow );
	return;
}

cv::Point3f point2dTo3d( cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    cv::Point3f p; // 3D ��

	//��Ҫ��������Ҫ������ϵ���ң�Ҫ�������һ��������һ��������Ľ�ѵ
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

    // ��ʾ good matches
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
        // x�����ҵģ�y�����µģ�y���У�x����
		ushort d = frame1.depth.ptr<ushort>( int(p.y) )[ int(p.x) ];
        if (d == 0)
            continue;
		pts_img.push_back( cv::Point2f( frame2.keypoint[goodMatches[i].trainIdx].pt ) );

        // ��(u,v,d)ת��(x,y,z)
        cv::Point3f pt ( p.x, p.y, d );
        cv::Point3f pd = point2dTo3d( pt, C );
        pts_obj.push_back( pd );
    }

    double camera_matrix_data[3][3] = {
        {C.fx, 0, C.cx},
        {0, C.fy, C.cy},
        {0, 0, 1}
    };

    // �����������
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    cv::Mat rvec, tvec, inliers;
    // ���pnp
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

//�����˶���С |min(r,2��-r)|+|��t|
double normofTransform( cv::Mat rvec, cv::Mat tvec ){
     return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}

PointCloud::Ptr joinPointCloud( PointCloud::Ptr original,int min_inliers,double max_norm,FRAME& oldFrame, FRAME& newFrame, CAMERA_INTRINSIC_PARAMETERS camera ){
	//PointCloud::Ptr cloud = image2PointCloud( oldFrame.rgb, oldFrame.depth, camera );
	PointCloud::Ptr newCloud = image2PointCloud( newFrame.rgb, newFrame.depth, camera );
	computeKeypoint(oldFrame);
	computeKeypoint(newFrame);
	PointCloud::Ptr flag( new PointCloud() );
        
	// �Ƚ�currFrame �� lastFrame
         PnP result = estimateMap( oldFrame, newFrame, camera );
		 cout<<" inliers = "<<result.inliers<<endl;
		 if ( result.inliers < min_inliers ) //inliers������������֡
             return flag;
         
	// �����˶���Χ�Ƿ�̫��
		 else if(result.inliers >= min_inliers){
		 double norm = normofTransform(result.rvec, result.tvec);
         cout<<"norm = "<<norm<<endl;
        
		 if ( norm >= max_norm )
             return flag;
		
		 else if(norm< max_norm){
		 Eigen::Isometry3d T = cvMat2Eigen( result.rvec, result.tvec );
		 cout<<"T = "<<T.matrix()<<endl;
		
 
     // �ϲ�����
     PointCloud::Ptr output (new PointCloud());
     pcl::transformPointCloud( *original, *output,T.matrix() );
     *newCloud += *output;
 
     //�˲�������
     static pcl::VoxelGrid<PointT> voxel;
     double gridsize = 0.05; //�����ʣ�ԽС�ؽ����Խ��ϸ
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
            // ��ȡ���ͼ��(m,n)����ֵ
            ushort d = depth.ptr<ushort>(m)[n];
            // d û��ֵ���������˵�
            if (d == 0)
                continue;
            // d ����ֵ�������������һ����
            PointT p;

            // ���������Ŀռ�����
            p.z = double(d) / C.scale;
            p.x = (n - C.cx) * p.z / C.fx;
            p.y = (m - C.cy) * p.z / C.fy;
            
            // ��rgbͼ���л�ȡ������ɫ
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            // ��p���뵽������
            cloud->points.push_back( p );
        }
    // ���ò��������
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    return cloud;
}

FRAME ReadImage(string filename, string fileExtect, string filenum){
	FRAME frame;
	string  fileFolder1 = filename+"/rgb/"+filenum+"��ɫͼ640480.jpg";
	string  fileFolder2 = filename+"/depth/"+filenum+"Ϊ16λ���ɫͼ��������ͼ640480.png";
	frame.rgb= cv::imread(fileFolder1);
	frame.depth = cv::imread(fileFolder2,-1);
	cout<<"done"<<endl;
	return frame; 
}
