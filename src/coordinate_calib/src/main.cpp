/*
 * @Author: mikey.zhaopeng 
 * @Date: 2021-08-30 18:11:21 
 * @Last Modified by: mikey.zhaopeng
 * @Last Modified time: 2021-08-30 18:32:18
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include  <opencv2/core/core_c.h>
#include  <opencv/cv.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <thread>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <string.h>
#include <atomic>
#include <math.h>
#include <sensor_msgs/PointField.h>
#include  <ros/io.h>
//#include <avoid_ob_msgs/avoid_result.h>

using namespace cv;
using namespace std;

ros::Subscriber  camerainfo_sub;
image_transport::Subscriber image_raw1_sub;

ros::Subscriber  corlor_camera_info_sub;
image_transport::Subscriber image_color_sub;

Mat pointcloud;

bool data_flag;
int imgw,imgh;
Mat image1_infra;

Mat intrinsic_matrix;   //
Mat distortion_coeffs;  //

Mat   src,src1;//   原图片

cv::Size board_size;   //qi pan ge ci cun
int Chessboard_width;  //棋盘格宽度 mm单位
vector<Point2f> corners;
vector<vector<Point2f>>  corners_Seq;   // 所有有效图片的点集合
vector<vector<Point3f>>  object_Points; // 世界坐标集合

bool calib_flag;

vector<float> rvec;
vector<float> tvec;

vector<vector<float>> rvec1;
vector<vector<float>> tvec1;

void init_param(){

  src=cv::imread("/home/flfjepl/catkin_ws_realsense/src/coordinate_calib/pic/22_Infrared.png",0);  //读取一个通道图像
  src1=cv::imread("/home/flfjepl/catkin_ws_realsense/src/coordinate_calib/pic/22_Infrared.png");  //读取

  if(!src.empty())

  ROS_INFO_STREAM_ONCE("src"<<src.channels());
  else
    
  ROS_INFO_STREAM_ONCE("nom src");

  
  data_flag=false;
  calib_flag=false;

  board_size.width=11;
  board_size.height=8;
  intrinsic_matrix.create(3,3,CV_32FC1);
  distortion_coeffs.create(1,5,CV_32FC1);

  Chessboard_width=25;

  vector<Point3f> tempPointSet;
     for(int i = 0; i<board_size.height; i++)
     {
       for (int j = 0; j<board_size.width; j++)
         {
             /* 假设定标板放在世界坐标系中z=0的平面上 */
             Point3f tempPoint;
             tempPoint.y = i*Chessboard_width;   //本来100?
             tempPoint.x = j*Chessboard_width;
             tempPoint.z = 0;
             tempPointSet.push_back(tempPoint);
         }
     }
  object_Points.push_back(tempPointSet);

}


void  imageColorCallback(const sensor_msgs::ImageConstPtr& msg)
   {

    if(!calib_flag) return;

     Mat image_color;


     cv_bridge::toCvShare(msg, "8UC3")->image.copyTo(image_color);

     if((!data_flag)&&(!image1_infra.empty()))   data_flag=true;

     Mat image=image_color;
     cvtColor(image_color,image,CV_BGR2GRAY);
     bool patternFind=findChessboardCorners(image_color,board_size,corners,CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE+CALIB_CB_FAST_CHECK);

     printf("patternFind: %d conrners sz: %d\n ",patternFind,corners.size());


     if(patternFind)
     {
      cornerSubPix(image,corners,Size(3,3),Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));
      corners_Seq.push_back(corners);
      for (int i = 0; i < corners.size(); i++)
       {
        if(i<15)
        circle(image_color, corners[i], 2, Scalar(255,0,0),2);
        else
        circle(image_color, corners[i], 2, Scalar(0,0,255),2);

        if(i==0){
         char buffer[4];
         sprintf(buffer, "%d", i+1);
         putText(image, buffer, corners[i], 1, 1, Scalar(255), 1);
        }
       }

       printf("sovepnp before\n");
       rvec.clear();
       tvec.clear();
       solvePnP(object_Points[0], corners, intrinsic_matrix, distortion_coeffs, rvec,  tvec,   false,   SOLVEPNP_ITERATIVE );
       printf("sovepnp after\n");

       printf("calibratecamera before\n");
//       double rms = cv::calibrateCamera(
//                   object_Points,
//                   corners_Seq,
//                   Size(640,480),
//                   intrinsic_matrix,
//                   distortion_coeffs,
//                   rvec1,
//                   tvec1, 0, cv::TermCriteria(3, 20, 1e-6));
       printf("calibratecamera after\n");

      }

  Mat m;

  if(patternFind){
   printf("rvec:");
   for(int i=0;i<rvec.size();i++)
       printf("  %f",rvec[i]);
   printf("\n");

   printf("tvec:");
   for(int i=0;i<tvec.size();i++)
       printf("  %f",tvec[i]);
   printf("\n");

//   printf("rvec1:");
//   for(int i=0;i<rvec1[0].size();i++)
//       printf("  %f",rvec1[0][i]);
//   printf("\n");

//   printf("tvec1:");
//   for(int i=0;i<tvec1[0].size();i++)
//       printf("  %f",tvec1[0][i]);
//   printf("\n");
}

      printf("3333333333333\n");
      /*
      double rms = cv::calibrateCamera(
                  object_Points,
                  corners_Seq,
                  image_size,
                  intrinsic_matrix,
                  distortion_coeffs,
                  rotation_vectors,
                  translation_vectors, 0, cv::TermCriteria(3, 20, 1e-6));*/

      cv::imshow("image_corlor", image_color);
      cv::waitKey(10);


    //     image1_3D_mutex.lock();
    //     cv_bridge::toCvShare(msg, "32FC3")->image.copyTo(image1_3D);
    //     image1_3D_mutex.unlock();

        // printf("imag1\n");


}

void  CameraCorlorInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info)
{
   sensor_msgs::CameraInfo camer_info=*cam_info;

   int len= camer_info.D.size();

   for(int i=0;i<len;i++){
     distortion_coeffs.data[i]=camer_info.D[i];
   }

   len=camer_info.K.size();
   for(int i=0;i<len;i++){
     intrinsic_matrix.data[i]=camer_info.K[i];
   }

   std::cout<<"D:camer_info"<<camer_info.D[0]<<"  "<<camer_info.D[1]<<"  "<<camer_info.D[2]<<std::endl;
   std::cout<<"K:"<<camer_info.K[0]<<"  "<<camer_info.K[1]<<"  "<<camer_info.K[2]<<std::endl;
   std::cout<<"K:"<<camer_info.K[3]<<"  "<<camer_info.K[4]<<"  "<<camer_info.K[5]<<std::endl;
   std::cout<<"K:"<<camer_info.K[6]<<"  "<<camer_info.K[7]<<"  "<<camer_info.K[8]<<std::endl;
   //  std::cout<<"P:"<<camer_info.P<<std::endl;
    std::cout<<"R:"<<camer_info.R[0]<<std::endl;
    std::cout<<"binning_x:"<<camer_info.binning_x<<std::endl;
    std::cout<<"binning_y:"<<camer_info.binning_y<<std::endl;

    calib_flag=true;
}


void  camera_calibration_extrisic_mat(){

   // bool patternFind=findChessboardCorners(src,board_size,corners,CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE+CALIB_CB_FAST_CHECK);
 bool patternFind=findChessboardCorners(src,board_size,corners,CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE);
    printf("patternFind: %d conrners sz: %d\n ",patternFind,corners.size());


     if(patternFind)
     {
      cornerSubPix(src,corners,Size(3,3),Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));   //输入图像必须为单通道灰度图
      //corners_Seq.push_back(corners);
      
      
      for (int i = 0; i < 8; i++)
       {
        if(i<15)
        circle(src, corners[i], 2, Scalar(125),-3);
        else
        circle(src, corners[i], 5, Scalar(125),-3);

/*
        if(i==0){
         char buffer[4];
         sprintf(buffer, "%d", i+1);
         putText(src1, buffer, corners[i], 1, 1, Scalar(255,0,0), 1);
        }*/
       }
    // drawChessboardCorners(src1, board_size, Mat(corners), patternFind);

      // ROS_INFO_STREAM("coner:"<<corners);

/*
       namedWindow("tu",  0);
         while(1)
         {

          imshow("tu",src);
          waitKey(10);

         }*/


        CvMat* image_points = cvCreateMat(corners.size(), 2, CV_32FC1);
	      CvMat* object_points = cvCreateMat(corners.size(), 3, CV_32FC1);

       // CvMat* image_points = cvCreateMat(corners.size(), 2, CV_32FC1);
	      CvMat* object_points1 = cvCreateMat(corners.size(), 3, CV_32FC1);


 
	
		if(corners.size() == board_size.area())
		{
			for(int i = 0; i < board_size.height; i++)
			{
				for(int j = 0; j < board_size.width; j++)
				{
					CV_MAT_ELEM(*image_points, float, i*board_size.width+j, 0) = corners[i*board_size.width+j].x;
					CV_MAT_ELEM(*image_points, float, i*board_size.width+j, 1) = corners[i*board_size.width+j].y;
					CV_MAT_ELEM(*object_points, float, i*board_size.width+j, 0) = i;
					CV_MAT_ELEM(*object_points, float, i*board_size.width+j, 1) = j;
					CV_MAT_ELEM(*object_points, float, i*board_size.width+j, 2) = 0.0f;

          CV_MAT_ELEM(*object_points1, float, i*board_size.width+j, 0) = i*Chessboard_width;
					CV_MAT_ELEM(*object_points1, float, i*board_size.width+j, 1) = j*Chessboard_width;
					CV_MAT_ELEM(*object_points1, float, i*board_size.width+j, 2) = 0.0f;
				}
			}
    }
    
  CvMat *intrinsic, *distortion;
	CvMat* rotation_vector = cvCreateMat(3, 1, CV_32FC1);
	CvMat* translation_vector = cvCreateMat(3, 1, CV_32FC1);


  	CvMat* rotation_vector1 = cvCreateMat(3, 1, CV_32FC1);
	CvMat* translation_vector1 = cvCreateMat(3, 1, CV_32FC1);

CvMat temp = intrinsic_matrix; //转化为CvMat类型，而不是复制数据  
cvCopy(&temp, intrinsic); //真正复制数据  
CvMat temp1 = distortion_coeffs; 
cvCopy(&temp1, distortion); //真正复制数据  

  cvFindExtrinsicCameraParams2(object_points, image_points, intrinsic, distortion, rotation_vector, translation_vector);

  cvFindExtrinsicCameraParams2(object_points1, image_points, intrinsic, distortion, rotation_vector1, translation_vector1);
			float x = Chessboard_width * CV_MAT_ELEM(*translation_vector, float, 0, 0);
			float y = Chessboard_width * CV_MAT_ELEM(*translation_vector, float, 1, 0);
			float z = Chessboard_width * CV_MAT_ELEM(*translation_vector, float, 2, 0);
			// 输出三维坐标，单位mm
			std::cout<<"x: "<<x<<" y: "<<y<<" z: "<<z<<std::endl;
   
      

    //Mat   rotation_mat  =*rotation_vector;
   //Mat   transfor_vec(translation_vector,false);
   //Mat   rotation_mat1(rotation_vector1,false);
    //Mat   transfor_vec1(translation_vector1,false);
    ROS_INFO_STREAM("rotation_mat:"<<rotation_vector);    
    ROS_INFO_STREAM("translation_vector:"<<translation_vector);

    ROS_INFO_STREAM("rotation_mat1:"<<rotation_vector);    
    ROS_INFO_STREAM("translation_vector:"<<translation_vector1);

/*
       printf("sovepnp before\n");
       rvec.clear();
       tvec.clear();
       solvePnP(object_Points[0], corners, intrinsic_matrix, distortion_coeffs, rvec,  tvec,   false,   SOLVEPNP_ITERATIVE );
       printf("sovepnp after\n");

       printf("calibratecamera before\n"); */
//       double rms = cv::calibrateCamera(
//                   object_Points,
//                   corners_Seq,
//                   Size(640,480),
//                   intrinsic_matrix,
//                   distortion_coeffs,
//                   rvec1,
//                   tvec1, 0, cv::TermCriteria(3, 20, 1e-6));
     //  printf("calibratecamera after\n");

      }



}
int main(int argc, char **argv)
{

  ros::init(argc, argv, "coord_calib");
  ros::NodeHandle nh;


  image_transport::ImageTransport it(nh);

  init_param();


camera_calibration_extrisic_mat();


 // camerainfo_sub = nh.subscribe("/camera/infra1/camera_info", 1 ,CameraInfoCallback);

 // pointcloud1_sub = nh.subscribe("/camera/depth/color/points", 1 , pointcloudCallback);

 // image_raw1_sub = it.subscribe("/camera/infra1/image_rect_raw", 1, imageCallback);

  //corlor_camera_info_sub = nh.subscribe("/camera/color/camera_info", 1 ,CameraCorlorInfoCallback);

  //image_color_sub = it.subscribe("/camera/color/image_raw", 1, imageColorCallback);
  
  printf("111111111111111\n");
  //ros::spin();

  //printf("2222222222222222222\n");
  //cv::destroyWindow("view");
}

