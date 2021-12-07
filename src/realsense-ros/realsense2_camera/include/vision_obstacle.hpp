#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include  <ros/ros.h>
#include  <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/message.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include  <Eigen/Dense>
#include  <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include   <pcl/filters/voxel_grid.h>
#include   <pcl/segmentation/sac_segmentation.h>
#include   <pcl/segmentation/extract_clusters.h>
#include   <pcl/filters/extract_indices.h>
#include   <pcl/filters/passthrough.h>
#include  <pcl/filters/radius_outlier_removal.h>
#include   <string.h>
using namespace cv;
using namespace std;
using namespace Eigen;

class   vision_obstacle
{
   public:
//内参矩阵
   Mat  intrinsic_matrix;
   Mat distortion_coeffs;  
   Mat  extrisic_mat;
   
  string  config_path;
//相机坐标系和标定板世界坐标系的 旋转  平移  (外参)
   Matrix<float,3,3>  rotata;  //旋转矩阵
   Vector3f   T_vec;   //平移向量
   Eigen::Matrix4f  transf;   //转换矩阵  旋转矩阵和平移向量的集合体

   pcl::PointCloud<pcl::PointXYZ>    cloud   ;    

  pcl::PointCloud<pcl::PointXYZ>    cloud_filter   ;  
   
  pcl::PointCloud<pcl::PointXYZ>    cloud_filter_pass   ;  

  pcl::PointCloud<pcl::PointXYZ>   cloud_out  ;


  //pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
   
    //pcl::PointCloud<pcl::PointXYZ>   pointcloud_base;
    pcl::PCLPointCloud2    pointcloud_camera;

    sensor_msgs::PointCloud2  pointcloud;   
    
    sensor_msgs::PointCloud2  cloud_out_pub;  

   bool  print_flag;   

   Mat  base_to_world;
    int  model;
    cv::Point3f  voxel_size, voxel_size_outdoor;
    float  distance_threshold, distance_threshold_outdoor;
    float epsAngel,epsAngel_outdoor;
    int   iteration_num , iteration_num_outdoor;
    Point2f  X_range,Y_range,Z_range;
    Point2f  X_range_outdoor,Y_range_outdoor,Z_range_outdoor;

    bool radis_rem_enable;
    float  radius_search;
    int   min_neighbors;

   vision_obstacle();

   int  read_config_param(string  config_path,bool  printflag);

   ~vision_obstacle();
   void   pointcloud_obstacle(sensor_msgs::PointCloud2&  msg);
 
     //点云稀疏
   void    PointCloud_DownSample(pcl::PointCloud<pcl::PointXYZ>&  src_pc  ,pcl::PointCloud<pcl::PointXYZ>&  dest_pc);

    //直通滤波器
   void    PointCloud_PassThrough(pcl::PointCloud<pcl::PointXYZ>&  src_pc  ,pcl::PointCloud<pcl::PointXYZ>&  dest_pc);

   void    PointCloud_PassThrough_Outdoor(pcl::PointCloud<pcl::PointXYZ>&  src_pc  ,pcl::PointCloud<pcl::PointXYZ>&  dest_pc);

  void    PointCloud_RadiusFilter(pcl::PointCloud<pcl::PointXYZ>&  src_pc  ,pcl::PointCloud<pcl::PointXYZ>&  dest_pc);
    //点云坐标 转换
   void   PointCloud_CoordTranform(pcl::PointCloud<pcl::PointXYZ>&  src_pc  ,pcl::PointCloud<pcl::PointXYZ>&  dest_pc,Eigen::Matrix4f  transf);
     //点云分割
   void  PointCloud_Segment(pcl::PointCloud<pcl::PointXYZ>&  src_pc  ,pcl::PointCloud<pcl::PointXYZ>&  floor_pc,pcl::PointCloud<pcl::PointXYZ>&  nofloor_pc);
  
  void  PointCloud_Segment_Outdoor(pcl::PointCloud<pcl::PointXYZ>&  src_pc  ,pcl::PointCloud<pcl::PointXYZ>&  floor_pc,pcl::PointCloud<pcl::PointXYZ>&  nofloor_pc);
    
};
   