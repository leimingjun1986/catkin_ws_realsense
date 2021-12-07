#include "vision_obstacle.hpp"
 #include  <ros/console.h>

// 构造函数
   vision_obstacle::vision_obstacle(ros::NodeHandle&  NH):nh(NH) {
     config_path="/data/vision_obstacle/config/config.xml";
     read_config_param(config_path,true);
     point_cloud_pub=nh.advertise<sensor_msgs::PointCloud2>("/pcloud_transform_base",1);
     point_cloud_pass_pub=nh.advertise<sensor_msgs::PointCloud2>("/pcloud_passthrough",1);
     point_cloud_nofloor_pub=nh.advertise<sensor_msgs::PointCloud2>("/pcloud_obstacle",1);
    point_cloud_floor_pub=nh.advertise<sensor_msgs::PointCloud2>("/pcloud_floor",1);
     point_cloud_sub =nh.subscribe("/camera/depth/color/points",1, &vision_obstacle::callback_pointcloud,this);
    
      print_flag=true;

      //用eigen进行矩阵运算
      //rotata<<0.0123 ,  -0.9999,  0.0120  , 0.5089  ,  0.0166  ,0.8606 ,-0.8607,-0.0045,0.5091;     //matlab 旋转矩阵转置
      // rotata<<0.0123 ,  0.5089  ,-0.8607,   -0.9999,    0.0166  , -0.0045, 0.0120  , 0.8606 ,0.5091;
     // T_vec<<50.0931 , -117.0355 , 1210.9;
     // T_vec= T_vec/1000.0 ;
      
      cv2eigen(extrisic_mat(Rect(0,0,3,3)),  rotata);    // opencv  mat转  eigen
      cv2eigen(extrisic_mat(Rect(3,0,1,3)),   T_vec);

     ROS_INFO_STREAM("extrisic_mat:"<<extrisic_mat);
     ROS_INFO_STREAM("rotata:"<<rotata);
     ROS_INFO_STREAM("T_vec:"<<T_vec);

      Matrix<float,3,3>  rotat_transpose=rotata.transpose(); 
      Vector3f  T_vec1 = (-rotat_transpose) *T_vec;

       transf  = Eigen::Matrix4f::Identity();//单位矩阵   
      //得到变换矩阵
       transf(0,0)=rotat_transpose(0,0);
       transf(0,1)=rotat_transpose(0,1);
       transf(0,2)=rotat_transpose(0,2);
       transf(1,0)=rotat_transpose(1,0);
       transf(1,1)=rotat_transpose(1,1);
       transf(1,2)=rotat_transpose(1,2);
       transf(2,0)=rotat_transpose(2,0);
       transf(2,1)=rotat_transpose(2,1);
       transf(2,2)=rotat_transpose(2,2);
       transf(0,3)=T_vec1(0,0);
       transf(1,3)=T_vec1(1,0);
       transf(2,3)=T_vec1(2,0);

      
        ROS_INFO_STREAM("TRANSF:"<<transf);
   }

//读取配置文件  算法所需所有参数
int  vision_obstacle::read_config_param(string  config_path,bool  printflag){

   cv::FileStorage  fs;
  bool re= fs.open(config_path,cv::FileStorage::READ);
   if(!re)
    {
      ROS_INFO_STREAM("Open config xml:"<<config_path<<"  error");
    }
    else{
      ROS_INFO_STREAM("Open  config xml:"<<config_path<<"  successful");
    }
   
   fs["extrinsic"]>>extrisic_mat;
   fs["base_to_world"]>>base_to_world;
  
  fs["model"]>>model;
   fs["voxel_grid"]>>voxel_size;
   fs["distance_threshold"]>>distance_threshold;
  fs["EpsAngle"]>>epsAngel;
  fs["X_range"]>>X_range;
  fs["Y_range"]>>Y_range;
  fs["Z_range"]>>Z_range;
  fs["iterations_num"]>>iteration_num;

   fs["voxel_grid_outdoor"]>>voxel_size_outdoor;
  fs["distance_threshold_outdoor"]>>distance_threshold_outdoor;
  fs["EpsAngle_outdoor"]>>epsAngel_outdoor;
  fs["X_range_outdoor"]>>X_range_outdoor;
  fs["Y_range_outdoor"]>>Y_range_outdoor;
  fs["Z_range_outdoor"]>>Z_range_outdoor;
  fs["iterations_num_outdoor"]>>iteration_num_outdoor; 

  fs["RadiusOutlierRemoval_Enable"]>>radis_rem_enable;
  fs["RadiusSearch"]>>radius_search;
  fs["MinNeighbors"]>>min_neighbors;

 if(printflag){
  std::cout<<"model:"<<model<<std::endl;
  std::cout<<"extrinsic:"<<extrisic_mat<<std::endl;
  std::cout<<"base_to _world:"<<base_to_world<<std::endl;
  std::cout<<"voxe_grid:"<<voxel_size<<std::endl;
  std::cout<<"distance_threshold"<<distance_threshold<<std::endl;
  std::cout<<"EpsAngle"<<epsAngel<<std::endl;
  std::cout<<"X_range"<<X_range<<std::endl;
  std::cout<<"Y_range"<<Y_range<<std::endl;
  std::cout<<"Z_range"<<Z_range<<std::endl;

   std::cout<<"voxe_grid_outdoor:"<<voxel_size_outdoor<<std::endl;
  std::cout<<"distance_threshold_outdoor"<<distance_threshold_outdoor<<std::endl;
  std::cout<<"EpsAngle_outdoor"<<epsAngel_outdoor<<std::endl;
  std::cout<<"X_range_outdoor"<<X_range_outdoor<<std::endl;
  std::cout <<"Y_range_outdoor"<<Y_range_outdoor<<std::endl;
  std::cout<<"Z_range_outdoor"<<Z_range_outdoor<<std::endl;
 }
  
  fs.release();
  return 1;
}


vision_obstacle::~vision_obstacle(){
       ;
   }
// 点云降采样
void  vision_obstacle::PointCloud_DownSample(pcl::PointCloud<pcl::PointXYZ>&  src_pc  ,pcl::PointCloud<pcl::PointXYZ>&  dest_pc){
// 创建一个长宽高分别是3cm的体素过滤器，src_pc作为输入点云数据，dest_pc作为输出数据
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>   src_pc_ptr   = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(src_pc);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (src_pc_ptr);  //需要传入点云数据结构指针
	sor.setLeafSize (voxel_size.x, voxel_size.y , voxel_size.z );
	sor.filter(dest_pc);
 // ROS_INFO_STREAM_ONCE("Pcloud_size:"<<src_pc.points.size()<<"  Pcloude_filter_size:"<<dest_pc.points.size());
}

void   vision_obstacle::PointCloud_CoordTranform(pcl::PointCloud<pcl::PointXYZ>&  src_pc  ,pcl::PointCloud<pcl::PointXYZ>&  dest_pc,Eigen::Matrix4f  transf){

         dest_pc.header=src_pc.header;
         dest_pc.height=src_pc.height;
         dest_pc.width =src_pc.width;
         dest_pc.is_dense=src_pc.is_dense;
         dest_pc.points.reserve (src_pc.points.size ());
      //cloud_out.points.assign (cloud_in.points.begin (), cloud_in.points.end ());
        dest_pc.points.resize (src_pc.points.size ());
        dest_pc.sensor_orientation_ = src_pc.sensor_orientation_;
        dest_pc.sensor_origin_      = src_pc.sensor_origin_;
         int  num=src_pc.size();

         for(int i=0;i<num;i++)
         {
                Vector3d  point;
                point[0]=src_pc.at(i).x;
                point[1]=src_pc.at(i).y;
                point[2]=src_pc.at(i).z;

               cloud_out[i].x   = static_cast<float> (transf(0,0)*point[0]+transf(0,1)*point[1]+transf(0,2)*point[2]+transf(0,3));
               cloud_out[i].y  = static_cast<float> (transf(1,0)*point[0]+transf(1,1)*point[1]+transf(1,2)*point[2]+transf(1,3));
               cloud_out[i].z  = static_cast<float> (transf(2,0)*point[0]+transf(2,1)*point[1]+transf(2,2)*point[2]+transf(2,3));

                //将世界坐标系变到机器人坐标系
                 cloud_out[i].x = base_to_world.at<float>(0,0)*cloud_out[i].x +    base_to_world.at<float>(0,1)*cloud_out[i].y  + base_to_world.at<float>(0,2)*cloud_out[i].z+base_to_world.at<float>(0,3);
                 cloud_out[i].y = base_to_world.at<float>(1,0)*cloud_out[i].x +    base_to_world.at<float>(1,1)*cloud_out[i].y  + base_to_world.at<float>(1,2)*cloud_out[i].z+base_to_world.at<float>(1,3);
                 cloud_out[i].z = base_to_world.at<float>(2,0)*cloud_out[i].x +    base_to_world.at<float>(2,1)*cloud_out[i].y  + base_to_world.at<float>(2,2)*cloud_out[i].z+base_to_world.at<float>(2,3);;
         }
}

void   vision_obstacle::PointCloud_PassThrough(pcl::PointCloud<pcl::PointXYZ>&  src_pc  ,pcl::PointCloud<pcl::PointXYZ>&  dest_pc){

    ROS_INFO_STREAM("passthrought before:"<<src_pc.points.size());
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(src_pc.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(Z_range.x, Z_range.y);
    pass.filter(dest_pc);

    ROS_INFO_STREAM("passthrought Z aixs  after:"<<dest_pc.points.size());

     pass.setInputCloud(dest_pc.makeShared());
    pass.setFilterFieldName("x");
    pass.setFilterLimits(X_range.x, X_range.y);
    pass.filter(dest_pc);

        ROS_INFO_STREAM("passthrought X aixs  after:"<<dest_pc.points.size());

    pass.setInputCloud(dest_pc.makeShared());
    pass.setFilterFieldName("y");
    pass.setFilterLimits(Y_range.x, Y_range.y);
    pass.filter(dest_pc);
        ROS_INFO_STREAM("passthrought Y aixs  after:"<<dest_pc.points.size());

}

void   vision_obstacle::PointCloud_PassThrough_Outdoor(pcl::PointCloud<pcl::PointXYZ>&  src_pc  ,pcl::PointCloud<pcl::PointXYZ>&  dest_pc){

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(src_pc.makeShared());

    pass.setFilterFieldName("z");
    pass.setFilterLimits(Z_range_outdoor.x, Z_range_outdoor.y);
    pass.filter(dest_pc);

     pass.setInputCloud(dest_pc.makeShared());
    pass.setFilterFieldName("x");
    pass.setFilterLimits(X_range_outdoor.x, X_range_outdoor.y);
    pass.filter(dest_pc);

    pass.setInputCloud(dest_pc.makeShared());
    pass.setFilterFieldName("y");
    pass.setFilterLimits(Y_range_outdoor.x, Y_range_outdoor.y);
    pass.filter(dest_pc);

}

void   vision_obstacle::PointCloud_RadiusFilter(pcl::PointCloud<pcl::PointXYZ>&  src_pc  ,pcl::PointCloud<pcl::PointXYZ>&  dest_pc){

      if(src_pc.points.size()>0)
      {
         pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;  //创建离群点滤波器radius_outlier
        	outrem.setInputCloud(src_pc.makeShared());
  	      outrem.setRadiusSearch(radius_search);  //设置半径范围
	       outrem.setMinNeighborsInRadius(min_neighbors);  //半径范围内最少点数，低于这个点数的点被删掉
  	     outrem.setKeepOrganized(false);
  	     outrem.filter(dest_pc);
      }
    
}

void  vision_obstacle::PointCloud_Segment(pcl::PointCloud<pcl::PointXYZ>&  src_pc  ,pcl::PointCloud<pcl::PointXYZ>&  floor_pc,pcl::PointCloud<pcl::PointXYZ>&  nofloor_pc){

 // 通过RANSAC估计二维平面进行地面点与非地面点的分割 
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr              inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr         coeffs(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(iteration_num);   // 迭代次数
    seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
    seg.setEpsAngle(epsAngel * (M_PI / 180.0f));

    seg.setDistanceThreshold(distance_threshold);    //距离阈值
    seg.setOptimizeCoefficients(true);

    seg.setInputCloud(src_pc.makeShared());
    // 点云分割
    seg.segment(*inliers, *coeffs);     //平面点太少 ,可能提取不了平面

        

    if (0 == inliers->indices.size())
    {
        std::cout << "Could Not Estimate A Planar Model For The Given Dataset With Conditional Range Space." << std::endl;
    }

    ROS_INFO_STREAM("segment inliers:"<<inliers->indices.size());
 
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(src_pc.makeShared());

    extract.setIndices(inliers);
    extract.setNegative(true);
    // 提取非地面点云
    extract.filter(nofloor_pc);
   
    // 提取地面点云
    //extract.setNegative(false);
   // extract.filter(floor_pc);
}

void  vision_obstacle::PointCloud_Segment_Outdoor(pcl::PointCloud<pcl::PointXYZ>&  src_pc  ,pcl::PointCloud<pcl::PointXYZ>&  floor_pc,pcl::PointCloud<pcl::PointXYZ>&  nofloor_pc){

 // 通过RANSAC估计二维平面进行地面点与非地面点的分割 
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr              inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr         coeffs(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(iteration_num_outdoor);   // 迭代次数
    seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
    seg.setEpsAngle(epsAngel_outdoor * (M_PI / 180.0f));

    seg.setDistanceThreshold(distance_threshold_outdoor);    //距离阈值
    seg.setOptimizeCoefficients(true);

    seg.setInputCloud(src_pc.makeShared());
    // 点云分割
    seg.segment(*inliers, *coeffs);

    if (0 == inliers->indices.size())
    {
        std::cout << "Could Not Estimate A Planar Model For The Given Dataset With Conditional Range Space." << std::endl;
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(src_pc.makeShared());

    extract.setIndices(inliers);
    extract.setNegative(true);
    // 提取非地面点云
     extract.filter(nofloor_pc);
   
    // 提取地面点云
    //extract.setNegative(false);
    //extract.filter(floor_pc);
}

void   vision_obstacle::callback_pointcloud(const  sensor_msgs::PointCloud2ConstPtr&  msg){

        //pointcloud=   *msg;  
        // pcl_conversions::toPCL(*msg, pointcloud_camera);
         //对点云坐标进行坐标系转换

       // read_config_param(config_path,false);    //每次处理前读配置文件参数

        pcl::fromROSMsg(*msg, cloud);  //将sensor_msgs::Pointcloud2    转为   pcl::PointCloud<<pcl::PointXYZ>

         //double st,sd;
         //st = ros::Time::now().toSec();

        ROS_INFO_STREAM("PointCloud downSample  before:  "<< cloud.points.size());
        PointCloud_DownSample(cloud,cloud);
        ROS_INFO_STREAM("PointCloud downSample  after:  "<< cloud.points.size());
       // sd=ros::Time::now().toSec();
        //ROS_INFO_STREAM("PointCloud_DownSample time:" << (sd - st)*1000 );

         //st = ros::Time::now().toSec();
        PointCloud_CoordTranform( cloud,cloud_out ,  transf); 
        //sd=ros::Time::now().toSec();
        //ROS_INFO_STREAM("PointCloud_CoordTranform  time:" << (sd - st)*1000 );
        //pcl::toROSMsg(cloud_out,cloud_out_pub);
         //  pcl::transformPointCloud();     //直接调用  成熟函数
          //将变换后的点云发布成ros消息
       // point_cloud_pub.publish(cloud_out_pub);      


     //  st = ros::Time::now().toSec();
        if(model<=1)
        PointCloud_PassThrough(cloud_out,cloud_filter_pass);
        if(model>=2)
        PointCloud_PassThrough_Outdoor(cloud_out,cloud_filter_pass);
        //sd=ros::Time::now().toSec();
        //ROS_INFO_STREAM("PointCloud_PassThrough  time:" << (sd - st)*1000 );

          if(radis_rem_enable)
         PointCloud_RadiusFilter(cloud_filter_pass,cloud_filter_pass);    //球形滤波器

        sensor_msgs::PointCloud2   pc_passthrought;
        pcl::toROSMsg(cloud_filter_pass,  pc_passthrought);

        if(cloud_filter_pass.points.size())
        point_cloud_floor_pub.publish(pc_passthrought);
          
          /*
        pcl::PointCloud<pcl::PointXYZ>  floor_pc;
        pcl::PointCloud<pcl::PointXYZ>  nofloor_pc;

        //model =1  室内   model =2  室外

           st = ros::Time::now().toSec();
        if(model<=1)
        PointCloud_Segment(cloud_filter_pass  ,  floor_pc,  nofloor_pc);
        if(model>=2)
        PointCloud_Segment_Outdoor(cloud_filter_pass  ,  floor_pc,  nofloor_pc);    //地面平面分割

        sd=ros::Time::now().toSec();
        ROS_INFO_STREAM("PointCloud_Segment  time:" << (sd - st)*1000 );

        
        
        if(radis_rem_enable)
         PointCloud_RadiusFilter(nofloor_pc,nofloor_pc);    //球形滤波器

          // 打印非地面点，此处可以将非地面点保存下来
          ROS_INFO_STREAM("PointCloud after segment floor  has: "<< nofloor_pc.points.size());
 
        sensor_msgs::PointCloud2   pc_floor;
        sensor_msgs::PointCloud2   pc_nofloor;

        pcl::toROSMsg(nofloor_pc,pc_nofloor);
        pcl::toROSMsg(floor_pc,pc_floor);

       if(nofloor_pc.points.size()>0)
        point_cloud_nofloor_pub.publish(pc_nofloor);
        point_cloud_floor_pub.publish(pc_floor);*/
       
       /*
        if(print_flag){
          ROS_INFO_STREAM("msg:"<<  msg->header);
         ROS_INFO_STREAM("field:" << msg->fields[0]);
         ROS_INFO_STREAM("field:" << msg->fields[1]);
         ROS_INFO_STREAM("field:" << msg->fields[2]);
        ROS_INFO_STREAM("field:" << msg->fields[3]);  //本消息 fields域只有四个
         ROS_INFO_STREAM("height:"<< msg->height);
         ROS_INFO_STREAM("point_step:"<<msg->point_step);
         ROS_INFO_STREAM("row_step:"<<msg->row_step);
         ROS_INFO_STREAM("size:"<<msg->data.size());
         ROS_INFO_STREAM("width:"<<msg->width);
         ROS_INFO_STREAM("is_dense:"<<msg->is_dense);
         ROS_INFO_STREAM("is_bigendian:"<<msg->is_bigendian);
      //  pcl::transformPointCloud();     //直接调用  成熟函数

        ROS_INFO_STREAM("cloud.header:"<<   cloud.header);
         ROS_INFO_STREAM("cloud.height:" << cloud.height);
         ROS_INFO_STREAM("cloud.width:" << cloud.width);
         ROS_INFO_STREAM("cloud.size:" << cloud.size());
        ROS_INFO_STREAM("cloud.sensor_origin_:" << cloud.sensor_origin_);
        //ROS_INFO_STREAM("cloud.sensor_orientation_" << cloud.sensor_orientation_);//本消息 fields域只有四个
         ROS_INFO_STREAM(" cloud.is_dense"<< cloud.is_dense);
      
          ROS_INFO_STREAM("cloud_out_pub:"<<  cloud_out_pub.header);
         ROS_INFO_STREAM("field:" << cloud_out_pub.fields[0]);
         ROS_INFO_STREAM("field:" << cloud_out_pub.fields[1]);
         ROS_INFO_STREAM("field:" << cloud_out_pub.fields[2]);
        //ROS_INFO_STREAM("field:" << cloud_out_pub.fields[3]);  //本消息 fields域只有四个
         ROS_INFO_STREAM("cloud_out_pub height:"<< cloud_out_pub.height);
         ROS_INFO_STREAM("cloud_out_pub point_step:"<<cloud_out_pub.point_step);
         ROS_INFO_STREAM("cloud_out_pub row_step:"<<cloud_out_pub.row_step);
         ROS_INFO_STREAM("cloud_out_pub size:"<<cloud_out_pub.data.size());
         ROS_INFO_STREAM("cloud_out_pub width:"<<cloud_out_pub.width);
         ROS_INFO_STREAM("cloud_out_pub is_dense:"<<cloud_out_pub.is_dense);
         ROS_INFO_STREAM(" cloud_out_pub  is_bigendian:"<<cloud_out_pub.is_bigendian);

         Vector3d  point1;
          point1[0]=cloud.at(1000).x;
          point1[1]=cloud.at(1000).y;
          point1[2]=cloud.at(1000).z;
        ROS_INFO_STREAM("cloud point value:"<<point1);
         print_flag=false;
        }*/
      
          
  }



