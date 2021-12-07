#include <ros/ros.h>
#include <ros/console.h>
#include  "vision_obstacle.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_transform");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  ros::NodeHandle nh("~");

  vision_obstacle  vision_ob(nh);
 
 /*
  ros::Rate loop_rate(6);
  while (ros::ok())
   {
     ros::spinOnce();
     loop_rate.sleep();
   }*/
  ros::spin();
}
