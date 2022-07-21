/*********************************************************************
 * This <test_thread> package is example code of planar two-link arm
 * using pthread and dynamixel sdk
 * author : ola31
 * Fab, 2022
 * ******************************************************************/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "test_thread/test_thread_node.h"
#include "geometry_msgs/Point32.h"

extern bool is_run;

//Initial position
//(robot will move to this position when start)
double init_pose_x = 0.0;
double init_posi_y = 0.15;


double Ep_x = init_pose_x;
double Ep_y = init_posi_y;

void endpoint_xy_Callback(const geometry_msgs::Point32::ConstPtr& msg)
{
  Ep_x = msg->x;
  Ep_y = msg->y;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_thread_node");
  ros::NodeHandle nh;

  pthread_t pthread;
  int thr_id;
  int status;
  char p1[] = "thread_1";

  sleep(1);

  thr_id = pthread_create(&pthread, NULL, p_function, (void*)p1);
  if(thr_id < 0){
    ROS_ERROR("pthread create error");
    exit(EXIT_FAILURE);
  }
  else{
    ROS_INFO("pthread start...");
  }
  sleep(1);

  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
  ros::Subscriber sub = nh.subscribe("/endpoint_xy", 1000, endpoint_xy_Callback);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    set_EP_goal(Ep_x,Ep_y);

    std_msgs::String msg;
    msg.data = "hello world";

    chatter_pub.publish(msg);


    ros::spinOnce();
    loop_rate.sleep();
  }

  is_run = false;
  return 0;


}




