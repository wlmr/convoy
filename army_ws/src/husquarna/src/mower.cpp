#include "ros/ros.h"
#include "std_msgs/UInt16.h"
#include "geometry_msgs/Twist.h"
#include <iostream>

using std::cout;
using std::endl;

const uint16_t MANUAL_MODE  = 0x90;
const uint16_t RANDOM_MODE  = 0x91;
const uint16_t PARK_MODE    = 0x100;
const uint16_t START_LOOP   = 0x111;
const uint16_t STOP_LOOP    = 0x17;


ros::Publisher vel_pub;
ros::Publisher mode_pub;



void wait_for_enter()
{
  cout << "Press enter to continue." << endl;
  getchar();
}


void draw_star()
{
  ros::Duration duration(1);
  geometry_msgs::Twist forwards;
  forwards.angular.z = 0.5;
  forwards.linear.x = 0.3;

  geometry_msgs::Twist backwards;
  backwards.linear.x = -0.3;
  backwards.angular.z = 0.5;

  geometry_msgs::Twist twists[2] = {forwards, backwards};

  for(int i = 0; i < 10; ++i)
  {
    if(!ros::ok())
    {
      ROS_ERROR("An error has occured in the loop.");
      break;
    }
    vel_pub.publish(twists[i % 2]);

    ros::spinOnce();
    duration.sleep();
  }
}


void draw_circle()
{
  ros::Duration duration(15);
  geometry_msgs::Twist forwards;
  forwards.angular.z = 0.5;
  forwards.linear.x = 0.3;

  vel_pub.publish(forwards);
  duration.sleep();
}


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  char key;

  

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  vel_pub  = n.advertise<geometry_msgs::Twist>("cmd_vel", 5);
  mode_pub = n.advertise<std_msgs::UInt16>("cmd_mode", 5);
  

  

  for(int i = 0; i < 100 && mode_pub.getNumSubscribers() == 0; ++i)
  {
    ROS_INFO("Waiting for subscribers...");
    sleep(1);
  }

  ROS_INFO("Subscriber found.");
  


  std_msgs::UInt16 manual_message;
  manual_message.data = MANUAL_MODE;

  mode_pub.publish(manual_message);


  cout << "Star-shape routine" << endl;
  wait_for_enter();

  draw_star();

  geometry_msgs::Twist stop;
  
  vel_pub.publish(stop);

  cout << "Star-shape routine" << endl;
  wait_for_enter();

  draw_circle();

  vel_pub.publish(stop);

  std_msgs::UInt16 random_message;
  random_message.data = RANDOM_MODE;

  mode_pub.publish(random_message);


  return 0;
}