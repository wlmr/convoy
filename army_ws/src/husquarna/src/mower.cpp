#include "ros/ros.h"
#include "std_msgs/UInt16.h"
#include "geometry_msgs/Twist.h"


#include <sstream>

const uint16_t MANUAL_MODE  = 0x90;
const uint16_t RANDOM_MODE  = 0x91;
const uint16_t PARK_MODE    = 0x100;
const uint16_t START_LOOP   = 0x111;
const uint16_t STOP_LOOP    = 0x17;

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */

  ros::Publisher vel_pub  = n.advertise<geometry_msgs::Twist>("cmd_vel", 5);
  ros::Publisher mode_pub = n.advertise<std_msgs::UInt16>("cmd_mode", 5);
  ros::Rate loop_rate(1);

  std_msgs::UInt16 manual_message;
  manual_message.data = MANUAL_MODE;

  mode_pub.publish(manual_message);

  geometry_msgs::Twist forwards;
  forwards.linear.x = 0.3;

  geometry_msgs::Twist stop;

  geometry_msgs::Twist backwards;
  backwards.linear.x = -0.3;
  backwards.angular.z = 1;

  geometry_msgs::Twist twists[3] = {forwards, stop, backwards};



  for(int i = 0; i < 20; ++i)
  {
    if(!ros::ok())
    {
      ROS_ERROR("An error has occured in the loop.");
      break;
    }
    vel_pub.publish(twists[i % 3]);

    ros::spinOnce();
    loop_rate.sleep();
  }

  std_msgs::UInt16 random_message;
  random_message.data = RANDOM_MODE;

  mode_pub.publish(random_message);


  return 0;
}