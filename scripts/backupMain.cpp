#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include <sstream>

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  //do parser here. 
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker2");
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::Publisher leftMotor_pub = n.advertise<std_msgs::Int16>("leftMotor", 1000);

  ros::Rate loop_rate(10);
  int count = 0;

  while (ros::ok())
  {
    std_msgs::Int16 msg;
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());

    leftMotor_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
