#include "ros/ros.h"
#include "std_msgs/String.h"

#include <mavros/mavros.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/mavlink_convert.h>
#include <mavlink/config.h>
#include <mavros/utils.h>
#include <mavros/mavros.h>
#include <mavros_msgs/mavlink_convert.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/StreamRate.h>
#include <nav_msgs/Odometry.h>

#include <std_msgs/Float32.h>
#include <vector>

using namespace std;

/// Variaveis globais
vector<float> x;
vector<float> y;
vector<float> z;
int amostra = 0;

void escutar_posicao(const nav_msgs::OdometryConstPtr& msg)
{
  ROS_INFO("Norte: %.2f\tLeste: %.2f\tUp: %.2f", msg->twist.twist.linear.x
                                               , msg->twist.twist.linear.y
                                               , msg->twist.twist.linear.z);
  x.push_back(msg->twist.twist.linear.x);
  y.push_back(msg->twist.twist.linear.y);
  z.push_back(msg->twist.twist.linear.z);

  amostra++;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "read_sensors_pixhawk_node");
  ros::NodeHandle nh;

  // Ajustando taxa de conversa do mavros
  ros::ServiceClient srv = nh.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");
  mavros_msgs::StreamRate rate;
  rate.request.stream_id = 0;
  rate.request.message_rate = 10; // 10 Hz das mensagens que vem
  rate.request.on_off = 1; // Nao sei
  if(srv.call(rate))
    ROS_INFO("Taxado mavros mudada para %d Hz", rate.request.message_rate);
  else
    ROS_INFO("Nao pode chamar o servico, taxa nao mudada.");

  ros::Subscriber sub = nh.subscribe("/mavros/global_position/local", 1000, escutar_posicao);

  while(amostra < 500)
    ros::spinOnce();

  ros::shutdown();

  return 0;
}
