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
#include <mavros_msgs/AttitudeTarget.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/Float32.h>
#include <vector>
#include <time.h>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, geometry_msgs::PoseStamped> syncPolicy;

/// Variaveis globais
vector<float> x;
vector<float> y;
vector<float> z;
int amostra = 0;
ros::Publisher caminho_pub;
visualization_msgs::Marker caminho;
ros::Publisher ponto_atual_pub;
visualization_msgs::Marker ponto_atual;

geometry_msgs::Point p;

void escutar_pose(const nav_msgs::OdometryConstPtr& msg, const geometry_msgs::PoseStampedConstPtr& msg2)
{
  // Leitura da posicao e orientacao
  ROS_INFO("Norte: %.2f\tLeste: %.2f\tUp: %.2f", msg->twist.twist.linear.x
                                               , msg->twist.twist.linear.y
                                               , msg->twist.twist.linear.z);
  x.push_back(msg->twist.twist.linear.x);
  y.push_back(msg->twist.twist.linear.y);
  z.push_back(msg->twist.twist.linear.z);
  ROS_INFO("X: %.3f\tY: %.3f\tZ: %.3f\t W:%.3f", msg2->pose.orientation.x
                                               , msg2->pose.orientation.y
                                               , msg2->pose.orientation.z
                                               , msg2->pose.orientation.w);

  amostra++; // Aqui controla quantas vezes leremos, para nao ser eterno e salvar log

  // Atualiza o caminho percorrido
  caminho.id = 0;
  caminho.header.frame_id = "/local_origin_ned";
  caminho.header.stamp = ros::Time::now();
  caminho.ns = "caminho";
  caminho.action = visualization_msgs::Marker::ADD;

  p.x = msg->twist.twist.linear.x;
  p.y = msg->twist.twist.linear.y;
  p.z = msg->twist.twist.linear.z;
  caminho.points.push_back(p);
  caminho.pose.position.x = p.x;
  caminho.pose.position.y = p.y;
  caminho.pose.position.z = p.z;

  // Atualiza ponto atual
  ponto_atual.id = 1;
  ponto_atual.header.frame_id = "/local_origin_ned";
  ponto_atual.header.stamp = caminho.header.stamp;
  ponto_atual.ns = "atual";
  ponto_atual.action = visualization_msgs::Marker::ADD;
  ponto_atual.pose.position.x = p.x;
  ponto_atual.pose.position.y = p.y;
  ponto_atual.pose.position.z = p.z;
  ponto_atual.pose.orientation.x = msg2->pose.orientation.x;
  ponto_atual.pose.orientation.y = msg2->pose.orientation.y;
  ponto_atual.pose.orientation.z = msg2->pose.orientation.z;
  ponto_atual.pose.orientation.w = msg2->pose.orientation.w;

  // Publicar nos topicos o caminho e o ponto atual
  caminho_pub.publish(caminho);
  ponto_atual_pub.publish(ponto_atual);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "read_sensors_pixhawk_node");
  ros::NodeHandle nh;

  // Aguardar a conexao segura do mavros para nao cair o no
  sleep(5);

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

  // Caracteristica do plot no RViz do caminho
  caminho.type = visualization_msgs::Marker::SPHERE_LIST;
  caminho.color.r = caminho.color.g = caminho.color.b = 1.0;
  caminho.color.a = 1.0;
  caminho.scale.x = caminho.scale.y = caminho.scale.z = 0.1;
  ponto_atual.type = visualization_msgs::Marker::ARROW;
  ponto_atual.color.r = 1.0; ponto_atual.color.g = ponto_atual.color.b = 0.0;
  ponto_atual.color.a = 1.0;
  ponto_atual.scale.x = ponto_atual.scale.y = ponto_atual.scale.z = 0.1;

  // Inicia publishers e subscribers
  caminho_pub     = nh.advertise<visualization_msgs::Marker>("caminho", 100);
  ponto_atual_pub = nh.advertise<visualization_msgs::Marker>("ponto_atual", 100);
  message_filters::Subscriber<nav_msgs::Odometry>         subpos(nh, "/mavros/global_position/local", 100);
  message_filters::Subscriber<geometry_msgs::PoseStamped> subori(nh, "/mavros/local_position/pose"  , 100);

  // Sincroniza as leituras dos topicos (sensores a principio) em um so callback
  Synchronizer<syncPolicy> sync(syncPolicy(100), subpos, subori);
  sync.registerCallback(boost::bind(&escutar_pose, _1, _2));

  while(amostra < 5000)
    ros::spinOnce();

  ros::shutdown();

  return 0;
}
