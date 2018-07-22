//Includes
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <nav_msgs/Odometry.h>

#include <tf/tf.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace pcl;
using namespace std;
using namespace tf;
using namespace message_filters;
//using namespace sensor_msgs;
using namespace nav_msgs;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Variaveis e definicoes globais
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Definicoes
typedef PointXYZRGB       PointT;
typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, Odometry> syncPolicy;

// Variaveis
PointCloud<PointT>::Ptr nuvem_acumulada;
tf::TransformListener *p_listener;
boost::shared_ptr<ros::Publisher> pub;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void filter_color(pcl::PointCloud<PointT>::Ptr cloud_in){

  // Limites RGB
  int rMax = 200;
  int rMin = 0;
  int gMax = 120;
  int gMin = 0;
  int bMax = 120;
  int bMin = 0;

  pcl::ConditionAnd<PointT>::Ptr color_cond (new pcl::ConditionAnd<PointT> ());
  color_cond->addComparison (pcl::PackedRGBComparison<PointT>::Ptr (new pcl::PackedRGBComparison<PointT> ("r", pcl::ComparisonOps::LT, rMax)));
  color_cond->addComparison (pcl::PackedRGBComparison<PointT>::Ptr (new pcl::PackedRGBComparison<PointT> ("r", pcl::ComparisonOps::GT, rMin)));
  color_cond->addComparison (pcl::PackedRGBComparison<PointT>::Ptr (new pcl::PackedRGBComparison<PointT> ("g", pcl::ComparisonOps::LT, gMax)));
  color_cond->addComparison (pcl::PackedRGBComparison<PointT>::Ptr (new pcl::PackedRGBComparison<PointT> ("g", pcl::ComparisonOps::GT, gMin)));
  color_cond->addComparison (pcl::PackedRGBComparison<PointT>::Ptr (new pcl::PackedRGBComparison<PointT> ("b", pcl::ComparisonOps::LT, bMax)));
  color_cond->addComparison (pcl::PackedRGBComparison<PointT>::Ptr (new pcl::PackedRGBComparison<PointT> ("b", pcl::ComparisonOps::GT, bMin)));

  // Constroi o filtro
  pcl::ConditionalRemoval<PointT> condrem (color_cond);
  condrem.setInputCloud (cloud_in);
  condrem.setKeepOrganized(true);

  // Aplica o filtro
  condrem.filter(*cloud_in);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void passthrough(pcl::PointCloud<PointT>::Ptr in, std::string field, float min, float max){
  pcl::PassThrough<PointT> ps;
  ps.setInputCloud(in);
  ps.setFilterFieldName(field);
  ps.setFilterLimits(min, max);

  ps.filter(*in);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void remove_outlier(PointCloud<PointT>::Ptr in, float mean, float deviation){
  StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(in);
  sor.setMeanK(mean);
  sor.setStddevMulThresh(deviation);
  sor.filter(*in);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void voxelgrid_nuvem(PointCloud<PointT>::Ptr in, float lf){
  VoxelGrid<PointT> grid;
  grid.setLeafSize(lf, lf, lf);
  grid.setInputCloud(in);
  grid.filter(*in);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////// Callback acumulador ////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void acumular_nuvem(const sensor_msgs::PointCloud2ConstPtr& msg_ptc, const OdometryConstPtr& msg_odo){
  // Variaveis locais
  PointCloud<PointT>::Ptr nuvem (new PointCloud<PointT>());
  PointCloud<PointT>::Ptr nuvem_transformada (new PointCloud<PointT>());
  sensor_msgs::PointCloud2 nuvem_mensagem;

  // Converter a mensagem em nuvem
  fromROSMsg (*msg_ptc, *nuvem);

  // Remover NaN se existir
  vector<int> indicesNAN;
  removeNaNFromPointCloud(*nuvem, *nuvem, indicesNAN);

  // Simplificar a nuvem por voxel grid
  float tamanho_leaf = 0.05;
  voxelgrid_nuvem(nuvem, tamanho_leaf);
  // Filtrar a regiao da nuvem que passa
  passthrough(nuvem, "z", -1,  1); // Vertical
  passthrough(nuvem, "x",  1, 10); // Profundidade
  passthrough(nuvem, "y", -1,  1); // Horizontal
  // Filtrar por cor
  //  filter_color(cloud);
  // Remover outliers
  remove_outlier(nuvem, 6, 0.5);

  /// Obter a odometria da mensagem
  // Rotacao
  Eigen::Quaternion<double> q;
  q.x() = (double)msg_odo->pose.pose.orientation.x;
  q.y() = (double)msg_odo->pose.pose.orientation.y;
  q.z() = (double)msg_odo->pose.pose.orientation.z;
  q.w() = (double)msg_odo->pose.pose.orientation.w;
  // Translacao
  Eigen::Vector3d offset(msg_odo->pose.pose.position.x, msg_odo->pose.pose.position.y, msg_odo->pose.pose.position.z);
  // Print para averiguar
  if(false){
    auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    cout << "Roll: " << RAD2DEG(euler[0]) << "\tPitch: " << RAD2DEG(euler[1]) << "\tYaw: " << RAD2DEG(euler[2]) << endl;
    cout << "X   : " << offset(0)         << "\tY    : " << offset(1)         << "\tZ  : " << offset(2)         << endl;
  }

  // Transformar a nuvem
  transformPointCloud<PointT>(*nuvem, *nuvem_transformada, offset, q);

  // Acumular a nuvem de forma simples
  ROS_INFO("Tamanho da nuvem atual = %ld", nuvem_transformada->points.size());
  (*nuvem_acumulada) += (*nuvem_transformada);
  ROS_INFO("Tamanho da nuvem acumulada = %ld", nuvem_acumulada->points.size());

  // Converter de volta a ros msg e enviar
  pcl::toROSMsg(*nuvem_acumulada, nuvem_mensagem);
  nuvem_mensagem.header.stamp = msg_ptc->header.stamp; // Mesmo tempo para sincronizar
  pub->publish(nuvem_mensagem);

  nuvem.reset();
  nuvem_transformada.reset();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////// Principal //////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "acumular_zed_node");
  ros::NodeHandle nh;

  // Iniciando a listener para a transformada que vem da odometria da ZED
  p_listener = (tf::TransformListener*) new tf::TransformListener;
  ros::Duration(2).sleep();

  // Iniciando nuvem acumulada com o frame segundo vindo do parametro
  nuvem_acumulada = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
  nuvem_acumulada->header.frame_id = ros::names::remap("/odom");

  // Iniciar o publisher no topico da nuvem acumulada
  pub  = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
  *pub = nh.advertise<sensor_msgs::PointCloud2>("/zed_neymar", 10);

  // Subscriber para a nuvem instantanea e odometria
  message_filters::Subscriber<sensor_msgs::PointCloud2> subptc(nh, "input_cloud", 100);
  message_filters::Subscriber<Odometry>    subodo(nh, "input_odom" , 100);
  // Sincroniza as leituras dos topicos (sensores e imagem a principio) em um so callback
  Synchronizer<syncPolicy> sync(syncPolicy(100), subptc, subodo);
  sync.registerCallback(boost::bind(&acumular_nuvem, _1, _2));

  ros::spin();

  return 0;
}
