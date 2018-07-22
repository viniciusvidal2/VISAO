//Includes
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

using namespace pcl;
using namespace std;
using namespace tf;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Variaveis e definicoes globais
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Definicoes
typedef PointXYZRGB       PointT;
typedef PointXYZRGBNormal PointTN;

// Variaveis
PointCloud<PointT>::Ptr nuvem_acumulada;
tf::TransformListener *p_listener;
boost::shared_ptr<ros::Publisher> pub;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void acumular_nuvem(const sensor_msgs::PointCloud2ConstPtr& msg){

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "salvar_zed_node");
  ros::NodeHandle nh;

//  ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);

  ros::spin();

  return 0;
}
