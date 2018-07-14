#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>

int ordem = 0;
std::string camera = "right";
std::string nome   = "labfull";
void leftCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  ROS_INFO("Tamanho da imagem: %d %d", cv_ptr->image.rows, cv_ptr->image.cols);
  cv::imwrite("/home/vinicius/stereo/"+camera+"/"+nome+"_"+boost::lexical_cast<std::string>(ordem)+".jpg", cv_ptr->image);
  std::cout << "gravamos uma imagem da posicao " << ordem << std::endl;
  ordem++;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_list");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  image_transport::Subscriber sub_left  = it.subscribe("/stereo/"+camera+"/image_rect",  1000, leftCallback);
//  image_transport::Subscriber sub_right = it.subscribe("/stereo/right/image_raw", 1000, rightCallback);

  ros::spin();

  return 0;
}
