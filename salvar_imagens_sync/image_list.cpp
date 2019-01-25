#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>


#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/flann.hpp>
#include <opencv2/stereo.hpp>

#include <opencv2/calib3d.hpp>

using namespace message_filters;
typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;

int ordem = 0;
std::string camera = "right";
std::string nome   = "pitch";

image_transport::Publisher pub_left;
image_transport::Publisher pub_right;
sensor_msgs::Image left;
sensor_msgs::Image right;

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

void cameras_callback(const sensor_msgs::ImageConstPtr& left, const sensor_msgs::ImageConstPtr& right){
  cv_bridge::CvImagePtr left_cv, right_cv;
  try
  {
    left_cv  = cv_bridge::toCvCopy(left , sensor_msgs::image_encodings::BGR8);
    right_cv = cv_bridge::toCvCopy(right, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::resize(left_cv->image , left_cv->image , cv::Size(), 0.5, 0.5, cv::INTER_AREA);
  cv::resize(right_cv->image, right_cv->image, cv::Size(), 0.5, 0.5, cv::INTER_AREA);

  pub_left.publish(left_cv->toImageMsg());
  pub_right.publish(right_cv->toImageMsg());

  std::cout << "\nTamanho da imagem: " << left_cv->image.cols << "\t" << left_cv->image.rows << std::endl;
//  cv::imwrite("/home/vinicius/stereo/left/" +boost::lexical_cast<std::string>(ordem)+".jpg",  left_cv->image);
//  cv::imwrite("/home/vinicius/stereo/right/"+boost::lexical_cast<std::string>(ordem)+".jpg", right_cv->image);
//  std::cout << "Gravamos as imagens da posicao " << ordem << std::endl;
//  ordem++;

//  ros::Rate rate(1);
//  rate.sleep();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_list");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  // Publicadores se quiser so reduzir a imagem e repassar
  pub_left  = it.advertise("/stereo/left/image_raw1" , 1);
  pub_right = it.advertise("/stereo/right/image_raw1", 1);

//  image_transport::Subscriber sub_left  = it.subscribe("/stereo/"+camera+"/image_rect_color",  1000, leftCallback);
  Subscriber<sensor_msgs::Image> subl(nh, "/stereo/left/image_raw" , 100);
  Subscriber<sensor_msgs::Image> subr(nh, "/stereo/right/image_raw", 100);
  Synchronizer<syncPolicy> sync(syncPolicy(100), subl, subr);
  sync.registerCallback(boost::bind(&cameras_callback, _1, _2));

  ros::spin();

  return 0;
}
