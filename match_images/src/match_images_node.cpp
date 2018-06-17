#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

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

#include <iostream>
#include <string>

#include "../include/imagem.h"

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

bool visualizar = true;
///////////////////////////////////////////////////////////////////////////////////////////////////////
void read_camera_calibration (Mat &matrix){
  // Usar os valores de left.yaml
  float fx = 1765.159257291513, fy = 1705.370761980893, cx = 755.431601581711, cy = 591.0292653288053;
  matrix = (Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "match_images_node");
  ros::NodeHandle nh;

  // Declarando as imagens que esta lendo
  Mat image_left, image_right;
  string path_left  = "/home/vinicius/register_ws/src/match_images/pares_stereo/left/lab_1.jpg";
  string path_right = "/home/vinicius/register_ws/src/match_images/pares_stereo/right/lab_1.jpg";

  image_left  = imread(path_left.c_str() , IMREAD_COLOR);
  image_right = imread(path_right.c_str(), IMREAD_COLOR);

  if (image_left.empty()){
    cout <<  "Could not open or find the image" << endl ;
    return -1;
  }

//  // Visualizar as imagens ou nao
//  if(visualizar){
//    imshow("left", image_right);
//    imshow("right", image_right);
//    waitKey(0);
//  }

  // Obter keypoints e features das imagens por metodo SURF
  int minHessian = 10000;
  vector<KeyPoint> keypoints_left, keypoints_right;
  Mat descriptors_left, descriptors_right;
  int min_matches = 100;
  FlannBasedMatcher matcher;
  vector<DMatch> matches;

  while (matches.size() < min_matches){ // Obter o minimo possivel de matches, senao abaixa o threshold do descritor SURF
    Ptr<SURF> detector = SURF::create( minHessian );

    detector->detectAndCompute(image_left , Mat(), keypoints_left , descriptors_left );
    detector->detectAndCompute(image_right, Mat(), keypoints_right, descriptors_right);

    if (visualizar){
      Mat image_kpts_left, image_kpts_right;
      drawKeypoints( image_left , keypoints_left , image_kpts_left , Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
      drawKeypoints( image_right, keypoints_right, image_kpts_right, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
      namedWindow("Keypoints left" , WINDOW_KEEPRATIO);
      namedWindow("Keypoints right", WINDOW_KEEPRATIO);
      imshow("Keypoints left" , image_kpts_left );
      imshow("Keypoints right", image_kpts_right );
      waitKey(1);
    }

    // Fazer match de features
    ROS_INFO("Quantos descriptors left: %d", *descriptors_left.size.p);
    ROS_INFO("Quantos descriptors right: %d", *descriptors_right.size.p);
    if (!descriptors_left.empty() && !descriptors_right.empty())
      matcher.match(descriptors_left, descriptors_right, matches);

    ROS_INFO("Quantas matches: %d", matches.size());
    if (matches.size() < min_matches){
      minHessian = 0.9*minHessian;
      matches.clear();
      keypoints_left.clear();
      keypoints_right.clear();
      descriptors_left.release();
      descriptors_right.release();
    }

  }

  // Limpar mas correspondencias tendo nocao da visao stereo
  float y_left, y_right;
  int min_pixel_diff = 5;
  vector<DMatch> good_matches;
  vector<Point2f> keypoints_filt_left, keypoints_filt_right;
  for (int i = 0; i < matches.size(); i++){
    y_left  = keypoints_left[matches[i].queryIdx].pt.y;
    y_right = keypoints_right[matches[i].trainIdx].pt.y;
    cout << "Y na esquerda: " << y_left  << " do Keypoint " << matches[i].queryIdx << endl;
    cout << "Y na direita:  " << y_right << " do Keypoint " << matches[i].trainIdx << endl;
    cout << "A diferenca em y " << abs(y_left - y_right) << endl;
    cout << "A distancia entre as marcacoes " << matches[i].distance << endl;
    if (abs(y_left - y_right) < min_pixel_diff){
      good_matches.push_back(matches[i]);
//      keypoints_filt_left.push_back(keypoints_left[matches[i].queryIdx].pt);
//      keypoints_filt_right.push_back(keypoints_right[matches[i].trainIdx].pt);
    }
    cout << endl << endl;
  }

  if (visualizar){
    Mat img_matches;
    drawMatches( image_left, keypoints_left, image_right, keypoints_right, good_matches, img_matches,
                 Scalar::all(-1), Scalar::all(-1),
                 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    namedWindow("All matches", WINDOW_GUI_EXPANDED);
    imshow("All Matches", img_matches);
    waitKey(0);
  }

  // Limpar correspondencias tendo nocao da distancia dos keypoints na imagem
  float min_dist = 1000000, max_dist = 0;
  for( int i = 0; i < matches.size(); i++ ){
    double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }
  cout << "-- Max dist : " << max_dist << endl;
  cout << "-- Min dist : " << min_dist << endl;
  float thresh_dist = 3*min_dist;
  std::vector< DMatch > better_matches;

  for(int i = 0; i < good_matches.size(); i++){
    if (good_matches[i].distance < thresh_dist){
      better_matches.push_back(good_matches[i]);
      keypoints_filt_left.push_back(keypoints_left[good_matches[i].queryIdx].pt);
      keypoints_filt_right.push_back(keypoints_right[good_matches[i].trainIdx].pt);
    }
  }

  // Matriz fundamental entre as fotos
  cout << "Quantas correspondencias: " << keypoints_filt_left.size() << "\t" << keypoints_filt_right.size() << endl;
  Mat F = findFundamentalMat(keypoints_filt_left, keypoints_filt_right);
  cout << "Matriz fundamental F:\n" << F << endl;

  // Ler o arquivo de calibracao YAML na pasta calibracao
  Mat K;
  read_camera_calibration(K);
  cout << "Camera matrix K:\n" << K << endl;

  // Matriz essencial
//  Mat E = K.t() * F * K;
  Mat E = findEssentialMat(keypoints_filt_left, keypoints_filt_right, K, RANSAC, 0.999, 0.5);
  cout << "Matriz essencial E:\n" << E << endl;

  // Descobrir a forma certa das quatro opcoes para matriz essencial
  Mat P1, R, t, P2;
  int inliers;
//  decomposeEssentialMat(E, R1, R2, t);
//  cout << "Matriz rotacao 1 R1:\n" << R1 << endl;
//  cout << "Matriz rotacao 2 R2:\n" << R2 << endl;
//  cout << "Matriz translacao t:\n" << t  << endl;
  P1 = (Mat1d(3, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0); // [I | 0]
  inliers = recoverPose(E, keypoints_filt_left, keypoints_filt_right, K, R, t);
  cout << "Matriz rotacao R:\n"    << R        << endl;
  cout << "Matriz translacao t:\n" << t        << endl;
  cout << "Inliers:\n"             << inliers  << endl;
  hconcat(R, t, P2);
  // Mapa de disparidade?
//  Mat disparity;
//  StereoMatcher::compute(image_left, image_right, disparity);

  // Triangulacao dos pontos


  ros::spinOnce();
}
///////////////////////////////////////////////////////////////////////////////////////////////////////
