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

#include "imagem.cpp"

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

///////////////////////////////////////////////////////////////////////////////////////////////////////
float degrees(float rad){
  return rad * 180.0/M_PI;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "match_images_node");
  ros::NodeHandle nh;
  // Objeto da classe que trabalha o pipeline da imagem, somente .cpp, porque ficou de viadagem
  Imagem im;
  im.set_visualizar(false);

  // Declarando as imagens que esta lendo
  Mat image_left, image_right;
  string path_left  = "/home/vinicius/visao_ws/src/VISAO/match_images/datasets/piano/im0.png";
  string path_right = "/home/vinicius/visao_ws/src/VISAO/match_images/datasets/piano/im1.png";

  image_left  = imread(path_left.c_str() , IMREAD_COLOR);
  image_right = imread(path_right.c_str(), IMREAD_COLOR);

  if (image_left.empty()){
    cout <<  "Could not open or find the image" << endl ;
    return -1;
  }

  // Obter keypoints, features e match das mesmas. Filtrar os matchs obtidos
  vector<Point2f> keypoints_filt_left, keypoints_filt_right;
  Mat descriptors_left, descriptors_right;
  vector<DMatch> better_matches;
  im.get_kpts_and_matches(image_left, image_right,
                          keypoints_filt_left, keypoints_filt_right, descriptors_left, descriptors_right,
                          9000, 800, better_matches);

  // Matriz fundamental entre as fotos
  Mat F = findFundamentalMat(keypoints_filt_left, keypoints_filt_right);
  cout << "Matriz fundamental F:\n" << F << endl;

  // Corrigir os matches para diminuir o erro geometrico
  vector<Point2f> keypoints_filt_left_new, keypoints_filt_right_new;
  correctMatches(F, keypoints_filt_left, keypoints_filt_right, keypoints_filt_left_new, keypoints_filt_right_new);

  // Ler o arquivo de calibracao YAML na pasta calibracao
  Mat K, dist_coef, rect; // matriz intrinseca, coeficientes de distorcao e matriz de retificacao
  im.read_camera_calibration("/home/vinicius/visao_ws/src/VISAO/match_images/calibracao/piano.yaml", K, dist_coef, rect);
  cout << "Camera matrix K:\n" << K << endl;

  // Matriz essencial
//  Mat E = K.t() * F * K;
  Mat E = findEssentialMat(keypoints_filt_left, keypoints_filt_right, K, RANSAC, 0.999, 1e-4);
  cout << "Matriz essencial E:\n" << E << endl;

  // Descobrir a forma certa das quatro opcoes para matriz essencial
  Mat R, t;
  int inliers;
//  decomposeEssentialMat(E, R1, R2, t);
  inliers = recoverPose(E, keypoints_filt_left, keypoints_filt_right, K, R, t);
  cout << "Matriz rotacao R:\n"    << R       << endl;
  cout << "Matriz translacao t:\n" << t       << endl;
  cout << "Inliers:  "             << inliers << endl;

  // Matriz de projecao
  Mat rt1, rt2, P1, P2;
  rt1 = (Mat1d(3, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0); // [I | 0]
  P1 = K*rt1; // Projecao inicial
//  cout << "Matriz Projecao P1:\n"  << P1      << endl;
  hconcat(R, t, rt2); // Matriz de parametros extrinsecos 2
//  cout << "Matriz extrinsecos:\n"  << rt2     << endl;
  P2 = K*rt2; // Matriz de projecao
//  cout << "Matriz Projecao P2:\n"  << P2      << endl;

  // Obtendo angulos RPY a partir da matriz de rotacao
  float roll, pitch, yaw; // [rad]
  im.rpy_from_rotation(R, roll, pitch, yaw);
  cout << endl << "roll: " << degrees(roll) << "\t" << "pitch: " << degrees(pitch) << "\t" << "yaw: " << degrees(yaw) << endl;

  // Triangulacao dos pontos - https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#triangulatepoints
  Mat points3d(Size(4, int(keypoints_filt_left_new.size())), CV_64FC1);
  triangulatePoints(P1, P2, keypoints_filt_left_new, keypoints_filt_right_new, points3d);

  // Visualizacao dos pontos 3D
  im.visualize_cloud(points3d);

  // Mapa de disparidade?
//  Mat disparity;
//  StereoMatcher::compute(image_left, image_right, disparity);


  ros::spinOnce();
}
///////////////////////////////////////////////////////////////////////////////////////////////////////
