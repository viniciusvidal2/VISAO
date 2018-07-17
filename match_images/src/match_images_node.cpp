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

#include "../../libraries/imagem.cpp"

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

///////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "match_images_node");
  ros::NodeHandle nh;
  // Objeto da classe que trabalha o pipeline da imagem, somente .cpp, porque ficou de viadagem
  Imagem im;
  im.set_visualizar(false);

  // Testar fator de escala
  float scale_fact = im.scale_factor("/home/vinicius/visao_ws/src/VISAO/match_images/calibracao/left.yaml",
                                     "/home/vinicius/visao_ws/src/VISAO/match_images/calibracao/right.yaml");
//  return -1;

  // Declarando as imagens que esta lendo
  Mat image_left, image_right;
  string path_left  = "/home/vinicius/visao_ws/src/VISAO/match_images/datasets/super/x.jpg";
  string path_right = "/home/vinicius/visao_ws/src/VISAO/match_images/datasets/super/xpraesquerda.jpg";

  image_left  = imread(path_left.c_str() , IMREAD_COLOR);
  image_right = imread(path_right.c_str(), IMREAD_COLOR);

  if (image_left.empty()){
    cout <<  "Could not open or find the image" << endl;
    return -1;
  }

  im.set_quadrados(image_left, 3, 3, false);

//  // Rotacionar a imagem de sacanagem
//  double angle = 30;

//  // get rotation matrix for rotating the image around its center in pixel coordinates
//  Point2f center((image_right.cols-1)/2.0, (image_right.rows-1)/2.0);
//  Mat rot = getRotationMatrix2D(center, angle, 1.0);
//  // determine bounding rectangle, center not relevant
//  Rect2f bbox = RotatedRect(Point2f(), image_right.size(), angle).boundingRect2f();
//  // adjust transformation matrix
//  rot.at<double>(0,2) += bbox.width/2.0 - image_right.cols/2.0;
//  rot.at<double>(1,2) += bbox.height/2.0 - image_right.rows/2.0;
//  cv::warpAffine(image_right, image_right, rot, bbox.size());

  // Obter keypoints, features e match das mesmas. Filtrar os matchs obtidos
  vector<Point2f> keypoints_filt_left, keypoints_filt_right;
  Mat descriptors_left, descriptors_right;
  vector<DMatch> better_matches;
  im.get_kpts_and_matches(image_left, image_right,
                          keypoints_filt_left, keypoints_filt_right, descriptors_left, descriptors_right,
                          30000, 20, better_matches);
//  return -1;
  // Matriz fundamental entre as fotos
  Mat F = findFundamentalMat(keypoints_filt_left, keypoints_filt_right);

  // Corrigir os matches para diminuir o erro geometrico
//  vector<Point2f> keypoints_filt_left_new, keypoints_filt_right_new;
//  correctMatches(F, keypoints_filt_left, keypoints_filt_right, keypoints_filt_left_new, keypoints_filt_right_new);

  // Ler o arquivo de calibracao YAML na pasta calibracao
  Mat K, dist_coef, rect; // matriz intrinseca, coeficientes de distorcao e matriz de retificacao
  im.read_camera_calibration("/home/vinicius/visao_ws/src/VISAO/match_images/calibracao/left.yaml", K, dist_coef, rect);

  // Matriz essencial
//  Mat E = K.t() * F * K;
  Mat E = findEssentialMat(keypoints_filt_left, keypoints_filt_right, K, RANSAC, 0.999, 1e-4);
  cout << "Matriz essencial E :\n" << E  << endl;

  // Descobrir a forma certa das quatro opcoes para matriz essencial
  Mat R, t, rod;
  int inliers;
  inliers = recoverPose(E, keypoints_filt_left, keypoints_filt_right, K, R, t);
  cout << "Inliers:  "             << inliers << " de " << keypoints_filt_left.size() << endl;

  // Matriz de projecao
  Mat rt1, rt2, P1, P2;
  rt1 = (Mat1d(3, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0); // [I | 0]
  P1 = K*rt1; // Projecao inicial
  hconcat(R, t, rt2); // Matriz de parametros extrinsecos 2
  P2 = K*rt2; // Matriz de projecao

  // Obtendo angulos RPY a partir da matriz de rotacao
  // YAW   -> eixo Z saindo do plano da imagem; camera positivo anti-horario de giro; imagem positivo horario de giro.
  // PITCH -> eixo X apontando para a direita da imagem; camera positivo sentido horario da origem para ponta do eixo ("para baixo")

  cout << endl << "X: " << scale_fact*t.at<double>(0, 0) << "\tY: " << scale_fact*t.at<double>(1, 0) << "\tZ: " << scale_fact*t.at<double>(2, 0) << endl;
  Rodrigues(R, rod);
  cout << endl << "roll: " << RAD2DEG(rod.at<double>(0, 0)) << "\t" << "pitch: " << RAD2DEG(rod.at<double>(1, 0)) <<
                  "\t" << "yaw: " << RAD2DEG(rod.at<double>(2, 0)) << endl;

  // Triangulacao dos pontos - https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#triangulatepoints
  Mat points3d(Size(4, int(keypoints_filt_left.size())), CV_64FC1);
  triangulatePoints(P1, P2, keypoints_filt_left, keypoints_filt_right, points3d);

  // Visualizacao dos pontos 3D
//  im.visualize_cloud(points3d);

  ros::spinOnce();
}
///////////////////////////////////////////////////////////////////////////////////////////////////////
