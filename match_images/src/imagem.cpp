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

#include <ros/ros.h>

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

class Imagem
{
public:
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void read_camera_calibration(Mat &matrix){
    // Usar os valores de left.yaml
    float fx = 1765.159257291513, fy = 1705.370761980893, cx = 755.431601581711, cy = 591.0292653288053;
    matrix = (Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void get_kpts_and_matches(Mat image_left, Mat image_right,
                            vector<Point2f> &keypoints_filt_left, vector<Point2f> &keypoints_filt_right,
                            Mat &descriptors_left, Mat &descriptors_right,
                            int min_hessian, int min_matches, vector<DMatch> &better_matches){

    FlannBasedMatcher matcher;
    vector<KeyPoint> keypoints_left, keypoints_right;
    std::vector<DMatch> matches;

    while (better_matches.size() < min_matches){ // Obter o minimo possivel de matches, senao abaixa o threshold do descritor SURF

      Ptr<SURF> detector = SURF::create(min_hessian);
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
//      ROS_INFO("Quantos descriptors left: %d", *descriptors_left.size.p);
//      ROS_INFO("Quantos descriptors right: %d", *descriptors_right.size.p);
      if (!descriptors_left.empty() && !descriptors_right.empty())
        matcher.match(descriptors_left, descriptors_right, matches);
//      ROS_INFO("Quantas matches: %d", matches.size());
      // Limpar mas correspondencias tendo nocao da visao stereo
      float y_left, y_right;
      int min_pixel_diff = 5;
      vector<DMatch> good_matches;

      for (int i = 0; i < matches.size(); i++){
        y_left  = keypoints_left[matches[i].queryIdx].pt.y;
        y_right = keypoints_right[matches[i].trainIdx].pt.y;
//        cout << "Y na esquerda: " << y_left  << " do Keypoint " << matches[i].queryIdx << endl;
//        cout << "Y na direita:  " << y_right << " do Keypoint " << matches[i].trainIdx << endl;
//        cout << "A diferenca em y " << abs(y_left - y_right) << endl;
//        cout << "A distancia entre as marcacoes " << matches[i].distance << endl;
        if (abs(y_left - y_right) < min_pixel_diff){
          good_matches.push_back(matches[i]);
        }
//        cout << endl << endl;
      }
//      ROS_INFO("Quantas good matches: %d", good_matches.size());

      // Limpar correspondencias tendo nocao da distancia dos keypoints na imagem
      float min_dist = 1000000, max_dist = 0;
      for( int i = 0; i < good_matches.size(); i++ ){
        double dist = good_matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
      }
      float thresh_dist = 3*min_dist;

      for(int i = 0; i < good_matches.size(); i++){
        if (good_matches[i].distance < thresh_dist){
          better_matches.push_back(good_matches[i]);
          keypoints_filt_left.push_back(keypoints_left[good_matches[i].queryIdx].pt);
          keypoints_filt_right.push_back(keypoints_right[good_matches[i].trainIdx].pt);
        }
      }
      // Limpando para proxima iteracao, se existir
//      ROS_INFO("Quantas better matches: %d", better_matches.size());
      if(better_matches.size() < min_matches){
        min_hessian = 0.9*min_hessian;
        matches.clear();
        keypoints_left.clear();
        keypoints_right.clear();
        descriptors_left.release();
        descriptors_right.release();
        better_matches.clear();
        keypoints_filt_left.clear();
        keypoints_filt_right.clear();
        good_matches.clear();
      }
    } // Fim do while
    if (visualizar){
      Mat img_matches;
      drawMatches( image_left, keypoints_left, image_right, keypoints_right, better_matches, img_matches,
                   Scalar::all(-1), Scalar::all(-1),
                   vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
      namedWindow("All matches", WINDOW_GUI_EXPANDED);
      imshow("All Matches", img_matches);
      waitKey(0);
    }
  } // Fim do void
//////////////////////////////////////////////////////////////////////////////////////////////////////////
  void set_visualizar(bool vis){
    visualizar = vis;
  }
//////////////////////////////////////////////////////////////////////////////////////////////////////////

private:
  bool visualizar;
};
