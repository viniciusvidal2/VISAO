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

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/thread/thread.hpp>
#include <iostream>
#include <string>

#include <ros/ros.h>

using namespace pcl;
using namespace pcl::visualization;
using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

typedef PointXYZRGB PointT;

class Imagem
{
public:
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void read_camera_calibration(string filename, Mat &K, Mat &dist_coef, Mat &rect){
    FileStorage fs;
    fs.open(filename, FileStorage::READ);
    fs["camera_matrix"] >> K;
    fs["distortion_coefficients"] >> dist_coef;
    fs["rectification_matrix"] >> rect;
    fs.release();
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void get_kpts_and_matches(Mat image_left, Mat image_right,
                            vector<Point2f> &keypoints_filt_left, vector<Point2f> &keypoints_filt_right,
                            Mat &descriptors_left, Mat &descriptors_right,
                            int min_hessian, int min_matches, vector<DMatch> &better_matches){

    FlannBasedMatcher matcher;
    vector<KeyPoint> keypoints_left, keypoints_right;
    vector<DMatch> matches;

    while (better_matches.size() < min_matches){ // Obter o minimo possivel de matches, senao abaixa o threshold do descritor SURF

      Ptr<SURF> detector = SURF::create(min_hessian);
      detector->detectAndCompute(image_left , Mat(), keypoints_left , descriptors_left );
      detector->detectAndCompute(image_right, Mat(), keypoints_right, descriptors_right);

      if (visualizar){
        Mat image_kpts_left, image_kpts_right;
        drawKeypoints( image_left , keypoints_left , image_kpts_left , Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        drawKeypoints( image_right, keypoints_right, image_kpts_right, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        namedWindow("Keypoints left" , WINDOW_NORMAL);
        namedWindow("Keypoints right", WINDOW_NORMAL);
        imshow("Keypoints left" , image_kpts_left );
        imshow("Keypoints right", image_kpts_right );
        waitKey(1);
      }

      // Fazer match de features
      if (!descriptors_left.empty() && !descriptors_right.empty())
        matcher.match(descriptors_left, descriptors_right, matches);
//      // Limpar mas correspondencias tendo nocao da visao stereo
//      float y_left, y_right;
//      int min_pixel_diff = 5;
//      vector<DMatch> good_matches;

//      for (int i = 0; i < matches.size(); i++){
//        y_left  = keypoints_left[matches[i].queryIdx].pt.y;
//        y_right = keypoints_right[matches[i].trainIdx].pt.y;
//        if (abs(y_left - y_right) < min_pixel_diff){
//          good_matches.push_back(matches[i]);
//        }
//      }

      // Limpar correspondencias tendo nocao da distancia dos keypoints na imagem
      float min_dist = 1000000, max_dist = 0;
      for( int i = 0; i < matches.size(); i++ ){
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
      }
      float thresh_dist = 2*min_dist;

      for(int i = 0; i < matches.size(); i++){
        if (matches[i].distance < thresh_dist){
          better_matches.push_back(matches[i]);
          keypoints_filt_left.push_back(keypoints_left[matches[i].queryIdx].pt);
          keypoints_filt_right.push_back(keypoints_right[matches[i].trainIdx].pt);
        }
      }
//      ROS_INFO("Quantos kpts do fim das contas %d %d", keypoints_filt_left.size(), keypoints_filt_right.size());
      // Limpando para proxima iteracao, se existir
      if(better_matches.size() < min_matches){
        min_hessian = 0.8*min_hessian;
        matches.clear();
        keypoints_left.clear();
        keypoints_right.clear();
        descriptors_left.release();
        descriptors_right.release();
        better_matches.clear();
        keypoints_filt_left.clear();
        keypoints_filt_right.clear();
//        good_matches.clear();
      }
    } // Fim do while

    if (visualizar){
      Mat img_matches, img_matches_disp;
      drawMatches( image_left, keypoints_left, image_right, keypoints_right, better_matches, img_matches,
                   Scalar::all(-1), Scalar::all(-1),
                   vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
      resize(img_matches, img_matches_disp, Size(img_matches.cols/4, img_matches.rows/4));
      namedWindow("All matches", WINDOW_GUI_EXPANDED);
      imshow("All Matches", img_matches_disp);
      waitKey(0);
    }
  } // Fim do void
//////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// Calculos a partir da matriz com teoria baseada nas fontes:
  /// http://planning.cs.uiuc.edu/node102.html#eqn:yprmat
  /// http://planning.cs.uiuc.edu/node103.html
  /// Algoritmo semelhante ao exemplo encontrado em opencv na fonte:
  /// https://www.learnopencv.com/rotation-matrix-to-euler-angles/
  void rpy_from_rotation(Mat R, float &r, float &p, float &y){
    bool singular;
    // ROLL
    singular = R.at<double>(2, 2) < 1e-6;
    if (!singular){
      r = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
    } else {
      if(R.at<double>(1, 1) > 0)
        r = M_PI_2;
      else
        r = M_PI + M_PI_2;
    }
    // PITCH
    float sp = sqrt(R.at<double>(2, 1)*R.at<double>(2, 1) + R.at<double>(2, 2)*R.at<double>(2, 2));
    singular = sp < 1e-5;
    if(!singular){
      p = atan2(-R.at<double>(2, 0), sp);
    } else {
      if(-R.at<double>(2, 0) > 0)
        p = M_PI_2;
      else
        p = M_PI + M_PI_2;
    }
    // YAW
    singular = R.at<double>(0, 0) < 1e-6;
    if(!singular){
      y = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    } else {
      if (R.at<double>(1, 0) > 0)
        y = M_PI_2;
      else
        y = M_PI + M_PI_2;
    }
  }
//////////////////////////////////////////////////////////////////////////////////////////////////////////
  void vector2mat(vector<Point2f> pl, vector<Point2f> pr, Mat &plm, Mat &prm){
    for(int i=0; i<pl.size(); i++){ // Os vetores tendem a ser do mesmo tamanho sempre, sao filtrados la atras
      plm.at<float>(i, 1) = pl[i].x;
      plm.at<float>(i, 2) = pl[i].y;
      prm.at<float>(i, 1) = pr[i].x;
      prm.at<float>(i, 2) = pr[i].y;
    }
  }
//////////////////////////////////////////////////////////////////////////////////////////////////////////
  void visualize_cloud(Mat c){
    int pts = c.cols;
    cloud =  (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
    cloud->points.reserve(pts);
    PointT point;
    for(int i=0; i < pts; i++){
      point.x = c.at<float>(0, i)/c.at<float>(3, i);
      point.y = c.at<float>(1, i)/c.at<float>(3, i);
      point.z = c.at<float>(2, i)/c.at<float>(3, i);
      point.r = 250.0f;
      point.g = 250.0f;
      point.b = 250.0f;
      cloud->push_back(point);
    }
    boost::shared_ptr<PCLVisualizer> vis (new PCLVisualizer("cloud_viewer"));
    PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    vis->addPointCloud<PointT>(cloud, rgb, "cloud");
    vis->addCoordinateSystem (1.0);
//    vis->resetCameraViewpoint("cloud");
    vis->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 4, "cloud");
    vis->initCameraParameters();
    vis->spin();
  }
//////////////////////////////////////////////////////////////////////////////////////////////////////////
  void set_visualizar(bool vis){
    visualizar = vis;
  }
//////////////////////////////////////////////////////////////////////////////////////////////////////////

private:
  bool visualizar;
  PointCloud<PointT>::Ptr cloud;

};
