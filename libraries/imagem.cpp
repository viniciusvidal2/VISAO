#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/optflow.hpp>
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
#include <numeric>

#include <ros/ros.h>

using namespace pcl;
using namespace pcl::visualization;
using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

typedef PointXYZRGB PointT;

// Estrutura para simplificar e passar para filtro
struct Pose{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
} ;

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
  void undistort_image(Mat &pic, Mat camera_matrix, Mat coefs){
    Mat temp; pic.copyTo(temp);
    undistort(temp, pic, camera_matrix, coefs);
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  float scale_factor(string path_left, string path_right){
    Mat proj_left, proj_right, Kl, Kr;
    FileStorage fl, fr;
    fl.open(path_left , FileStorage::READ);
    fl["projection_matrix"] >> proj_left;
    fl["camera_matrix"] >> Kl;
    fr.open(path_right, FileStorage::READ);
    fr["projection_matrix"] >> proj_right;
    fr["camera_matrix"] >> Kr;

    // Calcular matrizes [R|t]
    Mat rt_left(3, 4, CV_64F), rt_right(3, 4, CV_64F);
    rt_left  = Kl.inv()*proj_left;
    rt_right = Kr.inv()*proj_right;
    // Retirar translacao dali, em modulo para nao dar erro
    vector<double> translation = { abs(rt_left.at<double>(0, 3)-rt_right.at<double>(0, 3)),
                                   abs(rt_left.at<double>(1, 3)-rt_right.at<double>(1, 3)),
                                   abs(rt_left.at<double>(2, 3)-rt_right.at<double>(2, 3)) };
    // Garantir que estamos pegando o eixo que transladou mesmo
    scale_to_real_world = *max_element(translation.begin(), translation.end());

    return scale_to_real_world;
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void get_kpts_and_matches(Mat image_left, Mat image_right,
                            vector<Point2f> &keypoints_filt_left, vector<Point2f> &keypoints_filt_right,
                            Mat &descriptors_left, Mat &descriptors_right,
                            int min_hessian, int min_matches, vector<DMatch> &better_matches){

    FlannBasedMatcher matcher;
//    BFMatcher matcher(NORM_HAMMING);
    vector<KeyPoint> keypoints_left, keypoints_right;
    vector<DMatch> matches;

    while (better_matches.size() < min_matches){ // Obter o minimo possivel de matches, senao abaixa o threshold do descritor SURF

      Ptr<SURF> detector = SURF::create(min_hessian);
//      Ptr<AKAZE> detector = AKAZE::create(AKAZE::DESCRIPTOR_MLDB, 0, 3, min_hessian, 4, 4, KAZE::DIFF_PM_G2);
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

      // Limpar correspondencias tendo nocao da distancia dos keypoints na imagem
      float min_dist = 1000000, max_dist = 0;
      for( int i = 0; i < matches.size(); i++ ){
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
      }
      float thresh_dist = 3*min_dist;

      for(int i = 0; i < matches.size(); i++){
        if (matches[i].distance < thresh_dist){ // Aqui crio matches novo e vetor dos pontos dos keypoints ORGANIZADOS,
          better_matches.push_back(matches[i]); // porem daqui em diante apago os que nao forem bons
          keypoints_filt_left.push_back(keypoints_left[matches[i].queryIdx].pt);
          keypoints_filt_right.push_back(keypoints_right[matches[i].trainIdx].pt);
        }
      }
      // Filtrando por bins
      filter_bins(image_left, keypoints_filt_left, keypoints_filt_right, better_matches);
      // Filtrando por coeficiente angular
      filter_lines(image_left, keypoints_filt_left, keypoints_filt_right, better_matches);

      ROS_INFO("Quantos kpts do fim das contas %d %d", keypoints_filt_left.size(), keypoints_filt_right.size());
      ROS_INFO("Threshold para descritor: %d", min_hessian);
      // Limpando para proxima iteracao, se existir
      if(better_matches.size() < min_matches){
        min_hessian = 0.7*min_hessian;
        matches.clear();
        keypoints_left.clear();
        keypoints_right.clear();
        descriptors_left.release();
        descriptors_right.release();
        better_matches.clear();
        keypoints_filt_left.clear();
        keypoints_filt_right.clear();
      }

    } // Fim do while

    if (true){
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
  void filter_bins(Mat pic, vector<Point2f> &kptl, vector<Point2f> &kptr, vector<DMatch> &matches){
    // Keypoints ja correspondem um a um nesse estagio.
    float lim_x_left, lim_x_right, lim_y_down, lim_y_up;
    for(int i=0; i<kptl.size(); i++){
      // Guardar coordenadas dos pontos
      float xl = kptl[i].x, yl = kptl[i].y;
      float xr = kptr[i].x, yr = kptr[i].y;
      // Criar um bin em torno do ponto na imagem anterior (no caso da esquerda) atentando aos limites da imagem.
      lim_x_left  = (xl - w/2) >= 0        ? xl - w/2 : 0;
      lim_x_right = (xl + w/2) <  pic.cols ? xl + w/2 : pic.cols-1;
      lim_y_up    = (yl - h/2) >= 0        ? yl - h/2 : 0;
      lim_y_down  = (yl + h/2) <  pic.rows ? yl + h/2 : pic.rows-1;
      // Confeir se as coordenadas do ponto da imagem atual (ou da direita) estao dentro do bin
      if( !(lim_x_left < xr && xr < lim_x_right && lim_y_up < yr && yr < lim_y_down) ){ // Se nao estao, apaga
        kptl.erase(kptl.begin()+i);
        kptr.erase(kptr.begin()+i);
        matches.erase(matches.begin()+i);
        i = i - 1; // Para nao pular um elemento
      }
    } // Fim do for
  } // Fim do void
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void filter_lines(Mat pic, vector<Point2f> &kptl, vector<Point2f> &kptr, vector<DMatch> &matches){
    // Keypoints ja correspondem um a um nesse estagio.
    double mean_coef, sum_coef = 0, stdev_coef;
    float  rate = 1.0f; // Quantos desvios padroes vao passar
    vector<float> coefs(kptl.size());
    for(int i=0; i<kptl.size(); i++){
      // Guardar coordenadas dos pontos
      float xl = kptl[i].x,            yl = kptl[i].y;
      float xr = kptr[i].x + pic.cols, yr = kptr[i].y; // Como se a segunda imagem estivesse a direita
      // Posicionando imagens lado a lado, (anterior a esquerda, atual a direita), calcular coef. angular
      coefs[i] = (yr - yl)/(xr - xl);
    }
    // Criar media e desvio padrao do coef. angular
    mean_coef = accumulate(coefs.begin(), coefs.end(), 0.0)/coefs.size();
    for(vector<float>::iterator it=coefs.begin(); it!=coefs.end(); it++)
        sum_coef += (*it - mean_coef) * (*it - mean_coef);
    stdev_coef = sqrt( sum_coef/(coefs.size()-1) );
    cout << "MEDIA: " << mean_coef << "\tSTD_DEV: " << stdev_coef << endl;
    // Checar quem esta dentro de um desvio padrao
    for(int i=0; i<kptl.size(); i++){
      if( (coefs[i] < (mean_coef-stdev_coef*rate)) || (coefs[i] > (mean_coef+stdev_coef*rate)) ){ // Se fora dos limites
        kptl.erase(kptl.begin()+i);
        kptr.erase(kptr.begin()+i);
        matches.erase(matches.begin()+i);
        coefs.erase(coefs.begin()+i);
        i = i - 1; // Para nao pular um elemento
      }
    } // Fim do for
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
  void set_quadrados(Mat pict, int m, int n, bool vis){
    M = m;
    N = n;
    w = int(pict.cols/m);
    h = int(pict.rows/n);
    int key = 0;

    if(vis){
      Mat pic; pict.copyTo(pic);
      while(key!=115){
        for(int i=0; i<pic.cols; i=i+w)
          line(pic, Point(i, 0), Point(i, pic.rows), Scalar(255), 20, 7, 0);
        for(int j=0; j<pic.rows; j=j+h)
          line(pic, Point(0, j), Point(pic.cols, j), Scalar(255), 20, 7, 0);
        putText(pic, "Se bom aperte s", cvPoint(pic.cols/10,   pic.rows/4), FONT_HERSHEY_COMPLEX, 10,
                cvScalar(0, 250, 0), 10, CV_AA);
        putText(pic, "senao aperte n ", cvPoint(pic.cols/10, 3*pic.rows/4), FONT_HERSHEY_COMPLEX, 10,
                cvScalar(0, 250, 0), 10, CV_AA);

        namedWindow("testando", WINDOW_GUI_NORMAL);
        imshow("testando", pic);
        key = waitKey(0);
        if(key == 115){   // S
          return;
        } else if(key == 110){ // N
          M = M*2; N = N*2;
          w = w/2; h = h/2;
          pict.copyTo(pic);
          cout << "M: " << M << "\tN: " << N << endl;
        }
      }
    }
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////

private:
  bool visualizar;
  Mat camera_matrix, dist_coef, rect; // Vindos do arquivo de calibracao
  Mat previous_image, current_image; // Imagem anterior e atual para comparar
  Mat rt1, rt2, P1, P2;
  Pose pose;
  PointCloud<PointT>::Ptr cloud;
  int M; // Quantos quadrados no eixo  X (columns)
  int N; // Quantos quadrados no eixo -Y (rows)
  int w; // Largura dos quadrados no eixo  X
  int h; // Altura dos quadrados no eixo  -Y
  float scale_to_real_world; // Escala da visao monocular para o mundo real
};
