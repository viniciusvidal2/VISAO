#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

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

#include <nav_msgs/Odometry.h>

#include <vector>
#include <time.h>

#include "pose.h"

using namespace pcl;
using namespace pcl::visualization;
using namespace cv;
using namespace std;
using namespace nav_msgs;

class Kalman_completo
{

public:

  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void init(Pose_atual estimate, vector<double> error_est){
    /// Inicio Geral
    X_kp = Mat(6, 1, CV_64FC1);
    P_kp = Mat(6, 6, CV_64FC1);
    X_k  = Mat(6, 1, CV_64FC1);
    P_k  = Mat(6, 6, CV_64FC1);
    Y    = Mat(6, 1, CV_64FC1);
    u    = Mat(6, 1, CV_64FC1);
    R    = Mat(6, 6, CV_64FC1);
    KG   = Mat(6, 6, CV_64FC1);

    /// Inicio do estado do processo e da estimativa na iteracao k
    X_kp.at<double>(0, 0) = estimate.e;
    X_kp.at<double>(1, 0) = estimate.n;
    X_kp.at<double>(2, 0) = estimate.u;
    X_kp.at<double>(3, 0) = estimate.roll;
    X_kp.at<double>(4, 0) = estimate.pitch;
    X_kp.at<double>(5, 0) = estimate.yaw;
    X_k = X_kp;

    /// Inicio da matriz de covariancias de estados
    P_kp = Mat::diag(Mat(error_est));
    P_k  = P_kp;

//    cout << "Matriz de estados: \n" << X_kp << "\nCovariancias:\n" << P_k << endl;

    // Inicio da nuvem com o caminho filtrado
    nuvem = (pcl::PointCloud<PointXYZRGBNormal>::Ptr) new pcl::PointCloud<PointXYZRGBNormal>;
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////// GETS ///////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void get_estimate(Pose_atual &estimate){
    estimate.x     = X_k.at<double>(0, 0);
    estimate.y     = X_k.at<double>(1, 0);
    estimate.z     = X_k.at<double>(2, 0);
    estimate.roll  = X_k.at<double>(3, 0);
    estimate.pitch = X_k.at<double>(4, 0);
    estimate.yaw   = X_k.at<double>(5, 0);
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////// SETS /////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void set_salvar_caminho(bool salvar){
    vamos_salvar_nuvem = salvar;
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void set_debug(bool d){
    vamos_debugar = d;
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void set_measure_covariance_matrix(Odometry cov){
    // Aqui a covariancia da posicao, passada direto
    R.at<double>(0, 0) = cov.pose.covariance.at( 0);
    R.at<double>(1, 1) = cov.pose.covariance.at( 7);
    R.at<double>(2, 2) = cov.pose.covariance.at(14);
    // Valores de covariancias para angulos -> sensor muito bom, quase sempre desconsideradas
    R.at<double>(3, 3) = (cov.pose.covariance.at(21) > 10) ? cov.pose.covariance.at(21) : 0.01;
    R.at<double>(4, 4) = (cov.pose.covariance.at(28) > 10) ? cov.pose.covariance.at(28) : 0.01;
    R.at<double>(5, 6) = (cov.pose.covariance.at(35) > 10) ? cov.pose.covariance.at(35) : 0.01;
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////// PRINCIPAL ////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void filter(Pose_atual estimate_update, Pose_atual measurement, Pose_atual &filtered){
    /// Atualiza os estados do processo Xp do filtro com a leitura vinda da imagem(modelo)
    update_process_state(estimate_update);
    /// Calculo do ganho de Kalman com o que temos de covariancias
    calculate_KG();
    /// Atualiza estimativa frente ao novo ganho de Kalman e a nova leitura do sensor
    update_estimate(measurement);
    /// Nova covariancia da estimativa
    update_estimate_covariance();

    // Salvar na nuvem do caminho
    atualizar_nuvem();

    // Printar na tela o debug?
    if(vamos_debugar)
      print_debug();

    // Resultado filtrado
    get_estimate(filtered);
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void salvar_nuvem(string path){
    if(vamos_salvar_nuvem){
      ROS_INFO("Salvando o caminho...");
      // Ver o tempo para diferenciar bags gravadas automaticamente
      time_t t = time(0);
      struct tm * now = localtime( & t );
      std::string month, day, hour, minutes;
      month   = boost::lexical_cast<std::string>(now->tm_mon );
      day     = boost::lexical_cast<std::string>(now->tm_mday);
      hour    = boost::lexical_cast<std::string>(now->tm_hour);
      minutes = boost::lexical_cast<std::string>(now->tm_min );
      string date = "_" + month + "_" + day + "_" + hour + "h_" + minutes + "m";
      string filename = "/home/vinicius/visao_ws/src/VISAO/ler_tudo_junto/caminhos/"+path+date+".ply";
      // Salvando com o nome diferenciado
      if(!io::savePLYFileASCII(filename, *nuvem))
        cout << "\n\nSalvo na pasta caminhos com o nome "+path+date+".ply" << endl;
      // Visualizar a nuvem com PCL
      visualizar_nuvem();
    } else {
      ROS_INFO("NAO SALVAMOS O CAMINHO PORQUE NAO ERA PRA SALVAR MANEZAO");
    }
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////

private:

  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void update_process_state(Pose_atual est_up){
    // NOTA: a matriz de covariancia do processo nao e alterada aqui, somente na atulaizacao vinda da
    // iteracao anterior
    P_kp   = P_k;
    // Entradas do processo, vindas da ZED
    u.at<double>(0, 0) = est_up.dx;
    u.at<double>(1, 0) = est_up.dy;
    u.at<double>(2, 0) = est_up.dz;
    u.at<double>(3, 0) = est_up.droll;
    u.at<double>(4, 0) = est_up.dpitch;
    u.at<double>(5, 0) = est_up.dyaw;
    // Pega o estimado anteriormente e coloca no estado do processo
    X_kp   = X_k;
    // Adiciona as entradas no estado atual
    X_kp  += u; // Matriz B identidade aqui
//    cout << "Entrada da ZED \n" << u << "\nProcesso aumentado:\n" << X_kp << endl;
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void calculate_KG(){
    // Ganho de Kalman voando aqui
    KG  = P_kp*(P_kp + R).inv();
//    cout << "Ganho de Kalman:\n" << KG << endl;
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void update_estimate(Pose_atual meas){
    // Ler entrada do GPS
    Y.at<double>(0, 0) = meas.e;
    Y.at<double>(1, 0) = meas.n;
    Y.at<double>(2, 0) = meas.u;
    Y.at<double>(3, 0) = meas.roll;
    Y.at<double>(4, 0) = meas.pitch;
    Y.at<double>(5, 0) = meas.yaw;
    // Nova estimativa / estado do sistema
    X_k = X_kp + KG*( Y - X_kp );
//    cout << "Nova estimativa: \n" << X_k << endl;
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void update_estimate_covariance(){
    P_k = (Mat::eye(Size(6, 6), KG.type()) - KG)*P_kp;
//    cout << "Matriz de covariancias anterior:\n" << P_kp << endl << "Matriz de covariancias nova: \n" << P_k << endl;
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void print_debug(){
    cout << "#############################################"    << endl;
    cout << "ESTADO X: [" << X_k << "]\tMEDIDO: [" << Y << "]" << endl;
    cout << "Ganho   : [" << KG  << "]"                        << endl;
    cout << "Cov. do ESTADO: [" << P_k << "]"                  << endl;
    cout << "#############################################"    << endl;
    cout << endl;
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void atualizar_nuvem(){
    // Preenchendo o ponto com dados atuais
    point.x        = X_k.at<double>(0, 0);
    point.y        = X_k.at<double>(1, 0);
    point.z        = X_k.at<double>(2, 0);
    point.r = 0.0f; point.g = 250.0f; point.b = 0.0f; // Verde, melhor resultado
    point.normal_x = X_k.at<double>(3, 0);
    point.normal_y = X_k.at<double>(4, 0);
    point.normal_z = X_k.at<double>(5, 0);
    // Adicionando a nuvem
    nuvem->push_back(point);
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void visualizar_nuvem(){
    boost::shared_ptr<PCLVisualizer> vis_placa (new PCLVisualizer("caminho Kalman completo"));
    vis_placa->addPointCloud<PointXYZRGBNormal>(nuvem, "caminho Kalman completo");
    vis_placa->spin();
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// Variaveis privadas - Retiradas de ILectureOnline no youtube
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  Mat KG, X_kp, X_k, u, P_kp, P_k, R, Y;

  bool vamos_salvar_nuvem;
  bool vamos_debugar;

  PointCloud<PointXYZRGBNormal>::Ptr nuvem; // Nuvem com o caminho, onde as normais sao os angulos
  PointXYZRGBNormal point; // Ponto atual para a nuvem

};
