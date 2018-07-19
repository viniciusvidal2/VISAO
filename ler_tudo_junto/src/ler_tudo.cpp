#include "ros/ros.h"
#include <vector>
#include <time.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

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

#include <mavros/mavros.h>
#include <mavros/utils.h>
#include <mavros/mavros.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/mavlink_convert.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/StreamRate.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>

#include <nav_msgs/Odometry.h>

#include <geometry_msgs/PoseStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <std_msgs/Float32.h>

#include <tf/tf.h>
#include <eigen3/Eigen/Eigen>
#include <math.h>

// Nossos includes
#include "../../libraries/imagem2.cpp"
#include "../../libraries/placa.cpp"
#include "../../libraries/pose.h"

using namespace std;
using namespace cv;
using namespace message_filters;
// Mensagens
using namespace nav_msgs;
using namespace geometry_msgs;
using namespace sensor_msgs;

typedef sync_policies::ApproximateTime<Odometry, Image> syncPolicy;

//////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Variaveis globais
//////////////////////////////////////////////////////////////////////////////////////////////////////////
double roll, pitch, yaw; // Leituras de angulos da placa [RAD]
double R2D = 180.0/M_PI; // Conversao
bool so_sensores = true, so_imagem = false; // Flags de controle
bool primeira_vez = true;

ros::Subscriber subodo_, subima_; // Subscribers para dados em separado

Imagem2 im; // Objeto de tratamento da imagem

Placa placa; // Objeto de tratamento da placa

Pose_atual pose_leitura_placa; // Poses referentes a placa
Pose_atual pose_filtro_placa;
Pose_atual pose_leitura_imagem; // Poses referentes a imagem
Pose_atual pose_filtro_imagem;

int contador, amostras; // Conta quantas iteracoes passam que dai salvamos ou nao
string caminho_atual;

double min_hessian = 2000; // Threshold inicial de achar keypoints
int min_matches = 20; // Numero minimo de matches entre imagens
float rate_min_dist = 3.0f, ndevs = 1.5f; // Controle de filtragem
//////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Matematica retirada de:
/// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
void toEulerAngle(Quaternion q, double& roll, double& pitch, double& yaw)
{
  // roll (x-axis rotation)
  double sinr = 2.0*(q.w*q.x + q.y*q.z);
  double cosr = 1.0 - 2.0*(q.x*q.x + q.y*q.y);
  roll = atan2(sinr, cosr);

  // pitch (y-axis rotation)
  double sinp = 2.0*(q.w*q.y - q.z*q.x);
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI/2, sinp); // use 90 degrees if out of range
  else
    pitch = asin(sinp);

  // yaw (z-axis rotation)
  double siny = 2.0*(q.w*q.z + q.x*q.y);
  double cosy = 1.0 - 2.0*(q.y*q.y + q.z*q.z);
  yaw = atan2(siny, cosy);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void init_pose(Pose_atual &_pose){
  _pose.x     = 0; _pose.y      = 0; _pose.z    = 0;
  _pose.dx    = 0; _pose.dy     = 0; _pose.dz   = 0;
  _pose.roll  = 0; _pose.pitch  = 0; _pose.yaw  = 0;
  _pose.droll = 0; _pose.dpitch = 0; _pose.dyaw = 0;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void informacoes_sincronizadas(const OdometryConstPtr& odo, const ImageConstPtr& im){
  /// Armazenar os dados todos
  // Angulos atuais [RAD]
  // YAW:   eixo Z para cima da placa, sentido anti-horario positivo, 0 para leste, 180 em oeste, de -180 a 180
  // ROLL:  eixo X para a frente da placa, sentido horario e positivo, -180 a 180, 0 na horizontal
  // PITCH: eixo Y para a esquerda da placa, sentido mao direita, bico pra baixo positivo, empinada negativa, -90 a 90 (nao da uma volta)
  tf::Quaternion q(odo->pose.pose.orientation.x, odo->pose.pose.orientation.y,
                   odo->pose.pose.orientation.z, odo->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  // Posicao ENU [m]
  // PN: Posicao norte, Y da mensagem, cresce para norte do mapa
  // PE: Posicao leste, X da mensagem, cresce para leste do mapa
  // PZ: Posicao vertical, Z da mensagem, cresce para cima

  // Imagem
  cv_bridge::CvImagePtr imagemptr;
  imagemptr = cv_bridge::toCvCopy(im, image_encodings::BGR8);
  imwrite("pasta", imagemptr->image);
  // Salvar texto com as informacoes: posicao e orientacao
  // Salvar a imagem
  // Fechar o no por enquanto
  ros::shutdown();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void so_placa(const nav_msgs::OdometryConstPtr& odo){
  /// Armazenar os dados todos
  /// Angulos atuais
  /// Angulos atuais [RAD]
  /// YAW:   eixo Z para cima da placa, sentido anti-horario positivo, 0 para leste, 180 em oeste, de -180 a 180
  /// ROLL:  eixo X para a frente da placa, sentido horario e positivo, -180 a 180, 0 na horizontal
  /// PITCH: eixo Y para a esquerda da placa, sentido mao direita, bico pra baixo positivo, empinada negativa, -90 a 90 (nao da uma volta)
  tf::Quaternion q(odo->pose.pose.orientation.x, odo->pose.pose.orientation.y, odo->pose.pose.orientation.z, odo->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  pose_leitura_placa.roll = rad2deg(roll); pose_leitura_placa.pitch = rad2deg(pitch); pose_leitura_placa.yaw = rad2deg(yaw); // [DEG]
  /// Posicao
  /// Posicao ENU [m]
  /// PN: Posicao norte, Y da mensagem, cresce para norte do mapa
  /// PE: Posicao leste, X da mensagem, cresce para leste do mapa
  /// PZ: Posicao vertical, Z da mensagem, cresce para cima
  pose_leitura_placa.y = odo->pose.pose.position.y;
  pose_leitura_placa.x = odo->pose.pose.position.x;
  pose_leitura_placa.z = odo->pose.pose.position.z;
  // Print para averiguar
  cout << "roll: " << R2D*roll             << "\tpitch: " << R2D*pitch            << "\tyaw: " << R2D*yaw              << endl;
  cout << "pn:   " << pose_leitura_placa.y << "\tpe:    " << pose_leitura_placa.x << "\talt: " << pose_leitura_placa.z << endl;
  cout << "Iteracao: " << contador << endl;

  /// Pipeline
  if(primeira_vez){

    // Iniciar tudo
    placa.init();
    // Preencher a previous com o que vier
    placa.set_pose(pose_leitura_placa, 0); // Salvando na PREVIOUS
    // Ja foi a primeira vez, virar o flag
    primeira_vez = false;

  } else {

    if(contador <= amostras){
      // Setar a pose atual com as leituras
      placa.set_pose(pose_leitura_placa, 1); // Salvando na leitura atual
      // Calcular com o pipeline e pegar a pose com diferencas
      placa.process_and_return(pose_filtro_placa);
      // Atualiza contador
      contador++;
    } else {
      // Salvar a nuvem apos tantas iteracoes?
      placa.set_salvar_caminho(true);
      string nome = "placa";
      placa.salvar_nuvem(nome);
      ros::shutdown();
    }

  } // Fim do pipeline
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void so_imagem2(const ImageConstPtr msg){
  // Imagem vinda da mensagem
  cv_bridge::CvImagePtr imagemptr;
  imagemptr = cv_bridge::toCvCopy(msg, image_encodings::BGR8);

  // Aqui o pipeline decide entre primeira vez, para ajustar tudo, ou nao
  if(primeira_vez){

    // Calibracao e fator de escala, seguindo a camera da esquerda
    string path_left  = "calibracao, esquerda";
    string path_right = "calibracao direita";
    im.read_camera_calibration(path_left.c_str());
    im.scale_factor(path_left.c_str(), path_right.c_str());
    // Inicia as imagens
    im.set_image_current(imagemptr->image);
    im.set_image_previous(imagemptr->image);
    // Remover distorcao das imagens
    im.undistort_image(0); // Imagem previous somente
    // Ajustar bins para filtrar correspondencias
    im.set_quadrados(imagemptr->image, 5, 5, false);
    // Iniciar variaveis no geral
    im.init();
    im.set_pose(pose_leitura_imagem);
    // Ja esta tudo pronto para a proxima!
    primeira_vez = false;

  } else { // Aqui temos a acao com a imagem!

    if(contador <= amostras){
      // Imagem atual colocada
      im.set_image_current(imagemptr->image);
      // Remover distorcao da imagem para garantir
      im.undistort_image(1); // Imagem atual somente
      // Pipeline todo aqui
      im.pipeline(min_hessian, min_matches, rate_min_dist, ndevs);
      // Pegar a pose calculada a partir da anterior para o filtro
      im.get_pose(pose_filtro_imagem);
    } else {
      im.set_salvar_caminho(true);
      // Salvar a nuvem apos tantas iteracoes?
      string nome = "imagem";
      im.salvar_nuvem(nome);
      ros::shutdown();
    }

  } // Fim do pipeline
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ler_tudo");
  ros::NodeHandle nh;

  // Aguardar a conexao segura do mavros para nao cair o no
  sleep(2);

  // Ajustando taxa de conversa do mavros
  ros::ServiceClient srv = nh.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");
  mavros_msgs::StreamRate rate;
  rate.request.stream_id = 0;
  rate.request.message_rate = 10; // X Hz das mensagens que vem
  rate.request.on_off = 1; // Nao sei
  if(srv.call(rate))
    ROS_INFO("Taxa do mavros mudada para %d Hz", rate.request.message_rate);
  else
    ROS_INFO("Nao pode chamar o servico, taxa nao mudada.");
  // Armando o veiculo nesse caso
  ros::ServiceClient armar_srv = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  mavros_msgs::CommandBool armar;
  armar.request.value = true;
  if(armar_srv.call(armar))
    ROS_INFO("Veiculo armado para salvar origem.");
  else
    ROS_INFO("Nao foi possivel armar o veiculo.");

  // Visualizar ou nao as imagens da camera
  im.set_visualizar(false);
  // Contador de leituras da placa
  contador = 0; amostras = 2000;

  // Iniciar poses de leitura para placa e imagem
  init_pose(pose_leitura_placa);
  init_pose(pose_filtro_placa);
  init_pose(pose_leitura_imagem);
  init_pose(pose_filtro_imagem);

  // Subscribers para ler sincronizadas as informacoes
  if(!so_sensores && !so_imagem){
    message_filters::Subscriber<Odometry> subodo(nh, "/mavros/global_position/local", 100);
    message_filters::Subscriber<Image>    subima(nh, "/stereo/left/image_rect"      , 100);
    // Sincroniza as leituras dos topicos (sensores e imagem a principio) em um so callback
    Synchronizer<syncPolicy> sync(syncPolicy(100), subodo, subima);
    sync.registerCallback(boost::bind(&informacoes_sincronizadas, _1, _2));
  }
  // Subscriber para ler so a placa com a mensagem de pose
  if(so_sensores)
    subodo_ = nh.subscribe("/mavros/global_position/local", 100, so_placa);
  // Subscriber para ler so o topico de imagens
  if(so_imagem)
    subima_ = nh.subscribe("nomedotopicoaqui", 100, so_imagem2);

  ros::spin();

  return 0;
}
