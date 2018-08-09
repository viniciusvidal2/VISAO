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
#include "../../libraries/zed.cpp"
#include "../../libraries/kalman_simples.cpp"
#include "../../libraries/pose.h"

using namespace std;
using namespace cv;
using namespace message_filters;
// Mensagens
using namespace nav_msgs;
using namespace geometry_msgs;
using namespace sensor_msgs;

typedef sync_policies::ApproximateTime<Odometry, Image>    syncPolicy ;
typedef sync_policies::ApproximateTime<Odometry, Odometry> syncPolicy2;

//////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Variaveis globais
//////////////////////////////////////////////////////////////////////////////////////////////////////////
double roll, pitch, yaw, roll2, pitch2, yaw2; // Leituras de angulos da placa [RAD]
double R2D = 180.0/M_PI; // Conversao
bool primeira_vez = true; // SEMPRE comecamos na primeira vez

ros::Subscriber subodo_, subima_, subzed_; // Subscribers para dados em separado
ros::Publisher pub_im_odo; // Para enviar a odometria vinda so da imagem

Odometry imagem_odometry_msg;

Imagem2 im; // Objeto de tratamento da imagem

Placa placa; // Objeto de tratamento da placa
ros::Publisher pub_placa_odo; // Odometria vinda da placa

ZED zed; // Objeto de tratamento da ZED
ros::Publisher pub_zed_odo; // Odometria calculada e ajustada da ZED

Pose_atual pose_leitura_placa; // Poses referentes a placa
Pose_atual pose_filtro_placa;
Pose_atual pose_leitura_imagem; // Poses referentes a imagem
Pose_atual pose_filtro_imagem;
Pose_atual pose_leitura_zed; // Pose de leitura da zed, vamos ver
Pose_atual pose_filtro_zed;

Kalman_simples kalman; // Kalman simples que vai agir sobre posicao leste E para teste

int contador = 0, contador2 = 0, amostras = 730; // Conta quantas iteracoes passam que dai salvamos ou nao

double min_hessian = 11000; // Threshold inicial de achar keypoints
int min_matches = 10; // Numero minimo de matches entre imagens
float rate_min_dist = 3.0f, ndevs = 1.5f; // Controle de filtragem
//////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////
void init_pose(Pose_atual &_pose){
  _pose.x     = 0; _pose.y      = 0; _pose.z    = 0;
  _pose.e     = 0; _pose.n      = 0; _pose.u    = 0;
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
void so_placa_cb(const nav_msgs::OdometryConstPtr& odo){
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
    // Salvar a nuvem apos tantas iteracoes?
    placa.set_salvar_caminho(true);
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
      // Salvar se foi setado para tal
      string nome = "placa";
      placa.salvar_nuvem(nome);
      ros::shutdown();
    }

  } // Fim do pipeline
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void so_imagem_cb(const ImageConstPtr msg){
  // Imagem vinda da mensagem
  cv_bridge::CvImagePtr imagemptr;
  imagemptr = cv_bridge::toCvCopy(msg, image_encodings::BGR8);

  // Aqui o pipeline decide entre primeira vez, para ajustar tudo, ou nao
  if(primeira_vez){

    // Calibracao e fator de escala, seguindo a camera da esquerda
    string path_left  = "/home/mrs/visao_ws/src/VISAO/match_images/calibracao/zed_left.yaml";
    string path_right = "/home/mrs/visao_ws/src/VISAO/match_images/calibracao/zed_right.yaml";
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
    // Salvar a nuvem apos tantas iteracoes?
    im.set_salvar_caminho(true);
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
      // Pegar a mensagem de odometria e publicar
      im.get_odometry_msg(imagem_odometry_msg);
      imagem_odometry_msg.header.stamp = ros::Time::now(); //msg->header.stamp;
      pub_im_odo.publish(imagem_odometry_msg);
      // Atualiza contador
      cout << "Iteracao: " << contador << endl;
      contador++;
    } else {
      // Salvar se foi setado para tal
      string nome = "imagem";
      im.salvar_nuvem(nome);
      sleep(3);
      ros::shutdown();
    }

  } // Fim do pipeline
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void so_zed_cb(const nav_msgs::OdometryConstPtr& odo){
  /// Armazenar os dados todos
  /// Angulos atuais [RAD]
  /// YAW:   eixo Z para cima da ZED, sentido anti-horario positivo, 0 no inicio da movimentacao
  /// ROLL:  eixo X para a frente da ZED, sentido horario e positivo, 0 no inicio da movimentacao
  /// PITCH: eixo Y para a esquerda da ZED, sentido horario e positivo, bico pra baixo positivo, empinada negativa, 0 no inicio da movimentacao
  tf::Quaternion q(odo->pose.pose.orientation.x, odo->pose.pose.orientation.y, odo->pose.pose.orientation.z, odo->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll2, pitch2, yaw2);
  pose_leitura_zed.roll = rad2deg(roll2); pose_leitura_zed.pitch = rad2deg(pitch2); pose_leitura_zed.yaw = rad2deg(yaw2); // [DEG]
  /// Posicao Body Frame [m]
  /// X: para frente da ZED
  /// Y: para esquerda da ZED
  /// Z: para cima da ZED
  pose_leitura_zed.x = odo->pose.pose.position.x;
  pose_leitura_zed.y = odo->pose.pose.position.y;
  pose_leitura_zed.z = odo->pose.pose.position.z;
  // Print para averiguar
  cout << "roll: " << R2D*roll2          << "\tpitch: " << R2D*pitch2         << "\tyaw: " << R2D*yaw2           << endl;
  cout << "X:    " << pose_leitura_zed.x << "\tY:     " << pose_leitura_zed.y << "\tZ:   " << pose_leitura_zed.z << endl;
  cout << "Iteracao: " << contador2 << endl;

  /// Pipeline
  if(primeira_vez){
    // Setar a pose primeiro para o no da ZED

    // Iniciar tudo
    zed.init();
    // Preencher a previous com o que vier
    zed.set_pose(pose_leitura_zed, 0); // Salvando na PREVIOUS
    // Salvar a nuvem apos tantas iteracoes?
    zed.set_salvar_caminho(true);
    // Ja foi a primeira vez, virar o flag
    primeira_vez = false;

  } else {

    if(contador2 <= amostras){
      // Pose vazia quando so a zed esta funcionando
      Odometry msg_dull;
      // Setar a pose atual com as leituras
      zed.set_pose(pose_leitura_zed, 1); // Salvando na leitura atual
      // Calcular com o pipeline e pegar a pose com diferencas
      zed.process_and_return(pose_filtro_zed, msg_dull);
      // Atualiza contador
      contador2++;
    } else {
      // Salvar se foi setado para tal
      string nome = "zed";
      zed.salvar_nuvem(nome);
      ros::shutdown();
    }

  } // Fim do pipeline
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void placa_e_zed_cb(const nav_msgs::OdometryConstPtr& placa_msg, const nav_msgs::OdometryConstPtr& zed_msg){
  /////////////////// ----- PLACA ----- ///////////////////
  /// Angulos atuais
  /// Angulos atuais [RAD]
  /// YAW:   eixo Z para cima da placa, sentido anti-horario positivo, 0 para leste, 180 em oeste, de -180 a 180
  /// ROLL:  eixo X para a frente da placa, sentido horario e positivo, -180 a 180, 0 na horizontal
  /// PITCH: eixo Y para a esquerda da placa, sentido mao direita, bico pra baixo positivo, empinada negativa, -90 a 90 (nao da uma volta)
  tf::Quaternion qplaca(placa_msg->pose.pose.orientation.x, placa_msg->pose.pose.orientation.y, placa_msg->pose.pose.orientation.z, placa_msg->pose.pose.orientation.w);
  tf::Matrix3x3 mplaca(qplaca);
  mplaca.getRPY(roll, pitch, yaw);
  pose_leitura_placa.roll = rad2deg(roll); pose_leitura_placa.pitch = rad2deg(pitch); pose_leitura_placa.yaw = rad2deg(yaw); // [DEG]
  /// Posicao
  /// Posicao ENU [m]
  /// PN: Posicao norte, Y da mensagem, cresce para norte do mapa
  /// PE: Posicao leste, X da mensagem, cresce para leste do mapa
  /// PZ: Posicao vertical, Z da mensagem, cresce para cima
  pose_leitura_placa.n = placa_msg->pose.pose.position.y;
  pose_leitura_placa.e = placa_msg->pose.pose.position.x;
  pose_leitura_placa.u = placa_msg->pose.pose.position.z;
  /////////////////// ----- ZED ----- ///////////////////
  /// Angulos atuais [RAD]
  /// YAW:   eixo Z para cima da ZED, sentido anti-horario positivo, 0 no inicio da movimentacao
  /// ROLL:  eixo X para a frente da ZED, sentido horario e positivo, 0 no inicio da movimentacao
  /// PITCH: eixo Y para a esquerda da ZED, sentido horario e positivo, bico pra baixo positivo, empinada negativa, 0 no inicio da movimentacao
  tf::Quaternion qzed(zed_msg->pose.pose.orientation.x, zed_msg->pose.pose.orientation.y, zed_msg->pose.pose.orientation.z, zed_msg->pose.pose.orientation.w);
  tf::Matrix3x3 mzed(qzed);
  mzed.getRPY(roll2, pitch2, yaw2);
  pose_leitura_zed.roll = rad2deg(roll2); pose_leitura_zed.pitch = rad2deg(pitch2); pose_leitura_zed.yaw = rad2deg(yaw2); // [DEG]
  /// Posicao Body Frame [m]
  /// X: para frente da ZED
  /// Y: para esquerda da ZED                  Valores no frame da camera, serao transformados para o frame ENU da placa
  /// Z: para cima da ZED                      dentro da classe. Valores ds calculados a partir do previous
  pose_leitura_zed.x = zed_msg->pose.pose.position.x;
  pose_leitura_zed.y = zed_msg->pose.pose.position.y;
  pose_leitura_zed.z = zed_msg->pose.pose.position.z;

  // Mensagens de odometria para confirmar no RVIZ
  Odometry msg_placa_odo;
  Odometry msg_zed_odo;

  /////////////////// ----- PRIMEIRA VEZ ----- ///////////////////
  if(primeira_vez){
    // Iniciar PLACA
    placa.init();
    // Preencher a previous com o que vier
    placa.set_pose(pose_leitura_placa, 0); // Salvando na PREVIOUS
    // Salvar a nuvem apos tantas iteracoes?
    placa.set_salvar_caminho(true);
    // Iniciar ZED
    zed.init();
    // Preencher a previous com o que vier
    zed.set_pose(pose_leitura_placa, 0); // Salvando na PREVIOUS e na OFFSET com leitura da PLACA de uma vez para iniciar a estimativa
    zed.set_pose(pose_leitura_zed  , 2); // Salvando na PREVIOUS os offsets de leitura da ZED ao inicio da BAG (forcar o que ja andou ali a 0)
    // Salvar a nuvem apos tantas iteracoes?
    zed.set_salvar_caminho(true);
    // Iniciar o kalman simples
//    kalman.init(placa_msg->pose.covariance.at(0), pose_leitura_placa.x, placa_msg->pose.covariance.at(0), pose_leitura_placa.x);
    // Atualizar contador
    contador++;
    // Ja foi a primeira vez, virar o flag
    primeira_vez = false;
  } else {
  /////////////////// ----- PIPELINE ----- ///////////////////
    if(contador <= amostras){
      // Setar a pose atual com as leituras
      placa.set_pose(pose_leitura_placa, 1); // Salvando na leitura atual
      // Calcular com o pipeline e pegar a pose com diferencas
      placa.process_and_return(pose_filtro_placa);
      // A mensagem da placa ja e a pose definitiva, copia para enviar
      msg_placa_odo = *placa_msg;
      // Setar a pose atual com as leituras
      zed.set_pose(pose_leitura_zed, 1); // Salvando na leitura atual
      // Calcular com o pipeline e pegar a pose com diferencas e a mensagem de odometria
      zed.process_and_return(pose_filtro_zed, msg_zed_odo);
      // Publicar as duas poses para vefiricar no rviz
      msg_zed_odo.header.frame_id = msg_placa_odo.header.frame_id;
      msg_zed_odo.header.stamp    = msg_placa_odo.header.stamp;
//      msg_zed_odo = *zed_msg;
      pub_placa_odo.publish(msg_placa_odo);
      pub_zed_odo.publish(msg_zed_odo);
      // Atualiza contador
      contador++;
      cout << "\nIteracao: " << contador << endl;
      ////////////// AQUI ENTRA O FILTRO DE KALMAN


    } else {
      // Salvar se foi setado para tal
      string nome = "zed";
      zed.salvar_nuvem(nome);
      nome = "placa";
      placa.salvar_nuvem(nome);

      ros::shutdown();
    }
  }
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
  rate.request.message_rate = 5; // X Hz das mensagens que vem
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

  // Iniciar poses de leitura para placa e imagem
  init_pose(pose_leitura_placa);
  init_pose(pose_filtro_placa);
  init_pose(pose_leitura_imagem);
  init_pose(pose_filtro_imagem);
  init_pose(pose_leitura_zed);

  /// ------------ Odometria da placa e Imagem da ZED ------------- ///
  /// ------------------------------------------------------------- ///
//  message_filters::Subscriber<Odometry> subodo(nh, "/mavros/global_position/local", 100);
//  message_filters::Subscriber<Image>    subima(nh, "/zed/left/image_raw_color"    , 100);
//  // Sincroniza as leituras dos topicos (sensores e imagem a principio) em um so callback
//  Synchronizer<syncPolicy> sync1(syncPolicy(100), subodo, subima);
//  sync1.registerCallback(boost::bind(&informacoes_sincronizadas, _1, _2));

  /// ------------ Odometria da placa e Odometria da ZED ---------- ///
  /// --------------------------- AQUI!!--------------------------- ///
  message_filters::Subscriber<Odometry> subplaca(nh, "/mavros/global_position/local", 100);
  message_filters::Subscriber<Odometry> subzed(  nh, "/zed/odom"                    , 100);
  // Sincroniza as leituras dos topicos (sensores e imagem a principio) em um so callback
  Synchronizer<syncPolicy2> sync2(syncPolicy2(100), subplaca, subzed);
  sync2.registerCallback(boost::bind(&placa_e_zed_cb, _1, _2));
  pub_placa_odo = nh.advertise<Odometry>("/odometria_da_placa", 100);
  pub_zed_odo   = nh.advertise<Odometry>("/odometria_da_zed"  , 100);

  /// ----------------------- Somente a placa --------------------- ///
  /// ------------------------------------------------------------- ///
//  subodo_ = nh.subscribe("/mavros/global_position/local", 100, so_placa_cb );

  /// ------------------ Somente imagem da camera ----------------- ///
  /// ------------------------------------------------------------- ///
//  subima_ = nh.subscribe("/zed/left/image_raw_color"   , 100, so_imagem_cb);
//  pub_im_odo = nh.advertise<Odometry>("/so_imagem/odom", 100);

  /// ------------------ Somente a ZED odometria ------------------ ///
  /// ------------------------------------------------------------- ///
//  subzed_ = nh.subscribe("/zed/odom"                    , 100, so_zed_cb   );


  ros::spin();

  return 0;
}
