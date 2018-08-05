#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <nav_msgs/Odometry.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <boost/thread/thread.hpp>
#include <iostream>
#include <string>
#include <numeric>

#include <ros/ros.h>

#include "pose.h"

using namespace pcl;
using namespace pcl::visualization;
using namespace std;
using namespace nav_msgs;
using namespace Eigen;

class ZED
{
public:
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////// Inicio ////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void init(){
    // Iniciar pose atual
    pose.x     = 0; pose.y      = 0; pose.z    = 0;
    pose.e     = 0; pose.n      = 0; pose.u    = 0;
    pose.dx    = 0; pose.dy     = 0; pose.dz   = 0;
    pose.roll  = 0; pose.pitch  = 0; pose.yaw  = 0;
    pose.droll = 0; pose.dpitch = 0; pose.dyaw = 0;
    // Iniciar pose anterior
    pose_previous.x     = 0; pose_previous.y      = 0; pose_previous.z    = 0;
    pose_previous.e     = 0; pose_previous.n      = 0; pose_previous.u    = 0;
    pose_previous.dx    = 0; pose_previous.dy     = 0; pose_previous.dz   = 0;
    pose_previous.roll  = 0; pose_previous.pitch  = 0; pose_previous.yaw  = 0;
    pose_previous.droll = 0; pose_previous.dpitch = 0; pose_previous.dyaw = 0;
    // Iniciar pose de offset
    pose_offset.x       = 0; pose_offset.y        = 0; pose_offset.z      = 0;
    pose_offset.e       = 0; pose_offset.n        = 0; pose_offset.u      = 0;
    pose_offset.dx      = 0; pose_offset.dy       = 0; pose_offset.dz     = 0;
    pose_offset.roll    = 0; pose_offset.pitch    = 0; pose_offset.yaw    = 0;
    pose_offset.droll   = 0; pose_offset.dpitch   = 0; pose_offset.dyaw   = 0;
    // Por default nao queremos salvar a nuvem
    vamos_salvar_nuvem = false;
    // Inicio da nuvem
    nuvem = (pcl::PointCloud<PointXYZRGBNormal>::Ptr) new pcl::PointCloud<PointXYZRGBNormal>;
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////// SETS /////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void set_pose(Pose_atual _pose, int code){
    if(code == 0){ // Aqui atualizamos a pose previous e a offset com a POSE DA PLACA
      pose_previous = _pose;
      pose_offset   = _pose;
    } else if(code == 1){ // Aqui atualizamos a pose atual
      pose = _pose;
    }
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void set_salvar_caminho(bool salvar){
    vamos_salvar_nuvem = salvar;
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////// Principal ///////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void process_and_return(Pose_atual &_pose, Odometry &msg_odo){
    /// A pose atual ja foi atualizada no script principal, contem o que foi lido nesse instante em:
    /// X, Y, Z, ROLL, PITCH, YAW no frame da ZED
    // Temporarios para armazenar xy e transformar para EN
    if(true){
      cout << "########### ZED sozinha\n";
      cout << "X: " << pose.x << "\tY: " << pose.y << "\tZ: " << pose.z << endl;
    }
    // Rotacao a partir do offset de yaw inicial - x => E, y => N
    pose.e = pose.x; //pose.x*cos(deg2rad(pose_offset.yaw)) - pose.y*sin(deg2rad(pose_offset.yaw));
    pose.n = pose.y; //pose.x*sin(deg2rad(pose_offset.yaw)) + pose.y*cos(deg2rad(pose_offset.yaw));
    pose.u = pose.z;
    // Adicao da translacao do offset ENU (primeira mensagem da placa)
    pose.e += pose_offset.e;
    pose.n += pose_offset.n;
    pose.u += pose_offset.u;
    // Diferencas entre pose atual e previous
    pose.dx = pose.e - pose_previous.e;
    pose.dy = pose.n - pose_previous.n;
    pose.dz = pose.u - pose_previous.u;
    // Angulos atuais RPY com offsets da primeira mensagem - todos em DEGREES
    pose.roll  = bound180(pose.roll  + pose_offset.roll );
    pose.pitch = bound180(pose.pitch + pose_offset.pitch);
    pose.yaw   = bound180(pose.yaw   + pose_offset.yaw  );
    // Diferencas de angulos com a mensagem previous
    pose.droll  = bound180(pose.roll  - pose_previous.roll );
    pose.dpitch = bound180(pose.pitch - pose_previous.pitch);
    pose.dyaw   = bound180(pose.yaw   - pose_previous.yaw  );

    // Printar estado atual
    if(true){
      cout << "########### ZED\n";
      cout << "E: " << pose.e << "\tN: " << pose.n << "\tU: " << pose.u << endl;
      cout << "########### OFFSET\n";
      cout << "E: " << pose_offset.e << "\tN: " << pose_offset.n << "\tU: " << pose_offset.u << "\tYaw: " << pose_offset.yaw << endl << endl << endl << endl;
    }

    // Passar a mensagem atual para a previous
    pose_previous = pose;
    // Repassar para o script principal a pose calculada
    _pose = pose;

    /// Retornar para a mensagem de odometria
    // Calculo do quaternion relativo - forcar roll a 0 (futuramente considerar leitura do viso2)
    Quaternion<double> q = AngleAxisd(deg2rad(pose.pitch), Vector3d::UnitY()) *
                           AngleAxisd(deg2rad(pose.roll ), Vector3d::UnitX()) *
                           AngleAxisd(deg2rad(pose.yaw  ), Vector3d::UnitZ());
    // Inteirando mensagem de odometria
    msg_odo.pose.pose.position.x = pose.e; msg_odo.pose.pose.position.y = pose.n; msg_odo.pose.pose.position.z = pose.u;
    msg_odo.pose.pose.orientation.x = q.x(); msg_odo.pose.pose.orientation.y = q.y();
    msg_odo.pose.pose.orientation.z = q.z(); msg_odo.pose.pose.orientation.w = q.w();

    // Salvar a nuvem com a pose atual?
    if(vamos_salvar_nuvem)
      atualizar_nuvem();
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
  double bound180(double ang){
    // Manter o limite entre -180 e 180 graus, e o
    // sentido de giro e positivo para anti-horario
    if(ang >  180.0) ang = ang - 360.0;
    if(ang < -180.0) ang = ang + 360.0;

    return ang;
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  double wrap180(double ang_atual, double ang_previous){
    double delta = ang_atual - ang_previous;
    // Manter o limite entre -180 e 180 graus, e o
    // sentido de giro e positivo para anti-horario
    if(delta >  180.0) delta = delta - 360.0;
    if(delta < -180.0) delta = delta + 360.0;

    return delta;
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void atualizar_nuvem(){
    // Preenchendo o ponto com dados atuais
    point.x        = pose.x;
    point.y        = pose.y;
    point.z        = pose.z;
    point.r = 250.0f; point.g = 0.0f; point.b = 0.0f; // VERMELHO porque nao esta tao bacana
    point.normal_x = pose.roll;
    point.normal_y = pose.pitch;
    point.normal_z = pose.yaw;
    // Adicionando a nuvem
    nuvem->push_back(point);
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void visualizar_nuvem(){
    boost::shared_ptr<PCLVisualizer> vis_placa (new PCLVisualizer("caminho zed"));
    vis_placa->addPointCloud<PointXYZRGBNormal>(nuvem, "caminho zed");
    vis_placa->spin();
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// Variaveis privadas
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  Pose_atual pose_previous; // Somente a pose anterior, os deltas nao sao considerados, por mais que sejam atualizados
  Pose_atual pose;          // Tudo aqui e considerado para entrar no filtro
  Pose_atual pose_offset;   // Pose de offset com os dados iniciais da placa

  bool vamos_salvar_nuvem; // Vamos ou nao gravar a nuvem para ver o trajeto depois
  PointCloud<PointXYZRGBNormal>::Ptr nuvem; // Nuvem com o caminho, onde as normais sao os angulos
  PointXYZRGBNormal point; // Ponto atual para a nuvem
};
