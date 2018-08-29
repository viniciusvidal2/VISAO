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
    pose_offset_placa.x       = 0; pose_offset_placa.y        = 0; pose_offset_placa.z      = 0;
    pose_offset_placa.e       = 0; pose_offset_placa.n        = 0; pose_offset_placa.u      = 0;
    pose_offset_placa.dx      = 0; pose_offset_placa.dy       = 0; pose_offset_placa.dz     = 0;
    pose_offset_placa.roll    = 0; pose_offset_placa.pitch    = 0; pose_offset_placa.yaw    = 0;
    pose_offset_placa.droll   = 0; pose_offset_placa.dpitch   = 0; pose_offset_placa.dyaw   = 0;
    // Por default nao queremos salvar a nuvem
    vamos_salvar_nuvem = false;
    // Decaimento
    decaimento_bussola = 15.0;
    // Inicio da nuvem
    nuvem = (pcl::PointCloud<PointXYZRGBNormal>::Ptr) new pcl::PointCloud<PointXYZRGBNormal>;
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////// SETS /////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void set_pose(Pose_atual _pose, int code){
    if(code == 1){ // Aqui atualizamos a pose atual
      pose = _pose;
    } else if(code == 0){ // Aqui atualizamos a pose previous e a offset com a POSE DA PLACA
      pose_previous     = _pose;
      pose_offset_placa = _pose;
      pose_offset_placa.yaw += decaimento_bussola; // Guardar o decaimento observado na pratica
    } else if(code == 2){
      pose_offset_ZED = _pose; // Guardando o possivel offset vindo da propria ZED
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
    if(false){
      cout << "########### ZED sozinha\n";
      cout << "X: " << pose.x << "\tY: " << pose.y << "\tZ: " << pose.z << endl;
    }
    // Retirando ruido inicial leitura da propria ZED
    pose.x -= pose_offset_ZED.x; pose.y -= pose_offset_ZED.y; pose.z -= pose_offset_ZED.z;
    pose.roll -= pose_offset_ZED.roll; pose.pitch -= pose_offset_ZED.pitch; pose.yaw -= pose_offset_ZED.yaw;
    // Rotacao a partir do offset de yaw inicial - x => E, y => N
    pose.e = pose.x*cos(deg2rad(pose_offset_placa.yaw)) - pose.y*sin(deg2rad(pose_offset_placa.yaw));
    pose.n = pose.x*sin(deg2rad(pose_offset_placa.yaw)) + pose.y*cos(deg2rad(pose_offset_placa.yaw));
    pose.u = pose.z;
    // Adicao da translacao do offset ENU (primeira mensagem da placa)
    pose.e += pose_offset_placa.e;
    pose.n += pose_offset_placa.n;
    pose.u += pose_offset_placa.u;
    // Diferencas entre pose atual e previous
    pose.dx = pose.e - pose_previous.e;
    pose.dy = pose.n - pose_previous.n;
    pose.dz = pose.u - pose_previous.u;
    // Angulos atuais RPY com offsets da primeira mensagem - todos em DEGREES
    pose.roll  = bound180(pose.roll  + pose_offset_placa.roll );
    pose.pitch = bound180(pose.pitch + pose_offset_placa.pitch);
    pose.yaw   = bound180(pose.yaw   + pose_offset_placa.yaw  );
    // Diferencas de angulos com a mensagem previous
    pose.droll  = bound180(pose.roll  - pose_previous.roll );
    pose.dpitch = bound180(pose.pitch - pose_previous.pitch);
    pose.dyaw   = bound180(pose.yaw   - pose_previous.yaw  );

    // Printar estado atual
    if(false){
      cout << "########### ZED\n";
      cout << "E: " << pose.e << "\tN: " << pose.n << "\tU: " << pose.u << endl;
      cout << "########### OFFSET\n";
      cout << "E: " << pose_offset_placa.e << "\tN: " << pose_offset_placa.n << "\tU: " << pose_offset_placa.u << "\tYaw: " << pose_offset_placa.yaw << endl << endl << endl << endl;
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
  void atualizar_nuvem(){
    // Preenchendo o ponto com dados atuais
    point.x        = pose.e;
    point.y        = pose.n;
    point.z        = pose.u;
    point.r = 250.0f; point.g = 0.0f; point.b = 0.0f; // VERMELHO porque nao esta tao bacana
    point.normal_x = pose.roll;
    point.normal_y = pose.pitch;
    point.normal_z = pose.yaw;
    // Adicionando a nuvem
    nuvem->push_back(point);
    cout << "\n Leste da ZED: " << point.x << endl;
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
  Pose_atual pose_previous;     // Somente a pose anterior, os deltas nao sao considerados, por mais que sejam atualizados
  Pose_atual pose;              // Tudo aqui e considerado para entrar no filtro
  Pose_atual pose_offset_placa; // Pose de offset com os dados iniciais da placa
  Pose_atual pose_offset_ZED;   // Pose de offset com os dados iniciais da ZED

  double decaimento_bussola; // Decaimento natural da bussola para correcao

  bool vamos_salvar_nuvem; // Vamos ou nao gravar a nuvem para ver o trajeto depois
  PointCloud<PointXYZRGBNormal>::Ptr nuvem; // Nuvem com o caminho, onde as normais sao os angulos
  PointXYZRGBNormal point; // Ponto atual para a nuvem
};
