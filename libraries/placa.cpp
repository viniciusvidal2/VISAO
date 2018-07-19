#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/thread/thread.hpp>
#include <iostream>
#include <string>
#include <numeric>

#include <ros/ros.h>

#include "pose.h"

using namespace pcl;
using namespace pcl::visualization;
using namespace std;

class Placa
{
public:
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////// Inicio ////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void init(){
    // Iniciar pose atual
    pose.x     = 0; pose.y      = 0; pose.z    = 0;
    pose.dx    = 0; pose.dy     = 0; pose.dz   = 0;
    pose.roll  = 0; pose.pitch  = 0; pose.yaw  = 0;
    pose.droll = 0; pose.dpitch = 0; pose.dyaw = 0;
    // Iniciar pose anterior
    pose_previous.x     = 0; pose_previous.y      = 0; pose_previous.z    = 0;
    pose_previous.dx    = 0; pose_previous.dy     = 0; pose_previous.dz   = 0;
    pose_previous.roll  = 0; pose_previous.pitch  = 0; pose_previous.yaw  = 0;
    pose_previous.droll = 0; pose_previous.dpitch = 0; pose_previous.dyaw = 0;
    // Por default nao queremos salvar a nuvem
    vamos_salvar_nuvem = false;
    // Inicio da nuvem
    nuvem = (pcl::PointCloud<PointXYZRGBNormal>::Ptr) new pcl::PointCloud<PointXYZRGBNormal>;
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////// SETS /////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void set_pose(Pose_atual _pose, int code){
    if(code == 0){ // Aqui atualizamos a pose previous
      pose_previous = _pose;
    } else if(code == 1){ // Aqui atualizamos a pose atual
      pose = _pose;
    }
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void set_salvar_nuvem(bool salvar){
    vamos_salvar_nuvem = salvar;
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////// Principal ///////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void process_and_return(Pose_atual &_pose){
    /// A pose atual ja foi atualizada no script principal, contem o que foi lido nesse instante em:
    /// X, Y, Z, ROLL, PITCH, YAW
    /// Calcular as diferencas abaixo considerando sentidos ja estabelecidos
    // Diferencas de posicao
    pose.dx = pose.x - pose_previous.x;
    pose.dy = pose.y - pose_previous.y;
    pose.dz = pose.z - pose_previous.z;
    // Diferencas de angulo [DEGREES] - CONSIDERAR WRAP180
    pose.droll  = wrap180(pose.roll , pose_previous.roll );
    pose.dpitch = wrap180(pose.pitch, pose_previous.pitch);
    pose.dyaw   = wrap180(pose.yaw  , pose_previous.yaw  );
    // Repassar resultado para o script principal
    _pose = pose;
    // Preparar para proxima iteracao: atual->previous
    pose_previous = pose;
    // Salvar a nuvem com a pose atual?
    if(vamos_salvar_nuvem)
      atualizar_nuvem();
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void visualizar_nuvem(){
    boost::shared_ptr<PCLVisualizer> vis_placa (new PCLVisualizer("caminho"));
    vis_placa->addPointCloud<PointXYZRGBNormal>(nuvem, "caminho");
    vis_placa->spin();
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void salvar_nuvem(string path){
    if(vamos_salvar_nuvem){
      ROS_INFO("Salvando o caminho...");
      // Ver o tempo para diferenciar bags gravadas automaticamente
      time_t t = time(0);
      struct tm * now = localtime( & t );
      std::string year, month, day, hour, minutes;
      year    = boost::lexical_cast<std::string>(now->tm_year + 1900);
      month   = boost::lexical_cast<std::string>(now->tm_mon );
      day     = boost::lexical_cast<std::string>(now->tm_mday);
      hour    = boost::lexical_cast<std::string>(now->tm_hour);
      minutes = boost::lexical_cast<std::string>(now->tm_min );
      string date = "_" + year + "_" + month + "_" + day + "_" + hour + "h_" + minutes + "m";
      string filename = "/home/vinicius/Desktop/"+path+date+".ply";
      // Salvando com o nome diferenciado
      io::savePLYFileASCII(filename, *nuvem);
      ROS_INFO("Salvo na area de trabalho");
      // Visualizar a nuvem com PCL
      visualizar_nuvem();
    } else {
      ROS_INFO("NAO SALVAMOS O CAMINHO PORQUE NAO ERA PRA SALVAR MANEZAO");
    }
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////

private:
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
    point.r = 250.0f; point.g = 250.0f; point.b = 250.0f;
    point.normal_x = pose.roll;
    point.normal_y = pose.pitch;
    point.normal_z = pose.yaw;
    // Adicionando a nuvem
    nuvem->push_back(point);
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// Variaveis privadas
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  Pose_atual pose_previous; // Somente a pose anterior, os deltas nao sao considerados, por mais que sejam atualizados
  Pose_atual pose;          // Tudo aqui e considerado para entrar no filtro

  bool vamos_salvar_nuvem; // Vamos ou nao gravar a nuvem para ver o trajeto depois
  PointCloud<PointXYZRGBNormal>::Ptr nuvem; // Nuvem com o caminho, onde as normais sao os angulos
  PointXYZRGBNormal point; // Ponto atual para a nuvem
};
