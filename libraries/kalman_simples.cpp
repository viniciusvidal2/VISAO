#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl;
using namespace pcl::visualization;

class Kalman_simples{

public:
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void init(double error_measurement, double measurement, double error_estimate, double estimate){
    /// Erros de estimativa e de medicao
    P_kp = error_estimate;
    P_k  = error_estimate;
    R    = error_measurement;
    /// Primeira estimativa com inicio do sensor - offset
    X_kp = estimate;
    Y    = measurement;

    // Inicio da nuvem
    nuvem = (pcl::PointCloud<PointXYZRGBNormal>::Ptr) new pcl::PointCloud<PointXYZRGBNormal>;
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////// GETS ///////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  double get_estimate(){
    return X_k;
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
  /////////////////////////////////////////// PRINCIPAL ////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  double filter(double estimate_update, double measurement, double error_measurement){
    /// Atualiza os estados do processo Xp do filtro com a leitura vinda da imagem(modelo)
    update_process_state(estimate_update);
    /// Calculo do ganho de Kalman com o que temos de covariancias
    calculate_KG(error_measurement);
    /// Atualiza estimativa frente ao novo ganho de Kalman e a nova leitura do sensor
    update_estimate(measurement);
    /// Nova covariancia da estimativa
    update_estimate_covariance();

    // Salvar na nuvem do caminho
    atualizar_nuvem();

    // Printar na tela o debug?
    if(vamos_debugar)
      print_debug();

    // Retorna o valor da estimativa
    return X_k;
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
  void atualizar_nuvem(){
    // Preenchendo o ponto com dados atuais
    point.x        = X_k;
    point.y        = 0;
    point.z        = 0;
    point.r = 0.0f; point.g = 250.0f; point.b = 250.0f;
    point.normal_x = 0;
    point.normal_y = 0;
    point.normal_z = 0;
    // Adicionando a nuvem
    nuvem->push_back(point);
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void visualizar_nuvem(){
    boost::shared_ptr<PCLVisualizer> vis_placa (new PCLVisualizer("caminho Kalman simples"));
    vis_placa->addPointCloud<PointXYZRGBNormal>(nuvem, "caminho Kalman simples");
    vis_placa->spin();
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void update_process_state(double est_up){
    // NOTA: a matriz de covariancia do processo nao e alterada aqui, somente na atulaizacao vinda da
    // iteracao anterior
    P_kp   = P_k;
    deltaX = est_up;
    // Pega o estimado anteriormente e coloca no estado do processo
    X_kp   = X_k;
    X_kp  += deltaX;
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void calculate_KG(double error_meas){
    // Anotando o erro do gps aqui que muda com o tempo
    R = error_meas;
    // Ganho de Kalman voando aqui
    KG  = P_kp/(P_kp + R);
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void update_estimate(double meas){
    Y = meas; // Medido
    X_k = X_kp + KG*( Y - X_kp ); // Nova estimativa / estado do sistema
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  void update_estimate_covariance(){
    P_k = (1 - KG)*P_kp;
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
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// Variaveis privadas - Retiradas de ILectureOnline no youtube
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  double KG, X_kp, X_k, deltaX, P_kp, P_k, R, Y;
  bool vamos_salvar_nuvem;
  bool vamos_debugar;

  PointCloud<PointXYZRGBNormal>::Ptr nuvem;
  PointXYZRGBNormal point;

};
