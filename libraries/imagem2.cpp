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
    double y; // [m]
    double z;
    double roll;
    double pitch; // [DEG]
    double yaw;
    double droll;
    double dpitch; // diferenca de angulos [DEG]
    double dyaw;
} ;

class Imagem
{
public:
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////// Inicio ////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    void init(){
        E = Mat::zeros(3, 3); F = Mat::zeros(3, 3);
        rt_previous = (Mat1d(3, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0); // [I | 0]
        rt_current  = (Mat1d(3, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0); // [I | 0]
        if(!camera_matrix.empty()){
            P_previous  = camera_matrix*rt_previous;
            P_current   = camera_matrix*rt_current;
        } else {
            cout << "\n\n ARRUME A MATRIZ DE CALIBRACAO PRIMEIRO MALUCO \n\n";
        }
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    void read_camera_calibration(string filename){
        FileStorage fs;
        fs.open(filename, FileStorage::READ);
        fs["camera_matrix"] >> camera_matrix;
        fs["distortion_coefficients"] >> dist_coef;
        fs["rectification_matrix"] >> rect;
        fs.release();
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    void undistort_image(){
        Mat temp; image_current.copyTo(temp);
        undistort(temp, image_current, camera_matrix, dist_coef);
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    float scale_factor(string path_previous, string path_current){
        Mat proj_previous, proj_current, Kl, Kr;
        FileStorage fl, fr;
        fl.open(path_previous , FileStorage::READ);
        fl["projection_matrix"] >> proj_previous;
        fl["camera_matrix"] >> Kl;
        fr.open(path_current, FileStorage::READ);
        fr["projection_matrix"] >> proj_current;
        fr["camera_matrix"] >> Kr;

        // Calcular matrizes [R|t]
        Mat rt_previous(3, 4, CV_64F), rt_current(3, 4, CV_64F);
        rt_previous  = Kl.inv()*proj_previous;
        rt_current = Kr.inv()*proj_current;
        // Retirar translacao dali, em modulo para nao dar erro
        vector<double> translation = { abs(rt_previous.at<double>(0, 3)-rt_current.at<double>(0, 3)),
                                       abs(rt_previous.at<double>(1, 3)-rt_current.at<double>(1, 3)),
                                       abs(rt_previous.at<double>(2, 3)-rt_current.at<double>(2, 3)) };
        // Garantir que estamos pegando o eixo que transladou mesmo
        scale_to_real_world = *max_element(translation.begin(), translation.end());

        return scale_to_real_world;
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////// Pipeline 2D ////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    void get_kpts_and_matches(int min_hessian, int min_matches){
        // Liberar todos antes de adicionar nessa iteracao
        keypoints_filt_current.clear(); keypoints_filt_previous.clear();
        better_matches.clear();
        // Variaveis locais temporarias para guardar informacoes de kpts e matches gerais
        FlannBasedMatcher matcher;
        //    BFMatcher matcher(NORM_HAMMING);
        vector<KeyPoint> keypoints_previous, keypoints_current;
        vector<DMatch> matches;
        // Loop para pegar o minimo de matches exigido
        while (better_matches.size() < min_matches){ // Obter o minimo possivel de matches, senao abaixa o threshold do descritor SURF
            // Detectando keypoints no geral
            Ptr<SURF> detector = SURF::create(min_hessian);
            //      Ptr<AKAZE> detector = AKAZE::create(AKAZE::DESCRIPTOR_MLDB, 0, 3, min_hessian, 4, 4, KAZE::DIFF_PM_G2);
            detector->detectAndCompute(image_previous, Mat(), keypoints_previous, descriptors_previous);
            detector->detectAndCompute(image_current,  Mat(), keypoints_current,  descriptors_current );
            // visualizar se ok
            if (visualizar){
                Mat image_kpts_previous, image_kpts_current;
                drawKeypoints( image_previous , keypoints_previous , image_kpts_previous , Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
                drawKeypoints( image_current, keypoints_current, image_kpts_current, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
                namedWindow("Keypoints previous" , WINDOW_NORMAL);
                namedWindow("Keypoints current", WINDOW_NORMAL);
                imshow("Keypoints previous" , image_kpts_previous );
                imshow("Keypoints current", image_kpts_current );
                waitKey(1);
            }
            // Fazer match de features no geral
            if (!descriptors_previous.empty() && !descriptors_current.empty())
                matcher.match(descriptors_previous, descriptors_current, matches);
            // Limpar correspondencias tendo nocao da distancia dos keypoints na imagem
            float min_dist = 1000000, max_dist = 0;
            for( int i = 0; i < matches.size(); i++ ){
                double dist = matches[i].distance;
                if( dist < min_dist ) min_dist = dist;
                if( dist > max_dist ) max_dist = dist;
            }
            float thresh_dist = 3*min_dist; // Threshold para acabar com matches ruins
            for(int i = 0; i < matches.size(); i++){
                if (matches[i].distance < thresh_dist){ // Aqui crio matches novo e vetor dos pontos dos keypoints ORGANIZADOS,
                    better_matches.push_back(matches[i]); // porem daqui em diante apago os que nao forem bons
                    keypoints_filt_previous.push_back(keypoints_previous[matches[i].queryIdx].pt);
                    keypoints_filt_current.push_back(keypoints_current[matches[i].trainIdx].pt);
                }
            }
            // Filtrando por bins
            filter_bins();
            // Filtrando por coeficiente angular
            filter_lines(0.5); // Quantos desvios padroes vai ser o limite

            ROS_INFO("Quantos kpts do fim das contas %d %d", keypoints_filt_previous.size(), keypoints_filt_current.size());
            // Limpando para proxima iteracao, se existir
            if(better_matches.size() < min_matches){
                min_hessian = 0.7*min_hessian;
                matches.clear();
                keypoints_previous.clear();
                keypoints_current.clear();
                descriptors_previous.release();
                descriptors_current.release();
                better_matches.clear();
                keypoints_filt_previous.clear();
                keypoints_filt_current.clear();
            }

        } // Fim do while

        if (visualizar){
            Mat img_matches, img_matches_disp;
            drawMatches( image_previous, keypoints_previous, image_current, keypoints_current, better_matches, img_matches,
                         Scalar::all(-1), Scalar::all(-1),
                         vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
            resize(img_matches, img_matches_disp, Size(img_matches.cols/4, img_matches.rows/4));
            namedWindow("All matches", WINDOW_GUI_EXPANDED);
            imshow("All Matches", img_matches_disp);
            waitKey(0);
        }
    } // Fim do void
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    void filter_bins(){
        // Keypoints ja correspondem um a um nesse estagio.
        float lim_x_left, lim_x_right, lim_y_down, lim_y_up;
        for(int i=0; i<keypoints_filt_previous.size(); i++){
            // Guardar coordenadas dos pontos
            float xp = keypoints_filt_previous[i].x, yp = keypoints_filt_previous[i].y;
            float xc = keypoints_filt_current[i].x , yc = keypoints_filt_current[i].y;
            // Criar um bin em torno do ponto na imagem anterior (no caso da esquerda) atentando aos limites da imagem.
            lim_x_left  = (xp - w/2) >= 0                   ? xp - w/2 : 0;
            lim_x_right = (xp + w/2) <  image_previous.cols ? xp + w/2 : image_previous.cols-1;
            lim_y_up    = (yp - h/2) >= 0                   ? yp - h/2 : 0;
            lim_y_down  = (yp + h/2) <  image_previous.rows ? yp + h/2 : image_previous.rows-1;
            // Confeir se as coordenadas do ponto da imagem atual (ou da direita) estao dentro do bin
            if( !(lim_x_left < xc && xc < lim_x_right && lim_y_up < yc && yc < lim_y_down) ){ // Se nao estao, apaga
                keypoints_filt_previous.erase(keypoints_filt_previous.begin()+i);
                keypoints_filt_current.erase(keypoints_filt_current.begin()+i);
                better_matches.erase(better_matches.begin()+i);
                i = i - 1; // Para nao pular um elemento
            }
        } // Fim do for
    } // Fim do void
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    void filter_lines(float ndevs){
        // Keypoints ja correspondem um a um nesse estagio.
        double mean_coef, sum_coef = 0, stdev_coef;
        vector<float> coefs(keypoints_filt_previous.size());
        for(int i=0; i<keypoints_filt_previous.size(); i++){
            // Guardar coordenadas dos pontos
            float xl = keypoints_filt_previous[i].x                     , yl = keypoints_filt_previous[i].y;
            float xr = keypoints_filt_current[i].x + image_previous.cols, yr = keypoints_filt_current[i].y; // Como se a segunda imagem estivesse a direita
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
        for(int i=0; i<keypoints_filt_previous.size(); i++){
            if( (coefs[i] < (mean_coef-stdev_coef*ndevs)) || (coefs[i] > (mean_coef+stdev_coef*ndevs)) ){ // Se fora dos limites
                keypoints_filt_previous.erase(keypoints_filt_previous.begin()+i);
                keypoints_filt_current.erase(keypoints_filt_current.begin()+i);
                better_matches.erase(better_matches.begin()+i);
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
    /////////////////////////////////////////////// SETS /////////////////////////////////////////////////////
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
    void set_image_previous(Mat image){
        image.copyTo(image_previous);
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    void set_image_current(Mat image){
        image.copyTo(image_current);
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    void set_pose(Pose _pose){
        pose = _pose;
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    void set_rt_and_projection_matrix(int code){
        if(code == 0){ // Aqui para a previous
            rt_current.copyTo(rt_previous);
            P_current.copyTo(P_previous);
        } else if(code == 1){ // Aqui para a current
            hconcat(R, t, rt_current); // Matriz de parametros extrinsecos 2
            P_current = camera_matrix*rt_current;
        }
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////// Update ///////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    void update_pose(){
        // Posicao e acumulada!! -- FALTA AJUSTAR OS FRAMES
        pose.x += scale_to_real_world*t.at<double>(0, 0);
        pose.y += scale_to_real_world*t.at<double>(1, 0); // [m]
        pose.z += scale_to_real_world*t.at<double>(2, 0);
        // Diferencas entre angulos -- FALTA AJUSTAR OS FRAMES
        pose.droll  = rad2deg(rod.at<double>(0, 0));
        pose.dpitch = rad2deg(rod.at<double>(1, 0)); // [DEG]
        pose.dyaw   = rad2deg(rod.at<double>(2, 0));
        // Angulos atuais acumulados - somar com os anteriores
        pose.roll  += pose.droll;
        pose.pitch += pose.dpitch; // [DEG]
        pose.yaw   += pose.dyaw;
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////// PRINCIPAL /////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    void pipeline(){
        // Calcular keypoints, descriptors, fazer matches e filtra-las
        get_kpts_and_matches(2000, 20);
        // Calculo da matriz fundamental F
        F = findFundamentalMat(keypoints_filt_previous, keypoints_filt_current);
        // Calculo da matriz essencial E
        E = camera_matrix.t() * F * camera_matrix;
        //  Mat E = findEssentialMat(keypoints_filt_left, keypoints_filt_right, camera_matrix, RANSAC, 0.999, 1e-4);
        // Obtencao de rotacao e translacao
        inliers = recoverPose(E, keypoints_filt_previous, keypoints_filt_current, camera_matrix, R, t);
        cout << "Inliers:  "             << inliers << " de " << keypoints_filt_previous.size() << endl;
        // Ajuste de matrizes de projecao e parametros extrinsecos
        set_rt_and_projection_matrix(0); // 0 corresponde a previous
        set_rt_and_projection_matrix(1); // 1 corresponde a current
        // Angulos de rotacao
        Rodrigues(R, rod);
        // Update das variaveis atuais
        update_pose();
        // Ajuste para as proximas iteracoes (current -> previous)
        image_current.copyTo(image_previous);
        F.release(); E.release();
        keypoints_filt_previous.clear(); keypoints_filt_previous = keypoints_filt_current;
        descriptors_previous.release(); descriptors_current.copyTo(descriptors_previous);
        better_matches.clear();

    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
    bool visualizar;
    Mat camera_matrix, dist_coef, rect; // Vindos do arquivo de calibracao
    Mat image_previous, image_current; // Imagem anterior e atual para comparar
    PointCloud<PointT>::Ptr cloud;

    vector<Point2f> keypoints_filt_previous, keypoints_filt_current; // Keypoints filtrados
    Mat descriptors_previous, descriptors_current; // Descritores calculados no momento
    vector<DMatch>  better_matches; // Melhores matches do momento
    Mat E, F; // Matrizes essencial e fundamental

    Pose pose; // Pose total da camera
    Mat R, t, rod; // Matrizes para calculo de rotacao e translacao temporarias
    int inliers; // Quantos inliers apos descobrir pose atual
    Mat rt_previous, rt_current, P_previous, P_current; // Matrizes de parametros extrinsecos e matriz de projecao
    double roll_current, pitch_current, yaw_current; // Angulos atuais [DEG]
    double tx, ty, tz; // Translacoes atuais [m]

    int M; // Quantos quadrados no eixo  X (columns)
    int N; // Quantos quadrados no eixo -Y (rows)
    int w; // Largura dos quadrados no eixo  X
    int h; // Altura dos quadrados no eixo  -Y
    float scale_to_real_world; // Escala da visao monocular para o mundo real
};
