#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/susan.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_validation.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/pfh.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/common/angles.h>
#include <math.h>

#include <queue>
#include <vector>
#include <map>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::registration;
using namespace pcl::visualization;
using namespace Eigen;

//Definitions
typedef pcl::PointXYZRGB PointT;

boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_fim (new pcl::visualization::PCLVisualizer ("matched"));
boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_acc (new pcl::visualization::PCLVisualizer ("accumulated"));

float lf = 0.2f; // Leaf size for voxel grid
ros::Publisher pub;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool find_element(std::vector<int> v, int element){
  // Returns true if element is found
  bool is_there = false;
  for(std::vector<int>::iterator p=v.begin(); p!=v.end(); ++p){
    if(*p == element)
      is_there = true;
  }

  return is_there;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void color_cloud(PointCloud<PointT>::Ptr cloud){
  for(int i=0; i < cloud->points.size(); i++){
    cloud->points[i].r = 250.0f;
    cloud->points[i].g = 250.0f;
    cloud->points[i].b = 250.0f;
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void passthrough(pcl::PointCloud<PointT>::Ptr in, std::string field, float min, float max){
  pcl::PassThrough<PointT> ps;
  ps.setInputCloud(in);
  ps.setFilterFieldName(field);
  ps.setFilterLimits(min, max);

  ps.filter(*in);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void remove_outlier(PointCloud<PointT>::Ptr in, float mean, float deviation){
  StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(in);
  sor.setMeanK(mean);
  sor.setStddevMulThresh(deviation);
  sor.filter(*in);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float mean_points_distance(PointCloud<PointT>::Ptr inputCloud){
  const int totalcount = inputCloud->width * inputCloud->height;
  std::vector<float> EuclidianDistance(totalcount);
  float totalDistance = 0;

  search::KdTree<PointT>::Ptr kdtree(new search::KdTree<PointT>());
  kdtree->setInputCloud(inputCloud);

  const int K = 2; //first will be the distance with point it self and second will the nearest point that's why "2"
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  for (int i = 0; i < totalcount; ++i)  {
    if ( kdtree->nearestKSearch (inputCloud->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
      for (size_t j = 0; j < pointIdxNKNSearch.size (); ++j) {
        //saving all the distance in Vector
        EuclidianDistance[i] =  pointNKNSquaredDistance[j];
      }
    }
  }

  for(int i = 0; i < totalcount; i++) {
    //accumulating all distances
    totalDistance = totalDistance + EuclidianDistance[i];
  }

  //calculating the mean distance
  float meanDistance = totalDistance/totalcount;
  ROS_INFO("A distancia media dos pontos na nuvem: %.5f", meanDistance);
  //freeing the allocated memory
  EuclidianDistance.clear();

  return meanDistance;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_normals(pcl::PointCloud<PointT>::Ptr cloud_in, int k, float radius, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals){

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_ptr (new pcl::PointCloud<pcl::PointXYZ>()); // Gather only pose so can compute normals
  cloud_xyz_ptr->resize(cloud_in->points.size()); // allocate memory space
  ROS_INFO("Size of cloud to calculate normals: %d\tHow many neighbors: %d", cloud_in->points.size(), k);
  for(int i=0; i < cloud_in->points.size(); i++){
    cloud_xyz_ptr->points[i].x = cloud_in->points[i].x;
    cloud_xyz_ptr->points[i].y = cloud_in->points[i].y;
    cloud_xyz_ptr->points[i].z = cloud_in->points[i].z;
  }
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n(new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setInputCloud(cloud_xyz_ptr);
  ne.setSearchMethod(tree_n);
  ne.setKSearch(k);
//  ne.setRadiusSearch(radius);
  ne.compute(*cloud_normals);

//  std::vector<int> aux_indices;
//  removeNaNNormalsFromPointCloud(*cloud_normals, *cloud_normals, aux_indices);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void estimateKeypoints (const PointCloud<PointT>::Ptr src,
                        const PointCloud<PointT>::Ptr tgt,
                        const PointCloud<Normal>::Ptr src_normal,
                        const PointCloud<Normal>::Ptr tgt_normal,
                        PointCloud<PointXYZI>::Ptr keypoints_src,
                        PointCloud<PointXYZI>::Ptr keypoints_tgt,
                        PointIndicesConstPtr src_ind,
                        PointIndicesConstPtr tgt_ind,
                        int k,
                        float radius_src,
                        float radius_tgt,
                        int method)
{
  HarrisKeypoint3D<PointXYZRGB, PointXYZI> harris;
  double iss_salient_radius_ = 6 * lf;
  double iss_non_max_radius_ = 4 * lf;
  double iss_gamma_21_ (0.975);
  double iss_gamma_32_ (0.975);
  double iss_min_neighbors_ (5);
  int iss_threads_ (4);
  ISSKeypoint3D<PointT, PointXYZI> iss;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  switch (method){
  case 1: // Harris
    harris.setNumberOfThreads(8);
    harris.setRefine(false);
    harris.setThreshold(1e-9);

    harris.setRadius(radius_src);
    harris.setInputCloud(src);
    harris.setNormals(src_normal);
    harris.compute(*keypoints_src);
    ROS_INFO("Quantos keypoints source: %d", keypoints_src->size());
    harris.setRadius(radius_tgt);
    harris.setInputCloud(tgt);
    harris.setNormals(tgt_normal);
    harris.compute(*keypoints_tgt);
    ROS_INFO("Quantos keypoints target: %d", keypoints_tgt->size());
    //  ROS_INFO("Quantos indices: %d", tgt_ind->indices.size());
    break;
  case 2: // ISS3D
    iss.setSearchMethod(tree);
    iss.setSalientRadius (iss_salient_radius_);
    iss.setNonMaxRadius (iss_non_max_radius_);
    iss.setThreshold21 (iss_gamma_21_);
    iss.setThreshold32 (iss_gamma_32_);
    iss.setMinNeighbors (iss_min_neighbors_);
    iss.setNumberOfThreads (iss_threads_);    
    iss.setInputCloud (src);
    iss.compute (*keypoints_src);
    ROS_INFO("Quantos keypoints source: %d", keypoints_src->size());
    iss.setInputCloud (tgt);
    iss.compute (*keypoints_tgt);
    ROS_INFO("Quantos keypoints target: %d", keypoints_tgt->size());
    break;
  case 3: // SUSAN

    break;

  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void estimatePFH (const PointCloud<PointT>::Ptr src,
                   const PointCloud<PointT>::Ptr tgt,
                   const PointCloud<Normal>::Ptr normals_src,
                   const PointCloud<Normal>::Ptr normals_tgt,
                   const PointCloud<PointXYZI>::Ptr keypoints_src,
                   const PointCloud<PointXYZI>::Ptr keypoints_tgt,
                   PointCloud<PFHSignature125>::Ptr pfhs_src,
                   PointCloud<PFHSignature125>::Ptr pfhs_tgt,
                   int k,
                   float radius_src,
                   float radius_tgt)
{
  PFHEstimation<PointXYZI, Normal, PFHSignature125> pfh_est;
  pcl::search::KdTree<PointXYZI>::Ptr tree (new pcl::search::KdTree<PointXYZI>);
  pfh_est.setSearchMethod(tree);

  pfh_est.setInputCloud(keypoints_src);
  pfh_est.setInputNormals(normals_src);
  PointCloud<PointXYZI>::Ptr src_xyzi (new PointCloud<PointXYZI> ());
  PointCloudXYZRGBtoXYZI(*src, *src_xyzi);
  pfh_est.setSearchSurface(src_xyzi);
  pfh_est.setRadiusSearch(radius_src);
  pfh_est.compute(*pfhs_src);
  ROS_INFO("Tamanho das features source: %d", pfhs_src->size());
  pfh_est.setInputCloud(keypoints_tgt);
  pfh_est.setInputNormals(normals_tgt);
  PointCloud<PointXYZI>::Ptr tgt_xyzi (new PointCloud<PointXYZI> ());
  PointCloudXYZRGBtoXYZI(*tgt, *tgt_xyzi);
  pfh_est.setSearchSurface(tgt_xyzi);
  pfh_est.setRadiusSearch(radius_tgt);
  pfh_est.compute(*pfhs_tgt);
  ROS_INFO("Tamanho das features target: %d", pfhs_tgt->size());
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void estimateFPFH (const PointCloud<PointT>::Ptr src,
                   const PointCloud<PointT>::Ptr tgt,
                   const PointCloud<Normal>::Ptr normals_src,
                   const PointCloud<Normal>::Ptr normals_tgt,
                   const PointCloud<PointXYZI>::Ptr keypoints_src,
                   const PointCloud<PointXYZI>::Ptr keypoints_tgt,
                   PointCloud<FPFHSignature33>::Ptr fpfhs_src,
                   PointCloud<FPFHSignature33>::Ptr fpfhs_tgt,
                   int k,
                   float radius_src,
                   float radius_tgt)
{
  FPFHEstimation<PointXYZI, Normal, FPFHSignature33> fpfh_est;
  pcl::search::KdTree<PointXYZI>::Ptr tree (new pcl::search::KdTree<PointXYZI>);
  fpfh_est.setSearchMethod(tree);

  fpfh_est.setInputCloud(keypoints_src);
  fpfh_est.setInputNormals(normals_src);
  PointCloud<PointXYZI>::Ptr src_xyzi (new PointCloud<PointXYZI> ());
  PointCloudXYZRGBtoXYZI(*src, *src_xyzi);
  fpfh_est.setSearchSurface(src_xyzi);
  fpfh_est.setRadiusSearch(radius_src);
  fpfh_est.compute(*fpfhs_src);
  ROS_INFO("Tamanho das features source: %d", fpfhs_src->size());
  fpfh_est.setInputCloud(keypoints_tgt);
  fpfh_est.setInputNormals(normals_tgt);
  PointCloud<PointXYZI>::Ptr tgt_xyzi (new PointCloud<PointXYZI> ());
  PointCloudXYZRGBtoXYZI(*tgt, *tgt_xyzi);
  fpfh_est.setSearchSurface(tgt_xyzi);
  fpfh_est.setRadiusSearch(radius_tgt);
  fpfh_est.compute(*fpfhs_tgt);
  ROS_INFO("Tamanho das features target: %d", fpfhs_tgt->size());
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void findCorrespondences (const PointCloud<FPFHSignature33>::Ptr fpfhs_src,
                          const PointCloud<FPFHSignature33>::Ptr fpfhs_tgt,
                          CorrespondencesPtr all_correspondences)
{
  CorrespondenceEstimation<FPFHSignature33, FPFHSignature33> est;

  est.setInputSource(fpfhs_src);
  est.setInputTarget(fpfhs_tgt);
//  est.determineCorrespondences(*all_correspondences);
  est.determineReciprocalCorrespondences(*all_correspondences);
  ROS_INFO("Quantas correspondencias: %d", all_correspondences->size());
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float mean_correspondence_distance(CorrespondencesPtr correspondences){
  float sum = 0, mean;

  for (int i=0; i < correspondences->size(); i++){
    sum += correspondences->data()[i].distance;
  }
  mean = sum/correspondences->size();
  ROS_INFO("A distancia media entre as correspondencias: %.5f", mean);

  return mean;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void remove_correspondences_bins_distance(CorrespondencesPtr correspondences, const int bins, const int bests){
  float sum = 0, mean;
  // Gathering data from distances and organizing
  std::vector<float> distances (correspondences->size());
  std::vector<float>::iterator min, max;
  for (int i=0; i < correspondences->size(); i++){
    sum += correspondences->data()[i].distance;
    distances[i] = correspondences->data()[i].distance;
  }
  max = std::max_element(distances.begin(), distances.end());
  min = std::min_element(distances.begin(), distances.end());
  mean = sum/correspondences->size();
  ROS_INFO("A distancia media entre as correspondencias: %.5f\tMinimo: %.5f\tMaximo: %.5f", mean, *min, *max);

  // Dividing into bins the whole data
  float binsize = (*max - *min)/bins;
  std::vector<int> bin_count(bins);
  std::vector<int> labels(distances.size());
  for (int i=0; i<distances.size(); i++){
    float dist_comp = *min;
    int bin_fall = -1; // So the algorithm flows correctly
    while (distances[i] >= dist_comp){
      dist_comp += binsize;
      bin_fall += 1;
    } // If we leave this loop we reached the condition and have to place the data in the correct bin, which is bin_fall
    bin_fall = (bin_fall < bins) ? bin_fall : bin_fall-1; // So the largest distance dont get lost
    labels[i] = bin_fall;
    bin_count[bin_fall] += 1;
  }
  std::cout << "\nOs bins estao como abaixo: \n";
  for (std::vector<int>::const_iterator i=bin_count.begin(); i!=bin_count.end(); ++i)
    std::cout << *i << " ";
  std::cout << std::endl;

  // Choose the 'bests' bins among all
  std::vector<int> labels_filtered(bests); // Will gather the best bin labels
  std::priority_queue< std::pair<double, int> > organize_bins;
  for (int i = 0; i < bin_count.size(); i++) {
    organize_bins.push(std::pair<double, int>(bin_count[i], i));
  }
  int k = bests; // number of indices we need
  for (int i = 0; i < k; ++i) {
    int ki = organize_bins.top().second;
    labels_filtered[i] = ki; // Save the best label so far
    organize_bins.pop(); // Remove to next iteration
  }
  std::cout << "\nOs melhores bins sao: \n";
  for (std::vector<int>::const_iterator i=labels_filtered.begin(); i!=labels_filtered.end(); ++i)
    std::cout << *i << " ";
  std::cout << std::endl;

  // Check if correspondence is good via label marked, comparing with best bins labels
  Correspondences::iterator it = correspondences->begin();
  ROS_INFO("Quantas correspondencias tinham: %d", correspondences->size());

  for (std::vector<int>::iterator itl=labels.begin(); itl!=labels.end(); ++itl){
    if ( !find_element(labels_filtered, (int)(*itl)) ){ // The label is among the bests -> NEGATED
      correspondences->erase(it);
    } else {
      it++;
    }
    if (it == correspondences->end())
      break;
  }
  ROS_INFO("Quantas correspondencias sobraram: %d", correspondences->size());
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void remove_correspondences_lines(PointCloud<PointXYZI>::Ptr src_kp, PointCloud<PointXYZI>::Ptr tgt_kp, CorrespondencesPtr correspondences, float ndev){
  vector<double> ang_coefs_x(correspondences->size()), ang_coefs_y(correspondences->size()), ang_coefs_z(correspondences->size());
  double dx, dy, dz, norm;

  for(int i=0; i<correspondences->size(); i++){ // The unitary vectors between correspondences are calculated
    dx = tgt_kp->points[correspondences->data()[i].index_match].x - src_kp->points[correspondences->data()[i].index_query].x;
    dy = tgt_kp->points[correspondences->data()[i].index_match].y - src_kp->points[correspondences->data()[i].index_query].y;
    dz = tgt_kp->points[correspondences->data()[i].index_match].z - src_kp->points[correspondences->data()[i].index_query].z;
    norm = sqrt(dx*dx + dy*dy + dz*dz);
    dx = dx/norm; dy = dy/norm; dz = dz/norm;
    ang_coefs_x[i] = dx; ang_coefs_y[i] = dy; ang_coefs_z[i] = dz;
  }

  double stdev_x, stdev_y, stdev_z, mean_coef_x, mean_coef_y, mean_coef_z, sum_coef = 0;
  /// Mean and standard coef. for X
  mean_coef_x = accumulate(ang_coefs_x.begin(), ang_coefs_x.end(), 0.0)/ang_coefs_x.size();
  for(vector<double>::iterator it=ang_coefs_x.begin(); it!=ang_coefs_x.end(); it++)
      sum_coef += (*it - mean_coef_x) * (*it - mean_coef_x);
  stdev_x = sqrt( sum_coef/(ang_coefs_x.size()-1) );
  /// Mean and standard coef. for Y
  sum_coef = 0;
  mean_coef_y = accumulate(ang_coefs_y.begin(), ang_coefs_y.end(), 0.0)/ang_coefs_y.size();
  for(vector<double>::iterator it=ang_coefs_y.begin(); it!=ang_coefs_y.end(); it++)
      sum_coef += (*it - mean_coef_y) * (*it - mean_coef_y);
  stdev_y = sqrt( sum_coef/(ang_coefs_y.size()-1) );
  /// Mean and standard coef. for Z
  sum_coef = 0;
  mean_coef_z = accumulate(ang_coefs_z.begin(), ang_coefs_z.end(), 0.0)/ang_coefs_z.size();
  for(vector<double>::iterator it=ang_coefs_z.begin(); it!=ang_coefs_z.end(); it++)
      sum_coef += (*it - mean_coef_z) * (*it - mean_coef_z);
  stdev_z = sqrt( sum_coef/(ang_coefs_z.size()-1) );
  // Remove correspondence if one of the coeficients is not within the limits of ndev deviations
  ROS_INFO("Medias: %.4f\t%.4f\t%.4f", mean_coef_x, mean_coef_y, mean_coef_z);
  ROS_INFO("STDEV : %.4f\t%.4f\t%.4f", stdev_x    , stdev_y    , stdev_z    );
//  ROS_INFO("Minimo X: %.4f\tMaximo X: %.4f", *min_element(ang_coefs_x.begin(), ang_coefs_x.end()), *max_element(ang_coefs_x.begin(), ang_coefs_x.end()));
//  ROS_INFO("Minimo Y: %.4f\tMaximo Y: %.4f", *min_element(ang_coefs_y.begin(), ang_coefs_y.end()), *max_element(ang_coefs_y.begin(), ang_coefs_y.end()));
//  ROS_INFO("Minimo Z: %.4f\tMaximo Z: %.4f", *min_element(ang_coefs_z.begin(), ang_coefs_z.end()), *max_element(ang_coefs_z.begin(), ang_coefs_z.end()));
  ROS_INFO("Correspondencias antes: %d", correspondences->size());
  Correspondences::iterator it = correspondences->begin();
  for(int i=0; i<ang_coefs_x.size(); i++){
    // If any of the values is outside the limits, remove the correspondence
    if( ( (ang_coefs_x[i] < (mean_coef_x-stdev_x*ndev)) || (ang_coefs_x[i] > (mean_coef_x+stdev_x*ndev)) ) ||
        ( (ang_coefs_y[i] < (mean_coef_y-stdev_y*ndev)) || (ang_coefs_y[i] > (mean_coef_y+stdev_y*ndev)) ) ||
        ( (ang_coefs_z[i] < (mean_coef_z-stdev_z*ndev)) || (ang_coefs_z[i] > (mean_coef_z+stdev_z*ndev)) )    ){
      correspondences->erase(it);
    } else {
//      ROS_INFO("Minimo X: %.4f\tX: %.4f\tMaximo X: %.4f", mean_coef_x-stdev_x*ndev, ang_coefs_x[i], mean_coef_x+stdev_x*ndev);
//      ROS_INFO("Minimo Y: %.4f\tY: %.4f\tMaximo Y: %.4f", mean_coef_y-stdev_y*ndev, ang_coefs_y[i], mean_coef_y+stdev_y*ndev);
//      ROS_INFO("Minimo Z: %.4f\tZ: %.4f\tMaximo Z: %.4f", mean_coef_z-stdev_z*ndev, ang_coefs_z[i], mean_coef_z+stdev_z*ndev);
//      ROS_INFO("NORM: %.2f", sqrt(ang_coefs_x[i]*ang_coefs_x[i] + ang_coefs_y[i]*ang_coefs_y[i] + ang_coefs_z[i]*ang_coefs_z[i]));
//      cout << endl;
      it++;
    }
    // Security
    if(it == correspondences->end())
      break;
  } // Fim do for
  ROS_INFO("Correspondencias depois: %d\tQuantos desvios: %.2f", correspondences->size(), ndev);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void check_on_weights(CorrespondencesPtr correspondences, int k_bests){
  if (correspondences->size() < k_bests) // Here there are less correspondences than what is required, break the function
    return;
  // Get the weight values from the correspondences
  std::priority_queue< std::pair<double, int> > weight_map;
  std::vector<int> index_list_correspondences_raw(correspondences->size());
  Correspondences::iterator it;
  int pos = 0;
  for (it=correspondences->begin(); it !=correspondences->end(); it++){
    weight_map.push(std::pair<double, int>(it->weight, pos));
    index_list_correspondences_raw[pos] = pos;
    std::cout << "Peso na posicao " << pos <<": " << it->weight << std::endl;
    pos++;
  }
  // Separate the k_bests
  std::vector<int> best_indices(k_bests);
  for (int i = 0; i < k_bests; ++i) {
    int ki = weight_map.top().second; // We dont care about the weight, just its index
    std::cout << "index[" << i << "] = " << ki << "\tValor do peso: " << weight_map.top().first << std::endl;
    best_indices[i] = ki; // Save the best index so far
    weight_map.pop(); // Remove to next iteration
  }
//  std::vector<int>::iterator ite = best_indices.begin();
//  best_indices.erase(ite);
//  ite = best_indices.begin()+4;
//  best_indices.erase(ite);
//  std::cout << "Como esta o vetor de pesos: \n";
//  for (std::vector<int>::const_iterator i=best_indices.begin(); i!=best_indices.end(); ++i)
//    std::cout << *i << " ";
//  std::cout << std::endl;
  // Check if correspondence is good via label marked, comparing with best bins labels
//  std::cout << "Como estao as correspondencias: \n";
//  for (it=correspondences->begin(); it!=correspondences->end(); ++it)
//    std::cout << it->weight << " ";
//  std::cout << std::endl;
//  int search = 0;
  ROS_INFO("Quantas correspondencias tinham: %d", correspondences->size());
  it = correspondences->begin();
  for (std::vector<int>::iterator itl=index_list_correspondences_raw.begin(); itl!=index_list_correspondences_raw.end(); ++itl){
    if ( !find_element(best_indices, (int)(*itl)) ){ // The label is among the bests -> NEGATED
      correspondences->erase(it);
    } else {
      it++;
    }
    if (it == correspondences->end())
      break;
  }
  ROS_INFO("Quantas correspondencias sobraram: %d", correspondences->size());
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void rejectBadCorrespondences (const CorrespondencesPtr all_correspondences,
                               const PointCloud<PointXYZI>::Ptr keypoints_src,
                               const PointCloud<PointXYZI>::Ptr keypoints_tgt,
                               CorrespondencesPtr remaining_correspondences)
{
  if(all_correspondences->size() > 10){
//    check_on_weights(all_correspondences, 10);
    CorrespondenceRejectorDistance rej;
    rej.setInputSource<PointXYZI>(keypoints_src);
    rej.setInputTarget<PointXYZI>(keypoints_tgt);
    float m = mean_correspondence_distance(all_correspondences);
    rej.setMaximumDistance(m/2);
    ROS_INFO("Cortando em %.2f", m/2);
    rej.setInputCorrespondences(all_correspondences);
    rej.getCorrespondences(*remaining_correspondences);
    ROS_INFO("Number of correspondences remaining after rejection distance: %d\n", remaining_correspondences->size ());
    remove_correspondences_bins_distance(remaining_correspondences, 12, 2);
    remove_correspondences_lines(keypoints_src, keypoints_tgt, remaining_correspondences, 1);

//    *remaining_correspondences = *all_correspondences;
    pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZI> corr_rej_sac;
    corr_rej_sac.setInputSource(keypoints_src);
    corr_rej_sac.setInputTarget(keypoints_tgt);
    corr_rej_sac.setInlierThreshold(1e-9);
    corr_rej_sac.setMaximumIterations(1000);
    corr_rej_sac.setInputCorrespondences(remaining_correspondences);
    corr_rej_sac.getCorrespondences(*remaining_correspondences);
    ROS_INFO("Number of correspondences remaining after rejection ransac: %d\n", remaining_correspondences->size ());
  } else {
    *remaining_correspondences = *all_correspondences;
    ROS_INFO("Number of correspondences no rejection: %d\n", remaining_correspondences->size ());
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void computeTransformation (const PointCloud<PointXYZI>::Ptr keypoints_src,
                            const PointCloud<PointXYZI>::Ptr keypoints_tgt,
                            const CorrespondencesPtr good_correspondences,
                            Eigen::Matrix4f &transf)
{
  TransformationEstimationSVD<PointXYZI, PointXYZI> trans_est;
  TransformationEstimationPointToPlaneLLS<PointXYZRGBNormal, PointXYZRGBNormal, double> ptp;
  trans_est.estimateRigidTransformation (*keypoints_src, *keypoints_tgt, *good_correspondences, transf);
  std::cout << "\n\nMatriz de transformacao:\n" << transf << "\n\n";
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Affine3f create_rotation_matrix(double ax, double ay, double az) {
  Eigen::Affine3f rx =
      Eigen::Affine3f(Eigen::AngleAxisf(ax, Eigen::Vector3f(1, 0, 0)));
  Eigen::Affine3f ry =
      Eigen::Affine3f(Eigen::AngleAxisf(ay, Eigen::Vector3f(0, 1, 0)));
  Eigen::Affine3f rz =
      Eigen::Affine3f(Eigen::AngleAxisf(az, Eigen::Vector3f(0, 0, 1)));
  return rz * ry * rx;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void view (const PointCloud<PointT>::Ptr src,
           const PointCloud<PointT>::Ptr tgt,
           const PointCloud<Normal>::Ptr src_normals,
           const PointCloud<Normal>::Ptr tgt_normals,
           const PointCloud<PointXYZI>::Ptr src_kp,
           const PointCloud<PointXYZI>::Ptr tgt_kp,
           const CorrespondencesPtr correspondences)
{
  PointCloudColorHandlerRGBField<PointT> rgb_src(src);
  PointCloudColorHandlerRGBField<PointT> rgb_tgt(tgt);
  if (!vis_fim->updatePointCloud<PointT>(src, rgb_src, "source"))
  {
    vis_fim->addPointCloud<PointT>(src, rgb_src, "source");
//    vis_fim->addPointCloudNormals<PointT, Normal>(src, src_normals, 5, 0.5, "src_normals", 0);
//    vis_fim->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, "src_normals");
//    vis_fim->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 240, 255, 0, "src_normals");
    vis_fim->resetCameraViewpoint("source");
  }
  if (!vis_fim->updatePointCloud<PointT>(tgt, rgb_tgt, "target")){
    vis_fim->addPointCloud<PointT>(tgt, rgb_tgt, "target");
//    vis_fim->addPointCloudNormals<PointT, Normal>(tgt, tgt_normals, 5, 0.5, "tgt_normals", 0);
//    vis_fim->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, "tgt_normals");
//    vis_fim->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 240, 255, 0, "tgt_normals");
  }
  //  vis->setPointCloudRenderingProperties (PCL_VISUALIZER_OPACITY, 0.5, "source");
  //  vis->setPointCloudRenderingProperties (PCL_VISUALIZER_OPACITY, 0.7, "target");
  //  vis->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 6, "source");
  if (!vis_fim->updatePointCloud<PointXYZI>(src_kp, PointCloudColorHandlerCustom<PointXYZI>(src_kp, 0.0, 0.0, 255.0), "source_keypoints"))
    vis_fim->addPointCloud<PointXYZI> (src_kp, PointCloudColorHandlerCustom<PointXYZI>(src_kp, 0.0, 0.0, 255.0), "source_keypoints");
  if (!vis_fim->updatePointCloud<PointXYZI>(tgt_kp, PointCloudColorHandlerCustom<PointXYZI>(tgt_kp, 255.0, 0.0, 0.0), "target_keypoints"))
    vis_fim->addPointCloud<PointXYZI> (tgt_kp, PointCloudColorHandlerCustom<PointXYZI>(tgt_kp, 255.0, 0.0, 0.0), "target_keypoints");
  if (!vis_fim->updateCorrespondences<PointXYZI>(src_kp, tgt_kp, *correspondences, 1))
    vis_fim->addCorrespondences<PointXYZI>(src_kp, tgt_kp, *correspondences, 1, "correspondences");

  vis_fim->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 3, "source");
  vis_fim->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 3, "target");

  vis_fim->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 6, "source_keypoints");
  vis_fim->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 6, "target_keypoints");
  vis_fim->setShapeRenderingProperties (PCL_VISUALIZER_LINE_WIDTH, 15, "correspondences");
  vis_fim->setShapeRenderingProperties (PCL_VISUALIZER_COLOR, 0.0, 255.0, 0.0, "correspondences");
  vis_fim->initCameraParameters();
//  vis_fim->spinOnce(100);
  vis_fim->spin();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "register_node");
  ros::NodeHandle nh;

  // Init the visualizers
  vis_fim->setBackgroundColor(0, 0, 0);
  vis_fim->addCoordinateSystem (1.0);
  vis_acc->setBackgroundColor(0, 0, 0);
  vis_acc->addCoordinateSystem (1.0);

  // Read clouds from storage
  PointCloud<PointT>::Ptr source_cloud (new PointCloud<PointT>());
  PointCloud<PointT>::Ptr target_cloud (new PointCloud<PointT>());
  bool dragon = true;
  if (dragon) {
    PointCloud<PointXYZ>::Ptr src_xyz (new PointCloud<PointXYZ> ());
    PointCloud<PointXYZ>::Ptr tgt_xyz (new PointCloud<PointXYZ> ());
    loadPLYFile<PointXYZ>("/home/vinicius/register_ws/src/register/ptclouds/xyzrgb_dragon_back.ply",  *src_xyz);
    loadPLYFile<PointXYZ>("/home/vinicius/register_ws/src/register/ptclouds/xyzrgb_dragon_front.ply", *tgt_xyz);
    copyPointCloud(*src_xyz, *source_cloud);
    copyPointCloud(*tgt_xyz, *target_cloud);
    color_cloud(source_cloud);
    color_cloud(target_cloud);
  } else {
    loadPLYFile<PointT>("/home/vinicius/register_ws/src/register/laboratorio_clouds/ptCloud_1.ply", *source_cloud);
    loadPLYFile<PointT>("/home/vinicius/register_ws/src/register/laboratorio_clouds/ptCloud_2.ply", *target_cloud);
    std::vector<int> aux_indices;
    removeNaNFromPointCloud(*source_cloud, *source_cloud, aux_indices);
    removeNaNFromPointCloud(*target_cloud, *target_cloud, aux_indices);
    // Remove outliers in distance matters
    passthrough(source_cloud, "z", 0, 50);
    passthrough(target_cloud, "z", 0, 50);
  }
  // Mess up with source to make it harder
  Affine3f messrot = create_rotation_matrix(M_PI/6, M_PI/4, 0);
  Affine3f messt(Translation3f(Vector3f(-100, 0, 0)));
  Matrix4f mess = (messt*messrot).matrix(); //messt.matrix();//messt*messrot).matrix();
  transformPointCloud(*source_cloud, *source_cloud, mess);
  std::cout << "\nA matriz usada para danificar a source: \n" << mess << std::endl;
  std::cout << "\nA inversa da matriz usada para danificar a source: \n" << mess.inverse() << std::endl;
  // Reduce resolution to work better
  bool filter_cloud = true;
  if (filter_cloud) {
    VoxelGrid<PointT> grid;
    grid.setLeafSize(lf, lf, lf);
    grid.setInputCloud(source_cloud);
    grid.filter(*source_cloud);
    grid.setInputCloud(target_cloud);
    grid.filter(*target_cloud);
  }

  // Calculate mean distance so we go with this data to the rest of the algorithm
  float mean_src = mean_points_distance(source_cloud);
  float mean_tgt = mean_points_distance(target_cloud);
  int rate_kpt = 20, rate_des = 30; // To be used in the distance of the neighborhood
  // Remove outliers from clouds
  remove_outlier(source_cloud, 8, 1);
  remove_outlier(target_cloud, 8, 1);
  // Calculate all possible clouds with normals
  PointCloud<Normal>::Ptr src_normal (new PointCloud<Normal>());
  PointCloud<Normal>::Ptr tgt_normal (new PointCloud<Normal>());
  calculate_normals(source_cloud, 10, rate_kpt*mean_src, src_normal); // Use keypoints rate here as well, but number of neighbors is preferred for quality
  calculate_normals(target_cloud, 10, rate_kpt*mean_tgt, tgt_normal);
  // Calculate keypoints using the harris algorihtm
  PointCloud<PointXYZI>::Ptr src_kp (new PointCloud<PointXYZI> ());
  PointCloud<PointXYZI>::Ptr tgt_kp (new PointCloud<PointXYZI> ());
  PointIndicesConstPtr src_ind (new PointIndices ());
  PointIndicesConstPtr tgt_ind (new PointIndices ());
  estimateKeypoints(source_cloud, target_cloud, src_normal, tgt_normal, src_kp, tgt_kp, src_ind, tgt_ind, 10, rate_kpt*mean_src, rate_kpt*mean_tgt, 1);
  // Calculate features with FPFH algorithm
  PointCloud<FPFHSignature33>::Ptr src_fpfh (new PointCloud<FPFHSignature33> ());
  PointCloud<FPFHSignature33>::Ptr tgt_fpfh (new PointCloud<FPFHSignature33> ());
  estimateFPFH(source_cloud, target_cloud, src_normal, tgt_normal, src_kp, tgt_kp, src_fpfh, tgt_fpfh, 20, rate_des*mean_src, rate_des*mean_tgt);
  // Match the features
  CorrespondencesPtr all_correspondences (new Correspondences ());
  findCorrespondences(src_fpfh, tgt_fpfh, all_correspondences);
  // Reject bad ones
  CorrespondencesPtr good_correspondences (new Correspondences ());
  rejectBadCorrespondences(all_correspondences, src_kp, tgt_kp, good_correspondences);
  // Compute the transformation from the resultant correspondences
  Eigen::Matrix4f transformation;
  computeTransformation(src_kp, tgt_kp, good_correspondences, transformation);
  view(source_cloud, target_cloud, src_normal, tgt_normal, src_kp, tgt_kp, good_correspondences);
  // Transform the cloud with the resultant transformation and accumulate it
  transformPointCloud(*source_cloud, *source_cloud, transformation);
  PointCloud<PointXYZRGBNormal>::Ptr final (new PointCloud<PointXYZRGBNormal> ());
  PointCloud<PointXYZRGBNormal>::Ptr temp (new PointCloud<PointXYZRGBNormal> ());
  concatenateFields(*target_cloud, *tgt_normal, *final);
  concatenateFields(*source_cloud, *src_normal, *temp);
  *final += *temp;

  savePLYFile("/home/vinicius/register_ws/src/register/ptclouds/saida_final.ply", *final);
  savePLYFile("/home/vinicius/register_ws/src/register/ptclouds/saida_source_alterada.ply", *source_cloud);
  ROS_INFO("Rotina finalizada");
  // Check the final cloud after transformation
  PointCloudColorHandlerRGBField<PointXYZRGBNormal> rgb_final(final);
  if (!vis_acc->updatePointCloud<PointXYZRGBNormal>(final, rgb_final, "final"))
  {
    vis_acc->addPointCloud<PointXYZRGBNormal>(final, rgb_final, "final");
    vis_acc->resetCameraViewpoint("final");
  }
  vis_acc->spin();

  // Spin just once ROS
  ros::spinOnce();

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
