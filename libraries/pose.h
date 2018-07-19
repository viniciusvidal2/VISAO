#ifndef POSE_H
#define POSE_H

// Estrutura para simplificar e passar para filtro
struct Pose_atual{
  double x;
  double y;      // [m]
  double z;
  double dx;
  double dy;     // [m]
  double dz;
  double roll;
  double pitch;  // [DEG]
  double yaw;
  double droll;
  double dpitch; // diferenca de angulos [DEG]
  double dyaw;
} ;

#endif // POSE_H
