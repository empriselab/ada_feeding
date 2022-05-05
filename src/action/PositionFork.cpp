#include "feeding/action/MoveAboveForque.hpp"

#include <pr_tsr/plate.hpp>
#include "feeding/action/MoveAbove.hpp"
#include <libada/util.hpp>

using ada::util::createBwMatrixForTSR;
using aikido::constraint::dart::TSR;
#define PI 3.14159265


namespace feeding {
namespace action {

Eigen::Matrix3d getRotMatrix(float rot_x, float rot_y, double rot_z){
  Eigen::Matrix3d rot;
  rot_x = rot_x * PI/180; rot_y = rot_y * PI/180; rot_z = rot_z * PI/180;
  rot << cos(rot_z)*cos(rot_y),
  cos(rot_z)*sin(rot_y)*sin(rot_x) - sin(rot_z)*cos(rot_x),
  cos(rot_z)*sin(rot_y)*cos(rot_x) + sin(rot_z)*sin(rot_x),
  sin(rot_z)*cos(rot_y),
  sin(rot_z)*sin(rot_y)*sin(rot_x) + cos(rot_z)*cos(rot_x),
  sin(rot_z)*sin(rot_y)*cos(rot_x) - cos(rot_z)*sin(rot_x),
  -sin(rot_y), cos(rot_y) * sin(rot_x), cos(rot_y) * cos(rot_x);
  return rot;
}

void positionFork(FeedingDemo *feedingDemo, double z_rot, float horizontalTolerance=0.003, 
float verticalTolerance=0.008, float rotationTolerance=0.5){
  Eigen::Isometry3d target;
  Eigen::Isometry3d eeTransform;
  
  target.translation()[0] = 0.3;
  target.translation()[1] = -0.25;
  target.translation()[2] = 0.235;
  float x_rot = -60.;
  float y_rot = -15;
  Eigen::Matrix3d rot2 = getRotMatrix(x_rot, y_rot, z_rot);
  //rot2 <<  0.97, 0.22, -0.13, 0., 0.5, 0.87, 0.26, -0.84, 0.48;
  target.linear() = rot2;
  eeTransform.translation()[0] = 0.;
  eeTransform.translation()[1] = 0.;
  eeTransform.translation()[2] = 0.4;
  Eigen::Matrix3d rot;
  rot << 0, -1., 0., -1., 0., 0., 0., 0., -1;
  eeTransform.linear() = rot;



  moveAbove(target, eeTransform, horizontalTolerance, verticalTolerance,
               rotationTolerance, 0.0, feedingDemo);
  
}

} // namespace action
} // namespace feeding
