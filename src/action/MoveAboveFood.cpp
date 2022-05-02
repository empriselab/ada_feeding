#include "feeding/action/MoveAboveFood.hpp"

#include <aikido/constraint/dart/TSR.hpp>
#include <math.h>

#include <libada/util.hpp>

#include "feeding/AcquisitionAction.hpp"
#include "feeding/action/MoveAbove.hpp"
#include "feeding/util.hpp"
#define PI 3.14159265

using aikido::constraint::dart::TSR;

// Contains motions which are mainly TSR actions
namespace feeding {
namespace action {
Eigen::Matrix3d getRotMatrix(float rot_x, float rot_y, float rot_z){
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

bool moveAboveFood(std::string foodName, const Eigen::Isometry3d &foodTransform,
                   float rotateAngle, TiltStyle tiltStyle,
                   double rotationTolerance, FeedingDemo *feedingDemo,
                   double *angleGuess) {
  // Load necessary parameters from feedingDemo
  const std::shared_ptr<::ada::Ada> &ada = feedingDemo->getAda();
  double heightAboveFood = feedingDemo->mFoodTSRParameters.at("height");
  double horizontalTolerance =
      feedingDemo->mFoodTSRParameters.at("horizontalTolerance");
  double verticalTolerance =
      feedingDemo->mFoodTSRParameters.at("verticalTolerance");
  // NOTE: Although tiltTolerance was originally passed in as a param, it was
  // never used. double tiltTolerance =
  // feedingDemo->mFoodTSRParameters.at("tiltTolerance");

  Eigen::Isometry3d target;
  // TODO: remove hardcoded transform for food
  Eigen::Isometry3d eeTransform;
  Eigen::Matrix3d rot;
  rot << -1, 0., 0., 0., 1., 0., 0., 0., -1;
  eeTransform.linear() = rot;
  Eigen::AngleAxisd rotation =
      Eigen::AngleAxisd(-rotateAngle, Eigen::Vector3d::UnitZ());
  ROS_WARN_STREAM("Rotate Angle: " << rotateAngle);

  // Apply base rotation to food
  Eigen::Vector3d foodVec = foodTransform.rotation() * Eigen::Vector3d::UnitX();
  double baseRotateAngle = atan2(foodVec[1], foodVec[0]);
  if (angleGuess) {
    while (abs(baseRotateAngle - *angleGuess) > (M_PI / 2.0)) {
      baseRotateAngle += (*angleGuess > baseRotateAngle) ? M_PI : (-M_PI);
    }
  }
  ROS_WARN_STREAM("Food Rotate Angle: " << baseRotateAngle);
  Eigen::AngleAxisd baseRotation =
      Eigen::AngleAxisd(baseRotateAngle, Eigen::Vector3d::UnitZ());
  target = removeRotation(foodTransform);
  auto rotationFreeFoodNames = feedingDemo->mRotationFreeFoodNames;
  if (std::find(rotationFreeFoodNames.begin(), rotationFreeFoodNames.end(),
                foodName) == rotationFreeFoodNames.end()) {
    target.linear() = target.linear() * baseRotation;
  }
  target.translation()[2] = feedingDemo->mTableHeight;
  ROS_WARN_STREAM("Food Height: " << target.translation()[2]);

  if (tiltStyle == TiltStyle::NONE) {
    eeTransform.linear() = eeTransform.linear() * rotation;
    eeTransform.translation()[2] = heightAboveFood;
    // eeTransform.translation()[0] += 0.01;
    // eeTransform.translation()[1] += 0.02;
  } else if (tiltStyle == TiltStyle::VERTICAL) {
    eeTransform.linear() = eeTransform.linear() * rotation *
                           Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitX());
    eeTransform.translation()[2] = heightAboveFood;
  } else // angled
  {
    eeTransform.linear() =
        eeTransform.linear() * rotation *
        Eigen::AngleAxisd(-M_PI / 8, Eigen::Vector3d::UnitX());
    eeTransform.translation() =
        Eigen::AngleAxisd(
            rotateAngle,
            Eigen::Vector3d::UnitZ()) // Take into account action rotation
        * Eigen::Vector3d{0, -sin(M_PI * 0.25) * heightAboveFood * 0.7,
                          cos(M_PI * 0.25) * heightAboveFood * 0.9};
  }
  // What is foodTransform? The rotation matrix of food transform is identity
  std::cout << "Target rotation " << target.linear()<< std::endl;
  std::cout << "Hieght above food" <<  heightAboveFood << std::endl;
  ROS_INFO_STREAM("Changing eetransform val");
  eeTransform.translation()[2] = 0.4;
  // -60 in the x axis. z axis depends on food
  float x_rot = -60.;
  float y_rot = -15;
  float z_rot = -25;
  Eigen::Matrix3d rot2 = getRotMatrix(x_rot, y_rot, z_rot);
  //rot2 <<  0.97, 0.22, -0.13, 0., 0.5, 0.87, 0.26, -0.84, 0.48;
  target.linear() = rot2;
  //target.translation()[0] = target.translation()[0]+0.1;
  target.translation()[1] = target.translation()[1]-0.2;
  Eigen::Matrix3d test = getRotMatrix(x_rot, y_rot, z_rot);
  std::cout << "Testing getRotMatrix:\n" <<test << std::endl;
  auto retval =
      moveAbove(target, eeTransform, horizontalTolerance, verticalTolerance,
                rotationTolerance, 0.0, feedingDemo);
  ROS_INFO_STREAM("Moved above food");
  return retval;
}


} // namespace action
} // namespace feeding
