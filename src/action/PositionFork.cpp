#include "feeding/action/MoveAboveForque.hpp"

#include <pr_tsr/plate.hpp>
#include "feeding/action/MoveAbove.hpp"
#include <libada/util.hpp>

using ada::util::createBwMatrixForTSR;
using aikido::constraint::dart::TSR;

namespace feeding {
namespace action {

void positionFork(std::string foodName, const Eigen::Isometry3d &foodTransform,
                   float rotateAngle, TiltStyle tiltStyle,
                   double rotationTolerance, FeedingDemo *feedingDemo,
                   double *angleGuess) {
  // Load necessary parameters from feedingDemo
  const std::shared_ptr<::ada::Ada> &ada = feedingDemo->getAda();
  double heightAboveFood = feedingDemo->mFoodTSRParameters.at("height");
  double horizontalTolerance = 0.05;
  double verticalTolerance = 0.05;
  // NOTE: Although tiltTolerance was originally passed in as a param, it was
  // never used. double tiltTolerance =
  // feedingDemo->mFoodTSRParameters.at("tiltTolerance");

  Eigen::Isometry3d target;
  // eeTransform is the offset of the fork
  Eigen::Isometry3d eeTransform;
  eeTransform.translation()[2] = 0.1;

  // TODO: remove hardcoded transform for food
  

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



  target.translation()[2] = feedingDemo->mTableHeight;
  ROS_WARN_STREAM("Food Height: " << target.translation()[2]);

  

  auto retval =
      moveAbove(target, eeTransform, horizontalTolerance, verticalTolerance,
                rotationTolerance, 0.0, feedingDemo);


 



}

} // namespace action
} // namespace feeding
