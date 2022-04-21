#include "feeding/action/Scoop.hpp"

#include "feeding/TargetItem.hpp"
#include "feeding/action/MoveAbove.hpp"
#include "feeding/action/MoveInto.hpp"
#include "feeding/action/PickUpFork.hpp"
#include <map>



namespace feeding {
namespace action {




bool scoop(FeedingDemo *feedingDemo, double angle){
  const std::shared_ptr<::ada::Ada> &ada = feedingDemo->getAda();
  const aikido::constraint::dart::CollisionFreePtr &collisionFree =
      feedingDemo->getCollisionConstraint();
  double horizontalTolerance =
      feedingDemo->mPlateTSRParameters.at("horizontalTolerance");
  double verticalTolerance =
      feedingDemo->mPlateTSRParameters.at("verticalTolerance");

  // Get current pose of the robot. Ada getarm get configurations
  Eigen::VectorXd currentPose = ada->getArm()->getCurrentConfiguration();

  // Change last element of current pose. (current angle is 70 and want to make it -20
  // , so we subtract by 90 and convert to radians)
  std::stringstream prev;
  prev << currentPose;
  currentPose[5] = currentPose[5]+angle;

  ROS_INFO_STREAM("Hello");
  std::stringstream ss;
  ss << currentPose;
  ROS_INFO_STREAM(prev.str().c_str());
  ROS_INFO_STREAM(ss.str().c_str());


  // Edit last element of the robot

  auto trajectory = ada->getArm()->planToConfiguration(
      currentPose,
      ada->getArm()->getWorldCollisionConstraint());
  bool success = true;
  auto future = ada->getArm()->executeTrajectory(
      trajectory); // check velocity limits are set in FeedingDemo
  try {
    future.get();
  } catch (const std::exception &e) {
    dtwarn << "Exception in trajectoryExecution: " << e.what() << std::endl;
    success = false;
  }
  return success;
}

// bool moveWithEndEffectorTwist(
//     const Eigen::Vector6d& twists,
//     double duration,
//     bool respectCollision)
// {
//    //temporarily disabling
//   return mAda->moveArmWithEndEffectorTwist(
//     Eigen::Vector6d(getRosParam<std::vector<double>>("/scoop/twist1",
//     mNodeHandle).data()),
//     respectCollision ? mCollisionFreeConstraint : nullptr,
//     duration,
//     getRosParam<double>("/planning/timeoutSeconds", mNodeHandle),
//     getRosParam<double>(
//           "/planning/endEffectorTwist/positionTolerance", mNodeHandle),
//     getRosParam<double>(
//           "/planning/endEffectorTwist/angularTolerance", mNodeHandle));

// }

// void scoop(const std::shared_ptr<ada::Ada> &ada) {
//   //  temporarily disabling
//   std::vector<std::string> twists{
//     "/scoop/twist1", "/scoop/twist2", "/scoop/twist3"};

//   for (const auto & param : twists)
//   {
//     auto success =
//       ada->moveWithEndEffectorTwist(
//         Eigen::Vector6d(getRosParam<std::vector<double>>(param,
//         mNodeHandle).data()));
//     if (!success)
//     {
//       ROS_ERROR_STREAM("Failed to execute " << param << std::endl);
//       throw std::runtime_error("Failed to execute scoop");
//     }
//   }
// }

} // namespace action
} // namespace feeding
