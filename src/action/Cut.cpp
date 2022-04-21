#include "feeding/action/Cut.hpp"

#include <libada/util.hpp>

#include "feeding/TargetItem.hpp"
#include "feeding/action/MoveAbove.hpp"
#include "feeding/action/MoveOutOf.hpp"
#include "feeding/action/PutDownFork.hpp"
#include "feeding/perception/PerceptionServoClient.hpp"
#include "feeding/util.hpp"

namespace feeding {
namespace action {

bool moveFork(const std::shared_ptr<Perception> &perception, TargetItem item,
              const Eigen::Vector3d &endEffectorDirection, const Eigen::Vector3d &moveDir,
              FeedingDemo *feedingDemo, std::string controller = "move_until_touch_topic_controller") {
  // Load necessary parameters from feedingDemo
  const std::shared_ptr<::ada::Ada> &ada = feedingDemo->getAda();
  const ros::NodeHandle *nodeHandle = feedingDemo->getNodeHandle().get();
  const aikido::constraint::dart::CollisionFreePtr &collisionFree =
      feedingDemo->getCollisionConstraint();
  double heightIntoFood = feedingDemo->mFoodTSRParameters.at("heightInto");
  double planningTimeout = feedingDemo->mPlanningTimeout;
  double endEffectorOffsetPositionTolerance =
      feedingDemo->mEndEffectorOffsetPositionTolerance;
  double endEffectorOffsetAngularTolerance =
      feedingDemo->mEndEffectorOffsetAngularTolerance;
  std::shared_ptr<FTThresholdHelper> ftThresholdHelper =
      feedingDemo->getFTThresholdHelper();
  const Eigen::Vector6d &velocityLimits = feedingDemo->mVelocityLimits;

  ROS_INFO_STREAM("Move into " + TargetToString.at(item));

  if (item != FOOD && item != FORQUE)
    throw std::invalid_argument("MoveInto[" + TargetToString.at(item) +
                                "] not supported");

  if (controller.compare("move_until_touch_topic_controller")!=0){
    ada->switchControllers(
      std::vector<std::string> {controller}, 
      std::vector<std::string> {"move_until_touch_topic_controller"}
    );
  }

  std::cout << "endEffectorDirection " << endEffectorDirection.transpose()
            << std::endl;

  {
    int numDofs = ada->getArm()->getMetaSkeleton()->getNumDofs();
    // Collision constraint is not set because f/t sensor stops execution.
    // movedir is amount to move by
    auto trajectory =
        ada->getArm()->planToOffset(ada->getEndEffectorBodyNode()->getName(),
                                    moveDir);

    bool success = true;
    try {
      auto future = ada->getArm()->executeTrajectory(
          trajectory); // check velocity limits are set in FeedingDemo
      future.get();
    } catch (const std::exception &e) {
      dtwarn << "Exception in trajectoryExecution: " << e.what() << std::endl;
      success = false;
    }
  }

  if (controller.compare("move_until_touch_topic_controller")!=0){
    ada->switchControllers(
      std::vector<std::string> {"move_until_touch_topic_controller"}, 
      std::vector<std::string> {controller}
    );
  }

  return true;
}

} // namespace action
} // namespace feeding
