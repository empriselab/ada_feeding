#include "feeding/action/MoveAboveForque.hpp"

#include <pr_tsr/plate.hpp>

#include <libada/util.hpp>

using ada::util::createBwMatrixForTSR;

namespace feeding {
namespace action {

void push(
    const aikido::constraint::dart::CollisionFreePtr &collisionFree,
    FeedingDemo *feedingDemo) {
  ROS_INFO_STREAM("Nothing wrong here");
  // Load necessary parameters from feedingDemo
  const std::shared_ptr<::ada::Ada> &ada = feedingDemo->getAda();
  double forkHolderAngle = feedingDemo->mForkHolderAngle;
  std::vector<double> forkHolderTranslation =
      feedingDemo->mForkHolderTranslation;
  double planningTimeout = feedingDemo->mPlanningTimeout;
  int maxNumTrials = feedingDemo->mMaxNumTrials;
  int batchSize = feedingDemo->mBatchSize;
  int maxNumBatches = feedingDemo->mMaxNumBatches;
  int numMaxIterations = feedingDemo->mNumMaxIterations;


  // // Creating the trajectory
  // auto aboveForqueTSR = pr_tsr::getDefaultPlateTSR();
  // Eigen::Isometry3d forquePose = Eigen::Isometry3d::Identity();
  
  // forquePose.translation() =
  //     Eigen::Vector3d{forkHolderTranslation[0], forkHolderTranslation[1],
  //                     forkHolderTranslation[2]};
  // forquePose.linear() = Eigen::Matrix3d(
  //     Eigen::AngleAxisd(forkHolderAngle, Eigen::Vector3d::UnitX()));
  // aboveForqueTSR.mT0_w = forquePose;

  // aboveForqueTSR.mBw = createBwMatrixForTSR(0.0001, 0.0001, 0.0001, 0, 0, 0);

  // ROS_INFO_STREAM("Loaded in stuff");
  // // TODO: Remove hardcoded transform for plate
  // Eigen::Isometry3d eeTransformPlate;
  // Eigen::Matrix3d rot;
  // rot << 1., 0., 0., 0., -1, 0., 0., 0., -1;
  // eeTransformPlate.linear() = rot;
  // aboveForqueTSR.mTw_e.matrix() *= eeTransformPlate.matrix();

  // auto aboveForqueTSRPtr =
  //     std::make_shared<aikido::constraint::dart::TSR>(aboveForqueTSR);



  // // The trajectory stuff
  // auto trajectory = ada->getArm()->planToTSR(
  //     ada->getEndEffectorBodyNode()->getName(), aboveForqueTSRPtr,
  //     ada->getArm()->getWorldCollisionConstraint(),
  //     aikido::robot::util::PlanToTSRParameters(
  //       maxNumTrials,
  //       batchSize,
  //       maxNumBatches,
  //       numMaxIterations));
  // bool success = true;
  // // Executing the trajectory
  // auto future = ada->getArm()->executeTrajectory(
  //     trajectory); // check velocity limits are set in FeedingDemo



  // try {
  //   future.get();
  // } catch (const std::exception &e) {
  //   dtwarn << "Exception in trajectoryExecution: " << e.what() << std::endl;
  //   success = false;
  // }
  // if (!success)
  //   throw std::runtime_error("Trajectory execution failed");
}

} // namespace action
} // namespace feeding
