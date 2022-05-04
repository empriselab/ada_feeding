#include "feeding/action/CutAndScoop.hpp"

#include <libada/util.hpp>

#include "feeding/FeedingDemo.hpp"
#include "feeding/action/DetectAndMoveAboveFood.hpp"
#include "feeding/action/Grab.hpp"
#include "feeding/action/MoveAbovePlate.hpp"
#include "feeding/action/MoveInto.hpp"
#include "feeding/action/MoveOutOf.hpp"
#include "feeding/util.hpp"
#include "feeding/action/Push.hpp"
#include "feeding/action/Cut.hpp"
#include "feeding/action/Scoop.hpp"
#include "feeding/action/PositionFork.hpp"
#include "feeding/action/MoveAbove.hpp"
#include "cut_finder/find_cut.h"

#include <ros/topic.h>
#define PI 3.14159265

using ada::util::getRosParam;

static const std::vector<std::string> optionPrompts{"(1) success", "(2) fail"};
static const std::vector<std::string> actionPrompts{"(1) skewer", "(3) tilt",
                                                    "(5) angle"};

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

void pFork(FeedingDemo *feedingDemo, float horizontalTolerance=0.003, 
float verticalTolerance=0.008, float rotationTolerance=0.5){
  Eigen::Isometry3d target;
  Eigen::Isometry3d eeTransform;
  
  target.translation()[0] = 0.3;
  target.translation()[1] = -0.25;
  target.translation()[2] = 0.235;
  float x_rot = -60.;
  float y_rot = -15;
  double z_rot = 0;
  Eigen::Matrix3d rot2 = getRotMatrix(x_rot, y_rot, z_rot);
  rot2 <<  0.97, 0.22, -0.13, 0., 0.5, 0.87, 0.26, -0.84, 0.48;
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
//==============================================================================
bool cutAndScoop(const std::shared_ptr<Perception> &perception,
            const std::string &foodName, const Eigen::Isometry3d &plate,
            const Eigen::Isometry3d &plateEndEffectorTransform,
            FeedingDemo *feedingDemo) {
  // Load necessary parameters from feedingDemo
  const std::shared_ptr<::ada::Ada> &ada = feedingDemo->getAda();
  const ros::NodeHandle *nodeHandle = feedingDemo->getNodeHandle().get();
  const aikido::constraint::dart::CollisionFreePtr &collisionFree =
      feedingDemo->getCollisionConstraint();
  const std::unordered_map<std::string, double> &foodSkeweringForces =
      feedingDemo->mFoodSkeweringForces;
  double heightAboveFood = feedingDemo->mFoodTSRParameters.at("height");
  double rotationToleranceForFood =
      feedingDemo->mFoodTSRParameters.at("rotationTolerance");
  double moveOutofFoodLength = feedingDemo->mMoveOufOfFoodLength;
  std::chrono::milliseconds waitTimeForFood = feedingDemo->mWaitTimeForFood;
  // const Eigen::Vector6d& velocityLimits = feedingDemo.mVelocityLimits;
  const std::shared_ptr<FTThresholdHelper> &ftThresholdHelper =
      feedingDemo->getFTThresholdHelper();
  std::vector<std::string> rotationFreeFoodNames =
      feedingDemo->mRotationFreeFoodNames;

  // TODO (egordon): add option to disable override in feeding_demo.yaml
  int actionOverride = -1;
  if (feedingDemo->mPickUpAngleModes.count(foodName)) {
    actionOverride = feedingDemo->mPickUpAngleModes[foodName];
  }
ROS_INFO_STREAM("Home config cutting");
  ROS_INFO_STREAM("Move above plate");
  bool abovePlaceSuccess =
      moveAbovePlate(plate, plateEndEffectorTransform, feedingDemo, "home_config");

  // Obtain camera info
  // sensor_msgs::ImageConstPtr info = 
  // ros::topic::waitForMessage<sensor_msgs::Image>(
  //   "/camera/color/image_raw"
  // );
  // sensor_msgs::ImageConstPtr info = "tmp";
  // get_cut(info);


  double z_rot = -.0;
  double * zptr = &z_rot;
    // angle guess is the z axis
  pFork(feedingDemo);


  // detectAndMoveAboveFood(perception,
  //                      foodName, 0.5,
  //                      feedingDemo, zptr,
  //                      1) ;
  // take picture function that takes a picture and returns cutting position, angle and push direction                       
// bool vruh =
//       moveAbovePlate(plate, plateEndEffectorTransform, feedingDemo, "home_config_cutting");
ROS_INFO_STREAM("working");
  // Angle the fork in the correct cutting position. Fork base position is at 90, can move by +-90 degrees
    // Move to where to make the cut
    Eigen::Vector3d endEffectorDirection(1, 1, 0);
      
    // We need perception, foodName, rotation

    //detectAndMoveAboveFood(perception, foodName, rotationToleranceForFood,
                                 //feedingDemo, nullptr, actionOverride);
    // what is actionOverride(angleguess)? what is rotationTolerance for food?
    // define rotation tolerance by yourself
    // action override is the guess of the angle
  int isLeft = 1;

  if (isLeft == -1){
      bool rotateFork = scoop(feedingDemo, 3.14);
  }
  
  endEffectorDirection = Eigen::Vector3d(0,0,-5);
  ROS_INFO_STREAM("Cut");
  bool cutSuccess = moveFork(perception, TargetItem::FOOD,
                                    endEffectorDirection, 
                                    Eigen::Vector3d(0., 0., -0.11),feedingDemo,
                                    "move_until_touch_topic_controller");
//    moveFork(perception, TargetItem::FOOD,
//                                     endEffectorDirection, 
//                                     Eigen::Vector3d(0., 0., 0.03),feedingDemo,
//                                     "move_until_touch_topic_controller");


  endEffectorDirection = Eigen::Vector3d(5 , 0, 0) ;
  ROS_INFO_STREAM("Push Fork");
  bool pushSuccess = moveFork(perception, TargetItem::FOOD,
                                    endEffectorDirection, 
                                    Eigen::Vector3d(0.05*isLeft, 0., 0.),feedingDemo,
                                    "move_until_touch_topic_controller");

  ROS_INFO_STREAM("Scoop hello");
  bool scoopSuccess = scoop(feedingDemo, -1.57*isLeft);



  if (!abovePlaceSuccess) {
    talk("Sorry, I'm having a little trouble moving. Mind if I get a little "
         "help?");
    ROS_WARN_STREAM("Move above plate failed. Please restart");
    return false;
  }

  bool detectAndMoveAboveFoodSuccess = true;

}

void getCut(sensor_msgs::ImageConstPtr imagePtr){
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<cut_finder::find_cut>("find_cut");
  cut_finder::find_cut srv;
  srv.request.img = *imagePtr;
  if (client.call(srv)){
    ROS_INFO_STREAM("Called succesfully");
    std::cout << srv.response.trans << "\n" << srv.response.rot << std::endl;
  }
  else{
    ROS_ERROR("Could not call service find_cut");
  }

}



} // namespace action
} // namespace feeding
