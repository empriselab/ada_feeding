
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <ros/ros.h>
#include <libada/util.hpp>

#include "feeding/FeedingDemo.hpp"
#include "feeding/util.hpp"
#include "feeding/action/PickUpFork.hpp"
#include "feeding/action/PutDownFork.hpp"
#include "feeding/action/FeedFoodToPerson.hpp"
#include "feeding/action/Skewer.hpp"
#include "feeding/action/MoveAbove.hpp"
#include "feeding/action/MoveInFrontOfPerson.hpp"
#include "feeding/action/MoveDirectlyToPerson.hpp"
#include "feeding/action/CutAndScoop.hpp"
#include <cstdlib>
#include <ctime>

using ada::util::getRosParam;
using ada::util::waitForUser;

namespace feeding {

void brandonDemo(
    FeedingDemo& feedingDemo,
    std::shared_ptr<Perception>& perception,
    ros::NodeHandle nodeHandle)
{

  ROS_INFO_STREAM("==========  BRANDON DEMO ========== ");

  auto workspace = feedingDemo.getWorkspace();
  auto plate = workspace->getPlate()->getRootBodyNode()->getWorldTransform();

  talk("Hello, my name is aid uh. It's my pleasure to serve you today!");

  srand(time(NULL));

  while (true)
  {
    if (feedingDemo.getFTThresholdHelper())
        feedingDemo.getFTThresholdHelper()->setThresholds(STANDARD_FT_THRESHOLD);

    talk("What food would you like?");

    auto foodName = getUserFoodInput(false, nodeHandle);// "cantaloupe";//
    if (foodName == std::string("quit")) {
        break;
    }

    nodeHandle.setParam("/deep_pose/forceFood", false);
    nodeHandle.setParam("/deep_pose/publish_spnet", (true));
    nodeHandle.setParam("/deep_pose/invertSPNetDirection", false);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    ROS_INFO_STREAM("Running bite transfer study for " << foodName);



    talk(std::string("One ") + foodName + std::string(" coming right up!"), true);


    // ===== FORQUE PICKUP =====
    if (foodName == "pickupfork")
    {
      action::pickUpFork(
        plate,
        feedingDemo.getPlateEndEffectorTransform(),
        &feedingDemo);
    }
    else if (foodName == "putdownfork")
    {
      action::putDownFork(
        plate,
        feedingDemo.getPlateEndEffectorTransform(),
        &feedingDemo);
    }
    else
    {

      bool cutAndScoop = action::cutAndScoop(
        perception,
        foodName,
        plate,
        feedingDemo.getPlateEndEffectorTransform(),
        &feedingDemo);

      // if (feedingDemo.getFTThresholdHelper())
      //   feedingDemo.getFTThresholdHelper()->setThresholds(STANDARD_FT_THRESHOLD);

      // if (!skewer)
      // {
      //   ROS_WARN_STREAM("Restart from the beginning");
      //   continue;
      // }

      // // ===== IN FRONT OF PERSON =====
      // ROS_INFO_STREAM("Move forque in front of person");

      // auto tiltFoods = feedingDemo.mTiltFoodNames;
      // bool tilted = (std::find(tiltFoods.begin(), tiltFoods.end(), foodName) != tiltFoods.end());

      // action::feedFoodToPerson(
      //   perception,
      //   plate,
      //   feedingDemo.getPlateEndEffectorTransform(),
      //   tilted ? &feedingDemo.mTiltOffset : nullptr,
      //   &feedingDemo
      //   );

    }
  }

  // ===== DONE =====
  ROS_INFO("Demo finished.");
  talk("Thank you, I hope I was helpful!");
}
};
