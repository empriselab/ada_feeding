#include "feeding/nodes.hpp"
/**
 * Nodes for configuring movement actions
 **/

#include <Eigen/Core>
#include <behaviortree_cpp/behavior_tree.h>
#include <cmath>
#include <iostream>

#include <aikido/perception/DetectedObject.hpp>
using aikido::perception::DetectedObject;
#include "feeding/AcquisitionAction.hpp"

#include <aikido/common/util.hpp>
using aikido::common::FuzzyZero;

namespace feeding {
namespace nodes {

// Write PlanToPose Params from action
class ConfigMoveAbove : public BT::SyncActionNode {
public:
  ConfigMoveAbove(const std::string &name, const BT::NodeConfig &config,
                  ada::Ada *robot, ros::NodeHandle *nh)
      : BT::SyncActionNode(name, config), mAda(robot), mNode(nh) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<Eigen::Isometry3d>("obj_transform"),
            BT::InputPort<AcquisitionAction>("action"),
            BT::InputPort<Eigen::Isometry3d>("ee_transform"),
            BT::InputPort<bool>("yaw_agnostic"),
            BT::InputPort<double>("overshoot"),
            BT::OutputPort<std::vector<double>>("orig_pos"),
            BT::OutputPort<std::vector<double>>("orig_quat"),
            BT::OutputPort<std::vector<double>>("pos"),
            BT::OutputPort<std::vector<double>>("quat"),
            BT::OutputPort<std::vector<double>>("bounds")};
  }

  BT::NodeStatus tick() override {
    // Read Params
    // Default vertical skewer
    auto transformInput = getInput<Eigen::Isometry3d>("ee_transform");
    auto actionInput = getInput<AcquisitionAction>("action");

    // Concat two transforms if provided, else default
    Eigen::Isometry3d eeTransform = AcquisitionAction().pre_transform;
    if (actionInput || transformInput) {
      eeTransform = ((actionInput) ? actionInput.value().pre_transform
                                   : Eigen::Isometry3d::Identity()) *
                    ((transformInput) ? transformInput.value()
                                      : Eigen::Isometry3d::Identity());
    }

    // Origin is the food item
    auto objectInput = getInput<Eigen::Isometry3d>("obj_transform");
    if (!objectInput) {
      return BT::NodeStatus::FAILURE;
    }
    auto origin = objectInput.value();

    // Default normal yaw bounds
    auto yawAgnosticInput = getInput<bool>("yaw_agnostic");
    bool yawAgnostic = yawAgnosticInput ? yawAgnosticInput.value() : false;

    // Read Ros Params
    double distance;
    if (!mNode->getParam("move_above/distance", distance)) {
      ROS_WARN_STREAM("ConfigMoveAbove: Need distance param");
      return BT::NodeStatus::FAILURE;
    }
    

    std::vector<double> bounds{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    mNode->getParam("move_above/tsr_bounds", bounds);
    if (bounds.size() != 6) {
      ROS_WARN_STREAM("ConfigMoveAbove: TSR bounds must be size 6");
      return BT::NodeStatus::FAILURE;
    }
    // If yaw agnostic, unbounded
    if (yawAgnostic)
      bounds[5] = M_PI;
    setOutput<std::vector<double>>("bounds", bounds);

    // Output origin frame
    Eigen::Vector3d eOrigPos = origin.translation();
    std::vector<double> orig_pos{eOrigPos[0], eOrigPos[1], eOrigPos[2]};
    Eigen::Quaterniond eOrigQuat(origin.linear());
    std::vector<double> orig_quat{eOrigQuat.w(), eOrigQuat.x(), eOrigQuat.y(),
                                  eOrigQuat.z()};
    setOutput<std::vector<double>>("orig_pos", orig_pos);
    setOutput<std::vector<double>>("orig_quat", orig_quat);

    // Desired pose relative to origin
    // Scale by distance param
    eeTransform.translation() =
        distance * eeTransform.translation().normalized();

    

    // Output EE target pose
    Eigen::Vector3d ePos = eeTransform.translation();

    

    // Modify start position if it's a premanipulation action
    if (actionInput.value().action_type.compare("pre_manip") == 0){
      Eigen::Vector3d offset = actionInput.value().pre_offset;
      distance = offset.lpNorm<Eigen::Infinity>();
      std::cout << "Premanipulation action with distance" << distance; 
      Eigen::Vector3d tmp = (offset.array() * eeTransform.translation().normalized().array()).matrix();
      std::vector<double> pos{tmp[0], tmp[1], tmp[2]};
      setOutput<std::vector<double>>("pos", pos);
    }else{
      std::vector<double> pos{ePos[0], ePos[1], ePos[2]};
    }

    Eigen::Quaterniond eQuat(eeTransform.linear());
    std::vector<double> quat{eQuat.w(), eQuat.x(), eQuat.y(), eQuat.z()};

    // setOutput<std::vector<double>>("pos", pos);
    setOutput<std::vector<double>>("quat", quat);

    return BT::NodeStatus::SUCCESS;
  }

private:
  ada::Ada *mAda;
  ros::NodeHandle *mNode;
};

class ConfigMoveInto : public BT::SyncActionNode {
public:
  ConfigMoveInto(const std::string &name, const BT::NodeConfig &config,
                 ada::Ada *robot, ros::NodeHandle *nh)
      : BT::SyncActionNode(name, config), mAda(robot), mNode(nh) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<Eigen::Isometry3d>("obj_transform"),
            BT::InputPort<AcquisitionAction>("action"),
            BT::InputPort<double>("overshoot"),
            BT::InputPort<std::vector<double>>("world_off"),
            BT::OutputPort<std::vector<double>>("offset")};
  }

  BT::NodeStatus tick() override {
    std::cout << "Moving into object\n";
    // Read Object
    auto objectInput = getInput<Eigen::Isometry3d>("obj_transform");
    if (!objectInput) {
      return BT::NodeStatus::FAILURE;
    }
    auto objTransform = objectInput.value();

    // Input other arguments
    auto overshootInput = getInput<double>("overshoot");
    double overshoot = overshootInput ? overshootInput.value() : 0.0;
    auto offInput = getInput<std::vector<double>>("world_off");
    Eigen::Vector3d eOff = offInput ? Eigen::Vector3d(offInput.value().data())
                                    : Eigen::Vector3d::Zero();

    // If action provided, add offset from there
    auto actionInput = getInput<AcquisitionAction>("action");
    if (actionInput) {
      auto action = actionInput.value();
      eOff = eOff + (objTransform.linear() * action.pre_offset);
    }

    // Compute offset
    Eigen::Isometry3d eeTransform =
        mAda->getEndEffectorBodyNode()->getWorldTransform();
    Eigen::Vector3d eOffset =
        objTransform.translation() - eeTransform.translation();
    // Add overshoot and additional offset
    eOffset = eOffset.normalized() * (eOffset.norm() + overshoot) + eOff;

    std::vector<double> offset{eOffset.x(), eOffset.y(), eOffset.z()};

    setOutput("offset", offset);

    return BT::NodeStatus::SUCCESS;
  }

private:
  ada::Ada *mAda;
  ros::NodeHandle *mNode;
};


class ConfigJoints : public BT::SyncActionNode {
public:
  ConfigJoints(const std::string &name, const BT::NodeConfig &config,
                 ada::Ada *robot, ros::NodeHandle *nh)
      : BT::SyncActionNode(name, config), mAda(robot), mNode(nh) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<double>("offset"),
            BT::OutputPort<std::vector<double>>("config")};
  }

  BT::NodeStatus tick() override {
    std::cout << "Moving into object\n";
    // Read Object
    auto objectInput = getInput<double>("offset");
    if (!objectInput) {
      return BT::NodeStatus::FAILURE;
    }
    auto objTransform = objectInput.value();

    // compute config offset
    Eigen::VectorXd currentConfig = mAda->getArm()->getCurrentConfiguration();
    // Compute offset
    std::cout << "Robot config before: " << currentConfig << "\n";
    currentConfig[5] = currentConfig[5] + objTransform;
    std::cout << "Robot config: " << currentConfig << "\n";

    std::vector<double> off(currentConfig.data(), currentConfig.data() + currentConfig.size());
    for(int i=0; i < off.size(); i++)
      std::cout << off.at(i) << ' ';
    setOutput("config", off);

    return BT::NodeStatus::SUCCESS;
  }

private:
  ada::Ada *mAda;
  ros::NodeHandle *mNode;
};





class ConfigTwist : public BT::SyncActionNode {
public:
  ConfigTwist(const std::string &name, const BT::NodeConfig &config,
              ada::Ada *robot, ros::NodeHandle *nh)
      : BT::SyncActionNode(name, config), mAda(robot), mNode(nh) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<AcquisitionAction>("action"),
            BT::InputPort<bool>("is_extraction"),
            BT::InputPort<std::vector<double>>("approach"),
            BT::InputPort<std::vector<double>>("world_off"),
            BT::InputPort<std::vector<double>>("world_rot"),
            BT::InputPort<double>("z_max"),
            BT::InputPort<double>("z_min"),
            BT::InputPort<Eigen::Isometry3d>("obj_transform"),
            BT::InputPort<Eigen::Isometry3d>("pre_offset"),
            BT::InputPort<Eigen::Isometry3d>("translation"),
            BT::InputPort<std::vector<double>>("man_offset"),
            BT::InputPort<std::vector<double>>("man_rotation"),
            BT::OutputPort<std::vector<double>>("offset"),
            BT::OutputPort<std::vector<double>>("rotation"),
            BT::OutputPort<bool>("null_motion")};
  }

  BT::NodeStatus tick() override {
    std::cout << "Moving into object\n";
    // Input arguments
    auto offInput = getInput<std::vector<double>>("world_off");
    Eigen::Vector3d eOff = offInput ? Eigen::Vector3d(offInput.value().data())
                                    : Eigen::Vector3d::Zero();
    auto rotInput = getInput<std::vector<double>>("world_rot");
    Eigen::Vector3d eRot = rotInput ? Eigen::Vector3d(rotInput.value().data())
                                    : Eigen::Vector3d::Zero();

    Eigen::Isometry3d eeTransform =
        mAda->getEndEffectorBodyNode()->getWorldTransform();

    Eigen::Vector3d actionRotation;
    Eigen::Vector3d actionOffset;
    Eigen::Vector3d pre_offset;
    Eigen::Vector3d translation;
    double actionDuration;

    // If action provided, add twist from there
    auto actionInput = getInput<AcquisitionAction>("action");
    auto extractionInput = getInput<bool>("is_extraction");

    if (actionInput) {
      // Unpack action params
      auto action = actionInput.value();
      actionDuration = (extractionInput && extractionInput.value())
                                  ? action.ext_duration
                                  : action.grasp_duration;
      actionRotation =
          actionDuration * ((extractionInput && extractionInput.value())
                                ? action.ext_rot
                                : action.grasp_rot);
      actionOffset =
          actionDuration * ((extractionInput && extractionInput.value())
                                ? action.ext_offset
                                : action.grasp_offset);

      pre_offset = action.pre_offset;
      translation = action.pre_transform.translation();
    }else{ //otherwise, use params provided by node

      std::vector<double> rot = getInput<std::vector<double>>("man_rotation").value();
      actionRotation << rot[0], rot[1], rot[2];
      std::vector<double> off = getInput<std::vector<double>>("man_offset").value();
      actionOffset << off[0], off[1], off[2];
      // pre_offset = getInput<Eigen::Vector3d>("pre_offset").value();
      // translation = getInput<Eigen::Vector3d>("translation").value();
    }


    // Calculate offset
    // Tranform rotation to world frame from EE/utensil frame
    eRot += eeTransform.linear() * actionRotation;

    // Transform offset to world frame from "approach" frame
    // If not provided, impute approach vec from action
    auto approachInput = getInput<std::vector<double>>("approach");
    if (approachInput){
      Eigen::Vector3d approachVec =
        approachInput
            ? Eigen::Vector3d(approachInput.value().data())
            : pre_offset - translation;
    // Address vertical cases:
    // If vertical: default to +Y utensil frame (i.e. flat of fork)
    // If utensil flat (i.e. +Y is also vertical): default to +Z utensil frame
    if (FuzzyZero(approachVec[1]) && FuzzyZero(approachVec[0])) {
      approachVec = eeTransform.linear() * Eigen::Vector3d::UnitY();
    }
    if (FuzzyZero(approachVec[1]) && FuzzyZero(approachVec[0])) {
      approachVec = eeTransform.linear() * Eigen::Vector3d::UnitZ();
    }
    eOff += Eigen::AngleAxisd(atan2(approachVec[1], approachVec[0]),
                              Eigen::Vector3d::UnitZ()) *
            actionOffset;
    }
    else{
      eOff +=  actionOffset;

      std::vector<double> offset{eOff.x(), eOff.y(), eOff.z()};
      std::vector<double> rotation{eRot.x(), eRot.y(), eRot.z()};
      std::cout << "Original position for hardcode: " << eeTransform.translation()   ;
      std::cout << "Final position for hardcode: " << eOff;
      setOutput("offset", offset);
      setOutput("rotation", rotation);
      // setOutput("null_motion", FuzzyZero(eOff.norm()) && FuzzyZero(eRot.norm()));
    
      return BT::NodeStatus::SUCCESS;
    }
    
  

    // Clamp Z
    // if object provided, z relative to object
    auto objectInput = getInput<Eigen::Isometry3d>("obj_transform");
    double objOffset =
        objectInput ? objectInput.value().translation().z() : 0.0;
    double zHeight = eeTransform.translation().z() + eOff.z();
    if (eOff.z() > 0 && !FuzzyZero(eOff.z())) {
      double zMax =
          getInput<double>("z_max")
              ? std::min(getInput<double>("z_max").value() + objOffset, zHeight)
              : zHeight;
      double length =
          std::max(0.0, (zMax - eeTransform.translation().z()) / eOff.z());
      eOff *= length;
    }
    if (eOff.z() < 0 && !FuzzyZero(eOff.z())) {
      double zMin =
          getInput<double>("z_min")
              ? std::max(getInput<double>("z_min").value() + objOffset, zHeight)
              : zHeight;
      double length =
          std::max(0.0, (zMin - eeTransform.translation().z()) / eOff.z());
      eOff *= length;
    }
    std::cout<<"Original pose : ";
    std::cout << eeTransform.translation();
    std::cout << "\n Original rot: ";
    std::cout << eeTransform.linear();

    if (actionInput){
      auto action = actionInput.value();
      if (action.action_type.compare("pre_manip") == 0){
        eOff = (extractionInput && extractionInput.value())
                                  ? action.ext_offset
                                  : action.grasp_offset;
        eRot = (extractionInput && extractionInput.value())
                                  ? action.ext_rot
                                  : action.grasp_rot;
      }
        
    }
    std::cout<<"\nFinal pose ";
    std::cout << eOff;
    std::cout << "\n Final rot";
    std::cout << eRot;

    std::vector<double> offset{eOff.x(), eOff.y(), eOff.z()};
    std::vector<double> rotation{eRot.x(), eRot.y(), eRot.z()};

    setOutput("offset", offset);
    setOutput("rotation", rotation);
    setOutput("null_motion", FuzzyZero(eOff.norm()) && FuzzyZero(eRot.norm()));
    return BT::NodeStatus::SUCCESS;
  }

private:
  ada::Ada *mAda;
  ros::NodeHandle *mNode;
};

/// Node registration
static void registerNodes(BT::BehaviorTreeFactory &factory, ros::NodeHandle &nh,
                          ada::Ada &robot) {
  factory.registerNodeType<ConfigMoveAbove>("ConfigMoveAbove", &robot, &nh);
  factory.registerNodeType<ConfigMoveInto>("ConfigMoveInto", &robot, &nh);
  factory.registerNodeType<ConfigTwist>("ConfigTwist", &robot, &nh);
  factory.registerNodeType<ConfigJoints>("ConfigJoints", &robot, &nh);
}
static_block { feeding::registerNodeFn(&registerNodes); }

} // end namespace nodes
} // end namespace feeding
