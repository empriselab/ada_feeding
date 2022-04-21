#ifndef FEEDING_ACTION_CUT_HPP_
#define FEEDING_ACTION_CUT_HPP_

#include <libada/Ada.hpp>

#include "feeding/FeedingDemo.hpp"
#include "feeding/TargetItem.hpp"
#include "feeding/Workspace.hpp"
#include "feeding/perception/Perception.hpp"

#include "feeding/FTThresholdHelper.hpp"

namespace feeding {
namespace action {

bool moveFork(const std::shared_ptr<Perception> &perception, TargetItem item,
              const Eigen::Vector3d &endEffectorDirection, const Eigen::Vector3d &moveDir,
              FeedingDemo *feedingDemo,std::string controller);

} // namespace action
} // namespace feeding
#endif
