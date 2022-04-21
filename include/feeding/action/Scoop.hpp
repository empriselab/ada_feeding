#ifndef FEEDING_ACTION_SCOOP_HPP_
#define FEEDING_ACTION_SCOOP_HPP_

#include <libada/Ada.hpp>

#include "feeding/Workspace.hpp"

#include "feeding/action/MoveAbovePlate.hpp"

#include "feeding/action/MoveAbove.hpp"

namespace feeding {
namespace action {

bool scoop(FeedingDemo *feedingDemo, double angle);
// angle is the degree to rotate the 6th motor by
} // namespace action
} // namespace feeding

#endif