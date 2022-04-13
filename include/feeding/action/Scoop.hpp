#ifndef FEEDING_ACTION_SCOOP_HPP_
#define FEEDING_ACTION_SCOOP_HPP_

#include <libada/Ada.hpp>

#include "feeding/Workspace.hpp"

#include "feeding/action/MoveAbovePlate.hpp"

#include "feeding/action/MoveAbove.hpp"

namespace feeding {
namespace action {

bool scoop(FeedingDemo *feedingDemo);

} // namespace action
} // namespace feeding

#endif