#ifndef FEEDING_ACTION_POSITIONFORK_HPP_
#define FEEDING_ACTION_POSITIONFORK_HPP_

#include <libada/Ada.hpp>
#include "feeding/action/MoveAbove.hpp"
#include "feeding/FeedingDemo.hpp"
#include "feeding/Workspace.hpp"


namespace feeding {
namespace action {

void positionFork(FeedingDemo *feedingDemo, float horizontalTolerance=0.003, 
float verticalTolerance=0.008, float rotationTolerance=0.5);

} // namespace action
} // namespace feeding

#endif
