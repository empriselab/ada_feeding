#ifndef FEEDING_ACTION_POSITIONFORK_HPP_
#define FEEDING_ACTION_POSITIONFORK_HPP_

#include <libada/Ada.hpp>
#include "feeding/action/MoveAbove.hpp"
#include "feeding/FeedingDemo.hpp"
#include "feeding/Workspace.hpp"

namespace feeding {
namespace action {

void positionFork(std::string foodName, const Eigen::Isometry3d &foodTransform,
                   float rotateAngle, TiltStyle tiltStyle,
                   double rotationTolerance, FeedingDemo *feedingDemo,
                   double *angleGuess);

} // namespace action
} // namespace feeding

#endif
