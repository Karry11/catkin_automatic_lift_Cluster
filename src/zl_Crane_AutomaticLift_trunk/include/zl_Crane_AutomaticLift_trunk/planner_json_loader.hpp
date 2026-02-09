#pragma once

#include <string>

#include <zl_manipulation/AutoCraneArmManipulation.h>

namespace zl_Crane_AutomaticLift_trunk {

// Load planner JSON output and convert to joint trajectory.
// Returns false on error and sets err if provided.
bool BuildTrajectoryFromPlannerJson(const std::string& json_path,
                                    ompl::app::joint_trajectory_t* out,
                                    double dt_sec,
                                    std::string* err);

}  // namespace zl_Crane_AutomaticLift_trunk
