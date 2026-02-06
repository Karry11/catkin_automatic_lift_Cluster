#pragma once

#include <string>

#include <nlohmann/json.hpp>

#include "geometry.h"
#include "kinematics.h"

namespace path_generate {

struct PlannerConfig {
    double d_min_m{1.0};
    double delta_psi_deg{2.0};
    double delta_theta_deg{1.0};
    double delta_l_m{0.5};
    double eps_r_m{1e-4};
    bool search_pre_hoist{true};
    double pre_hoist_step_m{1.0};
    int pre_hoist_max_candidates{250};
    int waypoint_pre_hoist_max_candidates{60};
    int waypoint_max_nodes{60};
    bool auto_waypoints{true};
    int auto_waypoint_rounds{3};
    int auto_waypoint_max_candidates{120};
    double auto_psi_step_deg{30.0};
    double auto_theta_step_deg{5.0};
    double auto_rope_step_m{5.0};
    int auto_neighbor_k{18};
    bool optimize_min_segments{true};
    double cost_psi_deg{1.0};
    double cost_theta_deg{1.0};
    double cost_rope_m{1.0};
    double boom_clearance_m{0.0};
    double rope_clearance_m{0.0};
    double ground_z_min_m{0.0};

    void validate() const;
};

struct PlanResult {
    bool ok{false};
    nlohmann::json payload;
};

PlanResult plan_from_dict(const nlohmann::json& data);
PlanResult plan_from_json(const std::string& text);

}  // namespace path_generate
