#include "planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace path_generate {

using json = nlohmann::json;

struct TrajectoryResult {
    std::vector<json> traj;
    bool ok{false};
    std::string err;
};

struct ConfigsResult {
    bool ok{false};
    std::vector<Config> configs;
    std::string err;
};

struct SegmentPlanResult {
    bool ok{false};
    std::vector<json> actions;
    std::vector<Config> configs;
    std::string err;
};

struct GraphSearchResult {
    bool ok{false};
    std::vector<Config> configs;
    std::vector<int> node_path;
    std::vector<int> node_ks;
    std::string err;
};

void PlannerConfig::validate() const {
    if (d_min_m < 0.0) {
        throw std::invalid_argument("d_min_m must be >= 0.");
    }
    if (delta_psi_deg <= 0.0 || delta_theta_deg <= 0.0 || delta_l_m <= 0.0) {
        throw std::invalid_argument("Step sizes must be > 0.");
    }
    if (pre_hoist_step_m <= 0.0) {
        throw std::invalid_argument("pre_hoist_step_m must be > 0.");
    }
    if (pre_hoist_max_candidates <= 0) {
        throw std::invalid_argument("pre_hoist_max_candidates must be > 0.");
    }
    if (waypoint_pre_hoist_max_candidates <= 0) {
        throw std::invalid_argument("waypoint_pre_hoist_max_candidates must be > 0.");
    }
    if (waypoint_max_nodes <= 1) {
        throw std::invalid_argument("waypoint_max_nodes must be > 1.");
    }
    if (auto_waypoint_rounds <= 0) {
        throw std::invalid_argument("auto_waypoint_rounds must be > 0.");
    }
    if (auto_waypoint_max_candidates <= 0) {
        throw std::invalid_argument("auto_waypoint_max_candidates must be > 0.");
    }
    if (auto_psi_step_deg <= 0.0 || auto_theta_step_deg <= 0.0 || auto_rope_step_m <= 0.0) {
        throw std::invalid_argument("auto_* step sizes must be > 0.");
    }
    if (auto_neighbor_k <= 0) {
        throw std::invalid_argument("auto_neighbor_k must be > 0.");
    }
    if (cost_psi_deg < 0.0 || cost_theta_deg < 0.0 || cost_rope_m < 0.0) {
        throw std::invalid_argument("cost weights must be >= 0.");
    }
    if (boom_clearance_m < 0.0) {
        throw std::invalid_argument("boom_clearance_m must be >= 0.");
    }
    if (rope_clearance_m < 0.0) {
        throw std::invalid_argument("rope_clearance_m must be >= 0.");
    }
    if (!std::isfinite(ground_z_min_m)) {
        throw std::invalid_argument("ground_z_min_m must be finite.");
    }
}

static std::pair<bool, double> is_safe_point(const Vec3& point, const std::vector<AABB>& obstacles, double d_min_m) {
    double d = nearest_distance_to_obstacles(point, obstacles);
    return {d >= d_min_m, d};
}

static bool is_ground_safe(const Vec3& hook_xyz, double z_min_m) {
    return hook_xyz[2] >= z_min_m - 1e-9;
}

static Vec3 boom_tip_from_hook(const Vec3& hook_xyz, double rope_m) {
    return {hook_xyz[0], hook_xyz[1], hook_xyz[2] + rope_m};
}

static std::pair<bool, double> boom_is_safe(
    const Vec3& base_xyz,
    const Vec3& tip_xyz,
    const std::vector<AABB>& obstacles,
    double clearance_m) {
    for (const auto& obs : obstacles) {
        if (segment_intersects_aabb(base_xyz, tip_xyz, obs)) {
            return {false, 0.0};
        }
    }
    double d = nearest_segment_distance_to_obstacles(base_xyz, tip_xyz, obstacles);
    if (clearance_m > 0.0 && d < clearance_m) {
        return {false, d};
    }
    return {true, d};
}

static std::pair<bool, double> rope_is_safe(
    const Vec3& hook_xyz,
    const Vec3& tip_xyz,
    const std::vector<AABB>& obstacles,
    double clearance_m) {
    for (const auto& obs : obstacles) {
        if (segment_intersects_aabb(hook_xyz, tip_xyz, obs)) {
            return {false, 0.0};
        }
    }
    double d = nearest_segment_distance_to_obstacles(hook_xyz, tip_xyz, obstacles);
    if (clearance_m > 0.0 && d < clearance_m) {
        return {false, d};
    }
    return {true, d};
}

static TrajectoryResult configs_to_trajectory(
    const CraneModel& model,
    const std::vector<AABB>& obstacles,
    double d_min_m,
    const std::vector<Config>& configs,
    double boom_clearance_m,
    double rope_clearance_m,
    double ground_z_min_m) {
    TrajectoryResult result;
    result.ok = true;

    for (size_t idx = 0; idx < configs.size(); ++idx) {
        const Config& cfg = configs[idx];
        Vec3 xyz = forward_hook_xyz(model, cfg);
        if (!is_ground_safe(xyz, ground_z_min_m)) {
            result.ok = false;
            result.err = "Ground plane violation at k=" + std::to_string(idx) +
                " (z=" + std::to_string(xyz[2]) + "m < " + std::to_string(ground_z_min_m) + "m).";
            return result;
        }
        auto safe_pair = is_safe_point(xyz, obstacles, d_min_m);
        bool safe = safe_pair.first;
        double d_nearest = safe_pair.second;
        if (!safe) {
            result.ok = false;
            result.err = "Collision or clearance violation at k=" + std::to_string(idx) +
                " (d_nearest=" + std::to_string(d_nearest) + "m).";
            return result;
        }
        Vec3 tip_xyz = boom_tip_from_hook(xyz, cfg.rope_m);
        auto boom_pair = boom_is_safe(model.base_xyz_m, tip_xyz, obstacles, boom_clearance_m);
        bool boom_ok = boom_pair.first;
        double boom_d = boom_pair.second;
        if (!boom_ok) {
            result.ok = false;
            result.err = "Boom collision/clearance violation at k=" + std::to_string(idx) +
                " (d_boom=" + std::to_string(boom_d) + "m).";
            return result;
        }
        auto rope_pair = rope_is_safe(xyz, tip_xyz, obstacles, rope_clearance_m);
        bool rope_ok = rope_pair.first;
        double rope_d = rope_pair.second;
        if (!rope_ok) {
            result.ok = false;
            result.err = "Rope collision/clearance violation at k=" + std::to_string(idx) +
                " (d_rope=" + std::to_string(rope_d) + "m).";
            return result;
        }

        json step = json::object();
        step["k"] = static_cast<int>(idx);
        step["pos"] = {xyz[0], xyz[1], xyz[2]};
        step["psi_deg"] = normalize_deg(cfg.psi_deg);
        step["theta_deg"] = cfg.theta_deg;
        step["l_rope_m"] = cfg.rope_m;
        step["d_nearest_m"] = std::isfinite(d_nearest) ? json(d_nearest) : json(nullptr);
        step["d_boom_nearest_m"] = std::isfinite(boom_d) ? json(boom_d) : json(nullptr);
        step["d_rope_nearest_m"] = std::isfinite(rope_d) ? json(rope_d) : json(nullptr);
        result.traj.push_back(std::move(step));
    }

    return result;
}

static std::vector<Config> sample_slew(const Config& start, double end_psi_deg, double step_deg, const char* direction) {
    double delta = angular_delta_deg(start.psi_deg, end_psi_deg, direction);
    int steps = (std::abs(delta) > 1e-12) ? static_cast<int>(std::ceil(std::abs(delta) / step_deg)) : 0;
    if (steps == 0) {
        return {};
    }
    std::vector<Config> out;
    out.reserve(static_cast<size_t>(steps));
    for (int i = 1; i <= steps; ++i) {
        double psi = normalize_deg(start.psi_deg + (delta * static_cast<double>(i) / steps));
        out.push_back(Config{psi, start.theta_deg, start.rope_m});
    }
    return out;
}

static std::vector<Config> sample_hoist(const Config& start, double end_rope_m, double step_m) {
    double delta = end_rope_m - start.rope_m;
    int steps = (std::abs(delta) > 1e-12) ? static_cast<int>(std::ceil(std::abs(delta) / step_m)) : 0;
    if (steps == 0) {
        return {};
    }
    std::vector<Config> out;
    out.reserve(static_cast<size_t>(steps));
    for (int i = 1; i <= steps; ++i) {
        double rope = start.rope_m + (delta * static_cast<double>(i) / steps);
        out.push_back(Config{start.psi_deg, start.theta_deg, rope});
    }
    return out;
}

static std::vector<Config> sample_luff(const Config& start, double end_theta_deg, double step_deg) {
    double delta = end_theta_deg - start.theta_deg;
    int steps = (std::abs(delta) > 1e-12) ? static_cast<int>(std::ceil(std::abs(delta) / step_deg)) : 0;
    if (steps == 0) {
        return {};
    }
    std::vector<Config> out;
    out.reserve(static_cast<size_t>(steps));
    for (int i = 1; i <= steps; ++i) {
        double theta = start.theta_deg + (delta * static_cast<double>(i) / steps);
        out.push_back(Config{start.psi_deg, theta, start.rope_m});
    }
    return out;
}

static std::vector<Config> build_three_phase_configs(
    const PlannerConfig& planner,
    const Config& start,
    const Config& goal,
    const char* slew_dir) {
    std::vector<Config> configs;
    configs.push_back(start);
    auto slew = sample_slew(configs.back(), goal.psi_deg, planner.delta_psi_deg, slew_dir);
    configs.insert(configs.end(), slew.begin(), slew.end());
    auto hoist = sample_hoist(configs.back(), goal.rope_m, planner.delta_l_m);
    configs.insert(configs.end(), hoist.begin(), hoist.end());
    auto luff = sample_luff(configs.back(), goal.theta_deg, planner.delta_theta_deg);
    configs.insert(configs.end(), luff.begin(), luff.end());
    if (!(configs.back() == goal)) {
        configs.push_back(goal);
    }
    return configs;
}

static ConfigsResult try_three_phase_only(
    const CraneModel& model,
    const std::vector<AABB>& obstacles,
    const PlannerConfig& planner,
    const Config& start,
    const Config& goal) {
    planner.validate();
    const char* candidate_slew_dirs[] = {"shortest", "cw", "ccw"};
    std::string best_err;

    for (const char* slew_dir : candidate_slew_dirs) {
        std::vector<Config> configs = build_three_phase_configs(planner, start, goal, slew_dir);
        TrajectoryResult traj = configs_to_trajectory(
            model,
            obstacles,
            planner.d_min_m,
            configs,
            planner.boom_clearance_m,
            planner.rope_clearance_m,
            planner.ground_z_min_m);
        if (traj.ok) {
            return {true, configs, std::string()};
        }
        if (!traj.err.empty()) {
            best_err = traj.err;
        }
    }

    if (best_err.empty()) {
        best_err = "No feasible three-phase plan found.";
    }
    return {false, {}, best_err};
}

static double signed_angle_step_deg(double prev_deg, double cur_deg) {
    return std::fmod(cur_deg - prev_deg + 540.0, 360.0) - 180.0;
}

static double movement_cost(const std::vector<Config>& configs, const PlannerConfig& planner) {
    double cost = 0.0;
    for (size_t i = 0; i + 1 < configs.size(); ++i) {
        const Config& a = configs[i];
        const Config& b = configs[i + 1];
        cost += std::abs(signed_angle_step_deg(a.psi_deg, b.psi_deg)) * planner.cost_psi_deg;
        cost += std::abs(b.theta_deg - a.theta_deg) * planner.cost_theta_deg;
        cost += std::abs(b.rope_m - a.rope_m) * planner.cost_rope_m;
    }
    return cost;
}

static std::string classify_action(const Config& a, const Config& b) {
    double psi_delta = std::fmod(b.psi_deg - a.psi_deg, 360.0);
    if (psi_delta < 0.0) {
        psi_delta += 360.0;
    }
    if (std::abs(psi_delta) > 1e-9 &&
        std::abs(b.theta_deg - a.theta_deg) < 1e-9 &&
        std::abs(b.rope_m - a.rope_m) < 1e-9) {
        return "slew";
    }
    if (std::abs(b.rope_m - a.rope_m) > 1e-9 &&
        std::abs(b.psi_deg - a.psi_deg) < 1e-9 &&
        std::abs(b.theta_deg - a.theta_deg) < 1e-9) {
        return "hoist";
    }
    return "luff";
}

static json make_action(
    const std::vector<Config>& configs,
    int start_idx,
    int end_idx,
    const std::string& action_type,
    const PlannerConfig& planner) {
    const Config& a = configs[static_cast<size_t>(start_idx)];
    const Config& b = configs[static_cast<size_t>(end_idx)];
    if (action_type == "slew") {
        double delta = angular_delta_deg(a.psi_deg, b.psi_deg, "shortest");
        std::string direction = (delta >= 0.0) ? "cw" : "ccw";
        return json{
            {"type", "slew"},
            {"start_deg", normalize_deg(a.psi_deg)},
            {"end_deg", normalize_deg(b.psi_deg)},
            {"direction", direction},
            {"theta_hold_deg", a.theta_deg},
            {"l_hold_m", a.rope_m},
            {"step_deg", planner.delta_psi_deg},
            {"k_start", start_idx},
            {"k_end", end_idx},
            {"reason", "Slew with obstacle clearance constraint."}
        };
    }
    if (action_type == "hoist") {
        std::string direction = (b.rope_m < a.rope_m) ? "up" : "down";
        return json{
            {"type", "hoist"},
            {"start_m", a.rope_m},
            {"end_m", b.rope_m},
            {"direction", direction},
            {"psi_hold_deg", normalize_deg(a.psi_deg)},
            {"theta_hold_deg", a.theta_deg},
            {"step_m", planner.delta_l_m},
            {"k_start", start_idx},
            {"k_end", end_idx},
            {"reason", "Hoist with obstacle clearance constraint."},
            {"note", nullptr}
        };
    }
    std::string direction = (b.theta_deg > a.theta_deg) ? "up" : "down";
    return json{
        {"type", "luff"},
        {"start_deg", a.theta_deg},
        {"end_deg", b.theta_deg},
        {"direction", direction},
        {"psi_hold_deg", normalize_deg(a.psi_deg)},
        {"l_hold_m", a.rope_m},
        {"step_deg", planner.delta_theta_deg},
        {"k_start", start_idx},
        {"k_end", end_idx},
        {"reason", "Luff with obstacle clearance constraint."}
    };
}

static std::vector<json> actions_from_configs(const std::vector<Config>& configs, const PlannerConfig& planner) {
    if (configs.size() < 2) {
        return {};
    }

    std::vector<json> actions;
    std::string current_type = classify_action(configs[0], configs[1]);
    int seg_start_idx = 0;

    for (size_t i = 1; i + 1 < configs.size(); ++i) {
        std::string t = classify_action(configs[i], configs[i + 1]);
        if (t != current_type) {
            actions.push_back(make_action(configs, seg_start_idx, static_cast<int>(i), current_type, planner));
            seg_start_idx = static_cast<int>(i);
            current_type = t;
        }
    }
    actions.push_back(make_action(
        configs,
        seg_start_idx,
        static_cast<int>(configs.size() - 1),
        current_type,
        planner));

    int slew_idx = -1;
    for (size_t i = 0; i < actions.size(); ++i) {
        if (actions[i].at("type") == "slew") {
            slew_idx = static_cast<int>(i);
            break;
        }
    }
    if (slew_idx > 0) {
        bool has_hoist_after = false;
        for (size_t i = static_cast<size_t>(slew_idx + 1); i < actions.size(); ++i) {
            if (actions[i].at("type") == "hoist") {
                has_hoist_after = true;
                break;
            }
        }
        if (has_hoist_after && actions[static_cast<size_t>(slew_idx - 1)].at("type") == "hoist") {
            json& target = actions[static_cast<size_t>(slew_idx - 1)];
            if (target["note"].is_null()) {
                target["note"] = "Pre-hoist inserted for clearance before slew.";
            }
        }
    }

    return actions;
}

static SegmentPlanResult try_plan_segments(
    const CraneModel& model,
    const std::vector<AABB>& obstacles,
    const PlannerConfig& planner,
    const Config& start,
    const Config& goal,
    bool use_pre_hoist) {
    planner.validate();
    const char* candidate_slew_dirs[] = {"shortest", "cw", "ccw"};

    auto build_three_phase = [&](const char* slew_dir) -> SegmentPlanResult {
        SegmentPlanResult out;
        std::vector<Config> configs = build_three_phase_configs(planner, start, goal, slew_dir);
        TrajectoryResult traj = configs_to_trajectory(
            model,
            obstacles,
            planner.d_min_m,
            configs,
            planner.boom_clearance_m,
            planner.rope_clearance_m,
            planner.ground_z_min_m);
        if (!traj.ok) {
            out.ok = false;
            out.err = traj.err;
            return out;
        }
        out.ok = true;
        out.configs = std::move(configs);
        out.actions = actions_from_configs(out.configs, planner);
        return out;
    };

    std::string best_err;
    for (const char* slew_dir : candidate_slew_dirs) {
        SegmentPlanResult res = build_three_phase(slew_dir);
        if (res.ok) {
            return res;
        }
        if (!res.err.empty()) {
            best_err = res.err;
        }
    }

    if (!use_pre_hoist) {
        if (best_err.empty()) {
            best_err = "No feasible plan found.";
        }
        return {false, {}, {}, best_err};
    }

    double l_min = model.limits.l_min_m;
    double l_max = model.limits.l_max_m;
    double step = planner.pre_hoist_step_m;

    std::vector<double> candidates;
    int k = 0;
    while (static_cast<int>(candidates.size()) < planner.pre_hoist_max_candidates) {
        double down = start.rope_m - k * step;
        double up = start.rope_m + k * step;
        if (k == 0) {
            candidates.push_back(start.rope_m);
        } else {
            if (down >= l_min) {
                candidates.push_back(down);
            }
            if (up <= l_max) {
                candidates.push_back(up);
            }
        }
        if (down < l_min && up > l_max) {
            break;
        }
        ++k;
    }

    for (double l_mid : candidates) {
        for (const char* slew_dir : candidate_slew_dirs) {
            std::vector<Config> configs;
            configs.push_back(start);
            auto h1 = sample_hoist(configs.back(), l_mid, planner.delta_l_m);
            configs.insert(configs.end(), h1.begin(), h1.end());
            auto s1 = sample_slew(configs.back(), goal.psi_deg, planner.delta_psi_deg, slew_dir);
            configs.insert(configs.end(), s1.begin(), s1.end());
            auto h2 = sample_hoist(configs.back(), goal.rope_m, planner.delta_l_m);
            configs.insert(configs.end(), h2.begin(), h2.end());
            auto luff = sample_luff(configs.back(), goal.theta_deg, planner.delta_theta_deg);
            configs.insert(configs.end(), luff.begin(), luff.end());
            if (!(configs.back() == goal)) {
                configs.push_back(goal);
            }

            TrajectoryResult traj = configs_to_trajectory(
                model,
                obstacles,
                planner.d_min_m,
                configs,
                planner.boom_clearance_m,
                planner.rope_clearance_m,
                planner.ground_z_min_m);
            if (!traj.ok) {
                if (!traj.err.empty()) {
                    best_err = traj.err;
                }
                continue;
            }
            SegmentPlanResult out;
            out.ok = true;
            out.configs = std::move(configs);
            out.actions = actions_from_configs(out.configs, planner);
            for (auto& action : out.actions) {
                if (action.at("type") == "hoist" && action["note"].is_null()) {
                    action["note"] = "Pre-hoist inserted for clearance before slew.";
                    break;
                }
            }
            return out;
        }
    }

    if (best_err.empty()) {
        best_err = "No feasible plan found (including pre-hoist search).";
    }
    return {false, {}, {}, best_err};
}

static ConfigsResult try_segment(
    const CraneModel& model,
    const std::vector<AABB>& obstacles,
    const PlannerConfig& planner,
    const Config& start,
    const Config& goal,
    bool allow_pre_hoist) {
    if (!allow_pre_hoist) {
        return try_three_phase_only(model, obstacles, planner, start, goal);
    }
    PlannerConfig edge_planner = planner;
    edge_planner.pre_hoist_max_candidates = std::min(
        planner.pre_hoist_max_candidates,
        planner.waypoint_pre_hoist_max_candidates);
    SegmentPlanResult seg = try_plan_segments(model, obstacles, edge_planner, start, goal, true);
    if (!seg.ok) {
        std::string err = seg.err.empty() ? "No feasible segment found." : seg.err;
        return {false, {}, err};
    }
    return {true, seg.configs, std::string()};
}

static double vec_distance(const Vec3& a, const Vec3& b) {
    double dx = a[0] - b[0];
    double dy = a[1] - b[1];
    double dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

static long long py_round(double value) {
    return static_cast<long long>(std::nearbyint(value));
}

struct Candidate {
    double x;
    double y;
    double z;
    Config cfg;
    double d;
};

static std::pair<std::vector<Config>, std::vector<Vec3>> generate_auto_waypoint_candidates(
    const CraneModel& model,
    const std::vector<AABB>& obstacles,
    const PlannerConfig& planner,
    const Config& start_cfg,
    const Config& goal_cfg) {
    Vec3 start_xyz = forward_hook_xyz(model, start_cfg);
    Vec3 goal_xyz = forward_hook_xyz(model, goal_cfg);

    auto safe_cfg = [&](const Config& cfg) -> std::tuple<bool, Vec3, double> {
        Vec3 xyz = forward_hook_xyz(model, cfg);
        if (!is_ground_safe(xyz, planner.ground_z_min_m)) {
            return std::make_tuple(false, xyz, 0.0);
        }
        double d = nearest_distance_to_obstacles(xyz, obstacles);
        if (d < planner.d_min_m) {
            return std::make_tuple(false, xyz, d);
        }
        Vec3 tip = boom_tip_from_hook(xyz, cfg.rope_m);
        auto boom_pair = boom_is_safe(model.base_xyz_m, tip, obstacles, planner.boom_clearance_m);
        bool boom_ok = boom_pair.first;
        if (!boom_ok) {
            return std::make_tuple(false, xyz, d);
        }
        auto rope_pair = rope_is_safe(xyz, tip, obstacles, planner.rope_clearance_m);
        bool rope_ok = rope_pair.first;
        return std::make_tuple(rope_ok, xyz, d);
    };

    auto xyz_key = [&](const Vec3& xyz) {
        return std::make_tuple(
            static_cast<int>(py_round(xyz[0] * 100.0)),
            static_cast<int>(py_round(xyz[1] * 100.0)),
            static_cast<int>(py_round(xyz[2] * 100.0)));
    };

    auto unique_add = [&](
        std::vector<Candidate>& out,
        std::set<std::tuple<int, int, int>>& seen,
        const Config& cfg,
        const Vec3& xyz,
        double d) {
        auto key = xyz_key(xyz);
        if (seen.count(key) > 0) {
            return;
        }
        if (vec_distance(xyz, start_xyz) < 1e-8 || vec_distance(xyz, goal_xyz) < 1e-8) {
            return;
        }
        seen.insert(key);
        out.push_back(Candidate{xyz[0], xyz[1], xyz[2], cfg, d});
    };

    std::vector<Candidate> candidates;
    std::vector<Candidate> must;
    std::set<std::tuple<int, int, int>> seen_xyz;

    std::vector<double> seed_psis = {normalize_deg(start_cfg.psi_deg), normalize_deg(goal_cfg.psi_deg)};
    std::vector<double> seed_ropes = {start_cfg.rope_m, goal_cfg.rope_m};

    std::vector<double> theta_vals;
    double t = model.limits.theta_min_deg;
    while (t <= model.limits.theta_max_deg + 1e-9) {
        theta_vals.push_back(t);
        t += planner.auto_theta_step_deg;
    }
    theta_vals.push_back(start_cfg.theta_deg);
    theta_vals.push_back(goal_cfg.theta_deg);
    std::sort(theta_vals.begin(), theta_vals.end());
    theta_vals.erase(std::unique(theta_vals.begin(), theta_vals.end()), theta_vals.end());

    for (double psi : seed_psis) {
        for (double theta : theta_vals) {
            for (double rope : seed_ropes) {
                Config cfg{psi, theta, rope};
                auto safe_tuple = safe_cfg(cfg);
                bool ok = std::get<0>(safe_tuple);
                Vec3 xyz = std::get<1>(safe_tuple);
                double d = std::get<2>(safe_tuple);
                if (ok) {
                    unique_add(must, seen_xyz, cfg, xyz, d);
                }
            }
        }
    }

    for (int round_idx = 0; round_idx < planner.auto_waypoint_rounds; ++round_idx) {
        double psi_step = planner.auto_psi_step_deg / std::pow(2.0, round_idx);
        double theta_step = planner.auto_theta_step_deg / std::pow(2.0, round_idx);
        double rope_step = planner.auto_rope_step_m / std::pow(2.0, round_idx);

        std::set<double> psi_values = {normalize_deg(start_cfg.psi_deg), normalize_deg(goal_cfg.psi_deg)};
        int steps = static_cast<int>(std::ceil(360.0 / psi_step));
        for (int i = 0; i < steps; ++i) {
            psi_values.insert(normalize_deg(i * psi_step));
        }

        std::set<double> theta_values = {start_cfg.theta_deg, goal_cfg.theta_deg};
        t = model.limits.theta_min_deg;
        while (t <= model.limits.theta_max_deg + 1e-9) {
            theta_values.insert(t);
            t += theta_step;
        }

        std::set<double> rope_values = {start_cfg.rope_m, goal_cfg.rope_m};
        for (double base : {start_cfg.rope_m, goal_cfg.rope_m}) {
            for (int k = 1; k < 5; ++k) {
                rope_values.insert(base + k * rope_step);
                rope_values.insert(base - k * rope_step);
            }
        }
        rope_values.insert(model.limits.l_min_m);
        rope_values.insert(model.limits.l_max_m);

        for (double psi : psi_values) {
            for (double theta : theta_values) {
                if (!(model.limits.theta_min_deg - 1e-9 <= theta && theta <= model.limits.theta_max_deg + 1e-9)) {
                    continue;
                }
                for (double rope : rope_values) {
                    if (!(model.limits.l_min_m - 1e-9 <= rope && rope <= model.limits.l_max_m + 1e-9)) {
                        continue;
                    }
                    Config cfg{psi, theta, rope};
                    auto safe_tuple = safe_cfg(cfg);
                    bool ok = std::get<0>(safe_tuple);
                    Vec3 xyz = std::get<1>(safe_tuple);
                    double d = std::get<2>(safe_tuple);
                    if (!ok) {
                        continue;
                    }
                    unique_add(candidates, seen_xyz, cfg, xyz, d);
                }
            }
        }
    }

    std::vector<Candidate> combined = must;
    combined.insert(combined.end(), candidates.begin(), candidates.end());
    std::sort(combined.begin(), combined.end(), [&](const Candidate& a, const Candidate& b) {
        double score_a = a.d - 0.002 * vec_distance({a.x, a.y, a.z}, goal_xyz);
        double score_b = b.d - 0.002 * vec_distance({b.x, b.y, b.z}, goal_xyz);
        return score_a > score_b;
    });

    const int per_bin_limit = 2;
    std::map<std::pair<int, int>, int> bins;
    std::vector<Config> cfgs;
    std::vector<Vec3> xyzs;
    std::set<std::tuple<int, int, int>> selected_xyz;

    for (const auto& item : must) {
        Vec3 xyz{item.x, item.y, item.z};
        auto key = xyz_key(xyz);
        if (selected_xyz.count(key) > 0) {
            continue;
        }
        cfgs.push_back(item.cfg);
        xyzs.push_back(xyz);
        selected_xyz.insert(key);
        if (static_cast<int>(cfgs.size()) >= planner.auto_waypoint_max_candidates) {
            return {cfgs, xyzs};
        }
    }

    for (const auto& item : combined) {
        Vec3 xyz{item.x, item.y, item.z};
        auto key = xyz_key(xyz);
        if (selected_xyz.count(key) > 0) {
            continue;
        }
        double psi_norm = normalize_deg(item.cfg.psi_deg);
        int psi_bin = static_cast<int>(py_round(psi_norm / std::max(planner.auto_psi_step_deg, 1e-6)));
        int theta_bin = static_cast<int>(py_round(item.cfg.theta_deg / std::max(planner.auto_theta_step_deg, 1e-6)));
        std::pair<int, int> bin_key{psi_bin, theta_bin};
        if (bins[bin_key] >= per_bin_limit) {
            continue;
        }
        bins[bin_key] += 1;
        cfgs.push_back(item.cfg);
        xyzs.push_back(xyz);
        selected_xyz.insert(key);
        if (static_cast<int>(cfgs.size()) >= planner.auto_waypoint_max_candidates) {
            break;
        }
    }

    return {cfgs, xyzs};
}

static GraphSearchResult plan_with_waypoint_graph_search(
    const CraneModel& model,
    const std::vector<AABB>& obstacles,
    const PlannerConfig& planner,
    const std::vector<Config>& node_cfgs,
    const std::vector<Vec3>& node_xyzs,
    bool allow_pre_hoist,
    bool optimize_min_segments,
    int neighbor_k_opt) {
    GraphSearchResult result;
    int n = static_cast<int>(node_cfgs.size());
    if (n < 2) {
        result.err = "Waypoint search requires at least start and goal.";
        return result;
    }
    if (n > planner.waypoint_max_nodes) {
        result.err = "Too many waypoint nodes (" + std::to_string(n) + "); limit is " +
            std::to_string(planner.waypoint_max_nodes) + ".";
        return result;
    }
    if (static_cast<int>(node_xyzs.size()) != n) {
        result.err = "Internal error: node_xyzs length mismatch.";
        return result;
    }

    std::map<std::pair<int, int>, ConfigsResult> edge_cache;
    auto get_edge = [&](int i, int j) -> ConfigsResult {
        std::pair<int, int> key{i, j};
        auto it = edge_cache.find(key);
        if (it != edge_cache.end()) {
            return it->second;
        }
        ConfigsResult cfgs = try_segment(model, obstacles, planner, node_cfgs[i], node_cfgs[j], allow_pre_hoist);
        edge_cache.emplace(key, cfgs);
        return cfgs;
    };

    int goal_idx = n - 1;
    int neighbor_k = n - 1;
    if (neighbor_k_opt >= 0) {
        neighbor_k = std::min(neighbor_k_opt, n - 1);
    }

    auto neighbor_order = [&](int cur) {
        std::vector<std::pair<double, int>> dists;
        dists.reserve(static_cast<size_t>(n));
        Vec3 cur_xyz = node_xyzs[static_cast<size_t>(cur)];
        for (int j = 0; j < n; ++j) {
            if (j == cur) {
                continue;
            }
            Vec3 xyz = node_xyzs[static_cast<size_t>(j)];
            double dx = xyz[0] - cur_xyz[0];
            double dy = xyz[1] - cur_xyz[1];
            double dz = xyz[2] - cur_xyz[2];
            double d2 = dx * dx + dy * dy + dz * dz;
            dists.emplace_back(d2, j);
        }
        std::sort(dists.begin(), dists.end(), [](const auto& a, const auto& b) {
            return a.first < b.first;
        });
        std::vector<int> out;
        int limit = std::min(neighbor_k, static_cast<int>(dists.size()));
        for (int i = 0; i < limit; ++i) {
            int idx = dists[static_cast<size_t>(i)].second;
            if (idx != goal_idx) {
                out.push_back(idx);
            }
        }
        if (goal_idx != cur) {
            out.push_back(goal_idx);
        }
        return out;
    };

    std::vector<int> prev(static_cast<size_t>(n), -1);
    std::vector<std::vector<Config>> prev_edge(static_cast<size_t>(n));
    std::vector<bool> has_edge(static_cast<size_t>(n), false);

    if (optimize_min_segments) {
        std::pair<int, double> inf = {1000000000, std::numeric_limits<double>::infinity()};
        std::vector<std::pair<int, double>> dist(static_cast<size_t>(n), inf);
        dist[0] = {0, 0.0};

        struct Node {
            int segments;
            double cost;
            int idx;
        };
        auto cmp = [](const Node& a, const Node& b) {
            if (a.segments != b.segments) {
                return a.segments > b.segments;
            }
            return a.cost > b.cost;
        };
        std::priority_queue<Node, std::vector<Node>, decltype(cmp)> heap(cmp);
        heap.push({0, 0.0, 0});

        while (!heap.empty()) {
            Node cur = heap.top();
            heap.pop();
            if (cur.segments != dist[static_cast<size_t>(cur.idx)].first ||
                cur.cost != dist[static_cast<size_t>(cur.idx)].second) {
                continue;
            }
            if (cur.idx == goal_idx) {
                break;
            }
            for (int nxt : neighbor_order(cur.idx)) {
                ConfigsResult edge = get_edge(cur.idx, nxt);
                if (!edge.ok) {
                    continue;
                }
                double edge_cost = movement_cost(edge.configs, planner);
                std::pair<int, double> cand{cur.segments + 1, cur.cost + edge_cost};
                if (cand < dist[static_cast<size_t>(nxt)]) {
                    dist[static_cast<size_t>(nxt)] = cand;
                    prev[static_cast<size_t>(nxt)] = cur.idx;
                    prev_edge[static_cast<size_t>(nxt)] = edge.configs;
                    has_edge[static_cast<size_t>(nxt)] = true;
                    heap.push({cand.first, cand.second, nxt});
                }
            }
        }

        if (dist[static_cast<size_t>(goal_idx)] == inf) {
            result.err = "No feasible path found via waypoint search.";
            return result;
        }
    } else {
        std::vector<double> dist(static_cast<size_t>(n), std::numeric_limits<double>::infinity());
        dist[0] = 0.0;
        using Node = std::pair<double, int>;
        auto cmp = [](const Node& a, const Node& b) { return a.first > b.first; };
        std::priority_queue<Node, std::vector<Node>, decltype(cmp)> heap(cmp);
        heap.push({0.0, 0});
        std::set<int> visited;

        while (!heap.empty()) {
            Node top = heap.top();
            heap.pop();
            double cur_cost = top.first;
            int cur = top.second;
            if (visited.count(cur) > 0) {
                continue;
            }
            visited.insert(cur);
            if (cur == goal_idx) {
                break;
            }
            for (int nxt : neighbor_order(cur)) {
                if (visited.count(nxt) > 0) {
                    continue;
                }
                ConfigsResult edge = get_edge(cur, nxt);
                if (!edge.ok) {
                    continue;
                }
                double edge_cost = movement_cost(edge.configs, planner);
                double new_cost = cur_cost + edge_cost;
                if (new_cost + 1e-12 < dist[static_cast<size_t>(nxt)]) {
                    dist[static_cast<size_t>(nxt)] = new_cost;
                    prev[static_cast<size_t>(nxt)] = cur;
                    prev_edge[static_cast<size_t>(nxt)] = edge.configs;
                    has_edge[static_cast<size_t>(nxt)] = true;
                    heap.push({new_cost, nxt});
                }
            }
        }

        if (prev[static_cast<size_t>(goal_idx)] < 0) {
            result.err = "No feasible path found via waypoint search.";
            return result;
        }
    }

    std::vector<int> node_path;
    int cur = goal_idx;
    while (true) {
        node_path.push_back(cur);
        if (cur == 0) {
            break;
        }
        if (prev[static_cast<size_t>(cur)] < 0) {
            result.err = "Internal error reconstructing waypoint path.";
            return result;
        }
        cur = prev[static_cast<size_t>(cur)];
    }
    std::reverse(node_path.begin(), node_path.end());

    std::vector<Config> full_configs;
    std::vector<int> node_ks;
    node_ks.push_back(0);

    for (size_t idx = 1; idx < node_path.size(); ++idx) {
        int to_node = node_path[idx];
        if (!has_edge[static_cast<size_t>(to_node)]) {
            result.err = "Internal error: missing edge during reconstruction.";
            return result;
        }
        std::vector<Config> edge = prev_edge[static_cast<size_t>(to_node)];
        if (full_configs.empty()) {
            full_configs = edge;
        } else if (!edge.empty()) {
            full_configs.insert(full_configs.end(), edge.begin() + 1, edge.end());
        }
        node_ks.push_back(static_cast<int>(full_configs.size() - 1));
    }

    result.ok = true;
    result.configs = std::move(full_configs);
    result.node_path = std::move(node_path);
    result.node_ks = std::move(node_ks);
    return result;
}

PlanResult plan_from_dict(const json& data) {
    auto fail = [&](const std::string& reason) -> PlanResult {
        PlanResult res;
        res.ok = false;
        res.payload = json{{"status", "failure"}, {"message", "轨迹无法规划"}, {"reason", reason}};
        return res;
    };

    json crane = data.value("crane", json::object());
    if (!crane.contains("boom_length_m")) {
        return fail("Missing required field: crane.boom_length_m");
    }

    double boom_length_m = 0.0;
    Vec3 base_xyz{0.0, 0.0, 0.0};
    try {
        boom_length_m = crane.at("boom_length_m").get<double>();
        if (crane.contains("base_xyz_m")) {
            auto arr = crane.at("base_xyz_m");
            if (!arr.is_array() || arr.size() != 3) {
                return fail("crane.base_xyz_m must be a 3-element array.");
            }
            base_xyz = {arr[0].get<double>(), arr[1].get<double>(), arr[2].get<double>()};
        }
    } catch (...) {
        return fail("Invalid crane parameters (boom_length_m/base_xyz_m).");
    }

    json limits_in = crane.value("limits", json::object());
    Limits limits;
    try {
        limits.psi_min_deg = limits_in.value("psi_min_deg", 0.0);
        limits.psi_max_deg = limits_in.value("psi_max_deg", 360.0);
        limits.theta_min_deg = limits_in.value("theta_min_deg", 30.0);
        limits.theta_max_deg = limits_in.value("theta_max_deg", 90.0);
        limits.l_min_m = limits_in.value("l_min_m", 0.0);
        limits.l_max_m = limits_in.value("l_max_m", 100.0);
    } catch (...) {
        return fail("Invalid crane.limits.* values.");
    }

    CraneModel model{boom_length_m, base_xyz, limits};

    std::vector<AABB> obstacles;
    try {
        json obstacles_in = data.value("obstacles", json::array());
        if (!obstacles_in.is_array()) {
            return fail("Invalid obstacles format; expected obstacles[].{center_xyz_m,size_m}.");
        }
        for (const auto& oi : obstacles_in) {
            if (!oi.contains("center_xyz_m") || !oi.contains("size_m")) {
                return fail("Invalid obstacles format; expected obstacles[].{center_xyz_m,size_m}.");
            }
            auto center = oi.at("center_xyz_m");
            auto size = oi.at("size_m");
            if (!center.is_array() || !size.is_array() || center.size() != 3 || size.size() != 3) {
                return fail("Each obstacle must have center_xyz_m and size_m as 3-element arrays.");
            }
            obstacles.push_back(AABB{
                {center[0].get<double>(), center[1].get<double>(), center[2].get<double>()},
                {size[0].get<double>(), size[1].get<double>(), size[2].get<double>()}
            });
        }
    } catch (...) {
        return fail("Invalid obstacles format; expected obstacles[].{center_xyz_m,size_m}.");
    }

    json task = data.value("task", json::object());
    if (!task.contains("start_xyz_m") || !task.contains("goal_xyz_m")) {
        return fail("Missing required field: task.start_xyz_m and/or task.goal_xyz_m");
    }

    Vec3 start_xyz{};
    Vec3 goal_xyz{};
    try {
        auto start_arr = task.at("start_xyz_m");
        auto goal_arr = task.at("goal_xyz_m");
        if (!start_arr.is_array() || !goal_arr.is_array() || start_arr.size() != 3 || goal_arr.size() != 3) {
            return fail("task.start_xyz_m and task.goal_xyz_m must be 3-element arrays.");
        }
        start_xyz = {start_arr[0].get<double>(), start_arr[1].get<double>(), start_arr[2].get<double>()};
        goal_xyz = {goal_arr[0].get<double>(), goal_arr[1].get<double>(), goal_arr[2].get<double>()};
    } catch (...) {
        return fail("Invalid task.start_xyz_m/task.goal_xyz_m format.");
    }

    json planner_in = data.value("planner", json::object());
    PlannerConfig planner;
    try {
        bool optimize_min_segments = true;
        if (planner_in.contains("optimize_min_segments")) {
            optimize_min_segments = planner_in.at("optimize_min_segments").get<bool>();
        } else if (planner_in.contains("min_segments_first")) {
            optimize_min_segments = planner_in.at("min_segments_first").get<bool>();
        }
        planner.d_min_m = task.value("d_min_m", planner_in.value("d_min_m", 1.0));
        planner.delta_psi_deg = planner_in.value("delta_psi_deg", 2.0);
        planner.delta_theta_deg = planner_in.value("delta_theta_deg", 1.0);
        planner.delta_l_m = planner_in.value("delta_l_m", 0.5);
        planner.eps_r_m = planner_in.value("eps_r_m", 1e-4);
        planner.search_pre_hoist = planner_in.value("search_pre_hoist", true);
        planner.pre_hoist_step_m = planner_in.value("pre_hoist_step_m", 1.0);
        planner.pre_hoist_max_candidates = planner_in.value("pre_hoist_max_candidates", 250);
        planner.waypoint_pre_hoist_max_candidates = planner_in.value("waypoint_pre_hoist_max_candidates", 60);
        planner.waypoint_max_nodes = planner_in.value("waypoint_max_nodes", 60);
        planner.auto_waypoints = planner_in.value("auto_waypoints", true);
        planner.auto_waypoint_rounds = planner_in.value("auto_waypoint_rounds", 3);
        planner.auto_waypoint_max_candidates = planner_in.value("auto_waypoint_max_candidates", 120);
        planner.auto_psi_step_deg = planner_in.value("auto_psi_step_deg", 30.0);
        planner.auto_theta_step_deg = planner_in.value("auto_theta_step_deg", 5.0);
        planner.auto_rope_step_m = planner_in.value("auto_rope_step_m", 5.0);
        planner.auto_neighbor_k = planner_in.value("auto_neighbor_k", 18);
        planner.optimize_min_segments = optimize_min_segments;
        planner.cost_psi_deg = planner_in.value("cost_psi_deg", 1.0);
        planner.cost_theta_deg = planner_in.value("cost_theta_deg", 1.0);
        planner.cost_rope_m = planner_in.value("cost_rope_m", 1.0);
        planner.boom_clearance_m = planner_in.value("boom_clearance_m", 0.0);
        planner.rope_clearance_m = planner_in.value("rope_clearance_m", 0.0);
        planner.ground_z_min_m = planner_in.value("ground_z_min_m", 0.0);
        planner.validate();
    } catch (const std::exception& e) {
        return fail(std::string("Invalid planner parameters: ") + e.what());
    } catch (...) {
        return fail("Invalid planner parameters: unknown error");
    }

    Config start_cfg;
    Config goal_cfg;
    try {
        start_cfg = inverse_hook_to_config(model, start_xyz, planner.eps_r_m);
        goal_cfg = inverse_hook_to_config(model, goal_xyz, planner.eps_r_m);
    } catch (const std::exception& e) {
        return fail(e.what());
    }

    auto start_safe_pair = is_safe_point(start_xyz, obstacles, planner.d_min_m);
    bool start_safe = start_safe_pair.first;
    double start_d = start_safe_pair.second;
    auto goal_safe_pair = is_safe_point(goal_xyz, obstacles, planner.d_min_m);
    bool goal_safe = goal_safe_pair.first;
    double goal_d = goal_safe_pair.second;
    if (!start_safe) {
        return fail("Start point violates clearance (d_nearest=" + std::to_string(start_d) + "m).");
    }
    if (!goal_safe) {
        return fail("Goal point violates clearance (d_nearest=" + std::to_string(goal_d) + "m).");
    }
    if (!is_ground_safe(start_xyz, planner.ground_z_min_m)) {
        return fail("Start point is below ground_z_min_m=" + std::to_string(planner.ground_z_min_m) + "m.");
    }
    if (!is_ground_safe(goal_xyz, planner.ground_z_min_m)) {
        return fail("Goal point is below ground_z_min_m=" + std::to_string(planner.ground_z_min_m) + "m.");
    }

    Vec3 start_tip = boom_tip_from_hook(start_xyz, start_cfg.rope_m);
    auto boom_start_pair = boom_is_safe(model.base_xyz_m, start_tip, obstacles, planner.boom_clearance_m);
    bool boom_ok_start = boom_start_pair.first;
    double boom_d_start = boom_start_pair.second;
    if (!boom_ok_start) {
        return fail("Start boom collides or violates clearance (d_boom=" + std::to_string(boom_d_start) + "m).");
    }
    Vec3 goal_tip = boom_tip_from_hook(goal_xyz, goal_cfg.rope_m);
    auto boom_goal_pair = boom_is_safe(model.base_xyz_m, goal_tip, obstacles, planner.boom_clearance_m);
    bool boom_ok_goal = boom_goal_pair.first;
    double boom_d_goal = boom_goal_pair.second;
    if (!boom_ok_goal) {
        return fail("Goal boom collides or violates clearance (d_boom=" + std::to_string(boom_d_goal) + "m).");
    }
    auto rope_start_pair = rope_is_safe(start_xyz, start_tip, obstacles, planner.rope_clearance_m);
    bool rope_ok_start = rope_start_pair.first;
    double rope_d_start = rope_start_pair.second;
    if (!rope_ok_start) {
        return fail("Start rope collides or violates clearance (d_rope=" + std::to_string(rope_d_start) + "m).");
    }
    auto rope_goal_pair = rope_is_safe(goal_xyz, goal_tip, obstacles, planner.rope_clearance_m);
    bool rope_ok_goal = rope_goal_pair.first;
    double rope_d_goal = rope_goal_pair.second;
    if (!rope_ok_goal) {
        return fail("Goal rope collides or violates clearance (d_rope=" + std::to_string(rope_d_goal) + "m).");
    }

    std::string mode = "direct";
    std::vector<Vec3> waypoint_nodes_xyz;
    std::vector<Config> waypoint_nodes_cfg;

    json waypoints_in = task.value("waypoints_xyz_m", json::array());
    if (!waypoints_in.empty()) {
        try {
            for (const auto& wp : waypoints_in) {
                if (!wp.is_array() || wp.size() != 3) {
                    return fail("Each waypoint in task.waypoints_xyz_m must be a 3-element array.");
                }
                Vec3 xyz{wp[0].get<double>(), wp[1].get<double>(), wp[2].get<double>()};
                if (!is_ground_safe(xyz, planner.ground_z_min_m)) {
                    return fail("Waypoint is below ground_z_min_m=" + std::to_string(planner.ground_z_min_m) + "m: (" +
                        std::to_string(xyz[0]) + ", " + std::to_string(xyz[1]) + ", " + std::to_string(xyz[2]) + ")");
                }
                auto safe_pair = is_safe_point(xyz, obstacles, planner.d_min_m);
                bool safe = safe_pair.first;
                double d = safe_pair.second;
                if (!safe) {
                    return fail("Waypoint violates clearance (d_nearest=" + std::to_string(d) + "m): (" +
                        std::to_string(xyz[0]) + ", " + std::to_string(xyz[1]) + ", " + std::to_string(xyz[2]) + ")");
                }
                Config cfg = inverse_hook_to_config(model, xyz, planner.eps_r_m);
                Vec3 tip = boom_tip_from_hook(xyz, cfg.rope_m);
                auto boom_pair = boom_is_safe(model.base_xyz_m, tip, obstacles, planner.boom_clearance_m);
                bool boom_ok = boom_pair.first;
                double boom_d = boom_pair.second;
                if (!boom_ok) {
                    return fail("Waypoint boom collides or violates clearance (d_boom=" + std::to_string(boom_d) + "m): (" +
                        std::to_string(xyz[0]) + ", " + std::to_string(xyz[1]) + ", " + std::to_string(xyz[2]) + ")");
                }
                auto rope_pair = rope_is_safe(xyz, tip, obstacles, planner.rope_clearance_m);
                bool rope_ok = rope_pair.first;
                double rope_d = rope_pair.second;
                if (!rope_ok) {
                    return fail("Waypoint rope collides or violates clearance (d_rope=" + std::to_string(rope_d) + "m): (" +
                        std::to_string(xyz[0]) + ", " + std::to_string(xyz[1]) + ", " + std::to_string(xyz[2]) + ")");
                }
                waypoint_nodes_xyz.push_back(xyz);
                waypoint_nodes_cfg.push_back(cfg);
            }
        } catch (const std::exception& e) {
            return fail(e.what());
        } catch (...) {
            return fail("Invalid task.waypoints_xyz_m format; expected list of 3-element arrays.");
        }
    }

    std::vector<Config> configs;
    std::vector<int> node_path;
    std::vector<int> node_ks;
    bool has_node_path = false;
    std::string err;
    std::vector<Vec3> nodes_xyz_active;
    bool has_nodes_xyz_active = false;

    auto try_waypoint_search = [&](
        const std::vector<Config>& nodes_cfg,
        const std::vector<Vec3>& nodes_xyz,
        int neighbor_k,
        bool allow_pre_hoist) {
        nodes_xyz_active = nodes_xyz;
        has_nodes_xyz_active = true;
        GraphSearchResult res = plan_with_waypoint_graph_search(
            model,
            obstacles,
            planner,
            nodes_cfg,
            nodes_xyz,
            allow_pre_hoist,
            planner.optimize_min_segments,
            neighbor_k);
        if (res.ok) {
            configs = res.configs;
            node_path = res.node_path;
            node_ks = res.node_ks;
            has_node_path = true;
        } else {
            err = res.err;
        }
    };

    if (!waypoint_nodes_cfg.empty()) {
        mode = "waypoint_search";
        std::vector<Config> node_cfgs;
        std::vector<Vec3> node_xyzs;
        node_cfgs.reserve(waypoint_nodes_cfg.size() + 2);
        node_xyzs.reserve(waypoint_nodes_xyz.size() + 2);
        node_cfgs.push_back(start_cfg);
        node_xyzs.push_back(start_xyz);
        node_cfgs.insert(node_cfgs.end(), waypoint_nodes_cfg.begin(), waypoint_nodes_cfg.end());
        node_xyzs.insert(node_xyzs.end(), waypoint_nodes_xyz.begin(), waypoint_nodes_xyz.end());
        node_cfgs.push_back(goal_cfg);
        node_xyzs.push_back(goal_xyz);
        try_waypoint_search(node_cfgs, node_xyzs, -1, planner.search_pre_hoist);
    } else {
        SegmentPlanResult direct = try_plan_segments(model, obstacles, planner, start_cfg, goal_cfg, planner.search_pre_hoist);
        if (direct.ok) {
            configs = direct.configs;
        } else {
            err = direct.err;
            if (planner.auto_waypoints) {
                mode = "auto_waypoint_search";
                int auto_limit = std::max(0, planner.waypoint_max_nodes - 2);
                if (auto_limit == 0) {
                    return fail("waypoint_max_nodes too small for auto waypoint search.");
                }
                PlannerConfig tmp = planner;
                tmp.auto_waypoint_max_candidates = std::min(planner.auto_waypoint_max_candidates, auto_limit);
                tmp.validate();
                auto auto_pair = generate_auto_waypoint_candidates(
                    model,
                    obstacles,
                    tmp,
                    start_cfg,
                    goal_cfg);
                std::vector<Config> auto_cfgs = auto_pair.first;
                std::vector<Vec3> auto_xyzs = auto_pair.second;
                std::vector<Config> node_cfgs;
                std::vector<Vec3> node_xyzs;
                node_cfgs.reserve(auto_cfgs.size() + 2);
                node_xyzs.reserve(auto_xyzs.size() + 2);
                node_cfgs.push_back(start_cfg);
                node_xyzs.push_back(start_xyz);
                node_cfgs.insert(node_cfgs.end(), auto_cfgs.begin(), auto_cfgs.end());
                node_xyzs.insert(node_xyzs.end(), auto_xyzs.begin(), auto_xyzs.end());
                node_cfgs.push_back(goal_cfg);
                node_xyzs.push_back(goal_xyz);
                try_waypoint_search(node_cfgs, node_xyzs, planner.auto_neighbor_k, true);
            }
        }
    }

    if (configs.empty()) {
        return fail(err.empty() ? "No feasible plan found." : err);
    }

    TrajectoryResult traj = configs_to_trajectory(
        model,
        obstacles,
        planner.d_min_m,
        configs,
        planner.boom_clearance_m,
        planner.rope_clearance_m,
        planner.ground_z_min_m);
    if (!traj.ok) {
        return fail(traj.err.empty() ? "Trajectory invalid." : traj.err);
    }

    std::vector<json> actions = actions_from_configs(configs, planner);

    std::vector<json> waypoint_payload;
    if (!has_node_path) {
        waypoint_payload.push_back(json{{"role", "start"}, {"k", 0}, {"pos", {start_xyz[0], start_xyz[1], start_xyz[2]}}});
        waypoint_payload.push_back(json{{"role", "goal"}, {"k", static_cast<int>(traj.traj.size() - 1)}, {"pos", {goal_xyz[0], goal_xyz[1], goal_xyz[2]}}});
    } else {
        std::vector<Vec3> node_xyzs;
        if (has_nodes_xyz_active) {
            node_xyzs = nodes_xyz_active;
        } else {
            node_xyzs.push_back(start_xyz);
            node_xyzs.push_back(goal_xyz);
        }
        std::vector<int> ks;
        if (!node_ks.empty()) {
            ks = node_ks;
        } else {
            ks.assign(node_path.size(), 0);
        }
        for (size_t i = 0; i < node_path.size(); ++i) {
            int node_idx = node_path[i];
            std::string role = "waypoint";
            if (i == 0) {
                role = "start";
            } else if (i + 1 == node_path.size()) {
                role = "goal";
            }
            Vec3 xyz = node_xyzs[static_cast<size_t>(node_idx)];
            waypoint_payload.push_back(json{{"role", role}, {"k", ks[i]}, {"pos", {xyz[0], xyz[1], xyz[2]}}});
        }
    }

    json payload;
    payload["status"] = "success";
    payload["mode"] = mode;
    payload["actions"] = actions;
    payload["trajectory"] = traj.traj;
    payload["waypoints"] = waypoint_payload;
    payload["start_config"] = {
        {"psi_deg", normalize_deg(start_cfg.psi_deg)},
        {"theta_deg", start_cfg.theta_deg},
        {"l_rope_m", start_cfg.rope_m}
    };
    payload["goal_config"] = {
        {"psi_deg", normalize_deg(goal_cfg.psi_deg)},
        {"theta_deg", goal_cfg.theta_deg},
        {"l_rope_m", goal_cfg.rope_m}
    };

    return PlanResult{true, payload};
}

PlanResult plan_from_json(const std::string& text) {
    json parsed = json::parse(text);
    return plan_from_dict(parsed);
}

}  // namespace path_generate
