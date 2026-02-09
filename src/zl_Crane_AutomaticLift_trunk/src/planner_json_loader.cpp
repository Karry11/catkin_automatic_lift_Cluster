#include "planner_json_loader.hpp"

#include <cmath>
#include <fstream>
#include <sstream>

#include <nlohmann/json.hpp>

namespace zl_Crane_AutomaticLift_trunk {

namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

double signed_angle_step_deg(double prev_deg, double cur_deg) {
    return std::fmod(cur_deg - prev_deg + 540.0, 360.0) - 180.0;
}

bool get_required_double(const nlohmann::json& obj, const char* key, double* out, std::string* err) {
    if (!obj.contains(key)) {
        if (err) {
            *err = std::string("Missing required field: ") + key;
        }
        return false;
    }
    try {
        *out = obj.at(key).get<double>();
        return true;
    } catch (...) {
        if (err) {
            *err = std::string("Invalid numeric field: ") + key;
        }
        return false;
    }
}

}  // namespace

bool BuildTrajectoryFromPlannerJson(const std::string& json_path,
                                    ompl::app::joint_trajectory_t* out,
                                    double dt_sec,
                                    std::string* err) {
    if (!out) {
        if (err) {
            *err = "Output trajectory pointer is null.";
        }
        return false;
    }
    if (dt_sec <= 0.0 || !std::isfinite(dt_sec)) {
        dt_sec = 0.1;
    }

    std::ifstream file(json_path);
    if (!file.is_open()) {
        if (err) {
            *err = "Failed to open JSON file: " + json_path;
        }
        return false;
    }

    nlohmann::json root;
    try {
        file >> root;
    } catch (const std::exception& ex) {
        if (err) {
            *err = std::string("Failed to parse JSON: ") + ex.what();
        }
        return false;
    }

    const std::string status = root.value("status", "");
    if (status != "success") {
        std::string reason = root.value("reason", "");
        std::string message = root.value("message", "");
        if (err) {
            std::ostringstream ss;
            ss << "Planner status is not success.";
            if (!message.empty()) {
                ss << " message=" << message;
            }
            if (!reason.empty()) {
                ss << " reason=" << reason;
            }
            *err = ss.str();
        }
        return false;
    }

    if (!root.contains("trajectory") || !root["trajectory"].is_array()) {
        if (err) {
            *err = "Missing or invalid 'trajectory' array.";
        }
        return false;
    }

    const auto& traj = root["trajectory"];
    if (traj.size() < 2) {
        if (err) {
            *err = "Trajectory has fewer than 2 points.";
        }
        return false;
    }

    out->points.clear();
    out->points.reserve(traj.size());

    double t = 0.0;
    double prev_psi_deg = 0.0;
    double prev_theta_deg = 0.0;
    double prev_rope_m = 0.0;
    bool has_prev = false;

    for (const auto& step : traj) {
        double psi_deg = 0.0;
        double theta_deg = 0.0;
        double rope_m = 0.0;
        std::string local_err;
        if (!get_required_double(step, "psi_deg", &psi_deg, &local_err) ||
            !get_required_double(step, "theta_deg", &theta_deg, &local_err) ||
            !get_required_double(step, "l_rope_m", &rope_m, &local_err)) {
            if (err) {
                *err = local_err;
            }
            return false;
        }

        ompl::app::joint_trajectory_point_t point;
        point.positions.clear();
        point.velocities.clear();
        point.accelerations.clear();

        point.positions.push_back(psi_deg * kDegToRad);
        point.positions.push_back(theta_deg * kDegToRad);
        point.positions.push_back(rope_m);

        if (!has_prev) {
            point.velocities = {0.0, 0.0, 0.0};
        } else {
            const double psi_delta_deg = signed_angle_step_deg(prev_psi_deg, psi_deg);
            const double theta_delta_deg = theta_deg - prev_theta_deg;
            const double rope_delta_m = rope_m - prev_rope_m;

            point.velocities.push_back((psi_delta_deg * kDegToRad) / dt_sec);
            point.velocities.push_back((theta_delta_deg * kDegToRad) / dt_sec);
            point.velocities.push_back(rope_delta_m / dt_sec);
        }

        point.accelerations = {0.0, 0.0, 0.0};
        point.time_from_start = t;

        out->points.push_back(point);

        prev_psi_deg = psi_deg;
        prev_theta_deg = theta_deg;
        prev_rope_m = rope_m;
        has_prev = true;
        t += dt_sec;
    }

    return true;
}

}  // namespace zl_Crane_AutomaticLift_trunk
