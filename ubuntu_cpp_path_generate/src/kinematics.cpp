#include "kinematics.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace path_generate {

constexpr double kPi = 3.14159265358979323846;

static double clamp(double value, double low, double high) {
    return std::max(low, std::min(high, value));
}

double normalize_deg(double angle_deg) {
    double angle = std::fmod(angle_deg, 360.0);
    if (angle < 0.0) {
        angle += 360.0;
    }
    return angle;
}

double angular_delta_deg(double start_deg, double end_deg, const char* direction) {
    double start = normalize_deg(start_deg);
    double end = normalize_deg(end_deg);
    double cw = std::fmod(end - start + 360.0, 360.0);
    if (std::string(direction) == "cw") {
        return cw;
    }
    if (std::string(direction) == "ccw") {
        return -std::fmod(start - end + 360.0, 360.0);
    }
    if (std::string(direction) == "shortest") {
        double ccw = -std::fmod(start - end + 360.0, 360.0);
        return (std::abs(cw) <= std::abs(ccw)) ? cw : ccw;
    }
    throw std::invalid_argument("direction must be one of: cw, ccw, shortest");
}

void Limits::validate() const {
    if (!(0.0 <= psi_min_deg && psi_min_deg < psi_max_deg && psi_max_deg <= 360.0)) {
        throw std::invalid_argument("Invalid psi limits; expected within [0, 360].");
    }
    if (!(-89.0 < theta_min_deg && theta_min_deg <= theta_max_deg && theta_max_deg <= 179.0)) {
        throw std::invalid_argument("Invalid theta limits.");
    }
    if (l_min_m > l_max_m) {
        throw std::invalid_argument("Invalid rope limits.");
    }
}

void CraneModel::validate() const {
    limits.validate();
    if (!(boom_length_m > 0.0)) {
        throw std::invalid_argument("boom_length_m must be > 0.");
    }
}

bool Config::operator==(const Config& other) const {
    return psi_deg == other.psi_deg && theta_deg == other.theta_deg && rope_m == other.rope_m;
}

bool Config::operator!=(const Config& other) const {
    return !(*this == other);
}

Vec3 forward_hook_xyz(const CraneModel& model, const Config& config) {
    double psi = normalize_deg(config.psi_deg);
    psi = psi * kPi / 180.0;
    double theta = config.theta_deg * kPi / 180.0;

    double r_tip = model.boom_length_m * std::cos(theta);
    double z_tip = model.boom_length_m * std::sin(theta);

    double x = r_tip * std::cos(psi) + model.base_xyz_m[0];
    double y = r_tip * std::sin(psi) + model.base_xyz_m[1];
    double z = z_tip - config.rope_m + model.base_xyz_m[2];
    return {x, y, z};
}

Config inverse_hook_to_config(const CraneModel& model, const Vec3& hook_xyz_m, double eps_r_m) {
    model.validate();
    double x = hook_xyz_m[0] - model.base_xyz_m[0];
    double y = hook_xyz_m[1] - model.base_xyz_m[1];
    double z = hook_xyz_m[2] - model.base_xyz_m[2];

    double r = std::hypot(x, y);
    double psi_deg = (r < 1e-12) ? 0.0 : normalize_deg(std::atan2(y, x) * 180.0 / kPi);

    if (r > model.boom_length_m + eps_r_m) {
        throw std::invalid_argument("Point is out of reach (r > boom length).");
    }

    double ratio = clamp(r / model.boom_length_m, -1.0, 1.0);
    double theta_deg = std::acos(ratio) * 180.0 / kPi;

    if (!(model.limits.theta_min_deg - 1e-9 <= theta_deg && theta_deg <= model.limits.theta_max_deg + 1e-9)) {
        throw std::invalid_argument("Point is out of theta limits.");
    }

    double theta = theta_deg * kPi / 180.0;
    double z_tip = model.boom_length_m * std::sin(theta);
    double rope_m = z_tip - z;

    if (!(model.limits.l_min_m - 1e-9 <= rope_m && rope_m <= model.limits.l_max_m + 1e-9)) {
        throw std::invalid_argument("Point requires rope length out of limits.");
    }

    double r_tip = model.boom_length_m * std::cos(theta);
    if (std::abs(r_tip - r) > eps_r_m) {
        throw std::invalid_argument("Point radius does not match boom projection within eps_r_m.");
    }

    return Config{psi_deg, theta_deg, rope_m};
}

}  // namespace path_generate
