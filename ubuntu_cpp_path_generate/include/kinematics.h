#pragma once

#include "types.h"

namespace path_generate {

double normalize_deg(double angle_deg);

double angular_delta_deg(double start_deg, double end_deg, const char* direction);

struct Limits {
    double psi_min_deg{0.0};
    double psi_max_deg{360.0};
    double theta_min_deg{30.0};
    double theta_max_deg{90.0};
    double l_min_m{0.0};
    double l_max_m{100.0};

    void validate() const;
};

struct CraneModel {
    double boom_length_m{0.0};
    Vec3 base_xyz_m{{0.0, 0.0, 0.0}};
    Limits limits{};

    void validate() const;
};

struct Config {
    double psi_deg{0.0};
    double theta_deg{0.0};
    double rope_m{0.0};

    bool operator==(const Config& other) const;
    bool operator!=(const Config& other) const;
};

Vec3 forward_hook_xyz(const CraneModel& model, const Config& config);

Config inverse_hook_to_config(const CraneModel& model, const Vec3& hook_xyz_m, double eps_r_m);

}  // namespace path_generate
