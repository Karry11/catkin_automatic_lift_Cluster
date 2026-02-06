#include "cli.h"

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <limits.h>
#include <sys/stat.h>
#include <unistd.h>

#include "planner.h"

namespace path_generate {

struct Args {
    std::string input;
    std::string output;
    bool has_input{false};
    bool has_output{false};
    bool gui{false};
    double interval{0.5};
};

static void print_usage(const char* exe) {
    std::cerr << "Usage: " << (exe ? exe : "path_generate_cpp")
              << " [--input|-i <file>] [--output|-o <file>] [--gui] [--interval <seconds>]\n";
}

static bool path_exists(const std::string& path) {
    struct stat st;
    return ::stat(path.c_str(), &st) == 0;
}

static std::string normalize_dir(std::string path) {
    while (path.size() > 1 && path.back() == '/') {
        path.pop_back();
    }
    return path;
}

static std::string parent_path(const std::string& path) {
    std::string cleaned = normalize_dir(path);
    if (cleaned == "/") {
        return std::string();
    }
    std::string::size_type pos = cleaned.find_last_of('/');
    if (pos == std::string::npos) {
        return std::string();
    }
    if (pos == 0) {
        return std::string("/");
    }
    return cleaned.substr(0, pos);
}

static std::string join_path(const std::string& base, const std::string& leaf) {
    if (base.empty()) {
        return leaf;
    }
    if (base.back() == '/') {
        return base + leaf;
    }
    return base + "/" + leaf;
}

static std::string current_path() {
    char buffer[PATH_MAX];
    if (::getcwd(buffer, sizeof(buffer)) != nullptr) {
        return std::string(buffer);
    }
    return std::string(".");
}

static bool find_default_input_path(std::string* out_path) {
    if (!out_path) {
        return false;
    }
    std::string cur = current_path();
    for (int i = 0; i < 8; ++i) {
        std::string candidate = join_path(join_path(cur, "examples"), "input_three_obstacles_var_height.json");
        if (path_exists(candidate)) {
            *out_path = candidate;
            return true;
        }
        std::string parent = parent_path(cur);
        if (parent.empty() || parent == cur) {
            break;
        }
        cur = parent;
    }
    return false;
}

static std::string read_file(const std::string& path) {
    std::ifstream file(path.c_str(), std::ios::in | std::ios::binary);
    if (!file) {
        throw std::runtime_error("Failed to open input file.");
    }
    std::ostringstream buf;
    buf << file.rdbuf();
    return buf.str();
}

static void log_trajectory(const nlohmann::json& payload) {
    if (!payload.contains("trajectory") || !payload["trajectory"].is_array()) {
        return;
    }
    std::cerr << std::fixed << std::setprecision(3);
    for (const auto& step : payload["trajectory"]) {
        if (!step.contains("pos") || !step["pos"].is_array()) {
            continue;
        }
        int k = step.value("k", -1);
        const auto& pos = step["pos"];
        double x = pos[0].get<double>();
        double y = pos[1].get<double>();
        double z = pos[2].get<double>();
        double psi = step.value("psi_deg", 0.0);
        double theta = step.value("theta_deg", 0.0);
        double rope = step.value("l_rope_m", 0.0);
        std::cerr << "[step " << k << "] pos=(" << x << ", " << y << ", " << z << ")"
                  << " psi_deg=" << psi
                  << " theta_deg=" << theta
                  << " l_rope_m=" << rope << "\n";
    }
}

int run_cli(int argc, char** argv) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--input" || arg == "-i") {
            if (i + 1 >= argc) {
                std::cerr << "[error] --input requires a value.\n";
                print_usage(argv[0]);
                return 2;
            }
            args.input = argv[++i];
            args.has_input = true;
        } else if (arg == "--output" || arg == "-o") {
            if (i + 1 >= argc) {
                std::cerr << "[error] --output requires a value.\n";
                print_usage(argv[0]);
                return 2;
            }
            args.output = argv[++i];
            args.has_output = true;
        } else if (arg == "--gui") {
            args.gui = true;
        } else if (arg == "--interval") {
            if (i + 1 >= argc) {
                std::cerr << "[error] --interval requires a value.\n";
                print_usage(argv[0]);
                return 2;
            }
            try {
                args.interval = std::stod(argv[++i]);
            } catch (...) {
                std::cerr << "[error] Invalid --interval value.\n";
                return 2;
            }
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        } else {
            std::cerr << "[error] Unknown argument: " << arg << "\n";
            print_usage(argv[0]);
            return 2;
        }
    }

    if (args.gui) {
        std::cerr << "[info] --gui requested; GUI is not supported in the C++ version. Running headless.\n";
    }

    std::string input_path;
    if (args.has_input) {
        input_path = args.input;
    } else {
        std::string default_path;
        if (!find_default_input_path(&default_path)) {
            std::cerr << "[error] --input not provided and examples/input_three_obstacles_var_height.json not found.\n";
            return 2;
        }
        input_path = default_path;
        std::cerr << "[info] --input not provided; using default: " << input_path << "\n";
    }

    if (!path_exists(input_path)) {
        std::cerr << "[error] Input file not found: " << input_path << "\n";
        return 2;
    }

    std::string text;
    try {
        text = read_file(input_path);
    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << "\n";
        return 2;
    }

    PlanResult result;
    auto start_time = std::chrono::steady_clock::now();
    try {
        auto data = nlohmann::json::parse(text);
        result = plan_from_dict(data);
    } catch (const std::exception& e) {
        std::cerr << "[error] Failed to parse or plan: " << e.what() << "\n";
        return 2;
    }
    auto end_time = std::chrono::steady_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    if (result.ok) {
        log_trajectory(result.payload);
    } else {
        std::string reason = result.payload.value("reason", "");
        if (!reason.empty()) {
            std::cerr << "[failure] " << reason << "\n";
        }
    }
    std::cerr << "[timing] planning_ms=" << std::fixed << std::setprecision(3) << elapsed_ms << "\n";

    std::string out_text = result.payload.dump(2, ' ', false);
    if (args.has_output) {
        std::ofstream out_file(args.output.c_str(), std::ios::out | std::ios::binary);
        if (!out_file) {
            std::cerr << "[error] Failed to write output file: " << args.output << "\n";
            return 2;
        }
        out_file << out_text;
    } else {
        std::cout << out_text << "\n";
    }

    return result.ok ? 0 : 2;
}

}  // namespace path_generate
