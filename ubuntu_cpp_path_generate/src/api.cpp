#include "path_generate_api.h"

#include <cstdlib>
#include <cstring>
#include <string>

#include <nlohmann/json.hpp>

#include "planner.h"

namespace {

std::string build_failure_json(const std::string& reason) {
    nlohmann::json payload;
    payload["status"] = "failure";
    payload["message"] = "轨迹无法规划";
    payload["reason"] = reason;
    return payload.dump(2, ' ', false);
}

const char* dup_string(const std::string& value) {
    const std::size_t size = value.size();
    char* buffer = static_cast<char*>(std::malloc(size + 1));
    if (!buffer) {
        return nullptr;
    }
    std::memcpy(buffer, value.data(), size);
    buffer[size] = '\0';
    return buffer;
}

}  // namespace

const char* pg_plan_from_json(const char* input_json) {
    if (!input_json) {
        return dup_string(build_failure_json("input_json is null."));
    }
    try {
        path_generate::PlanResult result = path_generate::plan_from_json(std::string(input_json));
        std::string output = result.payload.dump(2, ' ', false);
        return dup_string(output);
    } catch (const std::exception& ex) {
        return dup_string(build_failure_json(std::string("Invalid JSON or planner error: ") + ex.what()));
    } catch (...) {
        return dup_string(build_failure_json("Unknown error."));
    }
}

void pg_free_string(const char* str) {
    if (!str) {
        return;
    }
    std::free(const_cast<char*>(str));
}
