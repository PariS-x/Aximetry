// A mission: the terrain to generate, the season and weather, where the rover
// starts, what it is trying to do, and the comms schedule with Earth.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "aximetry/environment.hpp"
#include "aximetry/json.hpp"
#include "aximetry/terrain.hpp"

namespace aximetry {

struct RelaySpec {
    double local_hour = 15.0;   // orbiter rises at this local time every sol
    double minutes = 10.0;
    double rate_kbps = 1000.0;
};

struct UplinkSpec {
    double send_at_s = 0.0;  // when the ground team sends it (mission time)
    std::string type = "note";
    int target_id = -1;
    double value = 0.0;
    std::string text;
};

struct Mission {
    std::string name = "Unnamed mission";
    std::string description;
    uint64_t seed = 1;
    TerrainConfig terrain;
    EnvironmentConfig environment;

    Vec2 start{10, 10};
    double start_heading_deg = 45.0;
    double start_local_hour = 9.5;
    double start_soc = 0.9;
    double start_box_temp_c = -5.0;
    double duration_sols = 1.0;

    bool has_goal = false;
    Vec2 goal;
    double goal_radius = 5.0;
    double goal_bonus = 200.0;

    double points_per_value = 100.0;
    std::vector<RelaySpec> relays;
    double owlt_min = 12.0;
    double storage_mbit = 2000.0;
    double product_mbit = 40.0;
    double sample_duration_s = 240.0;
    double sample_range_m = 1.5;
    std::vector<UplinkSpec> uplinks;

    double duration_s() const;
};

Mission mission_from_json(const json::Value& v);
Mission load_mission(const std::string& path);

}  // namespace aximetry
