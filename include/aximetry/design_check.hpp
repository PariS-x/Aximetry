// "Will it work on Mars?" — a static engineering review of a rover design.
// Every verdict comes from the same equations the simulator uses, and each
// failing item carries a plain-language explanation of the physics.
#pragma once

#include <string>
#include <vector>

#include "aximetry/environment.hpp"
#include "aximetry/rover.hpp"

namespace aximetry {

enum class Verdict { Pass, Warn, Fail };

struct DesignCheck {
    std::string category;
    std::string name;
    double value = 0.0;
    double limit = 0.0;
    std::string unit;
    Verdict verdict = Verdict::Pass;
    std::string why;
};

struct SoilPerformance {
    std::string soil_id, soil_name;
    double sinkage_cm = 0.0;
    double rolling_resistance = 0.0;  // Rc / W
    double slip_flat = 0.0;           // steady-state slip on level ground
    double slip_10deg = -1.0;         // -1 = cannot climb
    double max_slope_deg = 0.0;       // with slip <= 60 %
};

struct DesignReport {
    RoverModel model;
    std::vector<DesignCheck> checks;
    std::vector<SoilPerformance> soils;
    double night_min_temp_c = 0.0;
    double solar_wh_per_sol = 0.0;
    double baseline_wh_per_sol = 0.0;
    double night_energy_wh = 0.0;
    double usable_battery_wh = 0.0;
    double top_speed_cm_s = 0.0;
    int fails = 0, warns = 0;
};

// Steady-state slip needed to produce net forward force `need_per_wheel`
// (N per wheel) on `soil`. Returns -1 if not achievable with slip <= max_slip.
double steady_state_slip(const RoverModel& m, const Soil& soil, double load_per_wheel, double need_per_wheel,
                         double max_slip = 0.95);

DesignReport analyze_design(const RoverModel& model, const EnvironmentConfig& env);
std::string format_report(const DesignReport& r, bool color = true);
json::Value report_to_json(const DesignReport& r);

}  // namespace aximetry
