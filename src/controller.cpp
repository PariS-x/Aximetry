#include "aximetry/controller.hpp"

#include <cmath>
#include <stdexcept>

namespace aximetry {

double OrbitalMap::height_at(double x, double y) const {
    if (nx == 0) return 0.0;
    const double u = clampd(x / resolution_m, 0.0, nx - 1.000001);
    const double v = clampd(y / resolution_m, 0.0, ny - 1.000001);
    const int i = static_cast<int>(u), j = static_cast<int>(v);
    const double fu = u - i, fv = v - j;
    const double a = height[j * nx + i] * (1 - fu) + height[j * nx + i + 1] * fu;
    const double b = height[(j + 1) * nx + i] * (1 - fu) + height[(j + 1) * nx + i + 1] * fu;
    return a * (1 - fv) + b * fv;
}

double OrbitalMap::slope_deg_at(double x, double y) const {
    const double d = resolution_m;
    const double gx = (height_at(x + d, y) - height_at(x - d, y)) / (2 * d);
    const double gy = (height_at(x, y + d) - height_at(x, y - d)) / (2 * d);
    return rad2deg(std::atan(std::hypot(gx, gy)));
}

const std::string& OrbitalMap::soil_at(double x, double y) const {
    const int i = clampd(std::lround(x / resolution_m), 0, nx - 1);
    const int j = clampd(std::lround(y / resolution_m), 0, ny - 1);
    return soil_ids[soil_guess[j * nx + i]];
}

void Command::drive(double speed, double yaw_rate, const RoverInfo& rover) {
    const double half = 0.5 * rover.track_width * rover.skid_factor;
    wheel_speed_left = (speed - yaw_rate * half) / rover.wheel_radius;
    wheel_speed_right = (speed + yaw_rate * half) / rover.wheel_radius;
    // Keep the commanded curvature if either side would saturate.
    // Stay below no-load speed: a DC motor has no torque left at its top speed.
    const double limit = 0.85 * rover.max_wheel_speed;
    const double peak = std::max(std::fabs(wheel_speed_left), std::fabs(wheel_speed_right));
    if (peak > limit && peak > 0.0) {
        const double k = limit / peak;
        wheel_speed_left *= k;
        wheel_speed_right *= k;
    }
}

ControllerRegistry& ControllerRegistry::instance() {
    static ControllerRegistry r;
    return r;
}

void ControllerRegistry::add(ControllerEntry e) {
    for (const auto& x : entries_)
        if (x.name == e.name) throw std::runtime_error("duplicate controller name: " + e.name);
    entries_.push_back(std::move(e));
}

std::unique_ptr<Controller> ControllerRegistry::create(const std::string& name) const {
    for (const auto& e : entries_)
        if (e.name == name) return e.factory();
    std::string msg = "unknown controller '" + name + "'. Available:";
    for (const auto& e : entries_) msg += " " + e.name;
    throw std::runtime_error(msg);
}

}  // namespace aximetry
