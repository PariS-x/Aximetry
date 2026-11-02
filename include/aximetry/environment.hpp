// Mars surface environment: gravity, the sol, sun position, atmospheric dust
// opacity, insolation on a horizontal panel and diurnal air temperature.
#pragma once

#include <vector>

namespace aximetry {

namespace mars {
constexpr double kGravity = 3.721;          // m/s^2
constexpr double kSolSeconds = 88775.244;   // one mean solar day
constexpr double kSolarMean = 586.2;        // W/m^2 at mean Mars–Sun distance
constexpr double kEccentricity = 0.0934;
constexpr double kPerihelionLs = 251.0;     // deg
constexpr double kObliquity = 25.19;        // deg
}  // namespace mars

struct DustStorm {
    double start_sol = 1.0;     // mission sols (0 = start of mission)
    double ramp_sols = 0.5;     // time to build to peak
    double duration_sols = 1.0; // time at peak
    double peak_tau = 4.0;      // optical depth at peak
};

struct EnvironmentConfig {
    double latitude_deg = -4.6;   // Gale crater is -4.6
    double ls_deg = 150.0;        // solar longitude (season)
    double tau = 0.5;             // background dust optical depth
    double temp_mean_c = -55.0;
    double temp_amplitude_c = 33.0;
    double panel_dust_rate_per_sol = 0.0025;  // fractional loss of panel output per sol
    std::vector<DustStorm> storms;
};

class Environment {
public:
    Environment() = default;
    Environment(const EnvironmentConfig& cfg, double start_local_hour);

    const EnvironmentConfig& config() const { return cfg_; }
    double mission_sol(double t) const;      // fractional sols since mission start
    double local_hour(double t) const;       // Local mean solar time, 0..24 "Mars hours"
    double sun_elevation(double t) const;    // rad, negative at night
    double sun_azimuth(double t) const;      // rad, world frame (x = east, y = north)
    double optical_depth(double t) const;
    double top_of_atmosphere() const;        // W/m^2 for this season
    double irradiance(double t) const;       // W/m^2 on a horizontal surface
    double ambient_temp_c(double t) const;
    // Seconds from t until the sun next rises above min_elevation_rad.
    double time_to_sunrise(double t, double min_elevation_rad) const;

private:
    EnvironmentConfig cfg_;
    double start_hour_ = 9.0;
};

}  // namespace aximetry
