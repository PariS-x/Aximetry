#include "aximetry/environment.hpp"

#include <cmath>

#include "aximetry/math.hpp"

namespace aximetry {

Environment::Environment(const EnvironmentConfig& cfg, double start_local_hour)
    : cfg_(cfg), start_hour_(start_local_hour) {}

double Environment::mission_sol(double t) const { return t / mars::kSolSeconds; }

double Environment::local_hour(double t) const {
    const double sols = start_hour_ / 24.0 + t / mars::kSolSeconds;
    return (sols - std::floor(sols)) * 24.0;
}

namespace {
double declination(double ls_deg) {
    return std::asin(std::sin(deg2rad(mars::kObliquity)) * std::sin(deg2rad(ls_deg)));
}
}  // namespace

double Environment::sun_elevation(double t) const {
    const double lat = deg2rad(cfg_.latitude_deg);
    const double dec = declination(cfg_.ls_deg);
    const double h = deg2rad((local_hour(t) - 12.0) * 15.0);  // hour angle
    const double s = std::sin(lat) * std::sin(dec) + std::cos(lat) * std::cos(dec) * std::cos(h);
    return std::asin(clampd(s, -1.0, 1.0));
}

double Environment::sun_azimuth(double t) const {
    // Azimuth measured from north toward east, then converted to world angle
    // (x = east, y = north, counter-clockwise from +x).
    const double lat = deg2rad(cfg_.latitude_deg);
    const double dec = declination(cfg_.ls_deg);
    const double h = deg2rad((local_hour(t) - 12.0) * 15.0);
    const double az = std::atan2(-std::sin(h), std::tan(dec) * std::cos(lat) - std::sin(lat) * std::cos(h));
    return wrap_angle(kPi / 2.0 - az);
}

double Environment::optical_depth(double t) const {
    const double sol = mission_sol(t);
    double tau = cfg_.tau;
    for (const auto& s : cfg_.storms) {
        const double t0 = s.start_sol, t1 = t0 + s.ramp_sols, t2 = t1 + s.duration_sols, t3 = t2 + 2.0 * s.ramp_sols;
        double f = 0.0;
        if (sol >= t0 && sol < t1) f = (sol - t0) / std::max(1e-6, s.ramp_sols);
        else if (sol >= t1 && sol < t2) f = 1.0;
        else if (sol >= t2 && sol < t3) f = 1.0 - (sol - t2) / std::max(1e-6, t3 - t2);
        tau = std::max(tau, cfg_.tau + f * (s.peak_tau - cfg_.tau));
    }
    return tau;
}

double Environment::top_of_atmosphere() const {
    // Inverse-square with orbital eccentricity: r = a(1-e^2)/(1+e cos(Ls - Ls_p)).
    const double e = mars::kEccentricity;
    const double f = (1.0 + e * std::cos(deg2rad(cfg_.ls_deg - mars::kPerihelionLs))) / (1.0 - e * e);
    return mars::kSolarMean * f * f;
}

double Environment::irradiance(double t) const {
    const double mu = std::sin(sun_elevation(t));
    if (mu <= 0.0) return 0.0;
    const double tau = optical_depth(t);
    const double S = top_of_atmosphere();
    const double transmitted = std::exp(-tau / mu);
    // Diffuse (scattered) light matters a lot on Mars. Simple fit that keeps
    // ~80% of light at tau 0.5 and only a few percent in a global storm.
    const double diffuse = 0.6 * (1.0 - transmitted) * std::exp(-0.25 * tau);
    return S * mu * (transmitted + diffuse);
}

double Environment::ambient_temp_c(double t) const {
    const double extra_tau = std::max(0.0, optical_depth(t) - cfg_.tau);
    // Dust storms damp the day–night swing: warmer nights, colder days.
    const double amp = cfg_.temp_amplitude_c / (1.0 + 0.3 * extra_tau);
    const double phase = 2.0 * kPi * (local_hour(t) - 14.0) / 24.0;
    return cfg_.temp_mean_c + amp * std::cos(phase);
}

double Environment::time_to_sunrise(double t, double min_elevation_rad) const {
    const double step = 60.0;
    for (double dt = 0.0; dt < 2.0 * mars::kSolSeconds; dt += step)
        if (sun_elevation(t + dt) > min_elevation_rad) return dt;
    return 2.0 * mars::kSolSeconds;
}

}  // namespace aximetry
