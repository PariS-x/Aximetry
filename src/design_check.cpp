#include "aximetry/design_check.hpp"

#include <cmath>
#include <cstdio>
#include <sstream>

#include "aximetry/math.hpp"

namespace aximetry {

namespace {

constexpr double kDrivingLoadFactor = 2.0;  // dynamic wheel load while driving over rough ground
constexpr double kRockStrikeFactor = 3.0;   // single-wheel impact on a rock
constexpr double kRequiredSF = 1.5;          // required safety factor on yield
constexpr double kEarthG = 9.81;

Verdict sf_verdict(double sf) {
    if (sf >= kRequiredSF) return Verdict::Pass;
    if (sf >= 1.0) return Verdict::Warn;
    return Verdict::Fail;
}

std::string fmt(const char* f, double a) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), f, a);
    return buf;
}

std::string fmt2(const char* f, double a, double b) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), f, a, b);
    return buf;
}

}  // namespace

double steady_state_slip(const RoverModel& m, const Soil& soil, double load, double need, double max_slip) {
    // Scan slip upward; net force H(s) - Rc(s) grows with slip until slip
    // sinkage makes compaction resistance win. Torque limit applies at half speed.
    const double torque_cap = m.available_torque(0.5 * m.motor.no_load_speed) / m.wheel_radius();
    for (double s = 0.0; s <= max_slip + 1e-9; s += 0.005) {
        WheelContact c = evaluate_contact(soil, load, m.wheel, s);
        const double thrust = std::min(c.thrust, torque_cap);
        if (thrust - c.compaction >= need) return s;
    }
    return -1.0;
}

DesignReport analyze_design(const RoverModel& m, const EnvironmentConfig& env_cfg) {
    DesignReport r;
    r.model = m;
    const auto& d = m.design;
    const double g = mars::kGravity;
    const double weight = m.mass * g;
    const double wheel_load = weight / d.wheel_count;
    Environment env(env_cfg, 0.0);

    // --- Sweep one sol for thermal and power numbers
    double tmin = 1e9;
    double solar_wh = 0.0, night_wh = 0.0, base_wh = 0.0;
    const double step = 300.0;
    const double internal_sleep = d.avionics_sleep_w + d.rhu_count * 1.0;
    for (double t = 0.0; t < mars::kSolSeconds; t += step) {
        const double ta = env.ambient_temp_c(t);
        tmin = std::min(tmin, ta);
        const double night = env.sun_elevation(t) < 0.0;
        solar_wh += env.irradiance(t) * d.solar_area * d.solar_efficiency * step / 3600.0;
        const double internal = night ? internal_sleep : (d.avionics_awake_w + d.sensors_w + d.rhu_count);
        const double heater = clampd(d.box_conductance * (d.thermostat_c - ta) - internal, 0.0, d.heater_max_w);
        const double load = (night ? d.avionics_sleep_w : d.avionics_awake_w + d.sensors_w) + heater;
        base_wh += load * step / 3600.0;
        if (night) night_wh += load * step / 3600.0;
    }
    r.night_min_temp_c = tmin;
    r.solar_wh_per_sol = solar_wh;
    r.baseline_wh_per_sol = base_wh;
    r.night_energy_wh = night_wh;
    const double derate = 1.0 - m.battery.cold_derate_per_c * std::max(0.0, 20.0 - d.thermostat_c);
    r.usable_battery_wh = d.battery_wh * std::max(0.0, derate);
    r.top_speed_cm_s = m.motor.no_load_speed * m.wheel_radius() * 100.0;

    auto add = [&](std::string cat, std::string name, double value, double limit, std::string unit, Verdict v,
                   std::string why) {
        r.checks.push_back({std::move(cat), std::move(name), value, limit, std::move(unit), v, std::move(why)});
    };

    // ---------------- Structure
    {
        const double sigma = m.rim_stress(wheel_load * kDrivingLoadFactor);
        const double sf = m.wheel_mat.yield_strength / sigma;
        add("Structure", "Wheel rim bending (driving, 2 g)", sf, kRequiredSF, "SF", sf_verdict(sf),
            sf >= kRequiredSF ? "Rim carries rough-terrain loads with margin."
                              : fmt2("Rim stress %.0f MPa vs %.0f MPa strength: thicker rim, more spokes, or a stronger material.",
                                     sigma / 1e6, m.wheel_mat.yield_strength / 1e6));
    }
    {
        const double defl = m.rim_deflection(wheel_load * kDrivingLoadFactor);
        const double frac = defl / m.wheel_radius();
        Verdict v = frac < 0.03 ? Verdict::Pass : (frac < 0.08 ? Verdict::Warn : Verdict::Fail);
        add("Structure", "Wheel rim deflection", frac * 100.0, 3.0, "% of radius", v,
            v == Verdict::Pass ? "Wheel keeps its shape."
                               : "Rim flexes like a soft tyre between spokes; low stiffness (Young's modulus) or a thin rim lets it fold.");
    }
    {
        const double strike = wheel_load * kRockStrikeFactor;
        const double strength = RoverModel::strength_at(m.wheel_mat, tmin, true);
        const double sf = strength / m.rim_stress(strike);
        std::string why = sf >= kRequiredSF ? "Survives hitting a rock at night temperature." : "A rock strike at night would crack or crush the rim.";
        if (tmin < m.wheel_mat.brittle_below_c)
            why += " " + m.wheel_mat.name + " is brittle below " + fmt("%.0f C", m.wheel_mat.brittle_below_c) +
                   ", so impact strength drops.";
        add("Structure", "Rock strike on one wheel (3 g, coldest hour)", sf, kRequiredSF, "SF", sf_verdict(sf), why);
    }
    {
        const double P = wheel_load * kDrivingLoadFactor;
        const double traction = P * std::tan(deg2rad(35.0));
        const double sf = m.strut_mat.yield_strength / m.strut_stress(P, traction);
        add("Structure", "Suspension strut stress (axial + bending)", sf, kRequiredSF, "SF", sf_verdict(sf),
            sf >= kRequiredSF ? "Legs handle load plus tractive bending." : "Legs yield under driving loads: larger tube diameter is the cheapest fix (bending stiffness grows with diameter^4).");
    }
    {
        const double P = wheel_load * kDrivingLoadFactor;
        const double sf = m.strut_buckling_load() / P;
        add("Structure", "Suspension strut buckling (Euler)", sf, kRequiredSF, "SF", sf_verdict(sf),
            sf >= kRequiredSF ? "Legs are stiff enough not to buckle." : "Slender legs buckle sideways long before the material yields: stiffness (E) and tube diameter matter more than strength here.");
    }
    {
        const double sf = m.chassis_mat.yield_strength / m.chassis_stress(kDrivingLoadFactor);
        add("Structure", "Body box bending (2 g)", sf, kRequiredSF, "SF", sf_verdict(sf),
            sf >= kRequiredSF ? "Body carries the battery, avionics and payload." : "Body sags and fails under its own contents.");
    }
    {
        // Earth handling: the rover must be assembled and tested in 1 g Earth gravity.
        const double earth = kEarthG / g;
        const double sf_rim = m.wheel_mat.yield_strength / m.rim_stress(wheel_load * earth);
        const double sf_strut = std::min(m.strut_mat.yield_strength / m.strut_stress(wheel_load * earth, 0.0),
                                         m.strut_buckling_load() / (wheel_load * earth));
        const double sf = std::min(sf_rim, sf_strut);
        Verdict v = sf >= 1.2 ? Verdict::Pass : Verdict::Warn;
        add("Structure", "Can stand on Earth for testing (1 g)", sf, 1.2, "SF", v,
            v == Verdict::Pass ? "Can be tested in the lab without support rigs."
                               : "Too weak to stand in Earth gravity (2.6x Mars). Real teams then need gravity-offload rigs or a heavier test unit.");
    }
    for (const Material* mat : {&m.wheel_mat, &m.strut_mat, &m.chassis_mat}) {
        if (tmin < mat->brittle_below_c) {
            add("Structure", mat->name + " cold embrittlement", tmin, mat->brittle_below_c, "C", Verdict::Warn,
                "Night air reaches " + fmt("%.0f C", tmin) + ", below this material's brittle transition (" +
                    fmt("%.0f C", mat->brittle_below_c) + "). Impacts at night are far more dangerous.");
        }
    }

    // ---------------- Mobility
    {
        const double h_tip = rad2deg(std::min(m.tip_angle_lateral, m.tip_angle_longitudinal));
        Verdict v = h_tip >= 30.0 ? Verdict::Pass : (h_tip >= 20.0 ? Verdict::Warn : Verdict::Fail);
        add("Mobility", "Static tip-over angle", h_tip, 30.0, "deg", v,
            v == Verdict::Pass ? "Low, wide stance." : "Centre of gravity is high for the stance width: it will roll on moderate slopes.");
    }
    {
        const Soil firm = soil_by_id("m90_simulant");
        // Torque needed to climb 20 deg on the reference soil at walking slip.
        const double load = weight * std::cos(deg2rad(20.0)) / d.wheel_count;
        const double need = weight * std::sin(deg2rad(20.0)) / d.wheel_count;
        WheelContact c = evaluate_contact(firm, load, m.wheel, 0.3);
        const double torque_needed = (need + c.compaction) * m.wheel_radius();
        const double sf = m.motor.stall_torque / torque_needed;
        add("Mobility", "Motor torque for a 20 deg climb", sf, 1.5, "x needed", sf_verdict(sf),
            sf >= kRequiredSF ? "Motors have torque in reserve." : "Motors are undersized for this rover's weight and wheel size.");
    }
    {
        const double clearance_ratio = d.body_clearance / d.wheel_diameter;
        Verdict v = clearance_ratio >= 0.8 ? Verdict::Pass : Verdict::Warn;
        add("Mobility", "Belly clearance / wheel diameter", clearance_ratio, 0.8, "ratio", v,
            v == Verdict::Pass ? "Body clears rocks the wheels can climb." : "Body is low: rocks the wheels could climb will high-centre the rover.");
    }

    // ---------------- Power and thermal
    {
        Verdict v = r.usable_battery_wh >= 1.3 * r.night_energy_wh ? Verdict::Pass
                    : (r.usable_battery_wh >= r.night_energy_wh ? Verdict::Warn : Verdict::Fail);
        add("Power", "Battery covers the night (heaters + sleep)", r.usable_battery_wh, r.night_energy_wh, "Wh", v,
            v == Verdict::Pass ? "Enough stored energy to keep warm until sunrise."
                               : "Night heating alone drains the battery. At the thermostat temperature the battery only delivers " +
                                     fmt("%.0f%% of its rated capacity.", 100.0 * std::max(0.0, derate)));
    }
    {
        Verdict v = r.solar_wh_per_sol >= 1.4 * r.baseline_wh_per_sol ? Verdict::Pass
                    : (r.solar_wh_per_sol >= r.baseline_wh_per_sol ? Verdict::Warn : Verdict::Fail);
        add("Power", "Solar energy vs baseline needs per sol", r.solar_wh_per_sol, r.baseline_wh_per_sol, "Wh", v,
            v == Verdict::Pass ? "Energy left over for driving and science."
                               : "Array barely covers staying alive, leaving little or nothing for driving.");
    }
    {
        const double max_heat = d.heater_max_w + d.avionics_sleep_w + d.rhu_count;
        const double box_min = tmin + max_heat / d.box_conductance;
        Verdict v = box_min >= m.battery.min_discharge_c ? Verdict::Pass : Verdict::Fail;
        add("Thermal", "Heaters can hold battery above its minimum", box_min, m.battery.min_discharge_c, "C", v,
            v == Verdict::Pass ? "Warm box can be held at a safe temperature."
                               : "Even with heaters flat out, the box falls below what the battery tolerates. Better insulation (lower conductance) or more heater power needed.");
    }
    if (m.battery.survival_c > -20.0) {
        add("Thermal", m.battery.name + " freeze risk", m.battery.survival_c, -20.0, "C", Verdict::Warn,
            "This chemistry is damaged below " + fmt("%.0f C", m.battery.survival_c) + ". One brownout night and it is gone.");
    }

    // ---------------- Soil performance
    for (const auto& soil : soil_catalog()) {
        SoilPerformance sp;
        sp.soil_id = soil.id;
        sp.soil_name = soil.name;
        WheelContact c = evaluate_contact(soil, wheel_load, m.wheel, 0.0);
        sp.sinkage_cm = c.static_sinkage * 100.0;
        sp.rolling_resistance = c.compaction / wheel_load;
        sp.slip_flat = steady_state_slip(m, soil, wheel_load, 0.0);
        sp.slip_10deg = steady_state_slip(m, soil, weight * std::cos(deg2rad(10.0)) / d.wheel_count,
                                          weight * std::sin(deg2rad(10.0)) / d.wheel_count, 0.6);
        sp.max_slope_deg = 0.0;
        for (double a = 0.0; a <= 45.0; a += 0.5) {
            const double th = deg2rad(a);
            double s = steady_state_slip(m, soil, weight * std::cos(th) / d.wheel_count, weight * std::sin(th) / d.wheel_count, 0.6);
            if (s < 0.0) break;
            sp.max_slope_deg = a;
        }
        r.soils.push_back(sp);
    }

    for (const auto& c : r.checks) {
        if (c.verdict == Verdict::Fail) ++r.fails;
        if (c.verdict == Verdict::Warn) ++r.warns;
    }
    return r;
}

std::string format_report(const DesignReport& r, bool color) {
    const auto& m = r.model;
    const auto& d = m.design;
    std::ostringstream os;
    const char* red = color ? "\033[31m" : "";
    const char* yel = color ? "\033[33m" : "";
    const char* grn = color ? "\033[32m" : "";
    const char* bold = color ? "\033[1m" : "";
    const char* rst = color ? "\033[0m" : "";
    char buf[512];

    os << bold << "=== Design review: " << d.name << " ===" << rst << "\n";
    if (!d.description.empty()) os << d.description << "\n";
    os << "\n" << bold << "Mass budget" << rst << "\n";
    for (const auto& it : m.mass_breakdown) {
        std::snprintf(buf, sizeof(buf), "  %-32s %8.2f kg  (%4.1f%%)\n", it.name.c_str(), it.mass, 100.0 * it.mass / m.mass);
        os << buf;
    }
    std::snprintf(buf, sizeof(buf), "  %-32s %8.2f kg   weight on Mars %.0f N (on Earth %.0f N)\n", "TOTAL", m.mass,
                  m.mass * mars::kGravity, m.mass * kEarthG);
    os << buf;
    std::snprintf(buf, sizeof(buf), "  CG height %.2f m | tip-over %.0f deg sideways, %.0f deg fore/aft | top speed %.1f cm/s | climbs rocks up to %.0f cm\n",
                  m.cg_height, rad2deg(m.tip_angle_lateral), rad2deg(m.tip_angle_longitudinal), r.top_speed_cm_s,
                  m.climb_limit * 100.0);
    os << buf;

    os << "\n" << bold << "Engineering checks" << rst << " (coldest night air: " << fmt("%.0f C", r.night_min_temp_c) << ")\n";
    std::string last_cat;
    for (const auto& c : r.checks) {
        if (c.category != last_cat) {
            os << "  -- " << c.category << "\n";
            last_cat = c.category;
        }
        const char* tag = c.verdict == Verdict::Pass ? "PASS" : (c.verdict == Verdict::Warn ? "WARN" : "FAIL");
        const char* col = c.verdict == Verdict::Pass ? grn : (c.verdict == Verdict::Warn ? yel : red);
        std::snprintf(buf, sizeof(buf), "  %s[%s]%s %-46s %10.2f %s (need %.2f)\n", col, tag, rst, c.name.c_str(), c.value,
                      c.unit.c_str(), c.limit);
        os << buf;
        if (c.verdict != Verdict::Pass) os << "         " << c.why << "\n";
    }

    os << "\n" << bold << "Soil trafficability" << rst << " (Bekker-Wong / Janosi; slip capped at 60%)\n";
    std::snprintf(buf, sizeof(buf), "  %-36s %9s %9s %9s %10s %10s\n", "soil", "sink cm", "Rc/W", "slip 0deg", "slip 10deg", "max slope");
    os << buf;
    for (const auto& s : r.soils) {
        std::string s10 = s.slip_10deg < 0 ? "  stuck" : fmt("%6.0f%%", s.slip_10deg * 100.0);
        std::string s0 = s.slip_flat < 0 ? " stuck" : fmt("%5.0f%%", s.slip_flat * 100.0);
        std::snprintf(buf, sizeof(buf), "  %-36s %9.2f %9.3f %9s %10s %8.1f deg\n", s.soil_name.c_str(), s.sinkage_cm,
                      s.rolling_resistance, s0.c_str(), s10.c_str(), s.max_slope_deg);
        os << buf;
    }

    os << "\n" << bold << "Energy per sol" << rst << "\n";
    std::snprintf(buf, sizeof(buf),
                  "  solar %.0f Wh | baseline use %.0f Wh (of which night %.0f Wh) | usable battery when cold %.0f of %.0f Wh\n",
                  r.solar_wh_per_sol, r.baseline_wh_per_sol, r.night_energy_wh, r.usable_battery_wh, d.battery_wh);
    os << buf;

    os << "\n";
    if (r.fails > 0) os << red << bold << "VERDICT: NOT FLIGHTWORTHY" << rst << " (" << r.fails << " fail, " << r.warns << " warn)\n";
    else if (r.warns > 0) os << yel << bold << "VERDICT: FLIGHTWORTHY WITH RISKS" << rst << " (" << r.warns << " warn)\n";
    else os << grn << bold << "VERDICT: FLIGHTWORTHY" << rst << "\n";
    return os.str();
}

json::Value report_to_json(const DesignReport& r) {
    const auto& m = r.model;
    json::Value out = json::Value::object();
    out.set("name", m.design.name);
    out.set("mass_kg", m.mass);
    out.set("weight_mars_n", m.mass * mars::kGravity);
    out.set("weight_earth_n", m.mass * kEarthG);
    out.set("cg_height_m", m.cg_height);
    out.set("tip_lateral_deg", rad2deg(m.tip_angle_lateral));
    out.set("tip_longitudinal_deg", rad2deg(m.tip_angle_longitudinal));
    out.set("top_speed_cm_s", r.top_speed_cm_s);
    out.set("climb_limit_cm", m.climb_limit * 100.0);
    out.set("night_min_temp_c", r.night_min_temp_c);
    out.set("solar_wh_per_sol", r.solar_wh_per_sol);
    out.set("baseline_wh_per_sol", r.baseline_wh_per_sol);
    out.set("night_energy_wh", r.night_energy_wh);
    out.set("usable_battery_wh", r.usable_battery_wh);
    json::Value axles = json::Value::array();
    for (double x : m.axle_x) axles.push_back(x);
    out.set("axle_x", axles);
    json::Value mass = json::Value::array();
    for (const auto& it : m.mass_breakdown) {
        json::Value e = json::Value::object();
        e.set("item", it.name);
        e.set("kg", it.mass);
        mass.push_back(e);
    }
    out.set("mass_breakdown", mass);
    json::Value checks = json::Value::array();
    for (const auto& c : r.checks) {
        json::Value e = json::Value::object();
        e.set("category", c.category);
        e.set("name", c.name);
        e.set("value", c.value);
        e.set("limit", c.limit);
        e.set("unit", c.unit);
        e.set("verdict", c.verdict == Verdict::Pass ? "pass" : (c.verdict == Verdict::Warn ? "warn" : "fail"));
        e.set("why", c.why);
        checks.push_back(e);
    }
    out.set("checks", checks);
    json::Value soils = json::Value::array();
    for (const auto& s : r.soils) {
        json::Value e = json::Value::object();
        e.set("soil", s.soil_id);
        e.set("name", s.soil_name);
        e.set("sinkage_cm", s.sinkage_cm);
        e.set("rolling_resistance", s.rolling_resistance);
        e.set("slip_flat", s.slip_flat);
        e.set("slip_10deg", s.slip_10deg);
        e.set("max_slope_deg", s.max_slope_deg);
        soils.push_back(e);
    }
    out.set("soils", soils);
    out.set("fails", r.fails);
    out.set("warns", r.warns);
    out.set("verdict", r.fails ? "NOT FLIGHTWORTHY" : (r.warns ? "FLIGHTWORTHY WITH RISKS" : "FLIGHTWORTHY"));
    return out;
}

}  // namespace aximetry
