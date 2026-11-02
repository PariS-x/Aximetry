#include "aximetry/rover.hpp"

#include <cmath>
#include <stdexcept>

#include "aximetry/environment.hpp"
#include "aximetry/math.hpp"

namespace aximetry {

RoverDesign rover_design_from_json(const json::Value& v) {
    RoverDesign d;
    d.name = v.str("name", d.name);
    d.description = v.str("description", d.description);
    if (v.has("wheels")) {
        const auto& w = v.at("wheels");
        d.wheel_count = w.integer("count", d.wheel_count);
        d.wheel_diameter = w.num("diameter_m", d.wheel_diameter);
        d.wheel_width = w.num("width_m", d.wheel_width);
        d.rim_thickness = w.num("rim_thickness_mm", d.rim_thickness * 1000.0) / 1000.0;
        d.grouser_height = w.num("grouser_height_mm", d.grouser_height * 1000.0) / 1000.0;
        d.spokes = w.integer("spokes", d.spokes);
        d.wheel_material = w.str("material", d.wheel_material);
    }
    if (v.has("body")) {
        const auto& b = v.at("body");
        d.body_length = b.num("length_m", d.body_length);
        d.body_width = b.num("width_m", d.body_width);
        d.body_height = b.num("height_m", d.body_height);
        d.wall_thickness = b.num("wall_thickness_mm", d.wall_thickness * 1000.0) / 1000.0;
        d.body_clearance = b.num("ground_clearance_m", d.body_clearance);
        d.chassis_material = b.str("material", d.chassis_material);
    }
    if (v.has("suspension")) {
        const auto& s = v.at("suspension");
        d.suspension_material = s.str("material", d.suspension_material);
        d.strut_od = s.num("strut_diameter_mm", d.strut_od * 1000.0) / 1000.0;
        d.strut_wall = s.num("strut_wall_mm", d.strut_wall * 1000.0) / 1000.0;
        d.strut_length = s.num("strut_length_m", d.strut_length);
        d.track_width = s.num("track_width_m", d.track_width);
        d.wheelbase = s.num("wheelbase_m", d.wheelbase);
    }
    if (v.has("drive")) d.motor = v.at("drive").str("motor", d.motor);
    if (v.has("power")) {
        const auto& p = v.at("power");
        d.battery = p.str("battery", d.battery);
        d.battery_wh = p.num("battery_wh", d.battery_wh);
        d.solar_area = p.num("solar_area_m2", d.solar_area);
        d.solar_efficiency = p.num("solar_efficiency", d.solar_efficiency);
        d.avionics_awake_w = p.num("avionics_awake_w", d.avionics_awake_w);
        d.avionics_sleep_w = p.num("avionics_sleep_w", d.avionics_sleep_w);
        d.sensors_w = p.num("sensors_w", d.sensors_w);
    }
    if (v.has("thermal")) {
        const auto& t = v.at("thermal");
        d.rhu_count = t.integer("rhu_count", d.rhu_count);
        d.box_conductance = t.num("box_conductance_w_per_k", d.box_conductance);
        d.heater_max_w = t.num("heater_max_w", d.heater_max_w);
        d.thermostat_c = t.num("thermostat_c", d.thermostat_c);
    }
    if (v.has("payload")) {
        const auto& p = v.at("payload");
        d.payload_mass = p.num("instruments_kg", d.payload_mass);
        d.avionics_mass = p.num("avionics_kg", d.avionics_mass);
    }
    if (d.wheel_count != 4 && d.wheel_count != 6) throw std::runtime_error("wheels.count must be 4 or 6");
    if (d.spokes < 3) throw std::runtime_error("wheels.spokes must be at least 3");
    if (d.strut_wall * 2.0 >= d.strut_od) throw std::runtime_error("suspension: strut wall too thick for its diameter");
    // Validate catalog ids early so errors point at the rover file.
    material_by_id(d.wheel_material);
    material_by_id(d.chassis_material);
    material_by_id(d.suspension_material);
    motor_by_id(d.motor);
    battery_by_id(d.battery);
    return d;
}

RoverDesign load_rover_design(const std::string& path) { return rover_design_from_json(json::parse_file(path)); }

json::Value rover_design_to_json(const RoverDesign& d) {
    using json::Value;
    Value v = Value::object();
    v.set("name", d.name);
    v.set("description", d.description);
    Value w = Value::object();
    w.set("count", d.wheel_count);
    w.set("diameter_m", d.wheel_diameter);
    w.set("width_m", d.wheel_width);
    w.set("rim_thickness_mm", d.rim_thickness * 1000.0);
    w.set("grouser_height_mm", d.grouser_height * 1000.0);
    w.set("spokes", d.spokes);
    w.set("material", d.wheel_material);
    v.set("wheels", w);
    Value b = Value::object();
    b.set("length_m", d.body_length);
    b.set("width_m", d.body_width);
    b.set("height_m", d.body_height);
    b.set("wall_thickness_mm", d.wall_thickness * 1000.0);
    b.set("ground_clearance_m", d.body_clearance);
    b.set("material", d.chassis_material);
    v.set("body", b);
    Value s = Value::object();
    s.set("material", d.suspension_material);
    s.set("strut_diameter_mm", d.strut_od * 1000.0);
    s.set("strut_wall_mm", d.strut_wall * 1000.0);
    s.set("strut_length_m", d.strut_length);
    s.set("track_width_m", d.track_width);
    s.set("wheelbase_m", d.wheelbase);
    v.set("suspension", s);
    Value dr = Value::object();
    dr.set("motor", d.motor);
    v.set("drive", dr);
    Value p = Value::object();
    p.set("battery", d.battery);
    p.set("battery_wh", d.battery_wh);
    p.set("solar_area_m2", d.solar_area);
    p.set("solar_efficiency", d.solar_efficiency);
    p.set("avionics_awake_w", d.avionics_awake_w);
    p.set("avionics_sleep_w", d.avionics_sleep_w);
    p.set("sensors_w", d.sensors_w);
    v.set("power", p);
    Value t = Value::object();
    t.set("rhu_count", d.rhu_count);
    t.set("box_conductance_w_per_k", d.box_conductance);
    t.set("heater_max_w", d.heater_max_w);
    t.set("thermostat_c", d.thermostat_c);
    v.set("thermal", t);
    Value pl = Value::object();
    pl.set("instruments_kg", d.payload_mass);
    pl.set("avionics_kg", d.avionics_mass);
    v.set("payload", pl);
    return v;
}

RoverModel build_rover_model(const RoverDesign& d) {
    RoverModel m;
    m.design = d;
    m.wheel_mat = material_by_id(d.wheel_material);
    m.chassis_mat = material_by_id(d.chassis_material);
    m.strut_mat = material_by_id(d.suspension_material);
    m.motor = motor_by_id(d.motor);
    m.battery = battery_by_id(d.battery);

    m.wheel.radius = d.wheel_diameter / 2.0;
    m.wheel.width = d.wheel_width;
    m.wheel.grouser_height = d.grouser_height;

    const int n = d.wheel_count;
    const double R = m.wheel.radius;

    // ---- Masses
    const double rim = kPi * d.wheel_diameter * d.wheel_width * d.rim_thickness * m.wheel_mat.density;
    const double spokes = d.spokes * R * d.wheel_width * 0.5 * d.rim_thickness * m.wheel_mat.density;
    const double hub = 0.15;
    const double wheels_mass = n * (rim + spokes + hub);

    const double ri = d.strut_od / 2.0 - d.strut_wall;
    const double tube_area = kPi * (sq(d.strut_od / 2.0) - sq(ri));
    const double beam_length = n * d.strut_length + 2.0 * d.wheelbase;
    const double struts_mass = tube_area * beam_length * m.strut_mat.density;

    const double box_area = 2.0 * (d.body_length * d.body_width + d.body_length * d.body_height + d.body_width * d.body_height);
    const double chassis_mass = box_area * d.wall_thickness * m.chassis_mat.density;
    const double motors_mass = n * m.motor.mass;
    const double battery_mass = d.battery_wh / m.battery.specific_energy;
    const double solar_mass = d.solar_area * d.solar_areal_mass;
    const double rhu_mass = 0.04 * d.rhu_count;
    const double insulation_mass = 0.6 * box_area;  // aerogel + MLI, ~0.6 kg/m^2

    m.mass_breakdown = {
        {"Wheels", wheels_mass},
        {"Suspension struts + rockers", struts_mass},
        {"Body (warm electronics box)", chassis_mass},
        {"Drive motors", motors_mass},
        {"Battery", battery_mass},
        {"Solar array + deck", solar_mass},
        {"Avionics", d.avionics_mass},
        {"Science payload", d.payload_mass},
        {"Thermal (insulation, RHUs)", insulation_mass + rhu_mass},
    };
    double sum = 0.0;
    for (const auto& it : m.mass_breakdown) sum += it.mass;
    const double harness = 0.08 * sum;
    m.mass_breakdown.push_back({"Cabling + fasteners (8%)", harness});
    m.mass = sum + harness;

    // ---- Axles
    if (n == 6) m.axle_x = {d.wheelbase / 2.0, 0.0, -d.wheelbase / 2.0};
    else m.axle_x = {d.wheelbase / 2.0, -d.wheelbase / 2.0};

    // ---- Centre of gravity height (mass-weighted)
    const double body_z = d.body_clearance + d.body_height / 2.0;
    const double deck_z = d.body_clearance + d.body_height + 0.03;
    const double strut_z = (R + d.body_clearance) / 2.0;
    double mz = (wheels_mass + motors_mass) * R + struts_mass * strut_z +
                (chassis_mass + battery_mass + d.avionics_mass + d.payload_mass + insulation_mass + rhu_mass + harness) * body_z +
                solar_mass * deck_z;
    m.cg_height = mz / m.mass;

    // ---- Yaw inertia: body as a box, wheels/motors as point masses
    const double body_mass = m.mass - wheels_mass - motors_mass;
    m.inertia_z = body_mass * (sq(d.body_length) + sq(d.body_width)) / 12.0;
    const double per_wheel = (wheels_mass + motors_mass) / n;
    for (double x : m.axle_x) m.inertia_z += 2.0 * per_wheel * (sq(x) + sq(d.track_width / 2.0));

    // Only what sits inside the insulated box: battery, about half the avionics, inner panels.
    m.box_heat_capacity = (battery_mass + 0.5 * d.avionics_mass) * 900.0 + 0.2 * chassis_mass * 900.0 + 2000.0;

    const double climb = (n == 6 ? 1.0 : 0.5) * d.wheel_diameter;
    m.climb_limit = std::min(climb, d.body_clearance);
    m.tip_angle_lateral = std::atan2(d.track_width / 2.0, m.cg_height);
    m.tip_angle_longitudinal = std::atan2(d.wheelbase / 2.0, m.cg_height);
    m.footprint_radius = 0.5 * std::hypot(d.wheelbase + d.wheel_diameter, d.track_width + d.wheel_width);
    return m;
}

double RoverModel::available_torque(double wheel_speed) const {
    const double frac = 1.0 - std::fabs(wheel_speed) / motor.no_load_speed;
    return motor.stall_torque * std::max(0.0, frac);
}

double RoverModel::motor_power(double torque, double wheel_speed) const {
    // Motor-side torque is higher than wheel torque by the gearbox losses.
    const double t_motor = torque / motor.gear_efficiency;
    const double mech = t_motor * wheel_speed;
    const double copper = sq(t_motor) * motor.no_load_speed / motor.stall_torque;
    return (std::max(0.0, mech) + copper) / motor.driver_efficiency;
}

double RoverModel::rim_stress(double wheel_load) const {
    // Rim treated as a fixed–fixed curved beam between spokes with the contact
    // load at mid-span: M = P L / 8, section modulus b t^2 / 6.
    const double span = 2.0 * kPi * wheel.radius / design.spokes;
    const double M = wheel_load * span / 8.0;
    const double S = design.wheel_width * sq(design.rim_thickness) / 6.0;
    return M / S;
}

double RoverModel::rim_deflection(double wheel_load) const {
    const double span = 2.0 * kPi * wheel.radius / design.spokes;
    const double I = design.wheel_width * std::pow(design.rim_thickness, 3) / 12.0;
    return wheel_load * std::pow(span, 3) / (192.0 * wheel_mat.youngs_modulus * I);
}

double RoverModel::strut_stress(double wheel_load, double traction) const {
    const double ro = design.strut_od / 2.0, ri = ro - design.strut_wall;
    const double A = kPi * (sq(ro) - sq(ri));
    const double I = kPi / 4.0 * (std::pow(ro, 4) - std::pow(ri, 4));
    // Axial compression from the wheel load plus bending from the tractive force
    // acting at the end of the leg.
    return wheel_load / A + std::fabs(traction) * design.strut_length * ro / I;
}

double RoverModel::strut_buckling_load() const {
    const double ro = design.strut_od / 2.0, ri = ro - design.strut_wall;
    const double I = kPi / 4.0 * (std::pow(ro, 4) - std::pow(ri, 4));
    return sq(kPi) * strut_mat.youngs_modulus * I / sq(design.strut_length);  // pinned–pinned
}

double RoverModel::chassis_stress(double load_factor_g) const {
    // Body as a simply supported thin-walled box beam carrying its contents as a
    // uniform load between the suspension pivots: M = W L / 8.
    const auto& d = design;
    double carried = 0.0;
    for (const auto& it : mass_breakdown)
        if (it.name != "Wheels" && it.name != "Drive motors" && it.name != "Suspension struts + rockers") carried += it.mass;
    const double W = carried * mars::kGravity * load_factor_g;
    const double M = W * d.body_length / 8.0;
    const double b = d.body_width, h = d.body_height, t = d.wall_thickness;
    const double I = (b * std::pow(h, 3) - (b - 2 * t) * std::pow(h - 2 * t, 3)) / 12.0;
    return M * (h / 2.0) / I;
}

double RoverModel::strength_at(const Material& m, double temp_c, bool impact) {
    if (impact && temp_c < m.brittle_below_c) return m.yield_strength * m.cold_strength_factor;
    return m.yield_strength;
}

}  // namespace aximetry
