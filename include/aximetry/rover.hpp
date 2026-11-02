// Rover design (what the user chooses) and the derived physical model (what
// those choices imply: mass, inertia, centre of gravity, structural capacity).
#pragma once

#include <string>
#include <utility>
#include <vector>

#include "aximetry/json.hpp"
#include "aximetry/materials.hpp"
#include "aximetry/terramechanics.hpp"

namespace aximetry {

struct RoverDesign {
    std::string name = "Unnamed rover";
    std::string description;

    // Wheels
    int wheel_count = 6;              // 4 (rigid frame) or 6 (rocker-bogie)
    double wheel_diameter = 0.26;     // m
    double wheel_width = 0.16;        // m
    double rim_thickness = 0.003;     // m
    double grouser_height = 0.008;    // m
    int spokes = 6;                   // rim supported by this many flexures/spokes
    std::string wheel_material = "al7075";

    // Body (warm electronics box)
    double body_length = 1.0, body_width = 0.6, body_height = 0.35;  // m
    double wall_thickness = 0.002;    // m
    double body_clearance = 0.30;     // ground to underside of body, m
    std::string chassis_material = "al7075";

    // Suspension struts (one leg per wheel) and rocker beams
    std::string suspension_material = "al7075";
    double strut_od = 0.04;           // tube outer diameter, m
    double strut_wall = 0.002;        // tube wall, m
    double strut_length = 0.35;       // m
    double track_width = 1.2;         // m, between left and right wheel centres
    double wheelbase = 1.4;           // m, front axle to rear axle

    // Drive
    std::string motor = "bldc_m";

    // Power and thermal
    std::string battery = "li_ion";
    double battery_wh = 600.0;
    double solar_area = 1.3;          // m^2
    double solar_efficiency = 0.28;   // triple-junction GaAs
    double solar_areal_mass = 2.0;    // kg/m^2 incl. deck structure
    double avionics_mass = 12.0;      // kg
    double payload_mass = 8.0;        // kg (science instruments)
    double avionics_awake_w = 18.0;
    double avionics_sleep_w = 3.0;
    double sensors_w = 6.0;           // cameras + hazard scanner while awake
    int rhu_count = 8;                // radioisotope heater units, ~1 W each, always on
    double box_conductance = 0.35;    // W/K heat leak from warm box to ambient
    double heater_max_w = 40.0;
    double thermostat_c = -15.0;      // set point used when the controller leaves heaters on auto
};

RoverDesign rover_design_from_json(const json::Value& v);
RoverDesign load_rover_design(const std::string& path);
// Inverse of rover_design_from_json: the same schema as rover files, every field filled.
json::Value rover_design_to_json(const RoverDesign& d);

struct MassItem {
    std::string name;
    double mass;
};

// Derived physical model. Everything the simulator needs, computed once.
struct RoverModel {
    RoverDesign design;
    Material wheel_mat, chassis_mat, strut_mat;
    MotorSpec motor;
    BatteryChemistry battery;

    std::vector<MassItem> mass_breakdown;
    double mass = 0.0;          // kg
    double inertia_z = 0.0;     // kg m^2 about vertical axis
    double cg_height = 0.0;     // m above ground
    double box_heat_capacity = 0.0;  // J/K
    WheelGeometry wheel;
    std::vector<double> axle_x;  // longitudinal axle positions (front positive), m
    double climb_limit = 0.0;    // max rock height the suspension/clearance can pass, m
    double tip_angle_lateral = 0.0;       // rad
    double tip_angle_longitudinal = 0.0;  // rad
    double footprint_radius = 0.0;        // bounding radius used for rock collisions, m

    int wheels_per_side() const { return design.wheel_count / 2; }
    double wheel_radius() const { return wheel.radius; }
    // Motor torque available at wheel speed w (linear DC-motor torque–speed curve).
    double available_torque(double wheel_speed) const;
    // Electrical power to deliver torque T at speed w (copper loss + mechanical).
    double motor_power(double torque, double wheel_speed) const;

    // --- Structural stresses (Pa) for a given vertical wheel load (N) ---
    double rim_stress(double wheel_load) const;       // rim bending between spokes
    double rim_deflection(double wheel_load) const;   // m
    double strut_stress(double wheel_load, double traction) const;  // axial + bending
    double strut_buckling_load() const;               // Euler critical load, N
    double chassis_stress(double load_factor_g) const;  // body box as beam
    // Strength of a material at an ambient temperature (brittle factor for impacts).
    static double strength_at(const Material& m, double temp_c, bool impact);
};

RoverModel build_rover_model(const RoverDesign& d);

}  // namespace aximetry
