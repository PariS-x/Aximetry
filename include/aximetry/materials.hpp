// Engineering materials, motors and battery chemistries the rover can be built from.
// Material choice is not a label: density sets mass, yield strength and stiffness
// set structural margins, the brittle-transition temperature decides whether a
// part survives a -90 C Martian night, and wear rate decides how long wheels last.
#pragma once

#include <string>
#include <vector>

namespace aximetry {

struct Material {
    std::string id;
    std::string name;
    double density = 0.0;         // kg/m^3
    double yield_strength = 0.0;  // Pa (tensile yield, or ultimate for brittle/fibrous materials)
    double youngs_modulus = 0.0;  // Pa
    double brittle_below_c = -273.0;  // below this temperature the material embrittles
    double cold_strength_factor = 1.0;  // fraction of strength retained for impact loads when brittle
    double fatigue_exponent = 6.0;  // Basquin-style exponent used for rock-strike damage
    std::string note;
};

struct MotorSpec {
    std::string id;
    std::string name;
    double stall_torque = 0.0;   // N m at the wheel (after gearbox)
    double no_load_speed = 0.0;  // rad/s at the wheel
    double mass = 0.0;           // kg incl. gearbox
    double idle_power = 2.0;     // W per motor + controller while powered
    double driver_efficiency = 0.9;
    double gear_efficiency = 0.55;  // planetary + output stage, cold lubricant
};

struct BatteryChemistry {
    std::string id;
    std::string name;
    double specific_energy = 0.0;  // Wh/kg at pack level
    double min_discharge_c = -20;  // cannot deliver power below this
    double min_charge_c = -20;     // cannot accept charge below this
    double survival_c = -40;       // permanent damage below this
    double cold_derate_per_c = 0.008;  // capacity loss per degree below +20 C
    double charge_efficiency = 0.95;
    std::string note;
};

const std::vector<Material>& material_catalog();
const Material& material_by_id(const std::string& id);
const std::vector<MotorSpec>& motor_catalog();
const MotorSpec& motor_by_id(const std::string& id);
const std::vector<BatteryChemistry>& battery_catalog();
const BatteryChemistry& battery_by_id(const std::string& id);

}  // namespace aximetry
