#include "aximetry/materials.hpp"

#include <stdexcept>

namespace aximetry {

// Room-temperature handbook values (typical, not design-allowable). The cold
// behaviour columns are simplified teaching values: e.g. plain carbon structural
// steel has a ductile-to-brittle transition well above Martian night temperatures,
// while FCC metals (aluminium, austenitic stainless) stay ductile.
const std::vector<Material>& material_catalog() {
    static const std::vector<Material> m = {
        {"al7075", "Aluminium 7075-T6", 2810, 503e6, 71.7e9, -273, 1.0, 6.0,
         "Aerospace workhorse. FCC metal, stays tough in the cold. MER and MSL wheels are aluminium."},
        {"ti6al4v", "Titanium Ti-6Al-4V", 4430, 880e6, 113.8e9, -273, 1.0, 7.0,
         "Best strength-to-weight of the metals here, expensive, tough at cryogenic temperatures."},
        {"ss304", "Stainless steel 304", 8000, 215e6, 193e9, -273, 1.0, 6.0,
         "Austenitic (FCC) so no brittle transition, but heavy and fairly low yield."},
        {"a36", "Structural steel A36", 7850, 250e6, 200e9, -20, 0.3, 6.0,
         "Cheap carbon steel. BCC crystal structure: turns brittle in the cold (think Titanic hull)."},
        {"cfrp", "Carbon fibre composite (quasi-isotropic)", 1600, 600e6, 70e9, -150, 0.8, 8.0,
         "Very light and stiff. Matrix can microcrack under deep thermal cycling."},
        {"pla", "PLA plastic (3D print)", 1240, 50e6, 3.5e9, -10, 0.25, 5.0,
         "Hobby favourite. Weak, soft, and very brittle in Martian cold."},
        {"abs", "ABS plastic", 1050, 40e6, 2.2e9, -40, 0.35, 5.0,
         "Tougher than PLA, still far too compliant for load-bearing rover parts."},
        {"paper", "Paper / corrugated cardboard", 700, 10e6, 2.0e9, -273, 1.0, 3.0,
         "Fine for a model. Low strength and stiffness: thin walls buckle and crush long before metals notice."},
    };
    return m;
}

const Material& material_by_id(const std::string& id) {
    for (const auto& x : material_catalog())
        if (x.id == id) return x;
    throw std::runtime_error("unknown material id: " + id + " (see `aximetry catalog`)");
}

// Generic brushless DC motor + planetary gearhead units, specified at the wheel.
// For scale: MER's top speed was about 5 cm/s; with r = 0.13 m that is ~0.38 rad/s.
const std::vector<MotorSpec>& motor_catalog() {
    static const std::vector<MotorSpec> m = {
        {"bldc_s", "Small BLDC + gearhead", 12.0, 0.8, 0.45, 1.0, 0.9, 0.55},
        {"bldc_m", "Medium BLDC + gearhead", 45.0, 0.45, 1.1, 2.0, 0.9, 0.55},
        {"bldc_l", "Large BLDC + gearhead", 160.0, 0.35, 2.8, 3.0, 0.9, 0.55},
        {"bldc_xl", "Extra-large BLDC + gearhead", 500.0, 0.25, 6.0, 4.0, 0.9, 0.55},
    };
    return m;
}

const MotorSpec& motor_by_id(const std::string& id) {
    for (const auto& x : motor_catalog())
        if (x.id == id) return x;
    throw std::runtime_error("unknown motor id: " + id + " (see `aximetry catalog`)");
}

// Pack-level, illustrative values. Space-grade Li-ion cells (MER, MSL) use
// low-temperature electrolytes, which is what the li-ion entry represents.
const std::vector<BatteryChemistry>& battery_catalog() {
    static const std::vector<BatteryChemistry> b = {
        {"li_ion", "Lithium-ion (space grade, low-temp electrolyte)", 130, -20, -20, -40, 0.008, 0.95,
         "What modern rovers fly. Must be kept warm at night."},
        {"lifepo4", "Lithium iron phosphate", 90, -20, 0, -40, 0.010, 0.95,
         "Very safe and long-lived but heavier, and should not be charged below 0 C."},
        {"nimh", "Nickel-metal hydride", 60, -20, 0, -30, 0.010, 0.85,
         "Older technology: heavier and lossier."},
        {"lead_acid", "Lead-acid", 30, -20, -10, -15, 0.012, 0.80,
         "Car battery. Five times heavier per Wh than Li-ion, and the electrolyte freezes on Mars."},
    };
    return b;
}

const BatteryChemistry& battery_by_id(const std::string& id) {
    for (const auto& x : battery_catalog())
        if (x.id == id) return x;
    throw std::runtime_error("unknown battery chemistry: " + id + " (see `aximetry catalog`)");
}

}  // namespace aximetry
