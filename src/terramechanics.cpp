#include "aximetry/terramechanics.hpp"

#include <cmath>
#include <stdexcept>

#include "aximetry/math.hpp"

namespace aximetry {

const std::vector<Soil>& soil_catalog() {
    static const std::vector<Soil> soils = [] {
        std::vector<Soil> s;
        // Parameters for the two literature soils are taken from the Bekker–Wong
        // table compiled in arXiv:2606.06790 (Table 2): "dry sand" and the M90
        // Mars regolith simulant. The other entries are illustrative values
        // chosen to span firm-to-weak Martian terrain; they are labelled as such.
        Soil m90;
        m90.id = "m90_simulant";
        m90.name = "M90 Mars regolith simulant";
        m90.n = 1.3; m90.kc = 572.1e3; m90.kphi = 4915.3e3;
        m90.cohesion = 2000.0; m90.phi = deg2rad(35.0); m90.K = 0.0254; m90.density = 1520.0;
        m90.slip_sinkage = 0.4;
        m90.source = "Bekker-Wong parameters, M90 simulant (arXiv:2606.06790, Table 2)";
        s.push_back(m90);

        Soil sand;
        sand.id = "dry_sand";
        sand.name = "Dry sand";
        sand.n = 1.9; sand.kc = 10.3e3; sand.kphi = 5309.4e3;
        sand.cohesion = 1200.0; sand.phi = deg2rad(33.3); sand.K = 0.015; sand.density = 1600.0;
        sand.slip_sinkage = 0.5;
        sand.source = "Bekker-Wong parameters, dry sand (arXiv:2606.06790, Table 2)";
        s.push_back(sand);

        Soil drift;
        drift.id = "loose_drift";
        drift.name = "Loose aeolian drift (weak, Troy-like)";
        drift.n = 1.0; drift.kc = 1.0e3; drift.kphi = 9.0e5;
        drift.cohesion = 800.0; drift.phi = deg2rad(31.0); drift.K = 0.022; drift.density = 1300.0;
        drift.slip_sinkage = 0.7;
        drift.source = "Illustrative weak soil, tuned so MER-class rovers struggle on sloped drift";
        s.push_back(drift);

        Soil crust;
        crust.id = "duricrust";
        crust.name = "Cemented duricrust";
        crust.n = 1.0; crust.kc = 2.0e5; crust.kphi = 2.0e7;
        crust.cohesion = 6000.0; crust.phi = deg2rad(37.0); crust.K = 0.012; crust.density = 1700.0;
        crust.slip_sinkage = 0.15;
        crust.source = "Illustrative firm crust (low sinkage, good traction)";
        s.push_back(crust);

        Soil rock;
        rock.id = "bedrock";
        rock.name = "Bedrock / outcrop";
        rock.n = 1.0; rock.kc = 1.0e7; rock.kphi = 1.0e9;
        rock.cohesion = 20000.0; rock.phi = deg2rad(40.0); rock.K = 0.005; rock.density = 2600.0;
        rock.slip_sinkage = 0.0;
        rock.source = "Illustrative near-rigid surface";
        s.push_back(rock);
        return s;
    }();
    return soils;
}

const Soil& soil_by_id(const std::string& id) {
    for (const auto& s : soil_catalog())
        if (s.id == id) return s;
    throw std::runtime_error("unknown soil id: " + id);
}

double bekker_static_sinkage(const Soil& soil, double load, const WheelGeometry& w) {
    if (load <= 0.0) return 0.0;
    const double D = 2.0 * w.radius;
    const double denom = (3.0 - soil.n) * (soil.kc + w.width * soil.kphi) * std::sqrt(D);
    const double z = std::pow(3.0 * load / denom, 2.0 / (2.0 * soil.n + 1.0));
    // The closed form assumes small sinkage; clamp to a full wheel radius.
    return std::min(z, w.radius);
}

double compaction_resistance(const Soil& soil, double sinkage, double width) {
    if (sinkage <= 0.0) return 0.0;
    return (soil.kc + width * soil.kphi) * std::pow(sinkage, soil.n + 1.0) / (soil.n + 1.0);
}

double janosi_thrust(double max_thrust, double contact_length, double K, double slip) {
    const double a = std::fabs(slip);
    if (a < 1e-9 || contact_length <= 0.0) return 0.0;
    const double x = a * contact_length / K;
    // For tiny x use the series 1 - (1-e^-x)/x ≈ x/2 - x^2/6 to avoid cancellation.
    const double frac = (x < 1e-4) ? (x / 2.0 - x * x / 6.0) : (1.0 - (1.0 - std::exp(-x)) / x);
    return std::copysign(max_thrust * frac, slip);
}

WheelContact evaluate_contact(const Soil& soil, double load, const WheelGeometry& w, double slip) {
    return evaluate_contact_z0(soil, load, w, slip, bekker_static_sinkage(soil, load, w));
}

WheelContact evaluate_contact_z0(const Soil& soil, double load, const WheelGeometry& w, double slip, double z0) {
    WheelContact c;
    if (load <= 0.0) return c;  // wheel in the air
    c.static_sinkage = z0;
    // Slip sinkage: a driving wheel that slips digs itself in (excavation).
    c.sinkage = std::min(c.static_sinkage * (1.0 + soil.slip_sinkage * std::max(0.0, slip)), w.radius);

    // Contact geometry. Entry angle from sinkage (plus grouser penetration),
    // exit angle taken as 0.3 x entry, a common rule of thumb for loose soil.
    const double z_eff = std::min(c.sinkage + w.grouser_height, w.radius);
    const double theta1 = std::acos(clampd(1.0 - z_eff / w.radius, -1.0, 1.0));
    c.contact_length = w.radius * theta1 * 1.3;
    const double area = w.width * c.contact_length;

    c.max_thrust = area * soil.cohesion + load * std::tan(soil.phi);
    c.compaction = compaction_resistance(soil, c.sinkage, w.width);
    c.thrust = janosi_thrust(c.max_thrust, c.contact_length, soil.K, slip);
    return c;
}

double slip_ratio(double rim_speed, double ground_speed, double eps) {
    const double denom = std::max({std::fabs(rim_speed), std::fabs(ground_speed), eps});
    return clampd((rim_speed - ground_speed) / denom, -1.0, 1.0);
}

}  // namespace aximetry
