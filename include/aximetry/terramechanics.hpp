// Wheel–soil interaction using the classical Bekker–Wong / Janosi–Hanamoto
// formulation. These are the same closed-form models used for early sizing of
// the Sojourner, MER and MSL mobility systems: cheap enough to run at 20 Hz for
// six wheels, and they capture the effects that matter for planetary rovers —
// sinkage, compaction resistance, slip-dependent thrust and slip sinkage.
//
// Units are SI throughout: N, m, Pa, rad.
#pragma once

#include <string>
#include <vector>

namespace aximetry {

struct Soil {
    std::string id;           // short key used in mission files
    std::string name;         // human-readable
    double n = 1.0;           // sinkage exponent [-]
    double kc = 0.0;          // cohesive modulus of deformation [N/m^(n+1)]
    double kphi = 0.0;        // frictional modulus of deformation [N/m^(n+2)]
    double cohesion = 0.0;    // c [Pa]
    double phi = 0.0;         // internal friction angle [rad]
    double K = 0.02;          // shear deformation modulus (Janosi) [m]
    double density = 1500.0;  // bulk density [kg/m^3]
    double slip_sinkage = 0.3;  // extra sinkage per unit slip (empirical, see docs/physics.md)
    std::string source;       // where the numbers come from
};

// Built-in soil catalog. Look up by id ("m90_simulant", "loose_drift", ...).
const std::vector<Soil>& soil_catalog();
const Soil& soil_by_id(const std::string& id);

struct WheelGeometry {
    double radius = 0.13;          // m
    double width = 0.16;           // m
    double grouser_height = 0.0;   // m
};

// Everything the contact model computes for one wheel at one instant.
struct WheelContact {
    double static_sinkage = 0.0;  // z0 from Bekker [m]
    double sinkage = 0.0;         // z including slip sinkage [m]
    double contact_length = 0.0;  // projected shear length [m]
    double max_thrust = 0.0;      // (A c + W tan phi) — fully mobilised soil thrust [N]
    double compaction = 0.0;      // compaction (rolling) resistance Rc [N]
    double thrust = 0.0;          // slip-dependent soil thrust H(s) [N] (signed)
};

// Static sinkage of a rigid wheel (Bekker closed form, valid for n < 3):
//   z0 = [ 3W / ((3-n)(kc + b kphi) sqrt(D)) ]^(2/(2n+1))
double bekker_static_sinkage(const Soil& soil, double load, const WheelGeometry& w);

// Compaction resistance for sinkage z:  Rc = (kc + b kphi) z^(n+1) / (n+1)
double compaction_resistance(const Soil& soil, double sinkage, double width);

// Janosi–Hanamoto thrust for slip s in [-1, 1]:
//   H = Hmax * [1 - K/(|s| L) * (1 - exp(-|s| L / K))] * sign(s)
double janosi_thrust(double max_thrust, double contact_length, double K, double slip);

// Full contact evaluation. slip > 0 = driving, slip < 0 = braking/skidding.
WheelContact evaluate_contact(const Soil& soil, double load, const WheelGeometry& w, double slip);
// Same, with the static sinkage already known (lets the simulator cache it per step).
WheelContact evaluate_contact_z0(const Soil& soil, double load, const WheelGeometry& w, double slip, double z0);

// Signed slip ratio, regularised near zero speed so the solver stays smooth.
//   driving: s = (r w - v)/(r w),  braking: s = (r w - v)/v
double slip_ratio(double rim_speed, double ground_speed, double eps = 1e-3);

}  // namespace aximetry
