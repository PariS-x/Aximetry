// =============================================================================
//  Aximetry controller API — this is the file you write your rover code against.
// =============================================================================
//
//  A controller is a C++ class with two methods:
//
//    on_start(brief)  called once with everything known before the rover drives:
//                     orbital map, landmark catalogue, science targets, comms
//                     schedule and the rover's own specs.
//    on_step(obs)     called at 5 Hz while the rover is awake. You get noisy
//                     sensor readings, never the true state, and return a
//                     Command (wheel speeds, instruments, radio, heaters, sleep).
//
//  Register it with one line at the bottom of your .cpp file:
//
//    AXIMETRY_REGISTER_CONTROLLER(MyController, "my_controller", "What it does");
//
//  Drop the file anywhere under controllers/, re-run cmake --build, and run
//  `aximetry run --controller my_controller ...`. See docs/writing_a_controller.md.
// =============================================================================
#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "aximetry/math.hpp"

namespace aximetry {

// ---------------------------------------------------------------- before landing

// Coarse orbital data: a 2 m/pixel elevation model with noise, and a soil map
// guessed from thermal inertia (which is sometimes wrong).
struct OrbitalMap {
    double size_m = 0.0;
    double resolution_m = 2.0;
    int nx = 0, ny = 0;
    std::vector<float> height;          // row-major, ny rows of nx
    std::vector<uint8_t> soil_guess;    // index into soil_ids
    std::vector<std::string> soil_ids;  // e.g. "m90_simulant", "loose_drift"
    double height_at(double x, double y) const;
    double slope_deg_at(double x, double y) const;
    const std::string& soil_at(double x, double y) const;
};

struct Landmark {
    int id = 0;
    Vec2 pos;          // surveyed position (accurate)
    double radius = 0.5;
};

struct TargetInfo {
    int id = 0;
    Vec2 pos;
    std::string orbital_kind;   // best guess from orbit
    double orbital_prior = 0.0; // expected science value if the guess is right (0..1)
};

struct RelayPass {
    double start_s = 0.0;       // mission time when the orbiter rises
    double duration_s = 0.0;
    double rate_kbps = 0.0;
};

struct RoverInfo {
    std::string name;
    double mass_kg = 0.0;
    double wheel_radius = 0.0;
    double track_width = 0.0;
    double wheelbase = 0.0;
    double max_wheel_speed = 0.0;   // rad/s (motor no-load speed)
    double battery_wh = 0.0;
    double solar_area_m2 = 0.0;
    double footprint_radius = 0.0;  // m, for path planning clearance
    double climb_limit_m = 0.0;     // rocks shorter than this can be driven over
    double tip_angle_deg = 0.0;     // static tip-over angle (worst axis)
    double skid_factor = 1.5;       // effective track multiplier for skid-steer turning
};

struct MissionBrief {
    std::string mission_name;
    std::string description;
    double duration_s = 0.0;
    double start_local_hour = 9.0;
    double sol_seconds = 88775.244;

    Vec2 start_pos;              // landing estimate
    double start_heading = 0.0;  // rad, world frame (x east, y north)
    double start_pos_sigma = 0.5;

    bool has_goal = false;
    Vec2 goal;
    double goal_radius = 5.0;
    double goal_bonus = 0.0;
    double points_per_value = 100.0;  // science score = points * true value of each target received on Earth

    OrbitalMap map;
    std::vector<Landmark> landmarks;
    std::vector<TargetInfo> targets;

    std::vector<RelayPass> relay_passes;
    double owlt_s = 720.0;            // one-way light time Earth–Mars
    double storage_mbit = 2000.0;
    double product_mbit = 40.0;       // size of one spectrometer data product
    double sample_duration_s = 240.0; // spectrometer integration time
    double sample_range_m = 1.5;      // rover centre must be this close to the target

    RoverInfo rover;
};

// ---------------------------------------------------------------- every step

struct LandmarkObs {
    int id = 0;
    double range = 0.0;    // m
    double bearing = 0.0;  // rad, relative to rover heading (+ = left)
};

struct HazardRay {
    double angle = 0.0;  // rad relative to heading
    double range = 0.0;  // m to first hazard, or max range if clear
    bool hit = false;
    bool rock = false;   // true: rock too tall to climb, false: slope too steep
};

struct ScienceResult {
    int target_id = -1;
    double value = 0.0;   // measured science value (noisy)
    std::string kind;     // measured mineralogy
    double t = 0.0;
};

struct UplinkMessage {
    double t_received = 0.0;
    std::string type;     // "priority" or "note"
    int target_id = -1;
    double value = 0.0;
    std::string text;
};

struct Observation {
    double t = 0.0;            // mission time, s
    double local_hour = 0.0;   // Local mean solar time
    double dt = 0.0;           // since previous on_step

    // Proprioception
    double wheel_angle_left = 0.0, wheel_angle_right = 0.0;  // cumulative encoder angle, rad
    double wheel_rate_left = 0.0, wheel_rate_right = 0.0;    // rad/s
    double gyro_z = 0.0;     // yaw rate, rad/s (biased, noisy)
    double pitch = 0.0;      // rad, + = nose up (accelerometer, noisy)
    double roll = 0.0;       // rad, + = left side up

    // Exteroception
    bool sun_heading_valid = false;
    double sun_heading = 0.0;              // absolute heading from sun sensor (day only)
    std::vector<LandmarkObs> landmarks;    // catalogued boulders in view (id known)
    std::vector<HazardRay> hazards;        // forward hazard scanner
    // Visual odometry (stereo feature tracking, ~1 Hz, daylight only). Gives the
    // rover's true displacement since the previous VO frame, in that frame's body
    // axes, with ~2% noise. Fails often on featureless sand.
    bool vo_valid = false;
    double vo_dx = 0.0, vo_dy = 0.0, vo_dheading = 0.0, vo_dt = 0.0;

    // Power and thermal
    double battery_soc = 0.0;   // 0..1 of rated capacity
    double battery_wh = 0.0;
    double solar_w = 0.0;
    double load_w = 0.0;
    double box_temp_c = 0.0;
    double ambient_temp_c = 0.0;
    double sun_elevation = 0.0;  // rad

    // Science and comms
    std::vector<ScienceResult> new_results;
    bool sampling = false;
    int sampling_target = -1;
    double sample_progress = 0.0;  // 0..1
    bool relay_in_view = false;
    double data_queued_mbit = 0.0;
    std::vector<UplinkMessage> inbox;
    std::vector<std::string> events;  // collisions, wheel damage, warnings since last step
};

struct Command {
    double wheel_speed_left = 0.0;   // rad/s, clamped to motor no-load speed
    double wheel_speed_right = 0.0;
    int sample_target = -1;          // >= 0: run spectrometer on this target (must be stopped and in range)
    bool transmit = false;           // downlink queued data if a relay is in view
    double heater_w = -1.0;          // < 0: automatic thermostat; else commanded watts
    double sleep_s = 0.0;            // > 0: power down for this long (on_step not called meanwhile)
    bool end_mission = false;

    // Debug / visualisation only — not used by the physics.
    bool has_estimate = false;
    double est_x = 0.0, est_y = 0.0, est_heading = 0.0, est_sigma = 0.0;
    bool path_updated = false;
    std::vector<Vec2> planned_path;
    std::string status;  // short label, shown in the replay viewer
    std::string log;     // appended to the run log

    // Convenience: set wheel speeds from a body speed (m/s) and yaw rate (rad/s)
    // using skid-steer kinematics.
    void drive(double speed, double yaw_rate, const RoverInfo& rover);
};

// ---------------------------------------------------------------- interface

class Controller {
public:
    virtual ~Controller() = default;
    virtual void on_start(const MissionBrief& brief) = 0;
    virtual Command on_step(const Observation& obs) = 0;
    virtual void on_end() {}
};

using ControllerFactory = std::function<std::unique_ptr<Controller>()>;

struct ControllerEntry {
    std::string name;
    std::string description;
    ControllerFactory factory;
};

class ControllerRegistry {
public:
    static ControllerRegistry& instance();
    void add(ControllerEntry e);
    const std::vector<ControllerEntry>& entries() const { return entries_; }
    std::unique_ptr<Controller> create(const std::string& name) const;

private:
    std::vector<ControllerEntry> entries_;
};

struct ControllerRegistrar {
    ControllerRegistrar(const char* name, const char* description, ControllerFactory f) {
        ControllerRegistry::instance().add({name, description, std::move(f)});
    }
};

}  // namespace aximetry

#define AXIMETRY_REGISTER_CONTROLLER(CLASS, NAME, DESCRIPTION)                                   \
    static ::aximetry::ControllerRegistrar aximetry_registrar_##CLASS(NAME, DESCRIPTION, [] {    \
        return std::unique_ptr<::aximetry::Controller>(new CLASS());                             \
    })
