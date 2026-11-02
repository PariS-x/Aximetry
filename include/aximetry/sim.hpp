// The simulator: runs a rover design + controller through a mission.
//
// Per physics step:
//   terrain attitude → wheel normal loads (with weight transfer) → per-wheel
//   Bekker–Wong contact → backward-Euler solve of the coupled forward/yaw
//   dynamics → pose integration → rock collisions and rock strikes →
//   structural checks → motor power.
// Slower processes (sun, dust, temperatures, battery, heaters, comms,
// spectrometer) run on the same clock, and the controller is called at 5 Hz
// with noisy sensor data only.
#pragma once

#include <array>
#include <ostream>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "aximetry/controller.hpp"
#include "aximetry/environment.hpp"
#include "aximetry/mission.hpp"
#include "aximetry/rover.hpp"
#include "aximetry/terrain.hpp"

namespace aximetry {

struct SimOptions {
    double physics_dt = 0.1;    // s, while moving (backward Euler keeps this stable)
    double control_dt = 0.2;    // s, controller rate (5 Hz)
    double sleep_dt = 30.0;     // s, step while asleep / browned out
    double trace_interval_s = 2.0;
    double sleep_trace_interval_s = 300.0;
    bool record_trace = true;
    bool verbose = false;       // print events as they happen
    uint64_t noise_seed = 0;    // 0 → derived from mission seed
};

enum class RoverMode { Awake, Sleeping, Brownout, Dead };

struct SimEvent {
    double t = 0.0;
    std::string type;
    std::string message;
};

struct TraceSample {
    double t, x, y, z, heading;
    bool has_est;
    double est_x, est_y, est_heading, est_sigma;
    double v, soc, solar_w, load_w, box_c, ambient_c, tau, slip_l, slip_r, sinkage_cm, pitch_deg, roll_deg;
    int mode;  // 0 drive/awake, 1 sleeping, 2 brownout, 3 dead, 4 sampling, 5 transmitting
    std::string status;
};

struct PathSnapshot {
    double t;
    std::vector<Vec2> pts;
};

struct ScienceRecord {
    int target_id = -1;
    double measured_value = 0.0;
    double true_value = 0.0;
    double t_measured = 0.0;
    double t_downlinked = -1.0;
    bool received = false;  // reached Earth before mission end
};

struct RunResult {
    std::string rover_name, mission_name, controller_name;
    bool failed = false;
    std::string failure_type;
    std::string failure_message;
    std::string lesson;  // the real-mission parallel, for the pedagogy side

    double score = 0.0;
    double science_points = 0.0;
    double goal_points = 0.0;
    bool goal_reached = false;
    int targets_measured = 0;
    int targets_received = 0;

    double sim_time_s = 0.0;
    double wall_time_s = 0.0;
    double distance_m = 0.0;
    double drive_energy_wh = 0.0;
    double heater_energy_wh = 0.0;
    double solar_energy_wh = 0.0;
    double max_slip = 0.0;
    double max_sinkage_cm = 0.0;
    double min_box_temp_c = 1e9;
    double end_soc = 0.0;
    int collisions = 0;
    int rock_strikes = 0;
    int wheels_failed = 0;
    double max_wheel_damage = 0.0;

    std::vector<SimEvent> events;
    std::vector<TraceSample> trace;
    std::vector<PathSnapshot> paths;
    std::vector<std::string> log;
    std::vector<ScienceRecord> science;
};

class Simulator {
public:
    Simulator(const RoverModel& rover, const Mission& mission, Controller& controller, std::string controller_name,
              SimOptions opt = {});

    RunResult run();

    const Terrain& terrain() const { return terrain_; }
    const MissionBrief& brief() const { return brief_; }
    const Mission& mission() const { return mission_; }
    const RoverModel& rover() const { return rover_; }
    const Environment& environment() const { return env_; }
    const std::vector<RelayPass>& relay_passes() const { return brief_.relay_passes; }

private:
    // ---- setup
    void build_brief();

    // ---- physics
    struct WheelState {
        Vec2 world;          // contact point
        double load = 0.0;   // N
        int soil = 0;
        double slip = 0.0;
        double sinkage = 0.0;
        double thrust = 0.0;
        double resistance = 0.0;
        double torque = 0.0;
        double rate = 0.0;   // actual angular speed
        double damage = 0.0; // Miner sum (rock strikes)
        bool failed = false;
        bool limited = false;
    };
    struct StepContext {
        double pitch = 0.0, roll = 0.0;
        double cmd[2] = {0, 0};      // commanded wheel speeds L, R
        double torque_cap[2] = {0, 0};
        double z0[2][3] = {{0, 0, 0}, {0, 0, 0}};  // cached static sinkage per wheel
    };
    void physics_step(double dt);
    void compute_attitude_and_loads(StepContext& ctx);
    void wheel_forces(double v, double wz, const StepContext& ctx, double& F, double& M, bool commit);
    void handle_rocks(const Vec2& old_pos, double old_heading);
    void check_structure(double dynamic_factor);
    void update_power_thermal(double dt, double motor_w);
    void update_science_comms(double dt);
    Observation make_observation(double dt_since_last);
    void sense_hazards(Observation& obs);
    void sense_landmarks(Observation& obs);
    void record_trace(bool force);
    void event(const std::string& type, const std::string& msg);
    void fail(const std::string& type, const std::string& msg, const std::string& lesson);
    bool relay_in_view(double t) const;
    double relay_rate_kbps(double t) const;
    Vec2 wheel_world(int side, int axle) const;

    RoverModel rover_;
    Mission mission_;
    Controller& controller_;
    std::string controller_name_;
    SimOptions opt_;
    Terrain terrain_;
    Environment env_;
    MissionBrief brief_;
    Rng noise_;

    // ---- true state
    double t_ = 0.0;
    Vec2 pos_;
    double heading_ = 0.0;
    double v_ = 0.0, wz_ = 0.0;
    double pitch_ = 0.0, roll_ = 0.0;
    std::array<std::array<WheelState, 3>, 2> wheels_{};  // [side][axle], side 0 = left
    double wheel_angle_[2] = {0, 0};
    double battery_wh_ = 0.0;
    double box_c_ = 0.0;
    double dust_factor_ = 1.0;
    double gyro_bias_ = 0.0;
    RoverMode mode_ = RoverMode::Awake;
    double sleep_until_ = 0.0;
    Command cmd_;
    double last_solar_w_ = 0.0, last_load_w_ = 0.0, last_heater_w_ = 0.0;
    bool transmitting_ = false;

    // science / comms
    int sampling_target_ = -1;
    double sample_progress_s_ = 0.0;
    std::vector<int> queue_;            // indices into result_.science awaiting downlink
    double tx_bits_into_head_ = 0.0;
    std::vector<ScienceResult> pending_results_;
    std::vector<UplinkMessage> pending_inbox_;
    std::vector<bool> uplink_delivered_;
    std::vector<std::string> pending_events_;
    std::set<int> measured_;

    // contact bookkeeping
    std::set<std::pair<int, int>> touching_;  // (wheel index, rock id)
    int colliding_rock_ = -1;
    int last_reported_rock_ = -1;
    double last_collision_t_ = -1e9;
    double stuck_timer_ = 0.0;
    bool embedded_reported_ = false;
    double last_trace_t_ = -1e9;
    double last_landmark_t_ = -1e9;
    double obs_heading_ = 0.0;
    double obs_wheel_angle_[2] = {0, 0};
    double vo_t_ = 0.0;
    Vec2 vo_pos_;
    double vo_heading_ = 0.0;
    void sense_visual_odometry(Observation& obs);
    double since_obs_ = 0.0;
    std::vector<int> scratch_;

    RunResult result_;
};

// Serialise a run (terrain, rocks, targets, trace, events) for the replay viewer.
void write_replay_json(const Simulator& sim, const RunResult& r, std::ostream& os);
std::string format_result(const RunResult& r, bool color = true);

}  // namespace aximetry
