#include "aximetry/sim.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <sstream>

namespace aximetry {

namespace {
constexpr double kSpeedEps = 2e-3;   // m/s, regularisation of Coulomb-like terms
constexpr double kYawEps = 0.01;     // rad/s
constexpr double kSpectrometerW = 12.0;
constexpr double kRadioW = 25.0;
constexpr double kHeaterGain = 10.0;  // W per K for the automatic thermostat
constexpr int kNumHazardRays = 21;
constexpr double kHazardFov = 100.0;  // deg
constexpr double kHazardRange = 5.0;  // m
constexpr double kLandmarkRange = 15.0;
constexpr double kLandmarkFov = 120.0;

std::string fmtd(const char* f, double a) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), f, a);
    return buf;
}
std::string fmtd2(const char* f, double a, double b) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), f, a, b);
    return buf;
}
}  // namespace

Simulator::Simulator(const RoverModel& rover, const Mission& mission, Controller& controller, std::string controller_name,
                     SimOptions opt)
    : rover_(rover),
      mission_(mission),
      controller_(controller),
      controller_name_(std::move(controller_name)),
      opt_(opt),
      terrain_(Terrain::generate(mission.terrain, mission.seed)),
      env_(mission.environment, mission.start_local_hour),
      noise_(opt.noise_seed ? opt.noise_seed : mission.seed * 7919 + 17) {
    build_brief();
}

// --------------------------------------------------------------------------- brief
void Simulator::build_brief() {
    MissionBrief& b = brief_;
    b.mission_name = mission_.name;
    b.description = mission_.description;
    b.duration_s = mission_.duration_s();
    b.start_local_hour = mission_.start_local_hour;
    b.sol_seconds = mars::kSolSeconds;
    Rng r(mission_.seed * 31 + 5);
    b.start_pos_sigma = 0.5;
    b.start_pos = mission_.start + Vec2(r.normal(0, b.start_pos_sigma), r.normal(0, b.start_pos_sigma));
    b.start_heading = wrap_angle(deg2rad(mission_.start_heading_deg) + r.normal(0, deg2rad(2.0)));
    b.has_goal = mission_.has_goal;
    b.goal = mission_.goal;
    b.goal_radius = mission_.goal_radius;
    b.goal_bonus = mission_.goal_bonus;
    b.points_per_value = mission_.points_per_value;

    // Orbital map: 2 m DEM with noise, soil classes with errors.
    OrbitalMap& m = b.map;
    m.size_m = terrain_.size();
    m.resolution_m = 2.0;
    m.nx = m.ny = static_cast<int>(std::round(m.size_m / m.resolution_m)) + 1;
    for (const auto& s : terrain_.soils()) m.soil_ids.push_back(s.id);
    m.height.resize(m.nx * m.ny);
    m.soil_guess.resize(m.nx * m.ny);
    for (int j = 0; j < m.ny; ++j)
        for (int i = 0; i < m.nx; ++i) {
            const Vec2 p(i * m.resolution_m, j * m.resolution_m);
            m.height[j * m.nx + i] = static_cast<float>(terrain_.height(p) + r.normal(0, 0.08));
            int s = terrain_.soil_index(p);
            if (r.chance(0.15)) s = 0;  // thermal-inertia mapping misses some patches
            m.soil_guess[j * m.nx + i] = static_cast<uint8_t>(s);
        }
    for (const auto& rock : terrain_.rocks())
        if (rock.catalogued)
            b.landmarks.push_back({rock.id, rock.pos + Vec2(r.normal(0, 0.05), r.normal(0, 0.05)), rock.radius});
    for (const auto& t : terrain_.targets()) b.targets.push_back({t.id, t.pos, t.orbital_kind, t.orbital_prior});

    // Relay passes for every sol of the mission.
    const double T = b.duration_s;
    for (int sol = -1; sol <= static_cast<int>(mission_.duration_sols) + 1; ++sol)
        for (const auto& rs : mission_.relays) {
            const double start = ((sol * 24.0 + rs.local_hour) - mission_.start_local_hour) / 24.0 * mars::kSolSeconds;
            const double dur = rs.minutes * 60.0;
            if (start + dur > 0.0 && start < T) b.relay_passes.push_back({start, dur, rs.rate_kbps});
        }
    std::sort(b.relay_passes.begin(), b.relay_passes.end(),
              [](const RelayPass& a, const RelayPass& c) { return a.start_s < c.start_s; });
    b.owlt_s = mission_.owlt_min * 60.0;
    b.storage_mbit = mission_.storage_mbit;
    b.product_mbit = mission_.product_mbit;
    b.sample_duration_s = mission_.sample_duration_s;
    b.sample_range_m = mission_.sample_range_m;

    RoverInfo& ri = b.rover;
    ri.name = rover_.design.name;
    ri.mass_kg = rover_.mass;
    ri.wheel_radius = rover_.wheel_radius();
    ri.track_width = rover_.design.track_width;
    ri.wheelbase = rover_.design.wheelbase;
    ri.max_wheel_speed = rover_.motor.no_load_speed;
    ri.battery_wh = rover_.design.battery_wh;
    ri.solar_area_m2 = rover_.design.solar_area;
    ri.footprint_radius = rover_.footprint_radius;
    ri.climb_limit_m = rover_.climb_limit;
    ri.tip_angle_deg = rad2deg(std::min(rover_.tip_angle_lateral, rover_.tip_angle_longitudinal));
    ri.skid_factor = rover_.design.wheel_count == 6 ? 1.6 : 1.4;
}

// --------------------------------------------------------------------------- helpers
void Simulator::event(const std::string& type, const std::string& msg) {
    result_.events.push_back({t_, type, msg});
    pending_events_.push_back(type + ": " + msg);
    if (opt_.verbose) std::printf("[t=%8.1fs  %05.2fh] %-16s %s\n", t_, env_.local_hour(t_), type.c_str(), msg.c_str());
}

void Simulator::fail(const std::string& type, const std::string& msg, const std::string& lesson) {
    if (result_.failed) return;
    result_.failed = true;
    result_.failure_type = type;
    result_.failure_message = msg;
    result_.lesson = lesson;
    event("FAILURE", msg);
}

bool Simulator::relay_in_view(double t) const {
    for (const auto& p : brief_.relay_passes)
        if (t >= p.start_s && t < p.start_s + p.duration_s) return true;
    return false;
}

double Simulator::relay_rate_kbps(double t) const {
    for (const auto& p : brief_.relay_passes)
        if (t >= p.start_s && t < p.start_s + p.duration_s) return p.rate_kbps;
    return 0.0;
}

Vec2 Simulator::wheel_world(int side, int axle) const {
    const double y = (side == 0 ? 0.5 : -0.5) * rover_.design.track_width;
    return pos_ + Vec2(rover_.axle_x[axle], y).rotated(heading_);
}

// --------------------------------------------------------------------------- physics
void Simulator::compute_attitude_and_loads(StepContext& ctx) {
    const Vec2 g = terrain_.gradient(pos_);
    const Vec2 fwd = Vec2::from_angle(heading_);
    const Vec2 left(-fwd.y, fwd.x);
    ctx.pitch = std::atan(g.dot(fwd));
    ctx.roll = std::atan(g.dot(left));
    pitch_ = ctx.pitch;
    roll_ = ctx.roll;

    const double m = rover_.mass, G = mars::kGravity, h = rover_.cg_height;
    const double B = rover_.design.track_width;
    const int na = rover_.wheels_per_side();

    // Tip-over: the weight vector leaves the support polygon.
    if (std::fabs(ctx.roll) > rover_.tip_angle_lateral || std::fabs(ctx.pitch) > rover_.tip_angle_longitudinal) {
        fail("tip_over",
             fmtd2("Rover tipped over (pitch %.0f deg, roll %.0f deg).", rad2deg(ctx.pitch), rad2deg(ctx.roll)),
             "The centre of gravity passed outside the wheels. Rovers are built low and wide, and drivers keep a big margin "
             "below the tip angle because slopes measured from orbit hide local steps and ripples.");
        return;
    }

    const double N = m * G * std::cos(ctx.pitch) * std::cos(ctx.roll);
    double Wside[2];
    Wside[0] = N / 2.0 - m * G * std::sin(ctx.roll) * h / B;  // left side lighter when left is uphill
    Wside[1] = N - Wside[0];
    double sumx2 = 0.0;
    for (double x : rover_.axle_x) sumx2 += x * x;
    std::vector<double> Waxle(na);
    double wsum = 0.0;
    for (int i = 0; i < na; ++i) {
        Waxle[i] = std::max(0.0, N / na - m * G * std::sin(ctx.pitch) * h * rover_.axle_x[i] / sumx2);
        wsum += Waxle[i];
    }
    for (int s = 0; s < 2; ++s)
        for (int i = 0; i < na; ++i) {
            auto& w = wheels_[s][i];
            w.world = wheel_world(s, i);
            w.load = std::max(0.0, Waxle[i] / wsum * N * (Wside[s] / N));
            w.soil = terrain_.soil_index(w.world);
            ctx.z0[s][i] = bekker_static_sinkage(terrain_.soils()[w.soil], w.load, rover_.wheel);
        }

    const bool powered = (mode_ == RoverMode::Awake);
    for (int s = 0; s < 2; ++s) {
        const double w = ctx.cmd[s];
        if (!powered || std::fabs(w) < 1e-9) {
            ctx.torque_cap[s] = rover_.motor.stall_torque;  // gearbox holds
        } else {
            // Torque follows the motor curve at the speed the wheels are actually
            // turning: a loaded, slowed-down motor produces more torque.
            double actual = 0.0;
            for (int i = 0; i < na; ++i) actual += std::fabs(wheels_[s][i].rate);
            actual /= na;
            ctx.torque_cap[s] = std::max(rover_.available_torque(std::min(std::fabs(w), actual)), 0.05 * rover_.motor.stall_torque);
        }
    }
}

void Simulator::wheel_forces(double v, double wz, const StepContext& ctx, double& F, double& M, bool commit) {
    const double B = rover_.design.track_width;
    const double r = rover_.wheel_radius();
    const int na = rover_.wheels_per_side();
    double Fs[2] = {0, 0};
    double Mlat = 0.0;
    for (int s = 0; s < 2; ++s) {
        const double vs = v + (s == 0 ? -1.0 : 1.0) * wz * B / 2.0;
        const double rim = ctx.cmd[s] * r;
        const double cap = ctx.torque_cap[s] / r;
        for (int i = 0; i < na; ++i) {
            auto& w = wheels_[s][i];
            const Soil& soil = terrain_.soils()[w.soil];
            double H = 0.0, Rc = 0.0, slip = 0.0, sink = 0.0;
            bool limited = false;
            if (w.load > 0.0) {
                if (w.failed) {
                    // A broken wheel drags: no thrust, ploughing resistance.
                    WheelContact c = evaluate_contact_z0(soil, w.load, rover_.wheel, 0.0, ctx.z0[s][i]);
                    Rc = (3.0 * c.compaction + 0.3 * w.load) * std::tanh(vs / kSpeedEps);
                    sink = c.sinkage;
                } else {
                    slip = slip_ratio(rim, vs);
                    WheelContact c = evaluate_contact_z0(soil, w.load, rover_.wheel, slip, ctx.z0[s][i]);
                    H = c.thrust;
                    if (std::fabs(H) > cap) {
                        H = std::copysign(cap, H);
                        limited = true;
                    }
                    Rc = c.compaction * std::tanh(vs / kSpeedEps);
                    sink = c.sinkage;
                }
                Mlat += std::fabs(rover_.axle_x[i]) * w.load * std::tan(soil.phi) * 0.6;
            }
            Fs[s] += H - Rc;
            if (commit) {
                w.slip = slip;
                w.sinkage = sink;
                w.thrust = H;
                w.resistance = Rc;
                w.torque = H * r;
                w.limited = limited;
            }
        }
    }
    F = Fs[0] + Fs[1] - rover_.mass * mars::kGravity * std::sin(ctx.pitch);
    M = (Fs[1] - Fs[0]) * B / 2.0 - Mlat * std::tanh(wz / kYawEps);
}

void Simulator::physics_step(double dt) {
    StepContext ctx;
    const double w0 = rover_.motor.no_load_speed;
    ctx.cmd[0] = clampd(cmd_.wheel_speed_left, -w0, w0);
    ctx.cmd[1] = clampd(cmd_.wheel_speed_right, -w0, w0);
    if (mode_ != RoverMode::Awake) ctx.cmd[0] = ctx.cmd[1] = 0.0;

    compute_attitude_and_loads(ctx);
    if (result_.failed) return;

    // Parked with brakes on: if locked wheels can hold the slope, the rover is
    // simply static (this also keeps long overnight steps exact).
    if (ctx.cmd[0] == 0.0 && ctx.cmd[1] == 0.0 && std::fabs(v_) < 1e-3 && std::fabs(wz_) < 1e-3) {
        double hold = 0.0;
        for (int s = 0; s < 2; ++s)
            for (int i = 0; i < rover_.wheels_per_side(); ++i) {
                const auto& w = wheels_[s][i];
                if (w.load <= 0.0) continue;
                const Soil& soil = terrain_.soils()[w.soil];
                WheelContact c = evaluate_contact_z0(soil, w.load, rover_.wheel, -1.0, ctx.z0[s][i]);
                hold += std::fabs(c.thrust);
            }
        if (rover_.mass * mars::kGravity * std::fabs(std::sin(ctx.pitch)) < 0.9 * hold) {
            v_ = wz_ = 0.0;
            for (auto& side : wheels_)
                for (auto& w : side) { w.slip = 0.0; w.rate = 0.0; w.thrust = 0.0; w.torque = 0.0; w.limited = false; }
            check_structure(1.0);
            if (result_.failed) return;
            update_power_thermal(dt, 0.0);
            update_science_comms(dt);
            t_ += dt;
            return;
        }
    }

    // Backward Euler on (v, wz): stable for the stiff slip–force coupling and
    // for the long steps used while parked overnight.
    const double m = rover_.mass, I = rover_.inertia_z;
    double x0 = v_, x1 = wz_;
    auto residual = [&](double a, double b, double& r0, double& r1) {
        double F, M;
        wheel_forces(a, b, ctx, F, M, false);
        r0 = a - v_ - dt * F / m;
        r1 = b - wz_ - dt * M / I;
    };
    double r0, r1;
    residual(x0, x1, r0, r1);
    for (int it = 0; it < 25; ++it) {
        const double norm = std::hypot(r0, r1);
        if (norm < 1e-8) break;
        const double h = 1e-6;
        double a0, a1, b0, b1;
        residual(x0 + h, x1, a0, a1);
        residual(x0, x1 + h, b0, b1);
        Mat<2, 2> J;
        J(0, 0) = (a0 - r0) / h; J(1, 0) = (a1 - r1) / h;
        J(0, 1) = (b0 - r0) / h; J(1, 1) = (b1 - r1) / h;
        const double det = J(0, 0) * J(1, 1) - J(0, 1) * J(1, 0);
        if (std::fabs(det) < 1e-14) break;
        const Mat<2, 2> Ji = inverse(J);
        const double d0 = -(Ji(0, 0) * r0 + Ji(0, 1) * r1);
        const double d1 = -(Ji(1, 0) * r0 + Ji(1, 1) * r1);
        double alpha = 1.0;
        bool improved = false;
        for (int ls = 0; ls < 12; ++ls) {
            double n0, n1;
            residual(x0 + alpha * d0, x1 + alpha * d1, n0, n1);
            if (std::hypot(n0, n1) < norm) {
                x0 += alpha * d0; x1 += alpha * d1;
                r0 = n0; r1 = n1;
                improved = true;
                break;
            }
            alpha *= 0.5;
        }
        if (!improved) break;
    }
    v_ = x0;
    wz_ = x1;
    double F, M;
    wheel_forces(v_, wz_, ctx, F, M, true);

    // Actual wheel speeds (what the encoders see).
    const double B = rover_.design.track_width;
    const double r = rover_.wheel_radius();
    double side_rate[2] = {0, 0};
    for (int s = 0; s < 2; ++s) {
        const double vs = v_ + (s == 0 ? -1.0 : 1.0) * wz_ * B / 2.0;
        int count = 0;
        for (int i = 0; i < rover_.wheels_per_side(); ++i) {
            auto& w = wheels_[s][i];
            double rate = ctx.cmd[s];
            if (w.failed) rate = vs / r;
            else if (w.limited) {
                if (std::fabs(ctx.cmd[s]) < 1e-9) rate = 0.5 * vs / r;  // brakes slipping
                else {
                    // Find slip where soil thrust equals the torque limit.
                    WheelContact c = evaluate_contact(terrain_.soils()[w.soil], w.load, rover_.wheel, 0.5);
                    const double target = std::fabs(w.thrust);
                    double lo = 0.0, hi = 1.0;
                    for (int k = 0; k < 30; ++k) {
                        const double mid = 0.5 * (lo + hi);
                        if (janosi_thrust(c.max_thrust, c.contact_length, terrain_.soils()[w.soil].K, mid) < target) lo = mid;
                        else hi = mid;
                    }
                    const double vmag = std::fabs(vs);
                    rate = std::copysign(std::min(std::fabs(ctx.cmd[s]), vmag / (r * std::max(1e-3, 1.0 - lo))), ctx.cmd[s]);
                    if (vs * ctx.cmd[s] < 0) rate = 0.0;  // rolling backwards while trying to drive: stalled
                }
            }
            w.rate = rate;
            side_rate[s] += rate;
            ++count;
        }
        side_rate[s] /= std::max(1, count);
        wheel_angle_[s] += side_rate[s] * dt;
    }

    // Pose integration on the terrain.
    const Vec2 old_pos = pos_;
    const double old_heading = heading_;
    const double mid_heading = heading_ + 0.5 * wz_ * dt;
    heading_ = wrap_angle(heading_ + wz_ * dt);
    pos_ += Vec2::from_angle(mid_heading) * (v_ * std::cos(ctx.pitch) * dt);
    if (!terrain_.in_bounds(pos_, rover_.footprint_radius)) {
        pos_ = old_pos;
        v_ = 0.0;
        event("edge_of_map", "Reached the edge of the mapped area.");
    }
    if (std::fabs(v_) > 1e-5 || std::fabs(wz_) > 1e-5) handle_rocks(old_pos, old_heading);
    result_.distance_m += distance(old_pos, pos_);

    // Statistics and stuck detection.
    double sink_sum = 0.0;
    int n = 0;
    for (int s = 0; s < 2; ++s)
        for (int i = 0; i < rover_.wheels_per_side(); ++i) {
            const auto& w = wheels_[s][i];
            if (std::fabs(ctx.cmd[s]) > 1e-6) result_.max_slip = std::max(result_.max_slip, std::fabs(w.slip));
            result_.max_sinkage_cm = std::max(result_.max_sinkage_cm, w.sinkage * 100.0);
            sink_sum += w.sinkage;
            ++n;
        }
    const bool commanding = std::fabs(ctx.cmd[0]) + std::fabs(ctx.cmd[1]) > 0.05;
    if (commanding && std::fabs(v_) < 0.002 && std::fabs(wz_) < 0.005) stuck_timer_ += dt;
    else stuck_timer_ = 0.0;
    if (stuck_timer_ > 120.0 && !embedded_reported_ && t_ - last_collision_t_ > 30.0 && sink_sum / n > 0.25 * rover_.wheel_radius()) {
        embedded_reported_ = true;
        event("embedded", fmtd("Wheels spinning in place, sunk %.0f cm. The rover is embedded in soft soil.", 100.0 * sink_sum / n));
    }
    if (embedded_reported_ && stuck_timer_ == 0.0 && std::fabs(v_) > 0.005) embedded_reported_ = false;

    if (mission_.has_goal && !result_.goal_reached && distance(pos_, mission_.goal) <= mission_.goal_radius) {
        result_.goal_reached = true;
        event("goal_reached", "Rover reached the goal region.");
    }

    check_structure(std::fabs(v_) > 0.005 ? 1.5 : 1.0);
    if (result_.failed) return;

    double motor_w = 0.0;
    if (mode_ == RoverMode::Awake) {
        for (int s = 0; s < 2; ++s)
            for (int i = 0; i < rover_.wheels_per_side(); ++i) {
                const auto& w = wheels_[s][i];
                if (w.failed || std::fabs(ctx.cmd[s]) < 1e-9) continue;
                motor_w += rover_.motor_power(w.torque, w.rate) + rover_.motor.idle_power;
            }
    }
    result_.drive_energy_wh += motor_w * dt / 3600.0;
    update_power_thermal(dt, motor_w);
    update_science_comms(dt);
    t_ += dt;
}

void Simulator::handle_rocks(const Vec2& old_pos, double old_heading) {
    const double half_len = rover_.design.wheelbase / 2.0 + rover_.wheel_radius();
    const double half_wid = (rover_.design.track_width + rover_.design.wheel_width) / 2.0;
    terrain_.rocks_near(pos_, rover_.footprint_radius + 1.5, scratch_);
    const double ambient = env_.ambient_temp_c(t_);

    // Large rocks: hard obstacles, resolved like a rigid contact. If a step
    // pushes the footprint deeper into a rock, the rover is pushed back out
    // along the shallower body axis, so it can still slide along or turn away.
    auto penetration = [&](const Vec2& p, double h, const Rock& rk, double* px_out = nullptr, double* py_out = nullptr,
                           Vec2* pb_out = nullptr) {
        const Vec2 pb = (rk.pos - p).rotated(-h);
        const double px = half_len + rk.radius * 0.8 - std::fabs(pb.x);
        const double py = half_wid + rk.radius * 0.8 - std::fabs(pb.y);
        if (px_out) *px_out = px;
        if (py_out) *py_out = py;
        if (pb_out) *pb_out = pb;
        return (px > 0.0 && py > 0.0) ? std::min(px, py) : 0.0;
    };
    bool contact = false;
    for (int id : scratch_) {
        const Rock& rk = terrain_.rocks()[id];
        if (rk.height <= rover_.climb_limit) continue;
        double px, py;
        Vec2 pb;
        const double now = penetration(pos_, heading_, rk, &px, &py, &pb);
        const double before = penetration(old_pos, old_heading, rk);
        if (now <= before + 1e-6) continue;
        const double excess = now - before;
        if (px < py) {
            pos_ += Vec2(pb.x > 0 ? -excess : excess, 0.0).rotated(heading_);
            if (v_ * pb.x > 0.0) v_ = 0.0;  // stop pushing into it
        } else {
            pos_ += Vec2(0.0, pb.y > 0 ? -excess : excess).rotated(heading_);
        }
        contact = true;
        if (colliding_rock_ != id) {
            colliding_rock_ = id;
            // Count/report a rock once, unless the rover backed away from it for a while.
            if (last_reported_rock_ != id || t_ - last_collision_t_ > 120.0) {
                ++result_.collisions;
                event("collision", fmtd2("Blocked by a %.0f cm rock (can climb %.0f cm).", rk.height * 100.0, rover_.climb_limit * 100.0));
            }
            last_reported_rock_ = id;
        }
        last_collision_t_ = t_;
    }
    if (contact) return;
    colliding_rock_ = -1;

    // Small rocks: driven over, each one an impact on a wheel rim.
    std::set<std::pair<int, int>> now;
    for (int s = 0; s < 2; ++s)
        for (int i = 0; i < rover_.wheels_per_side(); ++i) {
            auto& w = wheels_[s][i];
            if (w.failed) continue;
            const Vec2 wp = wheel_world(s, i);
            for (int id : scratch_) {
                const Rock& rk = terrain_.rocks()[id];
                if (rk.height > rover_.climb_limit) continue;
                if (distance(wp, rk.pos) > rk.radius + rover_.design.wheel_width / 2.0) continue;
                const auto key = std::make_pair(s * 3 + i, id);
                now.insert(key);
                if (touching_.count(key)) continue;
                ++result_.rock_strikes;
                const double impact = 1.5 + 2.0 * rk.height / rover_.design.wheel_diameter;
                const double sigma = rover_.rim_stress(w.load * impact);
                const double strength = RoverModel::strength_at(rover_.wheel_mat, ambient, true);
                const char* side = s == 0 ? "left" : "right";
                const char* axle = (i == 0 ? "front" : (i == rover_.wheels_per_side() - 1 ? "rear" : "middle"));
                if (sigma > strength) {
                    w.failed = true;
                    ++result_.wheels_failed;
                    event("wheel_failed", std::string("The ") + axle + " " + side + " wheel rim fractured on a " +
                                              fmtd("%.0f cm rock", rk.height * 100.0) +
                                              fmtd2(" (%.0f MPa vs %.0f MPa strength).", sigma / 1e6, strength / 1e6));
                } else {
                    w.damage += std::pow(sigma / strength, rover_.wheel_mat.fatigue_exponent);
                    result_.max_wheel_damage = std::max(result_.max_wheel_damage, w.damage);
                    if (w.damage >= 1.0) {
                        w.failed = true;
                        ++result_.wheels_failed;
                        event("wheel_failed", std::string("The ") + axle + " " + side +
                                                  " wheel failed from accumulated rock-strike damage (fatigue).");
                    }
                }
            }
        }
    touching_.swap(now);
}

void Simulator::check_structure(double df) {
    const double strut_pcr = rover_.strut_buckling_load();
    for (int s = 0; s < 2; ++s)
        for (int i = 0; i < rover_.wheels_per_side(); ++i) {
            const auto& w = wheels_[s][i];
            const double P = w.load * df;
            if (P <= 0.0) continue;
            const double rim = rover_.rim_stress(P);
            if (!w.failed && rim > rover_.wheel_mat.yield_strength) {
                fail("structure", fmtd2("A wheel rim collapsed: %.0f MPa bending stress vs %.0f MPa strength.", rim / 1e6,
                                        rover_.wheel_mat.yield_strength / 1e6),
                     "Wheels carry the whole rover through a thin rim. Flight wheels are thin but stiff aluminium shells "
                     "supported by flexures; " + rover_.wheel_mat.name + " cannot carry this load at this thickness.");
                return;
            }
            if (P > strut_pcr) {
                fail("structure", fmtd2("A suspension leg buckled: %.0f N load vs %.0f N Euler critical load.", P, strut_pcr),
                     "Slender columns fail by buckling, set by stiffness (E x I / L^2), not by strength. " +
                         rover_.strut_mat.name + " legs this thin fold sideways.");
                return;
            }
            const double ss = rover_.strut_stress(P, w.thrust);
            if (ss > rover_.strut_mat.yield_strength) {
                fail("structure", fmtd2("A suspension leg yielded: %.0f MPa vs %.0f MPa.", ss / 1e6, rover_.strut_mat.yield_strength / 1e6),
                     "Axial load plus the bending from traction at the end of the leg exceeded the material's strength.");
                return;
            }
        }
    const double cs = rover_.chassis_stress(df);
    if (cs > rover_.chassis_mat.yield_strength) {
        fail("structure", fmtd2("The body box broke under its own contents: %.0f MPa vs %.0f MPa.", cs / 1e6,
                                rover_.chassis_mat.yield_strength / 1e6),
             "The warm electronics box carries battery, avionics and instruments as a beam between the suspension pivots.");
    }
}

// --------------------------------------------------------------------------- power & thermal
void Simulator::update_power_thermal(double dt, double motor_w) {
    const auto& d = rover_.design;
    const auto& bat = rover_.battery;
    const double ta = env_.ambient_temp_c(t_);
    dust_factor_ *= std::exp(-env_.config().panel_dust_rate_per_sol * dt / mars::kSolSeconds);
    const double solar = (mode_ == RoverMode::Dead) ? 0.0 : env_.irradiance(t_) * d.solar_area * d.solar_efficiency * dust_factor_;

    double avionics = 0.0, heater = 0.0, other = 0.0;
    if (mode_ == RoverMode::Awake) {
        avionics = d.avionics_awake_w + d.sensors_w;
        if (sampling_target_ >= 0) other += kSpectrometerW;
        if (transmitting_) other += kRadioW;
    } else if (mode_ == RoverMode::Sleeping) {
        avionics = d.avionics_sleep_w;
    }
    if (mode_ == RoverMode::Awake || mode_ == RoverMode::Sleeping) {
        if (cmd_.heater_w < 0.0) heater = clampd(kHeaterGain * (d.thermostat_c - box_c_), 0.0, d.heater_max_w);
        else heater = clampd(cmd_.heater_w, 0.0, d.heater_max_w);
    }
    const double load = avionics + heater + other + motor_w;
    const double net = solar - load;

    const double derate = box_c_ < 20.0 ? clampd(1.0 - bat.cold_derate_per_c * (20.0 - box_c_), 0.05, 1.0) : 1.0;
    const double floor_wh = d.battery_wh * (1.0 - derate);
    if (net >= 0.0) {
        if (box_c_ >= bat.min_charge_c) battery_wh_ = std::min(d.battery_wh, battery_wh_ + net * bat.charge_efficiency * dt / 3600.0);
    } else if (mode_ == RoverMode::Awake || mode_ == RoverMode::Sleeping) {
        const bool can_deliver = box_c_ >= bat.min_discharge_c && battery_wh_ > floor_wh;
        if (can_deliver) battery_wh_ += net * dt / 3600.0;
        if (!can_deliver || battery_wh_ <= floor_wh) {
            battery_wh_ = std::max(battery_wh_, floor_wh);
            mode_ = RoverMode::Brownout;
            v_ = wz_ = 0.0;
            sampling_target_ = -1;
            std::string why;
            if (box_c_ < bat.min_discharge_c)
                why = fmtd("Battery too cold to deliver power (%.0f C).", box_c_);
            else if (floor_wh > 0.1 * d.battery_wh)
                why = fmtd2("Battery still holds %.0f%% charge, but at %.0f C ", 100.0 * battery_wh_ / d.battery_wh, box_c_) +
                      fmtd("this chemistry can only deliver %.0f%% of it.", 100.0 * derate);
            else
                why = "Battery exhausted.";
            event("brownout", why + " Everything shuts off, including heaters.");
        }
    }
    if (mode_ == RoverMode::Brownout && battery_wh_ > floor_wh + 0.15 * d.battery_wh && box_c_ >= bat.min_discharge_c) {
        mode_ = RoverMode::Awake;
        event("recovered", "Solar array recharged the battery: rover rebooted.");
    }

    last_solar_w_ = solar;
    last_load_w_ = load;
    last_heater_w_ = heater;
    result_.solar_energy_wh += solar * dt / 3600.0;
    result_.heater_energy_wh += heater * dt / 3600.0;

    // Warm electronics box: C dT/dt = internal heat - G (T - T_ambient).
    const double internal = avionics + heater + 0.1 * other + d.rhu_count * 1.0;
    box_c_ += dt * (internal - d.box_conductance * (box_c_ - ta)) / rover_.box_heat_capacity;
    result_.min_box_temp_c = std::min(result_.min_box_temp_c, box_c_);
    if (box_c_ < bat.survival_c && mode_ != RoverMode::Dead) {
        mode_ = RoverMode::Dead;
        fail("battery_frozen",
             fmtd2("Battery froze at %.0f C (survival limit %.0f C). The rover is lost.", box_c_, bat.survival_c),
             "Without power the heaters stop, and a Martian night is cold enough to destroy batteries. Opportunity fell "
             "silent in June 2018 during a planet-encircling dust storm that starved its solar panels. Fixes: more battery, "
             "better insulation, RHUs, sleeping earlier, or a chemistry with a lower survival temperature.");
    }
}

// --------------------------------------------------------------------------- science & comms
void Simulator::update_science_comms(double dt) {
    // Spectrometer
    const int want = (mode_ == RoverMode::Awake) ? cmd_.sample_target : -1;
    if (want >= 0 && want < static_cast<int>(terrain_.targets().size()) && !measured_.count(want)) {
        const ScienceTarget& tg = terrain_.targets()[want];
        const bool in_range = distance(pos_, tg.pos) <= mission_.sample_range_m;
        const bool still = std::fabs(v_) < 0.005 && std::fabs(wz_) < 0.005;
        if (in_range && still) {
            if (sampling_target_ != want) {
                sampling_target_ = want;
                sample_progress_s_ = 0.0;
            }
            sample_progress_s_ += dt;
            if (sample_progress_s_ >= mission_.sample_duration_s) {
                measured_.insert(want);
                ScienceRecord rec;
                rec.target_id = want;
                rec.true_value = tg.true_value;
                rec.measured_value = clampd(tg.true_value + noise_.normal(0, 0.04), 0.0, 1.0);
                rec.t_measured = t_;
                result_.science.push_back(rec);
                pending_results_.push_back({want, rec.measured_value, tg.true_kind, t_});
                double queued = 0.0;
                for (size_t k = 0; k < queue_.size(); ++k) queued += mission_.product_mbit;
                if (queued + mission_.product_mbit <= mission_.storage_mbit) queue_.push_back(static_cast<int>(result_.science.size()) - 1);
                else event("storage_full", "Onboard storage full: data product discarded.");
                ++result_.targets_measured;
                event("sample_complete", fmtd("Target %.0f measured: ", want) + tg.true_kind + fmtd(" (value %.2f).", rec.measured_value));
                sampling_target_ = -1;
                sample_progress_s_ = 0.0;
            }
        } else if (sampling_target_ == want) {
            event("sample_aborted", in_range ? "Rover moved during integration." : "Target out of spectrometer range.");
            sampling_target_ = -1;
            sample_progress_s_ = 0.0;
        }
    } else {
        sampling_target_ = -1;
        sample_progress_s_ = 0.0;
    }

    // Downlink through the relay orbiter.
    transmitting_ = mode_ == RoverMode::Awake && cmd_.transmit && !queue_.empty() && relay_in_view(t_);
    if (transmitting_) {
        tx_bits_into_head_ += relay_rate_kbps(t_) * 1000.0 * dt;
        while (!queue_.empty() && tx_bits_into_head_ >= mission_.product_mbit * 1e6) {
            tx_bits_into_head_ -= mission_.product_mbit * 1e6;
            ScienceRecord& rec = result_.science[queue_.front()];
            rec.t_downlinked = t_;
            rec.received = t_ + brief_.owlt_s <= brief_.duration_s;
            queue_.erase(queue_.begin());
            event("downlink", fmtd("Target %.0f data relayed to Earth", rec.target_id) +
                                  (rec.received ? "." : " (arrives after the mission ends)."));
        }
        if (queue_.empty()) tx_bits_into_head_ = 0.0;
    }

    // Uplinks: travel one light-time, then wait for the next relay pass.
    if (uplink_delivered_.size() != mission_.uplinks.size()) uplink_delivered_.assign(mission_.uplinks.size(), false);
    for (size_t k = 0; k < mission_.uplinks.size(); ++k) {
        if (uplink_delivered_[k]) continue;
        const auto& u = mission_.uplinks[k];
        if (t_ >= u.send_at_s + brief_.owlt_s && relay_in_view(t_) && mode_ == RoverMode::Awake) {
            uplink_delivered_[k] = true;
            pending_inbox_.push_back({t_, u.type, u.target_id, u.value, u.text});
            event("uplink", "Message from Earth: " + u.text);
        }
    }
}

// --------------------------------------------------------------------------- sensors
void Simulator::sense_hazards(Observation& obs) {
    const Vec2 fwd = Vec2::from_angle(heading_);
    const Vec2 origin = pos_ + fwd * (rover_.design.wheelbase / 2.0 + rover_.wheel_radius());
    std::vector<int> near;
    terrain_.rocks_near(origin, kHazardRange + 0.5, near);
    std::vector<const Rock*> tall;
    for (int id : near)
        if (terrain_.rocks()[id].height > rover_.climb_limit) tall.push_back(&terrain_.rocks()[id]);
    const double hazard_slope = std::min(25.0, rad2deg(std::min(rover_.tip_angle_lateral, rover_.tip_angle_longitudinal)) - 8.0);
    for (int k = 0; k < kNumHazardRays; ++k) {
        const double rel = deg2rad(-kHazardFov / 2.0 + kHazardFov * k / (kNumHazardRays - 1));
        const Vec2 d = Vec2::from_angle(heading_ + rel);
        double best = kHazardRange;
        bool rock = false, hit = false;
        for (const Rock* rk : tall) {
            const Vec2 f = origin - rk->pos;
            const double b = f.dot(d);
            const double c = f.dot(f) - sq(rk->radius);
            const double disc = b * b - c;
            if (disc < 0) continue;
            const double tt = c < 0 ? 0.0 : -b - std::sqrt(disc);
            if (tt >= 0 && tt < best) { best = tt; rock = true; hit = true; }
        }
        for (double s = 0.25; s < best; s += 0.25)
            if (terrain_.slope_deg(origin + d * s) > hazard_slope) { best = s; hit = true; rock = false; break; }
        HazardRay ray;
        ray.angle = rel;
        ray.hit = hit;
        ray.rock = rock;
        ray.range = hit ? std::max(0.0, best + noise_.normal(0, 0.03)) : kHazardRange;
        obs.hazards.push_back(ray);
    }
}

void Simulator::sense_landmarks(Observation& obs) {
    if (t_ - last_landmark_t_ < 1.0) return;  // stereo + matching runs at ~1 Hz
    last_landmark_t_ = t_;
    std::vector<int> near;
    terrain_.rocks_near(pos_, kLandmarkRange, near);
    for (int id : near) {
        const Rock& rk = terrain_.rocks()[id];
        if (!rk.catalogued) continue;
        const Vec2 d = rk.pos - pos_;
        const double range = d.norm();
        const double bearing = wrap_angle(d.angle() - heading_);
        if (range > kLandmarkRange || std::fabs(bearing) > deg2rad(kLandmarkFov / 2.0)) continue;
        if (!noise_.chance(0.85)) continue;
        obs.landmarks.push_back({id, range + noise_.normal(0, 0.05 + 0.02 * range), wrap_angle(bearing + noise_.normal(0, deg2rad(1.0)))});
    }
}

void Simulator::sense_visual_odometry(Observation& obs) {
    if (t_ - vo_t_ < 1.0) return;
    const double dt = t_ - vo_t_;
    const Vec2 d = (pos_ - vo_pos_).rotated(-vo_heading_);
    const double dh = wrap_angle(heading_ - vo_heading_);
    vo_t_ = t_;
    vo_pos_ = pos_;
    vo_heading_ = heading_;
    if (dt > 5.0 || env_.sun_elevation(t_) < deg2rad(5.0)) return;  // no light / frame gap
    const std::string& soil = terrain_.soil(pos_).id;
    const double fail_p = (soil == "loose_drift" || soil == "dry_sand") ? 0.6 : 0.1;  // few features on sand
    if (noise_.chance(fail_p)) return;
    const double dist = d.norm();
    obs.vo_valid = true;
    obs.vo_dt = dt;
    obs.vo_dx = d.x + noise_.normal(0, 0.02 * dist + 0.002);
    obs.vo_dy = d.y + noise_.normal(0, 0.02 * dist + 0.002);
    obs.vo_dheading = dh + noise_.normal(0, deg2rad(0.2));
}

Observation Simulator::make_observation(double dt) {
    Observation o;
    o.t = t_;
    o.local_hour = env_.local_hour(t_);
    o.dt = dt;
    o.wheel_angle_left = wheel_angle_[0] + noise_.normal(0, 1e-4);
    o.wheel_angle_right = wheel_angle_[1] + noise_.normal(0, 1e-4);
    // Rate sensors report the average over the interval (delta-angle outputs),
    // like real IMUs and encoders, not an instantaneous sample.
    if (dt > 1e-6 && dt < 5.0) {
        o.wheel_rate_left = (wheel_angle_[0] - obs_wheel_angle_[0]) / dt + noise_.normal(0, 0.002);
        o.wheel_rate_right = (wheel_angle_[1] - obs_wheel_angle_[1]) / dt + noise_.normal(0, 0.002);
        o.gyro_z = wrap_angle(heading_ - obs_heading_) / dt;
    } else {
        o.gyro_z = wz_;
    }
    obs_heading_ = heading_;
    obs_wheel_angle_[0] = wheel_angle_[0];
    obs_wheel_angle_[1] = wheel_angle_[1];
    gyro_bias_ += noise_.normal(0, 1e-6) * std::sqrt(std::max(dt, 1e-3));
    o.gyro_z += gyro_bias_ + noise_.normal(0, 0.002);
    o.pitch = pitch_ + noise_.normal(0, deg2rad(0.5));
    o.roll = roll_ + noise_.normal(0, deg2rad(0.5));
    o.sun_elevation = env_.sun_elevation(t_);
    if (o.sun_elevation > deg2rad(5.0)) {
        o.sun_heading_valid = true;
        o.sun_heading = wrap_angle(heading_ + noise_.normal(0, deg2rad(1.5)));
    }
    sense_landmarks(o);
    sense_hazards(o);
    sense_visual_odometry(o);
    o.battery_soc = battery_wh_ / rover_.design.battery_wh;
    o.battery_wh = battery_wh_;
    o.solar_w = last_solar_w_;
    o.load_w = last_load_w_;
    o.box_temp_c = box_c_ + noise_.normal(0, 0.2);
    o.ambient_temp_c = env_.ambient_temp_c(t_);
    o.new_results.swap(pending_results_);
    o.sampling = sampling_target_ >= 0;
    o.sampling_target = sampling_target_;
    o.sample_progress = sampling_target_ >= 0 ? sample_progress_s_ / mission_.sample_duration_s : 0.0;
    o.relay_in_view = relay_in_view(t_);
    o.data_queued_mbit = queue_.size() * mission_.product_mbit;
    o.inbox.swap(pending_inbox_);
    o.events.swap(pending_events_);
    return o;
}

// --------------------------------------------------------------------------- trace
void Simulator::record_trace(bool force) {
    if (!opt_.record_trace) return;
    const double interval = mode_ == RoverMode::Awake ? opt_.trace_interval_s : opt_.sleep_trace_interval_s;
    if (!force && t_ - last_trace_t_ < interval) return;
    last_trace_t_ = t_;
    TraceSample s{};
    s.t = t_;
    s.x = pos_.x;
    s.y = pos_.y;
    s.z = terrain_.height(pos_);
    s.heading = heading_;
    s.has_est = cmd_.has_estimate;
    s.est_x = cmd_.est_x;
    s.est_y = cmd_.est_y;
    s.est_heading = cmd_.est_heading;
    s.est_sigma = cmd_.est_sigma;
    s.v = v_;
    s.soc = battery_wh_ / rover_.design.battery_wh;
    s.solar_w = last_solar_w_;
    s.load_w = last_load_w_;
    s.box_c = box_c_;
    s.ambient_c = env_.ambient_temp_c(t_);
    s.tau = env_.optical_depth(t_);
    double sl = 0, sr = 0, sink = 0;
    for (int i = 0; i < rover_.wheels_per_side(); ++i) {
        sl += wheels_[0][i].slip;
        sr += wheels_[1][i].slip;
        sink += wheels_[0][i].sinkage + wheels_[1][i].sinkage;
    }
    s.slip_l = sl / rover_.wheels_per_side();
    s.slip_r = sr / rover_.wheels_per_side();
    s.sinkage_cm = 100.0 * sink / rover_.design.wheel_count;
    s.pitch_deg = rad2deg(pitch_);
    s.roll_deg = rad2deg(roll_);
    switch (mode_) {
        case RoverMode::Awake: s.mode = sampling_target_ >= 0 ? 4 : (transmitting_ ? 5 : 0); break;
        case RoverMode::Sleeping: s.mode = 1; break;
        case RoverMode::Brownout: s.mode = 2; break;
        case RoverMode::Dead: s.mode = 3; break;
    }
    s.status = cmd_.status;
    result_.trace.push_back(s);
}

// --------------------------------------------------------------------------- main loop
RunResult Simulator::run() {
    const auto wall0 = std::chrono::steady_clock::now();
    result_ = RunResult{};
    result_.rover_name = rover_.design.name;
    result_.mission_name = mission_.name;
    result_.controller_name = controller_name_;

    t_ = 0.0;
    pos_ = mission_.start;
    heading_ = deg2rad(mission_.start_heading_deg);
    battery_wh_ = mission_.start_soc * rover_.design.battery_wh;
    box_c_ = mission_.start_box_temp_c;
    gyro_bias_ = noise_.normal(0, 2e-4);
    vo_pos_ = pos_;
    vo_heading_ = heading_;
    obs_heading_ = heading_;
    mode_ = RoverMode::Awake;
    cmd_ = Command{};

    // Settle the rover onto the ground and check it can even stand up.
    StepContext ctx;
    compute_attitude_and_loads(ctx);
    check_structure(1.0);

    controller_.on_start(brief_);
    record_trace(true);

    const double T = brief_.duration_s;
    while (t_ < T && !result_.failed && mode_ != RoverMode::Dead) {
        const RoverMode before = mode_;
        if (mode_ == RoverMode::Awake) {
            Observation obs = make_observation(since_obs_);
            since_obs_ = 0.0;
            cmd_ = controller_.on_step(obs);
            if (!std::isfinite(cmd_.wheel_speed_left)) cmd_.wheel_speed_left = 0.0;
            if (!std::isfinite(cmd_.wheel_speed_right)) cmd_.wheel_speed_right = 0.0;
            if (!cmd_.log.empty() && result_.log.size() < 5000)
                result_.log.push_back(fmtd("[%9.1f] ", t_) + cmd_.log);
            if (cmd_.path_updated) {
                std::vector<Vec2> pts;
                const size_t stride = std::max<size_t>(1, cmd_.planned_path.size() / 150);
                for (size_t k = 0; k < cmd_.planned_path.size(); k += stride) pts.push_back(cmd_.planned_path[k]);
                if (!cmd_.planned_path.empty()) pts.push_back(cmd_.planned_path.back());
                if (!result_.paths.empty() && t_ - result_.paths.back().t < 5.0) result_.paths.back() = {t_, pts};
                else result_.paths.push_back({t_, pts});
            }
            if (cmd_.end_mission) {
                event("end", "Controller ended the mission.");
                break;
            }
            if (cmd_.sleep_s > 0.0) {
                mode_ = RoverMode::Sleeping;
                sleep_until_ = std::min(T, t_ + cmd_.sleep_s);
                cmd_.wheel_speed_left = cmd_.wheel_speed_right = 0.0;
                cmd_.sample_target = -1;
                cmd_.transmit = false;
                record_trace(true);
                continue;
            }
            double elapsed = 0.0;
            while (elapsed < opt_.control_dt - 1e-9 && !result_.failed && mode_ == RoverMode::Awake && t_ < T) {
                const bool moving = std::fabs(v_) > 1e-4 || std::fabs(wz_) > 1e-4 ||
                                    std::fabs(cmd_.wheel_speed_left) + std::fabs(cmd_.wheel_speed_right) > 1e-9;
                double dt = moving ? opt_.physics_dt : opt_.control_dt - elapsed;
                dt = std::min(dt, opt_.control_dt - elapsed);
                physics_step(dt);
                elapsed += dt;
                since_obs_ += dt;
            }
        } else if (mode_ == RoverMode::Sleeping) {
            // Coast to a stop at physics resolution before taking long steps.
            const bool moving = std::fabs(v_) > 1e-3 || std::fabs(wz_) > 1e-3;
            const double dt = std::min(moving ? opt_.physics_dt : opt_.sleep_dt, std::max(1e-3, sleep_until_ - t_));
            physics_step(dt);
            since_obs_ += dt;
            if (t_ >= sleep_until_ - 1e-6 && mode_ == RoverMode::Sleeping) mode_ = RoverMode::Awake;
        } else {  // brownout
            const bool moving = std::fabs(v_) > 1e-3 || std::fabs(wz_) > 1e-3;
            const double dt = moving ? opt_.physics_dt : opt_.sleep_dt;
            physics_step(dt);
            since_obs_ += dt;
        }
        record_trace(before != mode_);
    }
    controller_.on_end();
    record_trace(true);

    // ---- Scoring
    result_.sim_time_s = t_;
    result_.end_soc = battery_wh_ / rover_.design.battery_wh;
    for (const auto& rec : result_.science)
        if (rec.received) {
            result_.science_points += rec.true_value * mission_.points_per_value;
            ++result_.targets_received;
        }
    if (result_.goal_reached) result_.goal_points = mission_.goal_bonus;
    result_.score = result_.science_points + result_.goal_points;
    if (result_.min_box_temp_c > 1e8) result_.min_box_temp_c = box_c_;
    result_.wall_time_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count();
    return result_;
}

// --------------------------------------------------------------------------- output
std::string format_result(const RunResult& r, bool color) {
    std::ostringstream os;
    const char* red = color ? "\033[31m" : "";
    const char* grn = color ? "\033[32m" : "";
    const char* bold = color ? "\033[1m" : "";
    const char* rst = color ? "\033[0m" : "";
    char buf[512];
    os << bold << "=== " << r.mission_name << " | " << r.rover_name << " | controller: " << r.controller_name << " ===" << rst << "\n";
    if (r.failed) {
        os << red << bold << "MISSION FAILED (" << r.failure_type << ")" << rst << ": " << r.failure_message << "\n";
        os << "  Why: " << r.lesson << "\n";
    } else {
        os << grn << bold << "Rover survived the mission." << rst << "\n";
    }
    std::snprintf(buf, sizeof(buf), "  SCORE %.1f  = science %.1f (%d of %d measured targets received on Earth) + goal %.0f%s\n",
                  r.score, r.science_points, r.targets_received, r.targets_measured, r.goal_points,
                  r.goal_reached ? " (reached)" : "");
    os << buf;
    std::snprintf(buf, sizeof(buf), "  Simulated %.2f sols in %.1f s wall time | drove %.1f m | %d collisions, %d rock strikes, %d wheels failed\n",
                  r.sim_time_s / mars::kSolSeconds, r.wall_time_s, r.distance_m, r.collisions, r.rock_strikes, r.wheels_failed);
    os << buf;
    std::snprintf(buf, sizeof(buf), "  Energy: solar in %.0f Wh | driving %.0f Wh | heaters %.0f Wh | end battery %.0f%%\n",
                  r.solar_energy_wh, r.drive_energy_wh, r.heater_energy_wh, 100.0 * r.end_soc);
    os << buf;
    std::snprintf(buf, sizeof(buf), "  Worst case: slip %.0f%% | sinkage %.1f cm | warm-box low %.1f C | wheel damage %.2f\n",
                  100.0 * r.max_slip, r.max_sinkage_cm, r.min_box_temp_c, r.max_wheel_damage);
    os << buf;
    return os.str();
}

}  // namespace aximetry
