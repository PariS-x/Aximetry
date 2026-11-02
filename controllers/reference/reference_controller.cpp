// reference — a complete autonomy stack for Aximetry rovers.
//
//   Localisation   EKF fusing slip-corrected wheel odometry, a bias-tracked gyro,
//                  the sun sensor and range/bearing sightings of catalogued boulders.
//                  Slip is estimated online by comparing landmark-corrected motion
//                  with what the encoders claim (the job visual odometry does on MER).
//   Mapping        orbital slope + soil cost map, updated live by the hazard scanner.
//   Planning       A* with line-of-sight smoothing, replanned when new hazards cut
//                  the route, when the rover gets stuck, or when the estimate jumps.
//   Control        pure pursuit with turn-in-place, slip-aware speed.
//   Science        adaptive target selection: expected value from orbital priors,
//                  corrected online by learning how often orbit misclassifies
//                  minerals; ranks targets by value per hour with one-step lookahead,
//                  subject to time, comms and goal deadlines.
//   Operations     sleeps at night, wakes for relay passes, detects dust storms from
//                  solar power and goes into power-saving mode, protects the battery.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

#include "ekf.hpp"
#include "planner.hpp"
#include "aximetry/controller.hpp"
#include "aximetry/terrain.hpp"

using namespace aximetry;
using namespace refctl;

namespace {

std::string fmt(const char* f, double a) {
    char b[256];
    std::snprintf(b, sizeof(b), f, a);
    return b;
}
std::string fmt2(const char* f, double a, double c) {
    char b[256];
    std::snprintf(b, sizeof(b), f, a, c);
    return b;
}

constexpr double kDriveStartHour = 9.0;
constexpr double kDriveEndHour = 16.5;
constexpr double kMinDriveSunElev = 0.17;  // ~10 deg
constexpr double kLookahead = 1.5;         // pure pursuit, m

struct Candidate {
    TargetInfo info;
    double priority_override = -1.0;  // set by an uplink from Earth
    bool done = false;
    bool abandoned = false;
    int failures = 0;
    double retry_after = -1.0;
};

}  // namespace

class ReferenceController : public Controller {
public:
    // Ablation switches (see the registrations at the bottom of the file).
    struct Options {
        bool landmarks = true;         // EKF uses landmark sightings
        bool visual_odometry = true;   // slip estimated from VO
        bool value_aware = true;       // rank targets by expected value per hour
    };
    ReferenceController() = default;
    explicit ReferenceController(Options o) : opt_(o) {}

    void on_start(const MissionBrief& b) override {
        brief_ = b;
        ekf_.init(b.start_pos, b.start_heading, std::max(0.3, b.start_pos_sigma), aximetry::deg2rad(3.0));
        params_.inflate = 0.5 * b.rover.track_width + 0.25;
        params_.max_slope_deg = std::min(20.0, b.rover.tip_angle_deg - 18.0);
        map_.build(b, params_);
        for (const auto& lm : b.landmarks) landmarks_[lm.id] = lm.pos;
        for (const auto& t : b.targets) cands_.push_back({t});
        v_cruise_ = 0.7 * b.rover.max_wheel_speed * b.rover.wheel_radius;

        // Last relay pass whose data still reaches Earth before the mission ends.
        for (const auto& p : b.relay_passes)
            if (p.start_s + 60.0 + b.owlt_s <= b.duration_s) last_useful_pass_start_ = std::max(last_useful_pass_start_, p.start_s);
        // Cumulative drive-window seconds at 5 minute resolution.
        const double step = 300.0;
        drive_cum_.push_back(0.0);
        for (double t = 0.0; t < b.duration_s + step; t += step) {
            const double h = hour_at(t + 0.5 * step);
            const bool window = h >= kDriveStartHour && h <= kDriveEndHour;
            drive_cum_.push_back(drive_cum_.back() + (window ? step : 0.0));
        }
        log_ = "Reference controller online. " + fmt("%.0f landmarks, ", b.landmarks.size()) +
               fmt("%.0f science targets, ", b.targets.size()) + fmt("%.0f relay passes.", b.relay_passes.size());
    }

    Command on_step(const Observation& o) override {
        Command c;
        estimate_state(o);
        c.log.swap(log_);
        handle_messages(o, c);
        update_hazards(o);

        c.transmit = o.relay_in_view && o.data_queued_mbit > 0.0;
        c.has_estimate = true;
        const Vec2 p = ekf_.pos();
        c.est_x = p.x;
        c.est_y = p.y;
        c.est_heading = ekf_.heading();
        c.est_sigma = ekf_.pos_sigma();
        last_t_ = o.t;

        operations(o, c);
        if (c.sleep_s > 0.0 || c.end_mission) {
            last_cmd_moving_ = false;
            return c;
        }
        last_cmd_moving_ = std::fabs(c.wheel_speed_left) + std::fabs(c.wheel_speed_right) > 1e-6;
        return c;
    }

private:
    // ------------------------------------------------------------------ estimation
    void estimate_state(const Observation& o) {
        const double r = brief_.rover.wheel_radius;
        if (first_) {
            last_wl_ = o.wheel_angle_left;
            last_wr_ = o.wheel_angle_right;
            first_ = false;
        }
        const double dl = o.wheel_angle_left - last_wl_, dr = o.wheel_angle_right - last_wr_;
        last_wl_ = o.wheel_angle_left;
        last_wr_ = o.wheel_angle_right;
        const double ds_enc = 0.5 * r * (dl + dr);

        // Zero-velocity update: when parked, whatever the gyro reads is bias.
        const bool parked = !last_cmd_moving_ && std::fabs(o.wheel_rate_left) < 0.01 && std::fabs(o.wheel_rate_right) < 0.01;
        if (parked && o.dt < 1.0) gyro_bias_ = 0.97 * gyro_bias_ + 0.03 * o.gyro_z;

        double dth = 0.0;
        if (o.dt < 1.0) dth = (o.gyro_z - gyro_bias_) * o.dt;  // after a long sleep the rover did not move
        const double ds = ds_enc * (1.0 - slip_est_);
        // Slip error is systematic, so odometry uncertainty is set generously.
        // Slip is a systematic error, so odometry noise is set generously, and
        // skid-steer turns scrub the rover sideways by an unknown amount.
        const double sig_ds = std::fabs(ds_enc) * (0.10 + 0.5 * slip_est_);
        const double sig_th = 0.003 * o.dt + 0.02 * std::fabs(dth) + 1e-4;
        const double sig_xy = 0.15 * std::fabs(dth) + 0.02 * std::fabs(ds_enc);
        ekf_.predict(ds, dth, sig_ds, sig_th, sig_xy);

        // Raw odometry pose (no slip correction) for the slip estimator.
        odo_h_ = ekf_.heading();
        odo_pos_ += Vec2::from_angle(odo_h_) * ds_enc;
        odo_window_ += std::fabs(ds_enc);

        if (o.sun_heading_valid) ekf_.update_heading(o.sun_heading, aximetry::deg2rad(2.0));
        for (const auto& lm : o.landmarks) {
            if (!opt_.landmarks) break;
            auto it = landmarks_.find(lm.id);
            if (it == landmarks_.end()) continue;
            if (ekf_.update_landmark(it->second, lm.range, lm.bearing, 1.5 * (0.05 + 0.02 * lm.range), aximetry::deg2rad(1.5))) {
                ++fixes_in_window_;
                rejections_ = 0;
            } else if (++rejections_ >= 6) {
                // Landmarks consistently disagree with us: we are more lost than the
                // filter thinks. Re-localise from scratch if two landmarks are in view,
                // otherwise widen the uncertainty (more each time) and let them pull us back.
                rejections_ = 0;
                if (relocalise_from_pair(o)) {
                    append_pending("Landmark sightings disagree with my estimate: re-localised from two landmarks.");
                } else {
                    inflate_ = std::min(inflate_ * 2.0, 40.0);
                    ekf_.inflate_position(inflate_);
                    append_pending(fmt("Landmark sightings disagree with my estimate: widening uncertainty to %.0f m.", inflate_));
                }
            }
        }
        if (ekf_.pos_sigma() < 0.5) inflate_ = 1.5;

        // Slip estimation, as on MER: compare how far visual odometry says we
        // moved with how far the wheels turned. Falls back to landmark-anchored
        // displacement when VO keeps failing (featureless sand).
        if (o.vo_valid && opt_.visual_odometry) {
            vo_dist_ += std::hypot(o.vo_dx, o.vo_dy) * (o.vo_dx >= 0 ? 1.0 : -1.0);
            vo_enc_ += enc_since_vo_;
        }
        enc_since_vo_ = o.vo_valid ? 0.0 : enc_since_vo_ + ds_enc;
        if (o.vo_valid && opt_.visual_odometry && std::fabs(vo_enc_) >= 1.0) {
            const double s = clampd(1.0 - vo_dist_ / vo_enc_, -0.1, 0.98);
            slip_est_ = clampd(0.6 * slip_est_ + 0.4 * s, 0.0, 0.95);
            vo_dist_ = vo_enc_ = 0.0;
        }
        if (odo_window_ >= 3.0) {
            const double chord_odo = aximetry::distance(odo_pos_, odo_start_);
            if (fixes_in_window_ >= 4 && ekf_.pos_sigma() < 0.15 && window_sigma_ < 0.15 && chord_odo > 2.0) {
                const double s = clampd(1.0 - aximetry::distance(ekf_.pos(), window_start_) / chord_odo, 0.0, 0.95);
                slip_est_ = 0.7 * slip_est_ + 0.3 * s;
            }
            odo_window_ = 0.0;
        }
        if (odo_window_ == 0.0) {
            fixes_in_window_ = 0;
            window_start_ = ekf_.pos();
            window_sigma_ = ekf_.pos_sigma();
            odo_start_ = odo_pos_;
        }
    }

    // Absolute pose from two identified landmarks: the vector between them in the
    // world and in the rover frame gives the heading, then either one gives position.
    bool relocalise_from_pair(const Observation& o) {
        const LandmarkObs* a = nullptr;
        const LandmarkObs* b = nullptr;
        double best = 0.0;
        for (size_t i = 0; i < o.landmarks.size(); ++i)
            for (size_t j = i + 1; j < o.landmarks.size(); ++j) {
                auto ia = landmarks_.find(o.landmarks[i].id), ib = landmarks_.find(o.landmarks[j].id);
                if (ia == landmarks_.end() || ib == landmarks_.end()) continue;
                const double sep = aximetry::distance(ia->second, ib->second);
                if (sep > best) { best = sep; a = &o.landmarks[i]; b = &o.landmarks[j]; }
            }
        if (!a || best < 3.0) return false;
        const Vec2 La = landmarks_[a->id], Lb = landmarks_[b->id];
        const Vec2 pa = Vec2::from_angle(a->bearing) * a->range, pb = Vec2::from_angle(b->bearing) * b->range;
        const double heading = wrap_angle((Lb - La).angle() - (pb - pa).angle());
        const Vec2 pos = La - pa.rotated(heading);
        ekf_.init(pos, heading, 0.6, aximetry::deg2rad(4.0));
        return true;
    }

    // ------------------------------------------------------------------ messages
    void handle_messages(const Observation& o, Command& c) {
        for (const auto& res : o.new_results) {
            for (auto& cand : cands_)
                if (cand.info.id == res.target_id) {
                    cand.done = true;
                    if (res.kind == cand.info.orbital_kind) orbit_right_ += 1.0;
                    else orbit_wrong_ += 1.0;
                }
            append(c, fmt("Measured target %.0f: ", res.target_id) + res.kind + fmt(" value %.2f. ", res.value) +
                          fmt("Orbital labels now trusted %.0f%%.", 100.0 * orbit_trust()));
            if (mode_ == Mode::Sampling) mode_ = Mode::Choose;
        }
        for (const auto& m : o.inbox) {
            if (m.type == "priority" && m.target_id >= 0)
                for (auto& cand : cands_)
                    if (cand.info.id == m.target_id) {
                        cand.priority_override = m.value;
                        cand.abandoned = false;
                    }
            append(c, "Uplink: " + m.text);
            if (mode_ == Mode::Drive) mode_ = Mode::Choose;  // re-plan with new priorities
        }
        for (const auto& e : o.events) {
            if (e.rfind("collision", 0) == 0) {
                // Something the scanner missed: mark it just ahead and back away.
                const Vec2 ahead = ekf_.pos() + Vec2::from_angle(ekf_.heading()) * (0.5 * brief_.rover.wheelbase + 0.5);
                map_.add_obstacle(ahead, 0.4);
                start_backup(c, "Bumped into an unmapped rock: backing off and replanning.");
            } else if (e.rfind("embedded", 0) == 0 && mode_ == Mode::Drive) {
                declare_stuck(c, "Sim reports wheels digging in.");
            } else if (e.rfind("wheel_failed", 0) == 0) {
                append(c, "Wheel failure reported. Continuing with reduced traction.");
                v_cruise_ *= 0.7;
            }
        }
    }

    void update_hazards(const Observation& o) {
        const double th = ekf_.heading();
        const Vec2 origin = ekf_.pos() + Vec2::from_angle(th) * (0.5 * brief_.rover.wheelbase + brief_.rover.wheel_radius);
        bool changed = false;
        for (const auto& ray : o.hazards) {
            if (!ray.hit) continue;
            const Vec2 pt = origin + Vec2::from_angle(th + ray.angle) * (ray.range + (ray.rock ? 0.2 : 0.0));
            changed |= map_.add_obstacle(pt, ray.rock ? 0.25 : 0.1);
        }
        if (changed && mode_ == Mode::Drive && path_blocked_ahead()) need_replan_ = true;
    }

    bool path_blocked_ahead() const {
        for (size_t k = path_idx_; k + 1 < path_.size(); ++k) {
            const Vec2 a = path_[k], b = path_[k + 1];
            const double L = aximetry::distance(a, b);
            const int n = std::max(1, static_cast<int>(L / 0.25));
            for (int i = 0; i <= n; ++i) {
                const Vec2 p = a + (b - a) * (static_cast<double>(i) / n);
                if (aximetry::distance(p, ekf_.pos()) > 8.0) return false;
                if (aximetry::distance(p, ekf_.pos()) > 1.2 && map_.blocked_at(p)) return true;
            }
        }
        return false;
    }

    // ------------------------------------------------------------------ operations
    void operations(const Observation& o, Command& c) {
        const double h = o.local_hour;
        const bool drive_window = h >= kDriveStartHour && h <= kDriveEndHour && o.sun_elevation > kMinDriveSunElev;

        // Dust storm detection: solar output far below clear-sky expectation.
        const double clear_sky = brief_.rover.solar_area_m2 * 0.28 * 590.0 * std::max(0.0, std::sin(o.sun_elevation)) * 0.8;
        if (o.sun_elevation > 0.3 && clear_sky > 1.0) {
            const double ratio = o.solar_w / clear_sky;
            storm_ratio_ = 0.98 * storm_ratio_ + 0.02 * ratio;
            const bool storm = storm_ratio_ < 0.45;
            if (storm != storm_mode_) {
                storm_mode_ = storm;
                append(c, storm ? "Solar output collapsed: dust storm. Entering power-saving mode."
                                : "Skies clearing. Leaving power-saving mode.");
            }
        }

        // Finished? Nothing left worth doing and no data waiting.
        if (nothing_left(o)) {
            c.end_mission = o.data_queued_mbit <= 0.0;
            if (!c.end_mission) sleep_until_next(o, c, "All work done; waiting for a relay pass.");
            else append(c, "All objectives complete. Ending mission.");
            return;
        }

        if (mode_ == Mode::Sampling) {  // keep integrating even past the drive window
            sample(o, c);
            if (mode_ == Mode::Sampling) return;
        }
        if (!drive_window) {
            sleep_until_next(o, c, h > kDriveEndHour || h < kDriveStartHour ? "Outside drive window." : "Sun too low.");
            return;
        }
        const double reserve = storm_mode_ ? 0.7 : 0.35;
        if (o.battery_soc < reserve && mode_ != Mode::Sampling) {
            c.sleep_s = 1800.0;
            c.status = "charging";
            append(c, fmt("Battery at %.0f%%: napping to recharge.", 100.0 * o.battery_soc));
            return;
        }

        switch (mode_) {
            case Mode::Choose: choose_destination(o, c); if (mode_ == Mode::Drive) drive(o, c); break;
            case Mode::Drive: drive(o, c); break;
            case Mode::Backup: backup(o, c); break;
            case Mode::Sampling: sample(o, c); break;
            case Mode::Idle: c.status = "idle"; if (o.t - idle_since_ > 600.0) mode_ = Mode::Choose; break;
        }
    }

    void sleep_until_next(const Observation& o, Command& c, const std::string& why) {
        if (o.relay_in_view && o.data_queued_mbit > 0.0) {  // stay up to finish the pass
            c.status = "downlinking";
            return;
        }
        const double h = o.local_hour;
        double until_drive = (std::fmod(kDriveStartHour - h + 24.0, 24.0)) / 24.0 * brief_.sol_seconds;
        if (until_drive < 60.0) until_drive = 600.0;
        double wake = until_drive;
        if (o.data_queued_mbit > 0.0)
            for (const auto& p : brief_.relay_passes)
                if (p.start_s + p.duration_s > o.t + 30.0) {
                    wake = std::min(wake, std::max(30.0, p.start_s - o.t + 5.0));
                    break;
                }
        if (storm_mode_) wake = std::min(wake, 3600.0);
        c.sleep_s = std::min(wake, 4.0 * 3600.0);
        c.status = "sleeping";
        if (why != last_sleep_reason_ || o.t - last_sleep_log_ > 20000.0) {
            append(c, why + fmt(" Sleeping %.0f min.", c.sleep_s / 60.0));
            last_sleep_reason_ = why;
            last_sleep_log_ = o.t;
        }
        if (mode_ == Mode::Drive) mode_ = Mode::Choose;  // re-evaluate after waking
    }

    // ------------------------------------------------------------------ science planning
    double orbit_trust() const { return (orbit_right_ + 4.0) / (orbit_right_ + orbit_wrong_ + 6.0); }

    double expected_value(const Candidate& cd) const {
        if (cd.priority_override >= 0.0) return cd.priority_override;
        const double trust = orbit_trust();
        double other = 0.0, wsum = 0.0;
        for (const auto& k : target_kinds())
            if (k.id != cd.info.orbital_kind) {
                other += k.frequency * k.mean_value;
                wsum += k.frequency;
            }
        return trust * cd.info.orbital_prior + (1.0 - trust) * other / wsum;
    }

    double hour_at(double t) const {
        const double sols = brief_.start_local_hour / 24.0 + t / brief_.sol_seconds;
        return (sols - std::floor(sols)) * 24.0;
    }
    double drive_seconds_between(double t0, double t1) const {
        if (t1 <= t0) return 0.0;
        auto cum = [&](double t) {
            const double idx = clampd(t / 300.0, 0.0, drive_cum_.size() - 2.0);
            const size_t i = static_cast<size_t>(idx);
            return drive_cum_[i] + (drive_cum_[i + 1] - drive_cum_[i]) * (idx - i);
        };
        return cum(t1) - cum(t0);
    }
    double travel_s(const Vec2& a, const Vec2& b) const {
        return 1.35 * aximetry::distance(a, b) / (v_cruise_ * std::max(0.3, 1.0 - slip_est_)) + 60.0;
    }

    bool nothing_left(const Observation& o) const {
        if (brief_.has_goal && !goal_reached_) return false;
        if (mode_ == Mode::Sampling || mode_ == Mode::Backup) return false;
        for (const auto& cd : cands_)
            if (!cd.done && !cd.abandoned && feasible(cd, o.t)) return false;
        return true;  // (deferred targets count as alive via the feasibility check)
    }

    bool feasible(const Candidate& cd, double t) const {
        const Vec2 p = ekf_.pos();
        const double drive = travel_s(p, cd.info.pos);
        const double avail = drive_seconds_between(t, last_useful_pass_start_);
        if (drive + brief_.sample_duration_s > avail) return false;
        if (brief_.has_goal && !goal_reached_) {
            const double to_goal = travel_s(cd.info.pos, brief_.goal);
            if (drive + brief_.sample_duration_s + to_goal > 0.85 * drive_seconds_between(t, brief_.duration_s)) return false;
        }
        return true;
    }

    void choose_destination(const Observation& o, Command& c) {
        const Vec2 p = ekf_.pos();
        double best = -1.0;
        int best_i = -1;
        for (size_t i = 0; i < cands_.size(); ++i) {
            const auto& a = cands_[i];
            if (a.done || a.abandoned || o.t < a.retry_after || !feasible(a, o.t)) continue;
            const double ta = travel_s(p, a.info.pos) + brief_.sample_duration_s;
            double next = 0.0, tn = 0.0;
            for (size_t j = 0; j < cands_.size(); ++j) {
                const auto& b = cands_[j];
                if (j == i || b.done || b.abandoned) continue;
                const double tb = travel_s(a.info.pos, b.info.pos) + brief_.sample_duration_s;
                const double u = expected_value(b) / (tb / 3600.0 + 0.5);
                if (u > next / (tn / 3600.0 + 0.5 + 1e-9)) { next = expected_value(b); tn = tb; }
            }
            double score = (expected_value(a) + 0.6 * next) / ((ta + 0.6 * tn) / 3600.0 + 0.5);
            if (!opt_.value_aware) score = 1.0 / ta;  // ablation: nearest target first
            if (score > best) { best = score; best_i = static_cast<int>(i); }
        }
        if (best_i >= 0) {
            dest_ = cands_[best_i].info.pos;
            dest_target_ = cands_[best_i].info.id;
            dest_tol_ = 0.6 * brief_.sample_range_m;
            append(c, fmt("Next: target %.0f (", dest_target_) + cands_[best_i].info.orbital_kind +
                          fmt2(" from orbit, expected value %.2f, %.0f m away).", expected_value(cands_[best_i]),
                               aximetry::distance(p, dest_)));
        } else if (brief_.has_goal && !goal_reached_ && o.t >= goal_retry_after_) {
            dest_ = brief_.goal;
            dest_target_ = -1;
            dest_tol_ = 0.5 * brief_.goal_radius;
            append(c, fmt("Heading for the goal, %.0f m away.", aximetry::distance(p, dest_)));
        } else {
            mode_ = Mode::Idle;
            idle_since_ = o.t;
            c.status = "idle";
            return;
        }
        mode_ = Mode::Drive;
        stuck_count_ = 0;
        search_step_ = 0;
        estops_ = 0;
        reset_progress();
        replan(c);
    }

    // ------------------------------------------------------------------ driving
    void replan(Command& c) {
        double cost = 0.0;
        path_ = plan_path(map_, ekf_.pos(), dest_, 3.0, &cost);
        if (path_.empty()) path_ = plan_path(map_, ekf_.pos(), dest_, 8.0, &cost);  // boxed in: dig out further
        path_idx_ = 0;
        last_plan_t_ = last_t_;
        need_replan_ = false;
        c.path_updated = true;
        c.planned_path = path_;
        if (path_.empty()) {
            append(c, "No safe path to destination right now.");
            defer_destination(last_t_);
        }
    }

    // Couldn't reach it this time: try others first, give up after 3 deferrals.
    void defer_destination(double t) {
        if (dest_target_ >= 0) {
            for (auto& cd : cands_)
                if (cd.info.id == dest_target_) {
                    cd.retry_after = t + 900.0;
                    if (++cd.failures >= 3) cd.abandoned = true;
                }
        } else if (++goal_failures_ >= 3) {
            goal_reached_ = true;
        } else {
            goal_retry_after_ = t + 900.0;
        }
        mode_ = Mode::Choose;
    }

    void abandon_destination() {
        if (dest_target_ >= 0) {
            for (auto& cd : cands_)
                if (cd.info.id == dest_target_) cd.abandoned = true;
        } else {
            goal_reached_ = true;  // treat an unreachable goal as closed
        }
        mode_ = Mode::Choose;
    }

    void drive(const Observation& o, Command& c) {
        const Vec2 p = ekf_.pos();
        if (aximetry::distance(p, dest_) < dest_tol_) {
            arrive(o, c);
            return;
        }
        if (need_replan_ || o.t - last_plan_t_ > 45.0) replan(c);
        if (mode_ != Mode::Drive) return;
        if (path_.size() < 2) path_ = {p, dest_};

        // Emergency stop: something close right in front.
        for (const auto& ray : o.hazards)
            if (ray.hit && ray.rock && ray.range < 0.5 && std::fabs(ray.angle) < 0.5) {
                if (++estops_ >= 4) {
                    estops_ = 0;
                    defer_destination(o.t);
                    choose_after_backup_ = true;
                    start_backup(c, "Keep meeting rocks on this route: trying another destination first.");
                    return;
                }
                start_backup(c, "Rock right ahead: stopping and backing off.");
                return;
            }

        // Stuck detection: lots of commanded driving, little estimated progress.
        if (o.t - progress_t_ > 60.0) {
            const double moved = aximetry::distance(p, progress_ref_);
            if (moved < 0.15 && !turning_) {  // slow-but-moving in sand is not stuck
                declare_stuck(c, fmt("Only %.1f m progress in a minute", moved) + fmt(" (slip estimate %.0f%%).", 100 * slip_est_));
                return;
            }
            progress_ref_ = p;
            progress_t_ = o.t;
        }

        // Pure pursuit.
        size_t best = path_idx_;
        double bestd = 1e9;
        for (size_t k = path_idx_; k < std::min(path_.size(), path_idx_ + 30); ++k) {
            const double d = aximetry::distance(path_[k], p);
            if (d < bestd) { bestd = d; best = k; }
        }
        path_idx_ = best;
        Vec2 look = path_.back();
        for (size_t k = path_idx_; k + 1 < path_.size(); ++k) {
            if (aximetry::distance(path_[k + 1], p) >= kLookahead) {
                look = path_[k + 1];
                break;
            }
        }
        const double alpha = wrap_angle((look - p).angle() - ekf_.heading());
        const double yaw_max = 0.15;
        if (std::fabs(alpha) > 0.7 || (turning_ && std::fabs(alpha) > 0.2)) {
            if (!turning_) {
                turn_start_t_ = o.t;
                turn_start_heading_ = ekf_.heading();
            } else if (o.t - turn_start_t_ > 60.0 && std::fabs(wrap_angle(ekf_.heading() - turn_start_heading_)) < 0.3) {
                turning_ = false;
                declare_stuck(c, "Cannot rotate here (wedged against something).");
                return;
            }
            turning_ = true;
            c.drive(0.0, std::copysign(yaw_max, alpha), brief_.rover);
            c.status = "turning";
            return;
        }
        turning_ = false;
        double v = v_cruise_ * (1.0 - 0.5 * std::fabs(alpha));
        if (slip_est_ > 0.3) v *= 0.6;  // slow down in soft ground
        if (std::fabs(o.pitch) > aximetry::deg2rad(12.0)) v *= 0.7;
        const double L = std::max(0.5, aximetry::distance(look, p));
        const double yaw = clampd(2.0 * v * std::sin(alpha) / L, -yaw_max, yaw_max);
        c.drive(v, yaw, brief_.rover);
        c.status = dest_target_ >= 0 ? "to target " + std::to_string(dest_target_) : "to goal";
    }

    void arrive(const Observation& o, Command& c) {
        if (dest_target_ < 0) {
            goal_reached_ = true;
            append(c, "Reached the goal (by my estimate).");
            mode_ = Mode::Choose;
            return;
        }
        mode_ = Mode::Sampling;
        sample_started_t_ = o.t;
        sample(o, c);
    }

    void sample(const Observation& o, Command& c) {
        c.sample_target = dest_target_;
        c.status = "sampling " + std::to_string(dest_target_);
        if (o.sampling) {
            sample_started_t_ = o.t;  // instrument is integrating: we're in range
            return;
        }
        // Not integrating after a few seconds → the target is out of range:
        // our estimate is off. Search a small pattern around the estimate.
        if (o.t - sample_started_t_ > 8.0) {
            static const double ox[] = {1.0, 0.0, -1.0, 0.0, 1.8, -1.8, 1.8, -1.8};
            static const double oy[] = {0.0, 1.0, 0.0, -1.0, 1.8, 1.8, -1.8, -1.8};
            if (search_step_ >= 8) {
                append(c, fmt("Could not get within instrument range of target %.0f. Moving on.", dest_target_));
                for (auto& cd : cands_)
                    if (cd.info.id == dest_target_) cd.abandoned = true;
                mode_ = Mode::Choose;
                return;
            }
            Vec2 tgt;
            for (const auto& cd : cands_)
                if (cd.info.id == dest_target_) tgt = cd.info.pos;
            dest_ = tgt + Vec2(ox[search_step_], oy[search_step_]);
            dest_tol_ = 0.35;
            ++search_step_;
            mode_ = Mode::Drive;
            reset_progress();
            replan(c);
            c.sample_target = -1;
        }
    }

    void start_backup(Command& c, const std::string& why) {
        append(c, why);
        mode_ = Mode::Backup;
        backup_until_ = last_t_ + 20.0;
        c.drive(-0.6 * v_cruise_, 0.0, brief_.rover);
        c.status = "backing up";
    }

    void reset_progress() {
        progress_ref_ = ekf_.pos();
        progress_t_ = last_t_;
        turning_ = false;
    }

    void backup(const Observation& o, Command& c) {
        if (o.t >= backup_until_) {
            reset_progress();
            if (choose_after_backup_) {
                choose_after_backup_ = false;
                mode_ = Mode::Choose;
                return;
            }
            mode_ = Mode::Drive;
            replan(c);
            return;
        }
        c.drive(-0.6 * v_cruise_, 0.0, brief_.rover);
        c.status = "backing up";
    }

    void declare_stuck(Command& c, const std::string& why) {
        ++stuck_count_;
        const Vec2 ahead = ekf_.pos() + Vec2::from_angle(ekf_.heading()) * 2.0;
        map_.add_cost(ahead, 3.0, 25.0);  // learn: this ground is bad
        slip_est_ = std::max(slip_est_, 0.5);
        if (stuck_count_ >= 3) {
            append(c, why + " Stuck three times on this leg: trying a different destination.");
            defer_destination(last_t_);
            choose_after_backup_ = true;
            start_backup(c, "Backing out along my tracks.");
            backup_until_ = last_t_ + 45.0;
            return;
        }
        start_backup(c, why + " Backing out and routing around.");
        backup_until_ = last_t_ + 30.0;
    }

    void append_pending(const std::string& s) {
        if (!log_.empty()) log_ += " | ";
        log_ += s;
    }

    static void append(Command& c, const std::string& s) {
        if (!c.log.empty()) c.log += " | ";
        c.log += s;
    }

    enum class Mode { Choose, Drive, Backup, Sampling, Idle };

    Options opt_;
    MissionBrief brief_;
    PoseEKF ekf_;
    PlannerParams params_;
    CostMap map_;
    std::map<int, Vec2> landmarks_;
    std::vector<Candidate> cands_;
    std::vector<double> drive_cum_;
    double last_useful_pass_start_ = -1.0;

    // estimation
    bool first_ = true;
    double last_wl_ = 0, last_wr_ = 0;
    double gyro_bias_ = 0.0;
    double slip_est_ = 0.05;
    double odo_window_ = 0.0;
    int fixes_in_window_ = 0;
    Vec2 window_start_;
    double window_sigma_ = 1.0;
    Vec2 odo_pos_, odo_start_;
    double vo_dist_ = 0.0, vo_enc_ = 0.0, enc_since_vo_ = 0.0;
    int rejections_ = 0;
    double inflate_ = 1.5;
    double odo_h_ = 0.0;
    bool last_cmd_moving_ = false;

    // behaviour
    Mode mode_ = Mode::Choose;
    Vec2 dest_;
    int dest_target_ = -1;
    double dest_tol_ = 1.0;
    std::vector<Vec2> path_;
    size_t path_idx_ = 0;
    bool need_replan_ = false;
    bool turning_ = false;
    double turn_start_t_ = 0.0, turn_start_heading_ = 0.0;
    double last_plan_t_ = -1e9;
    double last_t_ = 0.0;
    Vec2 progress_ref_;
    double progress_t_ = 0.0;
    int stuck_count_ = 0;
    double backup_until_ = 0.0;
    double sample_started_t_ = 0.0;
    int search_step_ = 0;
    double idle_since_ = 0.0;
    bool goal_reached_ = false;
    int goal_failures_ = 0;
    double goal_retry_after_ = -1.0;
    int estops_ = 0;
    bool choose_after_backup_ = false;
    double v_cruise_ = 0.05;
    double orbit_right_ = 0.0, orbit_wrong_ = 0.0;
    double storm_ratio_ = 1.0;
    bool storm_mode_ = false;
    std::string last_sleep_reason_;
    double last_sleep_log_ = -1e9;
    std::string log_;
};

AXIMETRY_REGISTER_CONTROLLER(ReferenceController, "reference",
                             "EKF localisation + A* planning + adaptive science selection + power-aware operations.");

// Ablations, for experiments: switch off one capability at a time.
namespace {
struct RefNoLandmarks : ReferenceController {
    RefNoLandmarks() : ReferenceController(Options{false, true, true}) {}
};
struct RefNoVO : ReferenceController {
    RefNoVO() : ReferenceController(Options{true, false, true}) {}
};
struct RefNearest : ReferenceController {
    RefNearest() : ReferenceController(Options{true, true, false}) {}
};
}  // namespace
AXIMETRY_REGISTER_CONTROLLER(RefNoLandmarks, "ablate_landmarks", "Reference stack without landmark updates (odometry + VO + sun only).");
AXIMETRY_REGISTER_CONTROLLER(RefNoVO, "ablate_vo", "Reference stack without visual-odometry slip estimation.");
AXIMETRY_REGISTER_CONTROLLER(RefNearest, "ablate_value", "Reference stack that visits the nearest target instead of the most valuable per hour.");
