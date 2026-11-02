// =============================================================================
//  My first Aximetry controller.
//
//  Edit this in the CODE tab of mission control and press COMPILE & LAUNCH.
//  The full API is in include/aximetry/controller.hpp; a guide is in
//  docs/writing_a_controller.md.
//
//  What this one does:
//    * keeps a position estimate from wheel encoders + gyro (dead reckoning)
//    * snaps that estimate to orbital landmarks (boulders) whenever it sees one
//    * drives to the science targets, best orbital guess first, then the goal
//    * swerves away from hazards, backs off after bumping a rock
//    * sleeps at night, sends data home
//  Ideas to improve it: estimate wheel slip, plan around soft sand, pick targets
//  by value per hour of driving, keep the battery above a reserve...
// =============================================================================
#include <algorithm>
#include <cmath>
#include <vector>

#include "aximetry/controller.hpp"

using namespace aximetry;

class MyRover : public Controller {
public:
    void on_start(const MissionBrief& b) override {
        brief_ = b;
        pos_ = b.start_pos;
        heading_ = b.start_heading;
        // Visit targets in order of how promising they look from orbit.
        for (const auto& t : b.targets) todo_.push_back(t);
        std::sort(todo_.begin(), todo_.end(),
                  [](const TargetInfo& a, const TargetInfo& c) { return a.orbital_prior > c.orbital_prior; });
        if (todo_.size() > 5) todo_.resize(5);
    }

    Command on_step(const Observation& o) override {
        Command c;
        update_estimate(o);
        const bool was_driving = driving_;
        driving_ = false;  // set again below if we command forward motion
        if (!was_driving) progress_t_ = o.t, progress_pos_ = pos_;

        // Draw my estimate in the replay viewer.
        c.has_estimate = true;
        c.est_x = pos_.x;
        c.est_y = pos_.y;
        c.est_heading = heading_;

        c.transmit = o.relay_in_view && o.data_queued_mbit > 0;   // send science home
        // A measurement finished: log it and cross that target off the list.
        for (const auto& r : o.new_results) {
            c.log = "Measured target " + std::to_string(r.target_id) + ": " + r.kind;
            todo_.erase(std::remove_if(todo_.begin(), todo_.end(), [&](const TargetInfo& t) { return t.id == r.target_id; }),
                        todo_.end());
        }

        // Night: sleep 30 minutes at a time (saves battery for the heaters).
        if (o.sun_elevation < 0.15) {
            c.sleep_s = 1800;
            c.status = "night";
            return c;
        }

        // Bumped into a rock, or wheels turning but going nowhere for 30 s?
        // Back up, then turn away for a while.
        bool stuck = false;
        for (const auto& e : o.events)
            if (e.rfind("collision", 0) == 0) stuck = true;
        if (o.t - progress_t_ > 30.0) {
            if (distance(pos_, progress_pos_) < 0.3) stuck = true;
            progress_pos_ = pos_;
            progress_t_ = o.t;
        }
        if (stuck && escape_steps_ == 0) {
            escape_steps_ = 120;  // 24 s at 5 Hz
            escape_dir_ = -escape_dir_;
        }
        if (escape_steps_ > 0) {
            --escape_steps_;
            if (escape_steps_ > 80) c.drive(-0.03, 0.0, brief_.rover);              // reverse
            else if (escape_steps_ > 50) c.drive(0.0, 0.15 * escape_dir_, brief_.rover);  // turn
            else c.drive(0.04, 0.0, brief_.rover);                                // drive clear
            c.status = "avoiding rock";
            return c;
        }

        // Busy with the spectrometer? Stay still until it finishes.
        if (o.sampling) {
            c.sample_target = current_target();
            c.status = "sampling";
            return c;
        }

        // Where am I going?
        Vec2 dest;
        if (!todo_.empty()) dest = todo_.front().pos;
        else if (brief_.has_goal && !goal_done_) dest = brief_.goal;
        else {
            c.status = "done";
            return c;
        }

        // Give each target a time budget: twice the straight-line drive time plus
        // 15 minutes. Daylight driving only, so the budget pauses at night.
        if (!todo_.empty() && current_target() != chasing_) {
            chasing_ = current_target();
            chase_budget_ = 2.0 * distance(pos_, todo_.front().pos) / 0.04 + 900.0;
        }
        chase_budget_ -= o.dt < 1.0 ? o.dt : 0.0;
        if (!todo_.empty() && chase_budget_ < 0.0) {
            c.log = "Target " + std::to_string(current_target()) + " is taking too long, skipping";
            todo_.erase(todo_.begin());
            return c;
        }

        const double dist = distance(pos_, dest);
        if (!todo_.empty() && dist < 0.7) {
            // Arrived (by my estimate). Try to sample; give up after 20 s if out of range.
            c.sample_target = current_target();
            if (++wait_steps_ > 100) {
                c.log = "Could not reach target " + std::to_string(current_target()) + ", skipping";
                todo_.erase(todo_.begin());
                wait_steps_ = 0;
            }
            c.status = "sampling";
            return c;
        }
        wait_steps_ = 0;
        if (todo_.empty() && dist < 1.5) {
            goal_done_ = true;
            c.log = "Reached the goal";
            return c;
        }
        // Something close ahead? Pick the clearer side and commit to it for 4 s
        // (deciding afresh every step makes the rover dither on the spot).
        if (avoid_steps_ == 0) {
            double left = 0, right = 0;
            bool blocked = false;
            for (const auto& ray : o.hazards) {
                if (ray.hit && ray.range < 1.2 && std::fabs(ray.angle) < 0.5) blocked = true;
                (ray.angle > 0 ? left : right) += ray.range;  // more free range = clearer side
            }
            if (blocked) {
                avoid_steps_ = 20;
                avoid_dir_ = left > right ? 1.0 : -1.0;
            }
        }
        if (avoid_steps_ > 0) {
            --avoid_steps_;
            c.drive(0.015, 0.15 * avoid_dir_, brief_.rover);  // creep forward while turning away
            c.status = "dodging";
            return c;
        }

        // Steer toward the destination; turn on the spot if pointing far off.
        const double error = wrap_angle((dest - pos_).angle() - heading_);
        const double speed = std::fabs(error) > 0.6 ? 0.0 : 0.04;
        c.drive(speed, clampd(1.0 * error, -0.15, 0.15), brief_.rover);
        driving_ = speed > 0.0;
        c.status = todo_.empty() ? "to goal" : "to target " + std::to_string(current_target());
        return c;
    }

private:
    void update_estimate(const Observation& o) {
        if (first_) {
            last_l_ = o.wheel_angle_left;
            last_r_ = o.wheel_angle_right;
            first_ = false;
        }
        // Distance from the wheels (wrong when they slip!), heading from the gyro.
        const double ds = 0.5 * brief_.rover.wheel_radius * ((o.wheel_angle_left - last_l_) + (o.wheel_angle_right - last_r_));
        last_l_ = o.wheel_angle_left;
        last_r_ = o.wheel_angle_right;
        if (o.dt < 1.0) heading_ = wrap_angle(heading_ + o.gyro_z * o.dt);
        pos_ += Vec2::from_angle(heading_) * ds;

        // The sun sensor gives an absolute heading in daylight: blend it in.
        if (o.sun_heading_valid) heading_ = wrap_angle(heading_ + 0.2 * wrap_angle(o.sun_heading - heading_));

        // Each landmark sighting says where I must be: landmark - range in that direction.
        for (const auto& lm : o.landmarks)
            for (const auto& known : brief_.landmarks)
                if (known.id == lm.id) {
                    const Vec2 fix = known.pos - Vec2::from_angle(heading_ + lm.bearing) * lm.range;
                    pos_ = pos_ + (fix - pos_) * 0.3;   // move 30% of the way toward the fix
                }
    }

    int current_target() const { return todo_.empty() ? -1 : todo_.front().id; }

    MissionBrief brief_;
    std::vector<TargetInfo> todo_;
    Vec2 pos_;
    double heading_ = 0.0;
    double last_l_ = 0.0, last_r_ = 0.0;
    bool first_ = true, goal_done_ = false;
    int wait_steps_ = 0;
    int escape_steps_ = 0;
    int avoid_steps_ = 0;
    double avoid_dir_ = 1.0;
    bool driving_ = false;
    Vec2 progress_pos_;
    double progress_t_ = 0.0;
    int chasing_ = -1;
    double chase_budget_ = 0.0;
    double escape_dir_ = 1.0;
};

AXIMETRY_REGISTER_CONTROLLER(MyRover, "my_rover", "My first controller: landmarks + dead reckoning, best targets first.");
