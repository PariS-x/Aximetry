// simple_drive — the "first attempt" controller.
//
// Navigates by dead reckoning only (wheel odometry + gyro), drives straight at
// the goal (or the best-looking target), and ignores the hazard scanner and
// landmarks. Run it on soft ground and watch the estimate drift away from the
// truth: wheel slip means the wheels turn further than the rover moves.
#include <cmath>

#include "aximetry/controller.hpp"

using namespace aximetry;

class SimpleDrive : public Controller {
public:
    void on_start(const MissionBrief& b) override {
        brief_ = b;
        x_ = b.start_pos.x;
        y_ = b.start_pos.y;
        h_ = b.start_heading;
        if (b.has_goal) {
            goal_ = b.goal;
            have_goal_ = true;
        } else if (!b.targets.empty()) {
            // Pick the target with the highest orbital prior.
            const TargetInfo* best = &b.targets[0];
            for (const auto& t : b.targets)
                if (t.orbital_prior > best->orbital_prior) best = &t;
            goal_ = best->pos;
            target_id_ = best->id;
            have_goal_ = true;
        }
    }

    Command on_step(const Observation& o) override {
        Command c;
        // --- Dead reckoning: distance from encoders, heading from the gyro.
        if (first_) {
            last_l_ = o.wheel_angle_left;
            last_r_ = o.wheel_angle_right;
            first_ = false;
        }
        const double r = brief_.rover.wheel_radius;
        const double ds = 0.5 * r * ((o.wheel_angle_left - last_l_) + (o.wheel_angle_right - last_r_));
        last_l_ = o.wheel_angle_left;
        last_r_ = o.wheel_angle_right;
        h_ = wrap_angle(h_ + o.gyro_z * o.dt);
        x_ += ds * std::cos(h_);
        y_ += ds * std::sin(h_);

        c.has_estimate = true;
        c.est_x = x_;
        c.est_y = y_;
        c.est_heading = h_;
        c.transmit = o.relay_in_view && o.data_queued_mbit > 0;

        // Sleep through the night.
        if (o.sun_elevation < 0.05) {
            c.status = "night";
            c.sleep_s = 1800.0;
            return c;
        }
        if (!have_goal_ || done_) {
            c.status = done_ ? "arrived (I think)" : "idle";
            if (done_ && target_id_ >= 0) c.sample_target = target_id_;
            return c;
        }
        const double dx = goal_.x - x_, dy = goal_.y - y_;
        const double dist = std::hypot(dx, dy);
        if (dist < 1.0) {
            done_ = true;
            c.log = "Believes it reached the goal";
            return c;
        }
        const double err = wrap_angle(std::atan2(dy, dx) - h_);
        const double speed = std::fabs(err) > 0.6 ? 0.0 : 0.8 * brief_.rover.max_wheel_speed * r;
        c.drive(speed, clampd(1.2 * err, -0.15, 0.15), brief_.rover);
        c.status = "driving";
        c.path_updated = !path_sent_;
        c.planned_path = {Vec2(x_, y_), goal_};
        path_sent_ = true;
        return c;
    }

private:
    MissionBrief brief_;
    double x_ = 0, y_ = 0, h_ = 0;
    double last_l_ = 0, last_r_ = 0;
    bool first_ = true, have_goal_ = false, done_ = false, path_sent_ = false;
    Vec2 goal_;
    int target_id_ = -1;
};

AXIMETRY_REGISTER_CONTROLLER(SimpleDrive, "simple_drive",
                             "Dead reckoning straight at the goal. No hazard avoidance, no landmarks.");
