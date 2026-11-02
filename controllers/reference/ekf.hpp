// Planar pose EKF: state [x, y, heading].
//   predict — wheel odometry (slip-corrected) + gyro
//   update  — range/bearing to catalogued boulders, absolute heading from the sun
#pragma once

#include "aximetry/math.hpp"

namespace refctl {

using aximetry::Mat;
using aximetry::Vec2;

class PoseEKF {
public:
    void init(const Vec2& p, double heading, double sigma_pos, double sigma_heading);
    // sigma_xy adds isotropic position noise (e.g. scrubbing while skid-steering).
    void predict(double ds, double dheading, double sigma_ds, double sigma_dheading, double sigma_xy = 0.0);
    // Widen the position uncertainty (used to re-acquire after repeated rejections).
    void inflate_position(double sigma);
    // Returns true if the measurement passed the Mahalanobis gate and was applied.
    bool update_landmark(const Vec2& landmark, double range, double bearing, double sigma_range, double sigma_bearing);
    bool update_heading(double heading, double sigma);

    Vec2 pos() const { return {x_(0, 0), x_(1, 0)}; }
    double heading() const { return x_(2, 0); }
    double pos_sigma() const;
    double heading_sigma() const;
    const Mat<3, 3>& cov() const { return P_; }

private:
    Mat<3, 1> x_;
    Mat<3, 3> P_;
};

}  // namespace refctl
