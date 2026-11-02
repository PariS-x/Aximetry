#include "ekf.hpp"

#include <cmath>

namespace refctl {

using aximetry::sq;
using aximetry::wrap_angle;

void PoseEKF::init(const Vec2& p, double heading, double sigma_pos, double sigma_heading) {
    x_(0, 0) = p.x;
    x_(1, 0) = p.y;
    x_(2, 0) = heading;
    P_ = Mat<3, 3>();
    P_(0, 0) = P_(1, 1) = sq(sigma_pos);
    P_(2, 2) = sq(sigma_heading);
}

void PoseEKF::predict(double ds, double dth, double sigma_ds, double sigma_dth, double sigma_xy) {
    const double th = x_(2, 0) + 0.5 * dth;
    const double c = std::cos(th), s = std::sin(th);
    x_(0, 0) += ds * c;
    x_(1, 0) += ds * s;
    x_(2, 0) = wrap_angle(x_(2, 0) + dth);

    Mat<3, 3> F = Mat<3, 3>::identity();
    F(0, 2) = -ds * s;
    F(1, 2) = ds * c;
    Mat<3, 2> G;
    G(0, 0) = c; G(0, 1) = -0.5 * ds * s;
    G(1, 0) = s; G(1, 1) = 0.5 * ds * c;
    G(2, 0) = 0; G(2, 1) = 1.0;
    Mat<2, 2> Qu;
    Qu(0, 0) = sq(sigma_ds);
    Qu(1, 1) = sq(sigma_dth);
    P_ = F * P_ * F.t() + G * Qu * G.t();
    P_(0, 0) += sq(sigma_xy);
    P_(1, 1) += sq(sigma_xy);
}

void PoseEKF::inflate_position(double sigma) {
    P_(0, 0) = std::max(P_(0, 0), sq(sigma));
    P_(1, 1) = std::max(P_(1, 1), sq(sigma));
    P_(0, 1) = P_(1, 0) = 0.0;
    P_(0, 2) = P_(2, 0) = P_(1, 2) = P_(2, 1) = 0.0;
}

bool PoseEKF::update_landmark(const Vec2& lm, double range, double bearing, double sr, double sb) {
    const double dx = lm.x - x_(0, 0), dy = lm.y - x_(1, 0);
    const double q = dx * dx + dy * dy;
    const double r = std::sqrt(q);
    if (r < 0.3) return false;
    Mat<2, 1> innov;
    innov(0, 0) = range - r;
    innov(1, 0) = wrap_angle(bearing - wrap_angle(std::atan2(dy, dx) - x_(2, 0)));
    Mat<2, 3> H;
    H(0, 0) = -dx / r; H(0, 1) = -dy / r; H(0, 2) = 0.0;
    H(1, 0) = dy / q;  H(1, 1) = -dx / q; H(1, 2) = -1.0;
    Mat<2, 2> R;
    R(0, 0) = sq(sr);
    R(1, 1) = sq(sb);
    const Mat<2, 2> S = H * P_ * H.t() + R;
    const Mat<2, 2> Si = aximetry::inverse(S);
    const double d2 = (innov.t() * Si * innov)(0, 0);
    if (d2 > 13.8) return false;  // chi-square, 2 dof, 99.9 %: rejects bad associations
    const Mat<3, 2> K = P_ * H.t() * Si;
    x_ = x_ + K * innov;
    x_(2, 0) = wrap_angle(x_(2, 0));
    // Joseph form keeps P symmetric positive definite.
    const Mat<3, 3> IKH = Mat<3, 3>::identity() - K * H;
    P_ = IKH * P_ * IKH.t() + K * R * K.t();
    return true;
}

bool PoseEKF::update_heading(double heading, double sigma) {
    const double innov = wrap_angle(heading - x_(2, 0));
    const double S = P_(2, 2) + sq(sigma);
    if (innov * innov / S > 10.8) return false;  // 1 dof, 99.9 %
    Mat<3, 1> K;
    for (int i = 0; i < 3; ++i) K(i, 0) = P_(i, 2) / S;
    for (int i = 0; i < 3; ++i) x_(i, 0) += K(i, 0) * innov;
    x_(2, 0) = wrap_angle(x_(2, 0));
    Mat<1, 3> H;
    H(0, 2) = 1.0;
    const Mat<3, 3> IKH = Mat<3, 3>::identity() - K * H;
    Mat<1, 1> R;
    R(0, 0) = sq(sigma);
    P_ = IKH * P_ * IKH.t() + K * R * K.t();
    return true;
}

double PoseEKF::pos_sigma() const { return std::sqrt(std::max(0.0, 0.5 * (P_(0, 0) + P_(1, 1)))); }
double PoseEKF::heading_sigma() const { return std::sqrt(std::max(0.0, P_(2, 2))); }

}  // namespace refctl
