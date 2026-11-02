// Small, dependency-free math helpers: 2D vectors, fixed-size matrices, RNG.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <random>

namespace aximetry {

constexpr double kPi = 3.14159265358979323846;

inline double deg2rad(double d) { return d * kPi / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / kPi; }
inline double sq(double x) { return x * x; }
inline double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

// Wrap an angle to (-pi, pi].
inline double wrap_angle(double a) {
    a = std::fmod(a + kPi, 2.0 * kPi);
    if (a <= 0.0) a += 2.0 * kPi;
    return a - kPi;
}

struct Vec2 {
    double x = 0.0, y = 0.0;
    Vec2() = default;
    Vec2(double x_, double y_) : x(x_), y(y_) {}
    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(double s) const { return {x * s, y * s}; }
    Vec2 operator/(double s) const { return {x / s, y / s}; }
    Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
    double dot(const Vec2& o) const { return x * o.x + y * o.y; }
    double cross(const Vec2& o) const { return x * o.y - y * o.x; }
    double norm() const { return std::sqrt(x * x + y * y); }
    double angle() const { return std::atan2(y, x); }
    static Vec2 from_angle(double a) { return {std::cos(a), std::sin(a)}; }
    // Rotate a body-frame vector into the world frame by heading psi.
    Vec2 rotated(double psi) const {
        const double c = std::cos(psi), s = std::sin(psi);
        return {c * x - s * y, s * x + c * y};
    }
};

inline double distance(const Vec2& a, const Vec2& b) { return (a - b).norm(); }

// Fixed-size row-major matrix. Enough for small filters (EKF etc.).
template <int R, int C>
struct Mat {
    std::array<double, R * C> d{};
    double& operator()(int r, int c) { return d[r * C + c]; }
    double operator()(int r, int c) const { return d[r * C + c]; }
    static Mat identity() {
        Mat m;
        for (int i = 0; i < std::min(R, C); ++i) m(i, i) = 1.0;
        return m;
    }
    Mat<C, R> t() const {
        Mat<C, R> m;
        for (int r = 0; r < R; ++r)
            for (int c = 0; c < C; ++c) m(c, r) = (*this)(r, c);
        return m;
    }
    Mat operator+(const Mat& o) const { Mat m; for (int i = 0; i < R * C; ++i) m.d[i] = d[i] + o.d[i]; return m; }
    Mat operator-(const Mat& o) const { Mat m; for (int i = 0; i < R * C; ++i) m.d[i] = d[i] - o.d[i]; return m; }
    Mat operator*(double s) const { Mat m; for (int i = 0; i < R * C; ++i) m.d[i] = d[i] * s; return m; }
};

template <int R, int K, int C>
Mat<R, C> operator*(const Mat<R, K>& a, const Mat<K, C>& b) {
    Mat<R, C> m;
    for (int r = 0; r < R; ++r)
        for (int c = 0; c < C; ++c) {
            double s = 0.0;
            for (int k = 0; k < K; ++k) s += a(r, k) * b(k, c);
            m(r, c) = s;
        }
    return m;
}

inline Mat<2, 2> inverse(const Mat<2, 2>& m) {
    const double det = m(0, 0) * m(1, 1) - m(0, 1) * m(1, 0);
    Mat<2, 2> r;
    r(0, 0) = m(1, 1) / det;
    r(0, 1) = -m(0, 1) / det;
    r(1, 0) = -m(1, 0) / det;
    r(1, 1) = m(0, 0) / det;
    return r;
}

// Seeded random source. Every stochastic thing in the simulator draws from one
// of these so runs are reproducible from the mission seed.
class Rng {
public:
    explicit Rng(uint64_t seed = 1) : eng_(seed) {}
    double uniform(double a = 0.0, double b = 1.0) { return std::uniform_real_distribution<double>(a, b)(eng_); }
    double normal(double mean = 0.0, double sd = 1.0) {
        if (sd <= 0.0) return mean;
        return std::normal_distribution<double>(mean, sd)(eng_);
    }
    int uniform_int(int a, int b) { return std::uniform_int_distribution<int>(a, b)(eng_); }
    bool chance(double p) { return uniform() < p; }
    uint64_t next_u64() { return eng_(); }

private:
    std::mt19937_64 eng_;
};

}  // namespace aximetry
