#include "aximetry/terrain.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aximetry {

const std::vector<TargetKind>& target_kinds() {
    // Values express astrobiological interest on a 0..1 scale. Hydrated silica
    // and clays preserve organics well, so they rank highest.
    static const std::vector<TargetKind> k = {
        {"basalt", "Unaltered basalt", 0.15, 0.40},
        {"sulfate", "Sulfate-bearing vein", 0.45, 0.25},
        {"clay", "Clay-bearing mudstone", 0.70, 0.20},
        {"silica", "Opaline silica (hydrothermal)", 0.92, 0.15},
    };
    return k;
}

double kind_mean_value(const std::string& kind) {
    for (const auto& k : target_kinds())
        if (k.id == kind) return k.mean_value;
    throw std::runtime_error("unknown target kind: " + kind);
}

namespace {

// Lattice value noise with smooth interpolation, summed over octaves (fBm).
class ValueNoise {
public:
    ValueNoise(uint64_t seed, int cells) : n_(cells), v_((cells + 1) * (cells + 1)) {
        Rng rng(seed);
        for (auto& x : v_) x = rng.uniform(-1.0, 1.0);
    }
    double at(double u, double w) const {  // u,w in lattice units
        u = std::fmod(std::fmod(u, n_) + n_, n_);
        w = std::fmod(std::fmod(w, n_) + n_, n_);
        const int i = static_cast<int>(u), j = static_cast<int>(w);
        const double fu = u - i, fw = w - j;
        const double su = fu * fu * (3 - 2 * fu), sw = fw * fw * (3 - 2 * fw);
        auto V = [&](int a, int b) { return v_[(b % (n_ + 1)) * (n_ + 1) + (a % (n_ + 1))]; };
        const double a = V(i, j) + su * (V(i + 1, j) - V(i, j));
        const double b = V(i, j + 1) + su * (V(i + 1, j + 1) - V(i, j + 1));
        return a + sw * (b - a);
    }

private:
    int n_;
    std::vector<double> v_;
};

double fbm(const ValueNoise& n, double x, double y, double wavelength, int octaves) {
    double sum = 0.0, amp = 1.0, norm = 0.0, f = 1.0 / wavelength;
    for (int o = 0; o < octaves; ++o) {
        sum += amp * n.at(x * f + o * 17.3, y * f + o * 31.7);
        norm += amp;
        amp *= 0.5;
        f *= 2.0;
    }
    return sum / norm;
}

double quantile_threshold(std::vector<double> v, double top_fraction) {
    if (top_fraction <= 0.0) return 1e30;
    if (top_fraction >= 1.0) return -1e30;
    const size_t k = static_cast<size_t>((1.0 - top_fraction) * (v.size() - 1));
    std::nth_element(v.begin(), v.begin() + k, v.end());
    return v[k];
}

}  // namespace

Terrain Terrain::generate(const TerrainConfig& cfg, uint64_t seed) {
    Terrain t;
    t.size_ = cfg.size_m;
    t.res_ = cfg.resolution_m;
    t.nx_ = t.ny_ = static_cast<int>(std::round(cfg.size_m / cfg.resolution_m)) + 1;
    const int N = t.nx_ * t.ny_;
    t.h_.assign(N, 0.0);
    t.soil_.assign(N, 0);

    Rng rng(seed);
    ValueNoise hills(rng.next_u64(), 64), driftn(rng.next_u64(), 64), crustn(rng.next_u64(), 64);

    // --- Craters: explicit + random
    std::vector<CraterSpec> craters = cfg.craters;
    for (int c = 0; c < cfg.random_craters; ++c) {
        const double R = rng.uniform(3.0, 12.0);
        craters.push_back({rng.uniform(R, cfg.size_m - R), rng.uniform(R, cfg.size_m - R), R, R * rng.uniform(0.12, 0.2)});
    }

    for (int j = 0; j < t.ny_; ++j) {
        for (int i = 0; i < t.nx_; ++i) {
            const double x = i * t.res_, y = j * t.res_;
            double h = cfg.relief_m * fbm(hills, x, y, cfg.feature_m, 5) + x * std::tan(deg2rad(cfg.incline_deg));
            for (const auto& c : craters) {
                const double r = std::hypot(x - c.x, y - c.y) / c.radius;
                if (r < 1.0) h -= c.depth * (1.0 - r * r);                         // bowl
                h += 0.22 * c.depth * std::exp(-sq((r - 1.0) / 0.18));              // raised rim
            }
            t.h_[j * t.nx_ + i] = h;
        }
    }

    // --- Soils
    t.soils_.push_back(soil_by_id(cfg.base_soil));
    auto soil_slot = [&](const std::string& id) {
        for (size_t k = 0; k < t.soils_.size(); ++k)
            if (t.soils_[k].id == id) return static_cast<int>(k);
        t.soils_.push_back(soil_by_id(id));
        return static_cast<int>(t.soils_.size() - 1);
    };
    const int drift = soil_slot("loose_drift");
    const int crust = soil_slot("duricrust");
    const int rock = soil_slot("bedrock");

    std::vector<double> dn(N), cn(N);
    for (int j = 0; j < t.ny_; ++j)
        for (int i = 0; i < t.nx_; ++i) {
            dn[j * t.nx_ + i] = fbm(driftn, i * t.res_, j * t.res_, 18.0, 3);
            cn[j * t.nx_ + i] = fbm(crustn, i * t.res_, j * t.res_, 22.0, 3);
        }
    const double dthr = quantile_threshold(dn, cfg.drift_fraction);
    const double cthr = quantile_threshold(cn, cfg.crust_fraction);
    for (int j = 0; j < t.ny_; ++j) {
        for (int i = 0; i < t.nx_; ++i) {
            const int k = j * t.nx_ + i;
            const Vec2 p(i * t.res_, j * t.res_);
            int s = 0;
            if (cn[k] > cthr) s = crust;
            if (dn[k] > dthr) s = drift;
            // Wind piles drift on crater floors.
            for (const auto& c : craters)
                if (std::hypot(p.x - c.x, p.y - c.y) < 0.45 * c.radius && dn[k] > dthr - 0.25) s = drift;
            t.soil_[k] = static_cast<uint8_t>(s);
        }
    }
    // Steep ground sheds loose material → exposed bedrock.
    for (int j = 0; j < t.ny_; ++j)
        for (int i = 0; i < t.nx_; ++i)
            if (t.slope_deg(Vec2(i * t.res_, j * t.res_)) > cfg.outcrop_slope_deg) t.soil_[j * t.nx_ + i] = static_cast<uint8_t>(rock);
    for (const auto& p : cfg.patches) {
        const int s = soil_slot(p.soil);
        for (int j = 0; j < t.ny_; ++j)
            for (int i = 0; i < t.nx_; ++i)
                if (std::hypot(i * t.res_ - p.x, j * t.res_ - p.y) < p.radius) t.soil_[j * t.nx_ + i] = static_cast<uint8_t>(s);
    }

    // --- Rocks
    auto clear_ok = [&](const Vec2& p, double r) {
        for (const auto& c : cfg.keep_clear)
            if (distance(p, c) < cfg.keep_clear_radius + r) return false;
        return true;
    };
    const int nrocks = static_cast<int>(cfg.rock_density * cfg.size_m * cfg.size_m);
    for (int k = 0; k < nrocks; ++k) {
        Rock r;
        r.id = k;
        const bool boulder = rng.chance(cfg.boulder_fraction);
        // Size-frequency: many small rocks, few big ones.
        r.radius = boulder ? rng.uniform(0.5, 1.4) : clampd(0.04 + rng.uniform() * rng.uniform() * 0.45, 0.04, 0.49);
        r.height = r.radius * rng.uniform(0.5, 0.9);
        r.catalogued = r.radius >= 0.5;
        int tries = 0;
        do {
            r.pos = Vec2(rng.uniform(1.0, cfg.size_m - 1.0), rng.uniform(1.0, cfg.size_m - 1.0));
        } while (!clear_ok(r.pos, r.radius) && ++tries < 50);
        if (tries >= 50) continue;
        r.id = static_cast<int>(t.rocks_.size());
        t.rocks_.push_back(r);
    }
    t.build_buckets();

    // --- Science targets
    const auto& kinds = target_kinds();
    auto pick_kind = [&]() {
        double u = rng.uniform(), acc = 0.0;
        for (const auto& k : kinds) {
            acc += k.frequency;
            if (u <= acc) return k.id;
        }
        return kinds.back().id;
    };
    auto make_target = [&](const Vec2& p, const std::string& kind) {
        ScienceTarget tg;
        tg.id = static_cast<int>(t.targets_.size());
        tg.pos = p;
        tg.true_kind = kind;
        tg.true_value = clampd(rng.normal(kind_mean_value(kind), 0.07), 0.0, 1.0);
        tg.orbital_kind = kind;
        if (rng.chance(cfg.orbital_misclassification)) {
            std::string other;
            do { other = kinds[rng.uniform_int(0, static_cast<int>(kinds.size()) - 1)].id; } while (other == kind);
            tg.orbital_kind = other;
        }
        tg.orbital_prior = kind_mean_value(tg.orbital_kind);
        t.targets_.push_back(tg);
    };
    for (const auto& f : cfg.fixed_targets) make_target(Vec2(f.x, f.y), f.kind);
    int guard = 0;
    while (static_cast<int>(t.targets_.size()) < cfg.science_targets + static_cast<int>(cfg.fixed_targets.size()) && ++guard < 5000) {
        Vec2 p(rng.uniform(8.0, cfg.size_m - 8.0), rng.uniform(8.0, cfg.size_m - 8.0));
        bool ok = clear_ok(p, 2.0) && t.slope_deg(p) < 20.0;
        for (const auto& o : t.targets_) ok = ok && distance(o.pos, p) > 10.0;
        std::vector<int> near;
        t.rocks_near(p, 2.5, near);
        for (int id : near) ok = ok && distance(t.rocks_[id].pos, p) > t.rocks_[id].radius + 1.2;
        if (ok) make_target(p, pick_kind());
    }
    return t;
}

void Terrain::build_buckets() {
    bnx_ = static_cast<int>(std::ceil(size_ / bucket_)) + 1;
    buckets_.assign(bnx_ * bnx_, {});
    for (const auto& r : rocks_) {
        const int bi = clampd(std::floor(r.pos.x / bucket_), 0, bnx_ - 1);
        const int bj = clampd(std::floor(r.pos.y / bucket_), 0, bnx_ - 1);
        buckets_[bj * bnx_ + bi].push_back(r.id);
    }
}

void Terrain::rocks_near(const Vec2& p, double radius, std::vector<int>& out) const {
    out.clear();
    const int i0 = std::max(0, static_cast<int>(std::floor((p.x - radius - 1.5) / bucket_)));
    const int i1 = std::min(bnx_ - 1, static_cast<int>(std::floor((p.x + radius + 1.5) / bucket_)));
    const int j0 = std::max(0, static_cast<int>(std::floor((p.y - radius - 1.5) / bucket_)));
    const int j1 = std::min(bnx_ - 1, static_cast<int>(std::floor((p.y + radius + 1.5) / bucket_)));
    for (int j = j0; j <= j1; ++j)
        for (int i = i0; i <= i1; ++i)
            for (int id : buckets_[j * bnx_ + i])
                if (distance(rocks_[id].pos, p) <= radius + rocks_[id].radius) out.push_back(id);
}

bool Terrain::in_bounds(const Vec2& p, double margin) const {
    return p.x >= margin && p.y >= margin && p.x <= size_ - margin && p.y <= size_ - margin;
}

double Terrain::height(const Vec2& p) const {
    const double u = clampd(p.x / res_, 0.0, nx_ - 1.000001);
    const double v = clampd(p.y / res_, 0.0, ny_ - 1.000001);
    const int i = static_cast<int>(u), j = static_cast<int>(v);
    const double fu = u - i, fv = v - j;
    const double a = h_[j * nx_ + i] * (1 - fu) + h_[j * nx_ + i + 1] * fu;
    const double b = h_[(j + 1) * nx_ + i] * (1 - fu) + h_[(j + 1) * nx_ + i + 1] * fu;
    return a * (1 - fv) + b * fv;
}

Vec2 Terrain::gradient(const Vec2& p) const {
    // Central differences over one cell: smooth enough for slope and attitude.
    const double d = res_;
    return {(height({p.x + d, p.y}) - height({p.x - d, p.y})) / (2 * d),
            (height({p.x, p.y + d}) - height({p.x, p.y - d})) / (2 * d)};
}

double Terrain::slope_deg(const Vec2& p) const { return rad2deg(std::atan(gradient(p).norm())); }

int Terrain::soil_index(const Vec2& p) const {
    const int i = clampd(std::lround(p.x / res_), 0, nx_ - 1);
    const int j = clampd(std::lround(p.y / res_), 0, ny_ - 1);
    return soil_[j * nx_ + i];
}

}  // namespace aximetry
