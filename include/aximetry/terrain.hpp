// Procedural Martian terrain: a heightmap with craters, a soil-type map, a rock
// field (with orbit-visible boulders that double as navigation landmarks) and
// science targets whose true value is hidden until a rover measures them.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "aximetry/math.hpp"
#include "aximetry/terramechanics.hpp"

namespace aximetry {

struct Rock {
    int id = 0;
    Vec2 pos;
    double radius = 0.1;   // m
    double height = 0.1;   // m
    bool catalogued = false;  // resolvable from orbit → usable as a landmark
};

struct ScienceTarget {
    int id = 0;
    Vec2 pos;
    std::string true_kind;
    std::string orbital_kind;  // what orbital spectroscopy suggests (may be wrong)
    double true_value = 0.0;   // 0..1, revealed only by the rover's spectrometer
    double orbital_prior = 0.0;
};

struct CraterSpec { double x, y, radius, depth; };
struct PatchSpec { double x, y, radius; std::string soil; };
struct TargetSpec { double x, y; std::string kind; };

struct TerrainConfig {
    double size_m = 160.0;
    double resolution_m = 0.5;
    double relief_m = 2.5;           // amplitude of rolling terrain
    double feature_m = 24.0;         // dominant hill wavelength
    double incline_deg = 0.0;        // tilt the whole map up along +x (for experiments)
    std::vector<CraterSpec> craters;
    int random_craters = 4;
    std::string base_soil = "m90_simulant";
    double drift_fraction = 0.12;    // fraction of area covered by loose drift
    double crust_fraction = 0.15;    // fraction of area that is cemented crust
    double outcrop_slope_deg = 18.0; // steeper than this is exposed bedrock
    std::vector<PatchSpec> patches;  // explicitly placed soil patches
    double rock_density = 0.04;      // rocks per m^2 (all sizes)
    double boulder_fraction = 0.04;  // fraction of rocks that are boulders (> 0.5 m)
    int science_targets = 10;
    std::vector<TargetSpec> fixed_targets;
    double orbital_misclassification = 0.3;
    std::vector<Vec2> keep_clear;    // points to keep free of rocks (start, goal)
    double keep_clear_radius = 4.0;
};

struct TargetKind {
    std::string id;
    std::string name;
    double mean_value;
    double frequency;
};
const std::vector<TargetKind>& target_kinds();
double kind_mean_value(const std::string& kind);

class Terrain {
public:
    static Terrain generate(const TerrainConfig& cfg, uint64_t seed);

    double size() const { return size_; }
    double resolution() const { return res_; }
    int nx() const { return nx_; }
    int ny() const { return ny_; }
    bool in_bounds(const Vec2& p, double margin = 0.0) const;

    double height(const Vec2& p) const;      // bilinear
    Vec2 gradient(const Vec2& p) const;      // dh/dx, dh/dy
    double slope_deg(const Vec2& p) const;
    int soil_index(const Vec2& p) const;
    const Soil& soil(const Vec2& p) const { return soils_[soil_index(p)]; }
    const std::vector<Soil>& soils() const { return soils_; }

    double height_cell(int i, int j) const { return h_[j * nx_ + i]; }
    int soil_cell(int i, int j) const { return soil_[j * nx_ + i]; }

    const std::vector<Rock>& rocks() const { return rocks_; }
    const std::vector<ScienceTarget>& targets() const { return targets_; }
    // Rock ids whose centres lie within `radius` of p.
    void rocks_near(const Vec2& p, double radius, std::vector<int>& out) const;

private:
    double size_ = 0.0, res_ = 0.5;
    int nx_ = 0, ny_ = 0;
    std::vector<double> h_;
    std::vector<uint8_t> soil_;
    std::vector<Soil> soils_;
    std::vector<Rock> rocks_;
    std::vector<ScienceTarget> targets_;
    double bucket_ = 4.0;
    int bnx_ = 0;
    std::vector<std::vector<int>> buckets_;
    void build_buckets();
};

}  // namespace aximetry
