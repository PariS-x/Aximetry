// Traversability cost map + A* path planner.
//
// The map starts from orbital data (DEM slopes, soil-class guesses, catalogued
// boulders) and is updated live from the hazard scanner. Costs encode what the
// terramechanics says is risky: steep ground and loose drift, especially both.
#pragma once

#include <vector>

#include "aximetry/controller.hpp"

namespace refctl {

using aximetry::Vec2;

struct PlannerParams {
    double resolution = 0.5;     // m
    double inflate = 0.9;        // obstacle inflation (rover half-width + margin), m
    double max_slope_deg = 20.0; // impassable above this
    double drift_cost = 8.0;     // extra cost per cell of loose drift
    double drift_max_slope_deg = 5.0;  // drift steeper than this is a trap
};

class CostMap {
public:
    void build(const aximetry::MissionBrief& brief, const PlannerParams& p);
    bool to_cell(const Vec2& p, int& i, int& j) const;
    Vec2 center(int i, int j) const { return {i * res_, j * res_}; }
    bool blocked(int i, int j) const { return obst_[j * nx_ + i] != 0; }
    double cost(int i, int j) const { return cost_[j * nx_ + i]; }
    bool blocked_at(const Vec2& p) const;

    // Mark a hazard at world point p (inflated). Returns true if anything changed.
    bool add_obstacle(const Vec2& p, double radius);
    // Penalise an area (e.g. where the rover got stuck).
    void add_cost(const Vec2& p, double radius, double extra);

    int nx() const { return nx_; }
    int ny() const { return ny_; }
    double res() const { return res_; }
    int version() const { return version_; }

private:
    int nx_ = 0, ny_ = 0;
    double res_ = 0.5;
    double inflate_ = 0.9;
    std::vector<float> cost_;
    std::vector<uint8_t> obst_;
    int version_ = 0;
};

// A* over the cost map. If the start cell is blocked (the rover is inside an
// inflated zone), the search may drive out to the nearest free cell, provided
// one exists within `escape_radius`. Returns an empty path if unreachable.
std::vector<Vec2> plan_path(const CostMap& map, const Vec2& start, const Vec2& goal, double escape_radius,
                            double* cost_out = nullptr);

double path_length(const std::vector<Vec2>& path);

}  // namespace refctl
