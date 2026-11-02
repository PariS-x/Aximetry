#include "planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace refctl {

using aximetry::clampd;

void CostMap::build(const aximetry::MissionBrief& b, const PlannerParams& p) {
    res_ = p.resolution;
    inflate_ = p.inflate;
    nx_ = ny_ = static_cast<int>(std::round(b.map.size_m / res_)) + 1;
    cost_.assign(nx_ * ny_, 1.0f);
    obst_.assign(nx_ * ny_, 0);
    const double edge = 1.6;
    for (int j = 0; j < ny_; ++j)
        for (int i = 0; i < nx_; ++i) {
            const Vec2 c = center(i, j);
            const int k = j * nx_ + i;
            if (c.x < edge || c.y < edge || c.x > b.map.size_m - edge || c.y > b.map.size_m - edge) {
                obst_[k] = 1;
                continue;
            }
            const double slope = b.map.slope_deg_at(c.x, c.y);
            const std::string& soil = b.map.soil_at(c.x, c.y);
            double cost = 1.0 + 3.0 * aximetry::sq(slope / p.max_slope_deg);
            if (soil == "loose_drift") {
                cost += p.drift_cost;
                // Sloped drift is where rovers get embedded: very expensive, but not
                // forbidden (the rover may already be in it and need to drive out).
                if (slope > p.drift_max_slope_deg) cost += 6.0 * p.drift_cost;
            } else if (soil == "dry_sand") {
                cost += 0.3 * p.drift_cost;
            }
            if (slope > p.max_slope_deg) obst_[k] = 1;
            cost_[k] = static_cast<float>(cost);
        }
    for (const auto& lm : b.landmarks) add_obstacle(lm.pos, lm.radius);
    version_ = 0;
}

bool CostMap::to_cell(const Vec2& p, int& i, int& j) const {
    i = static_cast<int>(std::lround(p.x / res_));
    j = static_cast<int>(std::lround(p.y / res_));
    return i >= 0 && j >= 0 && i < nx_ && j < ny_;
}

bool CostMap::blocked_at(const Vec2& p) const {
    int i, j;
    if (!to_cell(p, i, j)) return true;
    return blocked(i, j);
}

bool CostMap::add_obstacle(const Vec2& p, double radius) {
    const double R = radius + inflate_;
    const int r = static_cast<int>(std::ceil(R / res_));
    int ci, cj;
    to_cell(p, ci, cj);
    bool changed = false;
    for (int j = cj - r; j <= cj + r; ++j)
        for (int i = ci - r; i <= ci + r; ++i) {
            if (i < 0 || j < 0 || i >= nx_ || j >= ny_) continue;
            if (aximetry::distance(center(i, j), p) > R) continue;
            if (!obst_[j * nx_ + i]) {
                obst_[j * nx_ + i] = 1;
                changed = true;
            }
        }
    if (changed) ++version_;
    return changed;
}

void CostMap::add_cost(const Vec2& p, double radius, double extra) {
    const int r = static_cast<int>(std::ceil(radius / res_));
    int ci, cj;
    to_cell(p, ci, cj);
    for (int j = cj - r; j <= cj + r; ++j)
        for (int i = ci - r; i <= ci + r; ++i) {
            if (i < 0 || j < 0 || i >= nx_ || j >= ny_) continue;
            if (aximetry::distance(center(i, j), p) > radius) continue;
            cost_[j * nx_ + i] += static_cast<float>(extra);
        }
    ++version_;
}

double path_length(const std::vector<Vec2>& path) {
    double L = 0.0;
    for (size_t k = 1; k < path.size(); ++k) L += aximetry::distance(path[k - 1], path[k]);
    return L;
}

std::vector<Vec2> plan_path(const CostMap& map, const Vec2& start, const Vec2& goal_in, double escape, double* cost_out) {
    const int nx = map.nx(), ny = map.ny();
    int si, sj, gi, gj;
    if (!map.to_cell(start, si, sj)) return {};
    map.to_cell(goal_in, gi, gj);
    gi = std::max(0, std::min(nx - 1, gi));
    gj = std::max(0, std::min(ny - 1, gj));
    // Escape: if the start cell is blocked (the rover is inside an inflated zone,
    // e.g. after a position correction), allow driving out to the nearest free cell.
    double escape_r = 0.0;
    if (map.blocked(si, sj)) {
        const int r = static_cast<int>(std::ceil(escape / map.res()));
        double best = 1e9;
        for (int j = sj - r; j <= sj + r; ++j)
            for (int i = si - r; i <= si + r; ++i) {
                if (i < 0 || j < 0 || i >= nx || j >= ny || map.blocked(i, j)) continue;
                best = std::min(best, aximetry::distance(map.center(i, j), start));
            }
        if (best > escape) return {};
        escape_r = best + 0.5 * map.res() + 1e-6;
    }
    auto passable = [&](int i, int j) {
        return !map.blocked(i, j) || aximetry::distance(map.center(i, j), start) < escape_r;
    };
    // If the goal itself is blocked, aim for the nearest free cell within 3 m.
    if (!passable(gi, gj)) {
        double best = 1e9;
        int bi = -1, bj = -1;
        const int r = static_cast<int>(3.0 / map.res());
        for (int j = gj - r; j <= gj + r; ++j)
            for (int i = gi - r; i <= gi + r; ++i) {
                if (i < 0 || j < 0 || i >= nx || j >= ny || !passable(i, j)) continue;
                const double d = std::hypot(i - gi, j - gj);
                if (d < best) { best = d; bi = i; bj = j; }
            }
        if (bi < 0) return {};
        gi = bi;
        gj = bj;
    }

    const int N = nx * ny;
    std::vector<float> g(N, std::numeric_limits<float>::infinity());
    std::vector<int> parent(N, -1);
    std::vector<uint8_t> closed(N, 0);
    using Item = std::pair<float, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
    const int s = sj * nx + si, goal = gj * nx + gi;
    g[s] = 0.0f;
    auto h = [&](int i, int j) { return static_cast<float>(std::hypot(i - gi, j - gj) * map.res()); };
    open.push({h(si, sj), s});
    static const int di[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const int dj[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    static const float dl[8] = {1, 1, 1, 1, 1.41421356f, 1.41421356f, 1.41421356f, 1.41421356f};
    while (!open.empty()) {
        const int cur = open.top().second;
        open.pop();
        if (closed[cur]) continue;
        closed[cur] = 1;
        if (cur == goal) break;
        const int ci = cur % nx, cj = cur / nx;
        for (int k = 0; k < 8; ++k) {
            const int ni = ci + di[k], nj = cj + dj[k];
            if (ni < 0 || nj < 0 || ni >= nx || nj >= ny) continue;
            const int nk = nj * nx + ni;
            if (closed[nk] || !passable(ni, nj)) continue;
            // No corner cutting: a diagonal step needs both orthogonal neighbours free.
            if (k >= 4 && (!passable(ci + di[k], cj) || !passable(ci, cj + dj[k]))) continue;
            const float step = dl[k] * static_cast<float>(map.res()) * 0.5f *
                               static_cast<float>(map.cost(ci, cj) + map.cost(ni, nj));
            const float ng = g[cur] + step;
            if (ng < g[nk]) {
                g[nk] = ng;
                parent[nk] = cur;
                open.push({ng + h(ni, nj), nk});
            }
        }
    }
    if (!closed[goal]) return {};
    if (cost_out) *cost_out = g[goal];

    std::vector<Vec2> cells;
    for (int c = goal; c != -1; c = parent[c]) cells.push_back(map.center(c % nx, c / nx));
    std::reverse(cells.begin(), cells.end());
    cells.front() = start;

    // Line-of-sight shortcutting over cheap, unblocked ground.
    auto clear = [&](const Vec2& a, const Vec2& b) {
        const double L = aximetry::distance(a, b);
        const int n = std::max(1, static_cast<int>(L / (0.5 * map.res())));
        for (int k = 0; k <= n; ++k) {
            const Vec2 p = a + (b - a) * (static_cast<double>(k) / n);
            int i, j;
            if (!map.to_cell(p, i, j) || !passable(i, j) || map.cost(i, j) > 3.0) return false;
        }
        return true;
    };
    std::vector<Vec2> out{cells.front()};
    size_t anchor = 0;
    while (anchor < cells.size() - 1) {
        size_t next = anchor + 1;
        for (size_t k = cells.size() - 1; k > anchor + 1; --k)
            if (k - anchor < 60 && clear(cells[anchor], cells[k])) { next = k; break; }
        out.push_back(cells[next]);
        anchor = next;
    }
    return out;
}

}  // namespace refctl
