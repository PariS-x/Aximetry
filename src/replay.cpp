// Writes a compact JSON replay consumed by tools/viewer.html.
#include <cmath>
#include <ostream>

#include "aximetry/sim.hpp"

namespace aximetry {

namespace {
using json::escape_string;
using json::number_to_string;

std::string n(double v, int prec = 4) { return number_to_string(v, prec); }

template <typename F>
void column(std::ostream& os, const char* name, const std::vector<TraceSample>& tr, F get, int prec, bool last = false) {
    os << "\"" << name << "\":[";
    for (size_t i = 0; i < tr.size(); ++i) {
        if (i) os << ',';
        os << n(get(tr[i]), prec);
    }
    os << "]" << (last ? "" : ",");
}
}  // namespace

void write_replay_json(const Simulator& sim, const RunResult& r, std::ostream& os) {
    const Terrain& T = sim.terrain();
    const RoverModel& rv = sim.rover();
    const Mission& ms = sim.mission();
    const MissionBrief& b = sim.brief();

    os << "{\"version\":1,";
    os << "\"mission\":{\"name\":\"" << escape_string(ms.name) << "\",\"description\":\"" << escape_string(ms.description)
       << "\",\"duration_s\":" << n(b.duration_s, 8) << ",\"start_hour\":" << n(ms.start_local_hour)
       << ",\"sol_seconds\":" << n(mars::kSolSeconds, 9) << ",\"owlt_s\":" << n(b.owlt_s) << ",\"goal\":";
    if (ms.has_goal) os << "{\"x\":" << n(ms.goal.x) << ",\"y\":" << n(ms.goal.y) << ",\"r\":" << n(ms.goal_radius) << "}";
    else os << "null";
    os << ",\"start\":{\"x\":" << n(ms.start.x) << ",\"y\":" << n(ms.start.y) << "}},";

    os << "\"rover\":{\"name\":\"" << escape_string(rv.design.name) << "\",\"mass\":" << n(rv.mass)
       << ",\"track\":" << n(rv.design.track_width) << ",\"wheelbase\":" << n(rv.design.wheelbase)
       << ",\"wheel_d\":" << n(rv.design.wheel_diameter) << ",\"wheel_w\":" << n(rv.design.wheel_width)
       << ",\"wheels\":" << rv.design.wheel_count << ",\"battery_wh\":" << n(rv.design.battery_wh)
       << ",\"wheel_material\":\"" << escape_string(rv.design.wheel_material) << "\",\"chassis_material\":\""
       << escape_string(rv.design.chassis_material) << "\",\"suspension_material\":\"" << escape_string(rv.design.suspension_material)
       << "\",\"battery\":\"" << escape_string(rv.design.battery) << "\",\"rhu\":" << rv.design.rhu_count << "},";
    os << "\"controller\":\"" << escape_string(r.controller_name) << "\",";

    // Terrain (downsampled to at most ~400 cells per side), heights in cm.
    const int step = std::max(1, (T.nx() + 399) / 400);
    const int nx = (T.nx() - 1) / step + 1, ny = (T.ny() - 1) / step + 1;
    double hmin = 1e9, hmax = -1e9;
    for (int j = 0; j < T.ny(); j += step)
        for (int i = 0; i < T.nx(); i += step) {
            hmin = std::min(hmin, T.height_cell(i, j));
            hmax = std::max(hmax, T.height_cell(i, j));
        }
    os << "\"terrain\":{\"size\":" << n(T.size()) << ",\"res\":" << n(T.resolution() * step) << ",\"nx\":" << nx << ",\"ny\":" << ny
       << ",\"hmin\":" << n(hmin) << ",\"hmax\":" << n(hmax) << ",\"h\":[";
    bool first = true;
    for (int j = 0; j < T.ny(); j += step)
        for (int i = 0; i < T.nx(); i += step) {
            if (!first) os << ',';
            first = false;
            os << static_cast<long>(std::lround(T.height_cell(i, j) * 100.0));
        }
    os << "],\"soil\":[";
    first = true;
    for (int j = 0; j < T.ny(); j += step)
        for (int i = 0; i < T.nx(); i += step) {
            if (!first) os << ',';
            first = false;
            os << T.soil_cell(i, j);
        }
    os << "],\"soils\":[";
    for (size_t k = 0; k < T.soils().size(); ++k) {
        if (k) os << ',';
        os << "{\"id\":\"" << T.soils()[k].id << "\",\"name\":\"" << escape_string(T.soils()[k].name) << "\"}";
    }
    os << "]},";

    os << "\"rocks\":[";
    for (size_t k = 0; k < T.rocks().size(); ++k) {
        const Rock& rk = T.rocks()[k];
        if (k) os << ',';
        os << '[' << n(rk.pos.x, 5) << ',' << n(rk.pos.y, 5) << ',' << n(rk.radius, 3) << ',' << n(rk.height, 3) << ','
           << (rk.height > rv.climb_limit ? 1 : 0) << ',' << (rk.catalogued ? 1 : 0) << ']';
    }
    os << "],";

    os << "\"targets\":[";
    for (size_t k = 0; k < T.targets().size(); ++k) {
        const ScienceTarget& t = T.targets()[k];
        double t_meas = -1, t_down = -1;
        bool received = false;
        for (const auto& rec : r.science)
            if (rec.target_id == t.id) {
                t_meas = rec.t_measured;
                t_down = rec.t_downlinked;
                received = rec.received;
            }
        if (k) os << ',';
        os << "{\"id\":" << t.id << ",\"x\":" << n(t.pos.x, 5) << ",\"y\":" << n(t.pos.y, 5) << ",\"kind\":\"" << t.true_kind
           << "\",\"orbital\":\"" << t.orbital_kind << "\",\"value\":" << n(t.true_value, 3) << ",\"prior\":" << n(t.orbital_prior, 3)
           << ",\"t_measured\":" << n(t_meas, 7) << ",\"t_downlinked\":" << n(t_down, 7) << ",\"received\":" << (received ? "true" : "false")
           << "}";
    }
    os << "],";

    os << "\"relays\":[";
    for (size_t k = 0; k < b.relay_passes.size(); ++k) {
        if (k) os << ',';
        os << '[' << n(b.relay_passes[k].start_s, 8) << ',' << n(b.relay_passes[k].duration_s, 6) << ']';
    }
    os << "],";

    const auto& tr = r.trace;
    os << "\"trace\":{";
    column(os, "t", tr, [](const TraceSample& s) { return s.t; }, 8);
    column(os, "x", tr, [](const TraceSample& s) { return s.x; }, 6);
    column(os, "y", tr, [](const TraceSample& s) { return s.y; }, 6);
    column(os, "h", tr, [](const TraceSample& s) { return s.heading; }, 4);
    column(os, "ex", tr, [](const TraceSample& s) { return s.has_est ? s.est_x : NAN; }, 6);
    column(os, "ey", tr, [](const TraceSample& s) { return s.has_est ? s.est_y : NAN; }, 6);
    column(os, "eh", tr, [](const TraceSample& s) { return s.has_est ? s.est_heading : NAN; }, 4);
    column(os, "es", tr, [](const TraceSample& s) { return s.has_est ? s.est_sigma : NAN; }, 3);
    column(os, "v", tr, [](const TraceSample& s) { return s.v * 100.0; }, 3);
    column(os, "soc", tr, [](const TraceSample& s) { return s.soc * 100.0; }, 4);
    column(os, "solar", tr, [](const TraceSample& s) { return s.solar_w; }, 4);
    column(os, "load", tr, [](const TraceSample& s) { return s.load_w; }, 4);
    column(os, "box", tr, [](const TraceSample& s) { return s.box_c; }, 3);
    column(os, "amb", tr, [](const TraceSample& s) { return s.ambient_c; }, 3);
    column(os, "tau", tr, [](const TraceSample& s) { return s.tau; }, 3);
    column(os, "slip", tr, [](const TraceSample& s) { return 50.0 * (s.slip_l + s.slip_r); }, 3);
    column(os, "sink", tr, [](const TraceSample& s) { return s.sinkage_cm; }, 3);
    column(os, "pitch", tr, [](const TraceSample& s) { return s.pitch_deg; }, 3);
    column(os, "roll", tr, [](const TraceSample& s) { return s.roll_deg; }, 3);
    column(os, "mode", tr, [](const TraceSample& s) { return static_cast<double>(s.mode); }, 1);
    os << "\"status\":[";
    for (size_t i = 0; i < tr.size(); ++i) {
        if (i) os << ',';
        os << '"' << escape_string(tr[i].status) << '"';
    }
    os << "]},";

    os << "\"paths\":[";
    for (size_t k = 0; k < r.paths.size(); ++k) {
        if (k) os << ',';
        os << "{\"t\":" << n(r.paths[k].t, 8) << ",\"p\":[";
        for (size_t i = 0; i < r.paths[k].pts.size(); ++i) {
            if (i) os << ',';
            os << n(r.paths[k].pts[i].x, 5) << ',' << n(r.paths[k].pts[i].y, 5);
        }
        os << "]}";
    }
    os << "],";

    os << "\"events\":[";
    for (size_t k = 0; k < r.events.size(); ++k) {
        if (k) os << ',';
        os << "{\"t\":" << n(r.events[k].t, 8) << ",\"type\":\"" << escape_string(r.events[k].type) << "\",\"msg\":\""
           << escape_string(r.events[k].message) << "\"}";
    }
    os << "],";

    os << "\"result\":{\"score\":" << n(r.score, 6) << ",\"science\":" << n(r.science_points, 6) << ",\"goal\":" << n(r.goal_points)
       << ",\"goal_reached\":" << (r.goal_reached ? "true" : "false") << ",\"measured\":" << r.targets_measured
       << ",\"received\":" << r.targets_received << ",\"distance\":" << n(r.distance_m, 6) << ",\"failed\":"
       << (r.failed ? "true" : "false") << ",\"failure_type\":\"" << escape_string(r.failure_type) << "\",\"failure\":\""
       << escape_string(r.failure_message) << "\",\"lesson\":\""
       << escape_string(r.lesson) << "\",\"max_slip\":" << n(r.max_slip) << ",\"max_sinkage_cm\":" << n(r.max_sinkage_cm)
       << ",\"collisions\":" << r.collisions << ",\"rock_strikes\":" << r.rock_strikes << ",\"wheels_failed\":" << r.wheels_failed
       << ",\"drive_wh\":" << n(r.drive_energy_wh) << ",\"heater_wh\":" << n(r.heater_energy_wh) << ",\"solar_wh\":"
       << n(r.solar_energy_wh) << "},";

    os << "\"log\":[";
    const size_t nlog = std::min<size_t>(r.log.size(), 800);
    for (size_t k = 0; k < nlog; ++k) {
        if (k) os << ',';
        os << '"' << escape_string(r.log[k]) << '"';
    }
    os << "]}";
}

}  // namespace aximetry
