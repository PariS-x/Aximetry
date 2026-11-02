#include "aximetry/mission.hpp"

#include <stdexcept>

namespace aximetry {

double Mission::duration_s() const { return duration_sols * mars::kSolSeconds; }

Mission mission_from_json(const json::Value& v) {
    Mission m;
    m.name = v.str("name", m.name);
    m.description = v.str("description", m.description);
    m.seed = static_cast<uint64_t>(v.num("seed", 1));
    m.duration_sols = v.num("duration_sols", m.duration_sols);

    if (v.has("terrain")) {
        const auto& t = v.at("terrain");
        auto& c = m.terrain;
        c.size_m = t.num("size_m", c.size_m);
        c.resolution_m = t.num("resolution_m", c.resolution_m);
        c.relief_m = t.num("relief_m", c.relief_m);
        c.feature_m = t.num("feature_m", c.feature_m);
        c.incline_deg = t.num("incline_deg", c.incline_deg);
        c.random_craters = t.integer("random_craters", c.random_craters);
        c.base_soil = t.str("base_soil", c.base_soil);
        c.drift_fraction = t.num("drift_fraction", c.drift_fraction);
        c.crust_fraction = t.num("crust_fraction", c.crust_fraction);
        c.outcrop_slope_deg = t.num("outcrop_slope_deg", c.outcrop_slope_deg);
        c.rock_density = t.num("rock_density_per_m2", c.rock_density);
        c.boulder_fraction = t.num("boulder_fraction", c.boulder_fraction);
        if (t.has("craters"))
            for (const auto& cr : t.at("craters").as_array())
                c.craters.push_back({cr.num("x", 0), cr.num("y", 0), cr.num("radius", 10), cr.num("depth", 2)});
        if (t.has("soil_patches"))
            for (const auto& p : t.at("soil_patches").as_array())
                c.patches.push_back({p.num("x", 0), p.num("y", 0), p.num("radius", 5), p.str("soil", "loose_drift")});
    }
    if (v.has("science")) {
        const auto& s = v.at("science");
        m.terrain.science_targets = s.integer("random_targets", m.terrain.science_targets);
        m.terrain.orbital_misclassification = s.num("orbital_misclassification", m.terrain.orbital_misclassification);
        m.points_per_value = s.num("points_per_value", m.points_per_value);
        m.product_mbit = s.num("product_mbit", m.product_mbit);
        m.sample_duration_s = s.num("sample_duration_s", m.sample_duration_s);
        m.sample_range_m = s.num("sample_range_m", m.sample_range_m);
        if (s.has("fixed_targets"))
            for (const auto& f : s.at("fixed_targets").as_array())
                m.terrain.fixed_targets.push_back({f.num("x", 0), f.num("y", 0), f.str("kind", "basalt")});
    }
    if (v.has("environment")) {
        const auto& e = v.at("environment");
        auto& c = m.environment;
        c.latitude_deg = e.num("latitude_deg", c.latitude_deg);
        c.ls_deg = e.num("ls_deg", c.ls_deg);
        c.tau = e.num("tau", c.tau);
        c.temp_mean_c = e.num("temp_mean_c", c.temp_mean_c);
        c.temp_amplitude_c = e.num("temp_amplitude_c", c.temp_amplitude_c);
        c.panel_dust_rate_per_sol = e.num("panel_dust_rate_per_sol", c.panel_dust_rate_per_sol);
        if (e.has("dust_storms"))
            for (const auto& s : e.at("dust_storms").as_array())
                c.storms.push_back({s.num("start_sol", 1), s.num("ramp_sols", 0.5), s.num("duration_sols", 1), s.num("peak_tau", 4)});
    }
    if (v.has("start")) {
        const auto& s = v.at("start");
        m.start = Vec2(s.num("x", m.start.x), s.num("y", m.start.y));
        m.start_heading_deg = s.num("heading_deg", m.start_heading_deg);
        m.start_local_hour = s.num("local_hour", m.start_local_hour);
        m.start_soc = s.num("battery_soc", m.start_soc);
        m.start_box_temp_c = s.num("box_temp_c", m.start_box_temp_c);
    }
    if (v.has("goal")) {
        const auto& g = v.at("goal");
        m.has_goal = true;
        m.goal = Vec2(g.num("x", 0), g.num("y", 0));
        m.goal_radius = g.num("radius", m.goal_radius);
        m.goal_bonus = g.num("bonus", m.goal_bonus);
    }
    if (v.has("comms")) {
        const auto& c = v.at("comms");
        m.owlt_min = c.num("owlt_min", m.owlt_min);
        m.storage_mbit = c.num("storage_mbit", m.storage_mbit);
        if (c.has("relay_passes"))
            for (const auto& r : c.at("relay_passes").as_array())
                m.relays.push_back({r.num("local_hour", 15), r.num("minutes", 10), r.num("rate_kbps", 1000)});
        if (c.has("uplinks"))
            for (const auto& u : c.at("uplinks").as_array())
                m.uplinks.push_back({u.num("send_at_s", 0), u.str("type", "note"), u.integer("target", -1), u.num("value", 0),
                                     u.str("text", "")});
    }
    m.terrain.keep_clear.push_back(m.start);
    if (m.has_goal) m.terrain.keep_clear.push_back(m.goal);
    if (m.duration_sols <= 0) throw std::runtime_error("mission duration_sols must be > 0");
    return m;
}

Mission load_mission(const std::string& path) { return mission_from_json(json::parse_file(path)); }

}  // namespace aximetry
