// Aximetry test suite: physics sanity, model-vs-closed-form checks, planner and
// estimator unit tests, and end-to-end mission runs. No framework needed.
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

#include "../controllers/reference/ekf.hpp"
#include "../controllers/reference/planner.hpp"
#include "aximetry/design_check.hpp"
#include "aximetry/json.hpp"
#include "aximetry/sim.hpp"

using namespace aximetry;

namespace {
int g_fail = 0, g_checks = 0;
#define CHECK(cond)                                                                   \
    do {                                                                              \
        ++g_checks;                                                                   \
        if (!(cond)) {                                                                \
            ++g_fail;                                                                 \
            std::printf("    FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);           \
        }                                                                             \
    } while (0)
#define CHECK_NEAR(a, b, tol)                                                                     \
    do {                                                                                          \
        ++g_checks;                                                                               \
        const double _a = (a), _b = (b);                                                          \
        if (!(std::fabs(_a - _b) <= (tol))) {                                                     \
            ++g_fail;                                                                             \
            std::printf("    FAIL %s:%d: %s = %g, expected %g +/- %g\n", __FILE__, __LINE__, #a, _a, _b, \
                        (double)(tol));                                                           \
        }                                                                                         \
    } while (0)

std::string path(const std::string& rel) { return std::string(AXIMETRY_SOURCE_DIR) + "/" + rel; }
RoverModel mer() { return build_rover_model(load_rover_design(path("rovers/mer_class.json"))); }

Mission flat_mission(double incline_deg, const std::string& soil, double seconds) {
    Mission m;
    m.seed = 7;
    m.terrain.size_m = 60.0;
    m.terrain.relief_m = 0.0;
    m.terrain.random_craters = 0;
    m.terrain.incline_deg = incline_deg;
    m.terrain.base_soil = soil;
    m.terrain.drift_fraction = 0.0;
    m.terrain.crust_fraction = 0.0;
    m.terrain.outcrop_slope_deg = 89.0;
    m.terrain.rock_density = 0.0;
    m.terrain.science_targets = 0;
    m.start = Vec2(10.0, 30.0);
    m.start_heading_deg = 0.0;
    m.start_local_hour = 12.0;
    m.start_soc = 1.0;
    m.duration_sols = seconds / mars::kSolSeconds;
    return m;
}

struct Constant : Controller {
    double w;
    explicit Constant(double w_) : w(w_) {}
    void on_start(const MissionBrief&) override {}
    Command on_step(const Observation&) override {
        Command c;
        c.wheel_speed_left = c.wheel_speed_right = w;
        return c;
    }
};

struct Test {
    const char* name;
    std::function<void()> fn;
};
}  // namespace

static std::vector<Test> tests = {
    {"json: comments, trailing commas, round trip", [] {
         auto v = json::parse("// hello\n{ \"a\": 1.5, /* c */ \"b\": [1, 2, 3,], \"s\": \"x\\ny\", }");
         CHECK_NEAR(v.at("a").as_number(), 1.5, 1e-12);
         CHECK(v.at("b").size() == 3);
         CHECK(v.at("s").as_string() == "x\ny");
         auto w = json::parse(json::dump(v));
         CHECK(json::dump(w) == json::dump(v));
         bool threw = false;
         try { json::parse("{\"a\": }"); } catch (const std::exception&) { threw = true; }
         CHECK(threw);
     }},
    {"terramechanics: Bekker closed form and monotonicity", [] {
         const Soil s = soil_by_id("m90_simulant");
         WheelGeometry w{0.13, 0.16, 0.0};
         const double W = 100.0;
         const double expected = std::pow(3 * W / ((3 - s.n) * (s.kc + w.width * s.kphi) * std::sqrt(0.26)), 2 / (2 * s.n + 1));
         CHECK_NEAR(bekker_static_sinkage(s, W, w), expected, 1e-12);
         CHECK(bekker_static_sinkage(s, 200, w) > bekker_static_sinkage(s, 100, w));  // heavier sinks more
         WheelGeometry wide{0.13, 0.25, 0.0};
         CHECK(bekker_static_sinkage(s, 100, wide) < bekker_static_sinkage(s, 100, w));  // wider sinks less
         const Soil drift = soil_by_id("loose_drift");
         CHECK(bekker_static_sinkage(drift, 100, w) > bekker_static_sinkage(s, 100, w));  // weak soil sinks more
     }},
    {"terramechanics: Janosi thrust shape", [] {
         CHECK_NEAR(janosi_thrust(100, 0.1, 0.02, 0.0), 0.0, 1e-12);
         double prev = 0.0;
         for (double sl = 0.05; sl <= 1.0; sl += 0.05) {
             const double h = janosi_thrust(100, 0.1, 0.02, sl);
             CHECK(h > prev);
             CHECK(h < 100.0);
             prev = h;
         }
         CHECK_NEAR(janosi_thrust(100, 0.1, 0.02, -0.3), -janosi_thrust(100, 0.1, 0.02, 0.3), 1e-12);
         // Series branch matches the exact branch at the switch-over point.
         const double x = 1e-4, a = x * 0.02 / 0.1;
         CHECK_NEAR(janosi_thrust(1, 0.1, 0.02, a * 0.999), janosi_thrust(1, 0.1, 0.02, a * 1.001), 1e-6);
     }},
    {"environment: sun, storm, temperature", [] {
         EnvironmentConfig cfg;
         Environment env(cfg, 0.0);  // mission starts at midnight
         const double noon = 12.0 / 24.0 * mars::kSolSeconds;
         CHECK(env.irradiance(0.0) == 0.0);
         CHECK(env.irradiance(noon) > 300.0);
         CHECK(env.ambient_temp_c(14.0 / 24.0 * mars::kSolSeconds) > env.ambient_temp_c(4.0 / 24.0 * mars::kSolSeconds));
         cfg.storms.push_back({0.0, 0.1, 2.0, 5.0});
         Environment storm(cfg, 0.0);
         CHECK(storm.irradiance(noon) < 0.4 * env.irradiance(noon));
         CHECK(storm.optical_depth(noon) > 4.9);
     }},
    {"design check: MER-class passes, cardboard fails", [] {
         DesignReport ok = analyze_design(mer(), EnvironmentConfig{});
         CHECK(ok.fails == 0);
         CHECK(ok.model.mass > 100 && ok.model.mass < 250);
         DesignReport paper = analyze_design(build_rover_model(load_rover_design(path("rovers/paper_rover.json"))), EnvironmentConfig{});
         CHECK(paper.fails > 0);
     }},
    {"steady-state slip rises with slope", [] {
         RoverModel m = mer();
         const Soil s = soil_by_id("m90_simulant");
         const double W = m.mass * mars::kGravity;
         double prev = -1.0;
         for (double a = 0; a <= 15; a += 5) {
             const double th = deg2rad(a);
             const double sl = steady_state_slip(m, s, W * std::cos(th) / 6, W * std::sin(th) / 6);
             CHECK(sl > prev);
             prev = sl;
         }
     }},
    {"dynamics agree with the steady-state model (slip on a 10 deg incline)", [] {
         RoverModel m = mer();
         const double w = 0.7 * m.motor.no_load_speed;
         Mission ms = flat_mission(10.0, "m90_simulant", 200.0);
         Constant ctl(w);
         SimOptions o;
         o.trace_interval_s = 1.0;
         Simulator sim(m, ms, ctl, "constant", o);
         RunResult r = sim.run();
         CHECK(!r.failed);
         const auto& a = r.trace[r.trace.size() / 2];
         const auto& b = r.trace.back();
         const double along = std::hypot(b.x - a.x, b.y - a.y) / std::cos(deg2rad(10.0));
         const double sim_slip = 1.0 - along / (w * m.wheel_radius() * (b.t - a.t));
         const double W = m.mass * mars::kGravity;
         const double pred = steady_state_slip(m, soil_by_id("m90_simulant"), W * std::cos(deg2rad(10.0)) / 6,
                                               W * std::sin(deg2rad(10.0)) / 6);
         std::printf("      simulated slip %.3f vs steady-state %.3f\n", sim_slip, pred);
         CHECK_NEAR(sim_slip, pred, 0.06);
     }},
    {"parked rover holds still overnight on a slope", [] {
         RoverModel m = mer();
         Mission ms = flat_mission(12.0, "m90_simulant", 40000.0);
         Constant ctl(0.0);
         Simulator sim(m, ms, ctl, "parked");
         RunResult r = sim.run();
         CHECK(r.distance_m < 0.01);
     }},
    {"tip-over is detected on a slope beyond the tip angle", [] {
         RoverModel m = mer();
         Mission ms = flat_mission(rad2deg(m.tip_angle_longitudinal) + 3.0, "bedrock", 30.0);
         Constant ctl(0.0);
         Simulator sim(m, ms, ctl, "tip");
         RunResult r = sim.run();
         CHECK(r.failed && r.failure_type == "tip_over");
     }},
    {"determinism: same seed, same result", [] {
         RoverModel m = mer();
         Mission ms = load_mission(path("missions/01_first_drive.json"));
         SimOptions o;
         o.record_trace = false;
         auto c1 = ControllerRegistry::instance().create("reference");
         auto c2 = ControllerRegistry::instance().create("reference");
         RunResult a = Simulator(m, ms, *c1, "reference", o).run();
         RunResult b = Simulator(m, ms, *c2, "reference", o).run();
         CHECK(a.score == b.score);
         CHECK(a.distance_m == b.distance_m);
     }},
    {"EKF converges onto landmarks", [] {
         refctl::PoseEKF ekf;
         ekf.init(Vec2(2.0, -1.5), 0.2, 3.0, 0.3);  // start badly off
         const Vec2 truth(0, 0);
         const double th = 0.0;
         std::vector<Vec2> lms = {{8, 3}, {6, -5}, {12, 1}};
         Rng rng(3);
         for (int k = 0; k < 40; ++k) {
             ekf.update_heading(th + rng.normal(0, 0.02), 0.03);
             for (const auto& lm : lms) {
                 const Vec2 d = lm - truth;
                 ekf.update_landmark(lm, d.norm() + rng.normal(0, 0.05), wrap_angle(d.angle() - th) + rng.normal(0, 0.01), 0.1, 0.02);
             }
         }
         CHECK(distance(ekf.pos(), truth) < 0.15);
         CHECK(ekf.pos_sigma() < 0.2);
     }},
    {"A* routes around obstacles and reports enclosure", [] {
         MissionBrief b;
         b.map.size_m = 40;
         b.map.resolution_m = 2.0;
         b.map.nx = b.map.ny = 21;
         b.map.height.assign(21 * 21, 0.0f);
         b.map.soil_guess.assign(21 * 21, 0);
         b.map.soil_ids = {"m90_simulant"};
         refctl::PlannerParams p;
         p.inflate = 0.5;
         refctl::CostMap map;
         map.build(b, p);
         for (double y = 6; y <= 34; y += 0.5) map.add_obstacle(Vec2(20, y), 0.2);  // a wall with gaps at the ends
         auto path = refctl::plan_path(map, Vec2(10, 20), Vec2(30, 20), 2.0);
         CHECK(!path.empty());
         for (size_t k = 1; k < path.size(); ++k) {
             const Vec2 a = path[k - 1], c = path[k];
             for (int s = 0; s <= 20; ++s) CHECK(!map.blocked_at(a + (c - a) * (s / 20.0)) || distance(a, Vec2(10, 20)) < 1.0);
         }
         CHECK(refctl::path_length(path) > 20.0);
         for (double a = 0; a < 6.3; a += 0.05) map.add_obstacle(Vec2(30, 20) + Vec2::from_angle(a) * 4.0, 0.2);  // ring
         CHECK(refctl::plan_path(map, Vec2(10, 20), Vec2(30, 20), 2.0).empty());
     }},
    {"end to end: reference controller completes First Drive", [] {
         RoverModel m = mer();
         Mission ms = load_mission(path("missions/01_first_drive.json"));
         auto c = ControllerRegistry::instance().create("reference");
         SimOptions o;
         o.record_trace = false;
         RunResult r = Simulator(m, ms, *c, "reference", o).run();
         std::printf("      score %.1f, %d targets, goal %s\n", r.score, r.targets_received, r.goal_reached ? "yes" : "no");
         CHECK(!r.failed);
         CHECK(r.goal_reached);
         CHECK(r.targets_received >= 3);
     }},
    {"end to end: a lead-acid rover freezes in the dust storm", [] {
         RoverModel m = build_rover_model(load_rover_design(path("rovers/lead_acid_budget.json")));
         Mission ms = load_mission(path("missions/04_dust_storm.json"));
         auto c = ControllerRegistry::instance().create("reference");
         SimOptions o;
         o.record_trace = false;
         RunResult r = Simulator(m, ms, *c, "reference", o).run();
         CHECK(r.failed && r.failure_type == "battery_frozen");
     }},
};

int main() {
    std::printf("Aximetry tests\n");
    for (const auto& t : tests) {
        const int before = g_fail;
        std::printf("  %-70s", t.name);
        std::fflush(stdout);
        std::printf("\n");
        t.fn();
        std::printf("    %s\n", g_fail == before ? "ok" : "FAILED");
    }
    std::printf("%d checks, %d failures\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
