// aximetry — command-line front end.
//
//   aximetry catalog                         materials, motors, batteries, soils
//   aximetry controllers                     registered controllers
//   aximetry check   ROVER [--mission M]     engineering review of a design
//   aximetry compare ROVER...                side-by-side table of designs
//   aximetry run     --rover R --mission M --controller C [--out run.json] [--html replay.html]
//   aximetry experiment slip-slope --rover R [--soil ID] [--out file.csv]
//   aximetry experiment bench --rover R --mission M --controllers a,b --seeds N [--out file.csv]
#include <cstdio>
#include <chrono>
#include <cstring>
#include <thread>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <io.h>
#define AXIMETRY_ISATTY() _isatty(_fileno(stdout))
#else
#include <unistd.h>
#define AXIMETRY_ISATTY() isatty(fileno(stdout))
#endif

#include "aximetry/controller.hpp"
#include "aximetry/design_check.hpp"
#include "aximetry/gui_server.hpp"
#include "aximetry/mission.hpp"
#include "aximetry/rover.hpp"
#include "aximetry/sim.hpp"

#ifndef AXIMETRY_SOURCE_DIR
#define AXIMETRY_SOURCE_DIR "."
#endif

using namespace aximetry;

namespace {

struct Args {
    std::vector<std::string> positional;
    std::map<std::string, std::string> opts;
    bool flag(const std::string& k) const { return opts.count(k) > 0; }
    std::string get(const std::string& k, const std::string& def = "") const {
        auto it = opts.find(k);
        return it == opts.end() ? def : it->second;
    }
};

Args parse_args(int argc, char** argv, int start) {
    Args a;
    for (int i = start; i < argc; ++i) {
        std::string s = argv[i];
        if (s.rfind("--", 0) == 0) {
            const std::string key = s.substr(2);
            if (i + 1 < argc && std::strncmp(argv[i + 1], "--", 2) != 0) a.opts[key] = argv[++i];
            else a.opts[key] = "1";
        } else {
            a.positional.push_back(s);
        }
    }
    return a;
}

bool use_color(const Args& a) { return !a.flag("no-color") && AXIMETRY_ISATTY(); }

void usage() {
    std::puts(
        "Aximetry - Mars rover design and autonomy simulator\n"
        "\n"
        "  aximetry                               open the mission-control GUI in your browser\n"
        "  aximetry gui [--port N] [--no-browser] [--root PATH]\n"
        "  aximetry catalog                       list materials, motors, batteries and soils\n"
        "  aximetry controllers                   list compiled-in controllers\n"
        "  aximetry check ROVER.json [--mission M.json] [--json]\n"
        "                                         engineering review: will this design work on Mars?\n"
        "  aximetry compare ROVER.json ...        compare several designs side by side\n"
        "  aximetry run --rover R.json --mission M.json --controller NAME\n"
        "               [--out run.json] [--html replay.html] [--seed N] [--verbose]\n"
        "                                         simulate a mission and score it\n"
        "  aximetry experiment slip-slope --rover R.json [--soil ID] [--out file.csv]\n"
        "                                         slip vs slope: analytic model vs full dynamic simulation\n"
        "  aximetry experiment bench --rover R.json --mission M.json --controllers a,b [--seeds N] [--out file.csv]\n"
        "                                         score controllers over many randomised terrains\n");
}

// Project folder (holding tools/, rovers/, missions/): the source tree this was
// built from, else the current folder, else next to / above the executable.
std::string find_root(const char* argv0, const Args& a) {
    namespace fs = std::filesystem;
    std::vector<fs::path> c;
    if (a.flag("root")) c.push_back(a.get("root"));
    c.push_back(AXIMETRY_SOURCE_DIR);
    c.push_back(fs::current_path());
    c.push_back(fs::current_path().parent_path());
    std::error_code ec;
    const fs::path exe = fs::absolute(argv0, ec).parent_path();
    for (fs::path p = exe; !p.empty() && c.size() < 10; p = p.parent_path()) {
        c.push_back(p);
        if (p == p.parent_path()) break;
    }
    for (const auto& p : c)
        if (fs::exists(p / "tools" / "gui.html", ec)) return p.string();
    return AXIMETRY_SOURCE_DIR;
}

int cmd_gui(const char* argv0, const Args& a) {
    GuiOptions o;
    o.root = find_root(argv0, a);
    o.port = std::stoi(a.get("port", "8080"));
    o.open_browser = !a.flag("no-browser");
#ifdef AXIMETRY_GUI_COMPILER
    o.compiler = AXIMETRY_GUI_COMPILER;
    o.compiler_id = AXIMETRY_GUI_COMPILER_ID;
    o.cli_lib = AXIMETRY_GUI_CLI_LIB;
    o.core_lib = AXIMETRY_GUI_CORE_LIB;
    o.build_dir = AXIMETRY_GUI_BUILD_DIR;
#endif
    return run_gui_server(o);
}

int cmd_catalog() {
    std::puts("MATERIALS (id: density kg/m3 | yield MPa | E GPa | brittle below C)");
    for (const auto& m : material_catalog())
        std::printf("  %-8s %-40s %6.0f | %5.0f | %6.1f | %s\n             %s\n", m.id.c_str(), m.name.c_str(), m.density,
                    m.yield_strength / 1e6, m.youngs_modulus / 1e9,
                    m.brittle_below_c < -200 ? "never" : (std::to_string(static_cast<int>(m.brittle_below_c))).c_str(),
                    m.note.c_str());
    std::puts("\nMOTORS (id: stall torque at wheel N m | no-load speed rad/s | mass kg)");
    for (const auto& m : motor_catalog())
        std::printf("  %-8s %-30s %6.1f | %4.2f | %4.2f\n", m.id.c_str(), m.name.c_str(), m.stall_torque, m.no_load_speed, m.mass);
    std::puts("\nBATTERIES (id: Wh/kg | min discharge C | min charge C | survival C)");
    for (const auto& b : battery_catalog())
        std::printf("  %-9s %-46s %4.0f | %4.0f | %4.0f | %4.0f\n             %s\n", b.id.c_str(), b.name.c_str(), b.specific_energy,
                    b.min_discharge_c, b.min_charge_c, b.survival_c, b.note.c_str());
    std::puts("\nSOILS (id: n | kc | kphi | c Pa | phi deg | K m)");
    for (const auto& s : soil_catalog())
        std::printf("  %-13s %-38s %4.2f | %9.3g | %9.3g | %6.0f | %4.1f | %.4f\n                %s\n", s.id.c_str(), s.name.c_str(),
                    s.n, s.kc, s.kphi, s.cohesion, rad2deg(s.phi), s.K, s.source.c_str());
    return 0;
}

int cmd_controllers() {
    for (const auto& e : ControllerRegistry::instance().entries())
        std::printf("  %-16s %s\n", e.name.c_str(), e.description.c_str());
    return 0;
}

EnvironmentConfig env_from_args(const Args& a) {
    if (a.flag("mission")) return load_mission(a.get("mission")).environment;
    return EnvironmentConfig{};
}

int cmd_check(const Args& a) {
    if (a.positional.empty()) { usage(); return 2; }
    int worst = 0;
    for (const auto& path : a.positional) {
        RoverModel m = build_rover_model(load_rover_design(path));
        DesignReport r = analyze_design(m, env_from_args(a));
        if (a.flag("json")) std::cout << json::dump(report_to_json(r)) << "\n";
        else std::cout << format_report(r, use_color(a)) << "\n";
        if (r.fails) worst = 1;
    }
    return worst;
}

int cmd_compare(const Args& a) {
    if (a.positional.empty()) { usage(); return 2; }
    const EnvironmentConfig env = env_from_args(a);
    std::printf("%-30s %8s %6s %6s %10s %10s %9s %9s %8s\n", "design", "mass kg", "fails", "warns", "slope m90", "slope drift",
                "solar Wh", "night Wh", "tip deg");
    for (const auto& path : a.positional) {
        RoverModel m = build_rover_model(load_rover_design(path));
        DesignReport r = analyze_design(m, env);
        double m90 = 0, drift = 0;
        for (const auto& s : r.soils) {
            if (s.soil_id == "m90_simulant") m90 = s.max_slope_deg;
            if (s.soil_id == "loose_drift") drift = s.max_slope_deg;
        }
        std::printf("%-30s %8.1f %6d %6d %10.1f %10.1f %9.0f %9.0f %8.0f\n", m.design.name.substr(0, 30).c_str(), m.mass, r.fails,
                    r.warns, m90, drift, r.solar_wh_per_sol, r.night_energy_wh,
                    rad2deg(std::min(m.tip_angle_lateral, m.tip_angle_longitudinal)));
    }
    return 0;
}

std::string load_viewer(const Args& a) {
    std::vector<std::string> candidates;
    if (a.flag("viewer")) candidates.push_back(a.get("viewer"));
    candidates.push_back(std::string(AXIMETRY_SOURCE_DIR) + "/tools/viewer.html");
    candidates.push_back("tools/viewer.html");
    candidates.push_back("../tools/viewer.html");
    for (const auto& c : candidates) {
        std::ifstream in(c, std::ios::binary);
        if (in) {
            std::stringstream ss;
            ss << in.rdbuf();
            return ss.str();
        }
    }
    throw std::runtime_error("could not find tools/viewer.html (pass --viewer PATH)");
}

int cmd_run(const Args& a) {
    if (!a.flag("rover") || !a.flag("mission")) { usage(); return 2; }
    RoverModel rover = build_rover_model(load_rover_design(a.get("rover")));
    Mission mission = load_mission(a.get("mission"));
    if (a.flag("seed")) mission.seed = std::stoull(a.get("seed"));
    const std::string cname = a.get("controller", "reference");
    auto controller = ControllerRegistry::instance().create(cname);
    SimOptions opt;
    opt.verbose = a.flag("verbose");
    if (a.flag("time-limit")) {
        // Watchdog for untrusted controller code (e.g. an infinite loop in on_step).
        const int limit = std::stoi(a.get("time-limit"));
        std::thread([limit] {
            std::this_thread::sleep_for(std::chrono::seconds(limit));
            std::fprintf(stderr, "error: run exceeded %d s wall-clock limit (infinite loop in the controller?)\n", limit);
            std::fflush(stderr);
            std::_Exit(4);
        }).detach();
    }
    Simulator sim(rover, mission, *controller, cname, opt);
    if (!a.flag("quiet")) std::printf("Simulating %s with %s (%s)...\n", mission.name.c_str(), rover.design.name.c_str(), cname.c_str());
    RunResult r = sim.run();
    std::cout << format_result(r, use_color(a));

    std::string replay;
    if (a.flag("out") || a.flag("html")) {
        std::ostringstream os;
        write_replay_json(sim, r, os);
        replay = os.str();
    }
    if (a.flag("out")) {
        std::ofstream out(a.get("out"), std::ios::binary);
        out << replay;
        std::printf("  Replay data written to %s\n", a.get("out").c_str());
    }
    if (a.flag("html")) {
        std::string page = load_viewer(a);
        const std::string marker = "/*__REPLAY_DATA__*/null";
        const size_t at = page.find(marker);
        if (at == std::string::npos) throw std::runtime_error("viewer.html is missing the replay data marker");
        page.replace(at, marker.size(), replay);
        std::ofstream out(a.get("html"), std::ios::binary);
        out << page;
        std::printf("  Replay page written to %s (open it in any browser)\n", a.get("html").c_str());
    }
    return r.failed ? 1 : 0;
}

// Drives straight ahead at a fixed wheel speed. Used by the slip-slope experiment.
class ConstantSpeed : public Controller {
public:
    explicit ConstantSpeed(double w) : w_(w) {}
    void on_start(const MissionBrief&) override {}
    Command on_step(const Observation&) override {
        Command c;
        c.wheel_speed_left = c.wheel_speed_right = w_;
        return c;
    }

private:
    double w_;
};

int cmd_slip_slope(const Args& a) {
    RoverModel rover = build_rover_model(load_rover_design(a.get("rover")));
    const std::string soil_id = a.get("soil", "m90_simulant");
    const Soil& soil = soil_by_id(soil_id);
    std::ostream* out = &std::cout;
    std::ofstream file;
    if (a.flag("out")) {
        file.open(a.get("out"));
        out = &file;
    }
    *out << "slope_deg,predicted_slip,simulated_slip,simulated_sinkage_cm,drive_power_w\n";
    const double w = 0.7 * rover.motor.no_load_speed;
    const double drive_s = 200.0;
    for (double slope = 0.0; slope <= 30.0 + 1e-9; slope += 2.5) {
        const double th = deg2rad(slope);
        const double W = rover.mass * mars::kGravity;
        double pred = steady_state_slip(rover, soil, W * std::cos(th) / rover.design.wheel_count,
                                        W * std::sin(th) / rover.design.wheel_count, 0.95);
        Mission m;
        m.name = "slip-slope";
        m.seed = 1;
        m.terrain.size_m = 60.0;
        m.terrain.relief_m = 0.0;
        m.terrain.random_craters = 0;
        m.terrain.incline_deg = slope;
        m.terrain.base_soil = soil_id;
        m.terrain.drift_fraction = 0.0;
        m.terrain.crust_fraction = 0.0;
        m.terrain.outcrop_slope_deg = 89.0;
        m.terrain.rock_density = 0.0;
        m.terrain.science_targets = 0;
        m.start = Vec2(8.0, 30.0);
        m.start_heading_deg = 0.0;
        m.start_local_hour = 12.0;
        m.start_soc = 1.0;
        m.duration_sols = drive_s / mars::kSolSeconds;
        ConstantSpeed ctl(w);
        SimOptions opt;
        opt.record_trace = true;
        opt.trace_interval_s = 1.0;
        Simulator sim(rover, m, ctl, "constant_speed", opt);
        RunResult r = sim.run();
        // Measure over the second half of the run (after acceleration transients).
        double sim_slip = 1.0, sink = 0.0, power = 0.0;
        if (r.trace.size() > 4 && !r.failed) {
            const auto& t0 = r.trace[r.trace.size() / 2];
            const auto& t1 = r.trace.back();
            const double dist = std::hypot(t1.x - t0.x, t1.y - t0.y);
            const double dt = t1.t - t0.t;
            // Ground distance along the incline (x is horizontal).
            const double along = dist / std::cos(th);
            sim_slip = 1.0 - along / (w * rover.wheel_radius() * dt);
            sink = t1.sinkage_cm;
            power = t1.load_w;
        }
        char buf[200];
        std::snprintf(buf, sizeof(buf), "%.1f,%.4f,%.4f,%.3f,%.1f\n", slope, pred < 0 ? 1.0 : pred, std::max(0.0, sim_slip), sink, power);
        *out << buf;
    }
    if (a.flag("out")) std::printf("Wrote %s\n", a.get("out").c_str());
    return 0;
}

int cmd_bench(const Args& a) {
    RoverModel rover = build_rover_model(load_rover_design(a.get("rover")));
    const Mission base = load_mission(a.get("mission"));
    std::vector<std::string> names;
    std::stringstream ss(a.get("controllers", "reference"));
    for (std::string item; std::getline(ss, item, ',');)
        if (!item.empty()) names.push_back(item);
    const int seeds = std::stoi(a.get("seeds", "5"));
    std::ostream* out = &std::cout;
    std::ofstream file;
    if (a.flag("out")) {
        file.open(a.get("out"));
        out = &file;
    }
    *out << "controller,seed,score,science,goal_reached,measured,received,distance_m,collisions,failed,failure\n";
    for (const auto& name : names) {
        double total = 0.0;
        int failures = 0;
        for (int s = 1; s <= seeds; ++s) {
            Mission m = base;
            m.seed = base.seed + s;
            auto ctl = ControllerRegistry::instance().create(name);
            SimOptions opt;
            opt.record_trace = false;
            Simulator sim(rover, m, *ctl, name, opt);
            RunResult r = sim.run();
            total += r.score;
            failures += r.failed;
            *out << name << ',' << m.seed << ',' << r.score << ',' << r.science_points << ',' << r.goal_reached << ','
                 << r.targets_measured << ',' << r.targets_received << ',' << r.distance_m << ',' << r.collisions << ','
                 << r.failed << ",\"" << r.failure_type << "\"\n";
            out->flush();
        }
        std::fprintf(stderr, "%-16s mean score %.1f over %d seeds, %d failures\n", name.c_str(), total / seeds, seeds, failures);
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
#ifdef _WIN32
    // Let the Windows console understand the colour codes used in reports.
    HANDLE out = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD mode = 0;
    if (out != INVALID_HANDLE_VALUE && GetConsoleMode(out, &mode)) SetConsoleMode(out, mode | 0x0004 /* VT processing */);
#endif
    if (argc < 2) {  // double-clicked or run bare: open the GUI
        try {
            return cmd_gui(argv[0], Args{});
        } catch (const std::exception& e) {
            std::fprintf(stderr, "error: %s\n", e.what());
            return 3;
        }
    }
    const std::string cmd = argv[1];
    try {
        if (cmd == "gui") return cmd_gui(argv[0], parse_args(argc, argv, 2));
        if (cmd == "help" || cmd == "--help" || cmd == "-h") { usage(); return 0; }
        if (cmd == "catalog") return cmd_catalog();
        if (cmd == "controllers") return cmd_controllers();
        if (cmd == "check") return cmd_check(parse_args(argc, argv, 2));
        if (cmd == "compare") return cmd_compare(parse_args(argc, argv, 2));
        if (cmd == "run") return cmd_run(parse_args(argc, argv, 2));
        if (cmd == "experiment" && argc >= 3) {
            const std::string which = argv[2];
            Args a = parse_args(argc, argv, 3);
            if (which == "slip-slope") return cmd_slip_slope(a);
            if (which == "bench") return cmd_bench(a);
        }
        usage();
        return 2;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 3;
    }
}
