#include "aximetry/gui_server.hpp"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#ifdef _MSC_VER
#pragma comment(lib, "ws2_32.lib")
#endif
using socket_t = SOCKET;
static const socket_t kBadSocket = INVALID_SOCKET;
static void close_socket(socket_t s) { closesocket(s); }
#else
#include <arpa/inet.h>
#include <csignal>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
using socket_t = int;
static const socket_t kBadSocket = -1;
static void close_socket(socket_t s) { ::close(s); }
#endif

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <map>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "aximetry/controller.hpp"
#include "aximetry/design_check.hpp"
#include "aximetry/json.hpp"
#include "aximetry/mission.hpp"
#include "aximetry/rover.hpp"
#include "aximetry/sim.hpp"

namespace fs = std::filesystem;

namespace aximetry {

namespace {

// ------------------------------------------------------------------ HTTP plumbing
struct Request {
    std::string method, path, query, body;
    std::string host, origin, content_type;
};

struct Response {
    int status = 200;
    std::string type = "application/json";
    std::string body;
    std::string extra_headers;
};

const char* status_text(int s) {
    switch (s) {
        case 200: return "OK";
        case 400: return "Bad Request";
        case 404: return "Not Found";
        case 403: return "Forbidden";
        case 405: return "Method Not Allowed";
        case 413: return "Payload Too Large";
        default: return "Internal Server Error";
    }
}

bool read_request(socket_t s, Request& r) {
    std::string buf;
    char tmp[16384];
    size_t header_end = std::string::npos;
    while (header_end == std::string::npos) {
        const int n = recv(s, tmp, sizeof(tmp), 0);
        if (n <= 0) return false;
        buf.append(tmp, static_cast<size_t>(n));
        header_end = buf.find("\r\n\r\n");
        if (buf.size() > (1u << 20) && header_end == std::string::npos) return false;
    }
    const std::string head = buf.substr(0, header_end);
    std::istringstream hs(head);
    std::string line, target, version;
    std::getline(hs, line);
    std::istringstream rl(line);
    rl >> r.method >> target >> version;
    const size_t q = target.find('?');
    r.path = target.substr(0, q);
    if (q != std::string::npos) r.query = target.substr(q + 1);

    size_t length = 0;
    while (std::getline(hs, line)) {
        const size_t c = line.find(':');
        if (c == std::string::npos) continue;
        std::string key = line.substr(0, c);
        for (auto& ch : key) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        std::string val = line.substr(c + 1);
        while (!val.empty() && (val.front() == ' ' || val.front() == '\t')) val.erase(val.begin());
        while (!val.empty() && (val.back() == '\r' || val.back() == ' ')) val.pop_back();
        if (key == "content-length") length = static_cast<size_t>(std::strtoull(val.c_str(), nullptr, 10));
        else if (key == "host") r.host = val;
        else if (key == "origin") r.origin = val;
        else if (key == "content-type") r.content_type = val;
    }
    if (length > (32u << 20)) return false;
    r.body = buf.substr(header_end + 4);
    while (r.body.size() < length) {
        const int n = recv(s, tmp, sizeof(tmp), 0);
        if (n <= 0) return false;
        r.body.append(tmp, static_cast<size_t>(n));
    }
    r.body.resize(length);
    return true;
}

void send_all(socket_t s, const std::string& data) {
    size_t off = 0;
    while (off < data.size()) {
        const int chunk = static_cast<int>(std::min<size_t>(data.size() - off, 1u << 20));
        const int n = send(s, data.data() + off, chunk, 0);
        if (n <= 0) return;
        off += static_cast<size_t>(n);
    }
}

void send_response(socket_t s, const Response& r) {
    std::ostringstream os;
    os << "HTTP/1.1 " << r.status << ' ' << status_text(r.status) << "\r\n"
       << "Content-Type: " << r.type << "\r\n"
       << "Content-Length: " << r.body.size() << "\r\n"
       << "Cache-Control: no-store\r\n"
       << r.extra_headers << "Connection: close\r\n\r\n";
    send_all(s, os.str());
    send_all(s, r.body);
}

Response json_response(const json::Value& v, int status = 200) {
    Response r;
    r.status = status;
    r.body = json::dump(v, -1);
    return r;
}

Response error_response(int status, const std::string& msg) {
    json::Value v = json::Value::object();
    v.set("error", msg);
    return json_response(v, status);
}

std::string read_file(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    if (!in) throw std::runtime_error("cannot open " + p.string());
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

// A bare file name such as "02_crater_science.json" (no folders, no "..").
bool safe_name(const std::string& n) {
    if (n.empty() || n.size() > 120) return false;
    for (char c : n)
        if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-' || c == '.')) return false;
    return n.find("..") == std::string::npos;
}

std::vector<fs::path> json_files(const fs::path& dir) {
    std::vector<fs::path> out;
    std::error_code ec;
    for (const auto& e : fs::directory_iterator(dir, ec))
        if (e.is_regular_file() && e.path().extension() == ".json") out.push_back(e.path());
    std::sort(out.begin(), out.end());
    return out;
}

// ------------------------------------------------------------------ the app
class GuiApp {
public:
    GuiApp(GuiOptions o, int port) : opt_(std::move(o)), root_(opt_.root), port_(port) {}

    Response handle(const Request& req) {
        // Only answer this computer's own browser tab: reject other Host names
        // (DNS rebinding) and cross-site POSTs from other web pages (which could
        // otherwise make the GUI compile and run code).
        const std::string p = std::to_string(port_);
        const bool host_ok = req.host == "127.0.0.1:" + p || req.host == "localhost:" + p;
        if (!host_ok) return error_response(403, "forbidden host");
        if (req.method == "POST") {
            const bool origin_ok = req.origin.empty() || req.origin == "http://127.0.0.1:" + p || req.origin == "http://localhost:" + p;
            if (!origin_ok || req.content_type.find("application/json") == std::string::npos)
                return error_response(403, "forbidden: requests must come from the Aximetry page");
        }
        try {
            if (req.method == "GET" && (req.path == "/" || req.path == "/index.html")) {
                Response r;
                r.type = "text/html; charset=utf-8";
                r.body = read_file(root_ / "tools" / "gui.html");
                return r;
            }
            if (req.method == "GET" && req.path == "/favicon.ico") return Response{204, "text/plain", "", ""};
            if (req.method == "GET" && req.path == "/api/catalog") return catalog();
            if (req.path == "/api/check") return req.method == "POST" ? check(req.body) : error_response(405, "use POST");
            if (req.path == "/api/run") return req.method == "POST" ? run(req.body) : error_response(405, "use POST");
            if (req.path == "/api/save") return req.method == "POST" ? save(req.body) : error_response(405, "use POST");
            if (req.method == "GET" && req.path == "/api/code/list") return code_list();
            if (req.method == "GET" && req.path == "/api/code/load") return code_load(req.query);
            if (req.path == "/api/code/compile") return req.method == "POST" ? code_compile(req.body) : error_response(405, "use POST");
            if (req.method == "GET" && req.path.rfind("/replay/", 0) == 0) return replay(req.path.substr(8));
            return error_response(404, "not found: " + req.path);
        } catch (const std::exception& e) {
            return error_response(400, e.what());
        }
    }

private:
    GuiOptions opt_;
    fs::path root_;
    int port_ = 0;
    std::map<std::string, fs::file_time_type> compiled_;  // source file -> mtime it was compiled at
    std::map<int, std::string> replays_;
    int next_id_ = 1;

    Response catalog() {
        using json::Value;
        Value out = Value::object();

        Value mats = Value::array();
        for (const auto& m : material_catalog()) {
            Value e = Value::object();
            e.set("id", m.id);
            e.set("name", m.name);
            e.set("density", m.density);
            e.set("yield_mpa", m.yield_strength / 1e6);
            e.set("youngs_gpa", m.youngs_modulus / 1e9);
            e.set("brittle_below_c", m.brittle_below_c < -200 ? Value() : Value(m.brittle_below_c));
            e.set("note", m.note);
            mats.push_back(e);
        }
        out.set("materials", mats);

        Value motors = Value::array();
        for (const auto& m : motor_catalog()) {
            Value e = Value::object();
            e.set("id", m.id);
            e.set("name", m.name);
            e.set("stall_torque", m.stall_torque);
            e.set("no_load_speed", m.no_load_speed);
            e.set("mass", m.mass);
            motors.push_back(e);
        }
        out.set("motors", motors);

        Value bats = Value::array();
        for (const auto& b : battery_catalog()) {
            Value e = Value::object();
            e.set("id", b.id);
            e.set("name", b.name);
            e.set("wh_per_kg", b.specific_energy);
            e.set("min_discharge_c", b.min_discharge_c);
            e.set("min_charge_c", b.min_charge_c);
            e.set("survival_c", b.survival_c);
            e.set("note", b.note);
            bats.push_back(e);
        }
        out.set("batteries", bats);

        Value ctls = Value::array();
        for (const auto& c : ControllerRegistry::instance().entries()) {
            Value e = Value::object();
            e.set("name", c.name);
            e.set("description", c.description);
            ctls.push_back(e);
        }
        out.set("controllers", ctls);

        Value missions = Value::array();
        for (const auto& p : json_files(root_ / "missions")) {
            try {
                const Mission m = load_mission(p.string());
                Value e = Value::object();
                e.set("file", p.filename().string());
                e.set("name", m.name);
                e.set("description", m.description);
                e.set("sols", m.duration_sols);
                e.set("seed", static_cast<double>(m.seed));
                e.set("size_m", m.terrain.size_m);
                e.set("has_goal", m.has_goal);
                e.set("targets", m.terrain.science_targets + static_cast<int>(m.terrain.fixed_targets.size()));
                e.set("storm", !m.environment.storms.empty());
                missions.push_back(e);
            } catch (const std::exception& e) {
                std::fprintf(stderr, "skipping mission %s: %s\n", p.string().c_str(), e.what());
            }
        }
        out.set("missions", missions);

        Value rovers = Value::array();
        for (const auto& p : json_files(root_ / "rovers")) {
            try {
                Value e = Value::object();
                e.set("file", p.filename().string());
                e.set("design", rover_design_to_json(load_rover_design(p.string())));
                rovers.push_back(e);
            } catch (const std::exception& e) {
                std::fprintf(stderr, "skipping rover %s: %s\n", p.string().c_str(), e.what());
            }
        }
        out.set("rovers", rovers);
        return json_response(out);
    }

    Mission mission_from_request(const json::Value& body) {
        const std::string file = body.str("mission", "");
        if (!safe_name(file)) throw std::runtime_error("bad mission name");
        return load_mission((root_ / "missions" / file).string());
    }

    Response check(const std::string& body_text) {
        const json::Value body = json::parse(body_text, "request");
        const RoverModel model = build_rover_model(rover_design_from_json(body.at("rover")));
        EnvironmentConfig env;
        if (body.has("mission") && !body.at("mission").is_null()) env = mission_from_request(body).environment;
        return json_response(report_to_json(analyze_design(model, env)));
    }

    Response run(const std::string& body_text) {
        const json::Value body = json::parse(body_text, "request");
        if (body.has("code_file") && body.at("code_file").is_string()) return run_user_code(body);
        const RoverModel model = build_rover_model(rover_design_from_json(body.at("rover")));
        Mission mission = mission_from_request(body);
        if (body.has("seed") && body.at("seed").is_number()) mission.seed = static_cast<uint64_t>(body.at("seed").as_number());
        const std::string cname = body.str("controller", "reference");
        auto controller = ControllerRegistry::instance().create(cname);

        std::printf("  launching %s on %s with %s ...\n", model.design.name.c_str(), mission.name.c_str(), cname.c_str());
        std::fflush(stdout);
        Simulator sim(model, mission, *controller, cname);
        const RunResult r = sim.run();
        std::printf("  done in %.1f s: score %.1f%s\n", r.wall_time_s, r.score, r.failed ? " (rover lost)" : "");
        std::fflush(stdout);

        std::ostringstream os;
        write_replay_json(sim, r, os);
        const int id = next_id_++;
        replays_[id] = os.str();
        while (replays_.size() > 8) replays_.erase(replays_.begin());

        json::Value out = json::Value::object();
        out.set("id", id);
        out.set("score", r.score);
        out.set("science", r.science_points);
        out.set("goal_points", r.goal_points);
        out.set("goal_reached", r.goal_reached);
        out.set("measured", r.targets_measured);
        out.set("received", r.targets_received);
        out.set("distance_m", r.distance_m);
        out.set("failed", r.failed);
        out.set("failure", r.failure_message);
        out.set("lesson", r.lesson);
        out.set("wall_time_s", r.wall_time_s);
        out.set("sols", r.sim_time_s / mars::kSolSeconds);
        out.set("text", format_result(r, false));
        return json_response(out);
    }

    Response save(const std::string& body_text) {
        const json::Value body = json::parse(body_text, "request");
        const RoverDesign d = rover_design_from_json(body.at("rover"));  // validates
        std::string slug;
        for (char c : d.name) {
            if (std::isalnum(static_cast<unsigned char>(c))) slug += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            else if (!slug.empty() && slug.back() != '_') slug += '_';
        }
        while (!slug.empty() && slug.back() == '_') slug.pop_back();
        if (slug.empty()) slug = "design";
        const std::string file = "custom_" + slug.substr(0, 40) + ".json";
        const fs::path path = root_ / "rovers" / file;
        std::ofstream out(path, std::ios::binary);
        if (!out) throw std::runtime_error("cannot write " + path.string());
        out << "// Saved from the Aximetry GUI.\n" << json::dump(rover_design_to_json(d), 2) << "\n";
        json::Value v = json::Value::object();
        v.set("file", file);
        return json_response(v);
    }


    // ---------------------------------------------------------- user code
    fs::path code_dir() const { return root_ / "my_controllers"; }
    fs::path runner_dir() const { return fs::path(opt_.build_dir) / "user_runners"; }

    static std::string controller_name_in(const std::string& code) {
        static const std::regex re(R"(AXIMETRY_REGISTER_CONTROLLER\s*\(\s*\w+\s*,\s*"([^"]+)\")");
        std::smatch m;
        if (std::regex_search(code, m, re)) return m[1].str();
        return "";
    }

    std::string compiler_problem() const {
        if (opt_.compiler.empty()) return "this build of Aximetry does not know its compiler; rebuild with CMake";
        if (opt_.compiler_id == "MSVC")
            return "compiling from the GUI needs GCC or Clang (MinGW works). With Visual Studio, copy your file into "
                   "controllers/ and rebuild instead";
        if (!fs::exists(opt_.cli_lib) || !fs::exists(opt_.core_lib)) return "build libraries not found; run cmake --build build first";
        return "";
    }

    Response code_list() {
        std::error_code ec;
        fs::create_directories(code_dir(), ec);
        json::Value out = json::Value::object();
        json::Value files = json::Value::array();
        std::vector<fs::path> paths;
        for (const auto& e : fs::directory_iterator(code_dir(), ec))
            if (e.is_regular_file() && e.path().extension() == ".cpp") paths.push_back(e.path());
        std::sort(paths.begin(), paths.end());
        for (const auto& pth : paths) {
            json::Value f = json::Value::object();
            f.set("file", pth.filename().string());
            f.set("controller", controller_name_in(read_file(pth)));
            files.push_back(f);
        }
        out.set("files", files);
        const std::string problem = compiler_problem();
        out.set("compiler_ok", problem.empty());
        out.set("problem", problem);
        out.set("compiler", opt_.compiler);
        return json_response(out);
    }

    Response code_load(const std::string& query) {
        const std::string prefix = "file=";
        const std::string file = query.rfind(prefix, 0) == 0 ? query.substr(prefix.size()) : "";
        if (!safe_name(file) || fs::path(file).extension() != ".cpp") return error_response(400, "bad file name");
        json::Value out = json::Value::object();
        out.set("file", file);
        const fs::path pth = code_dir() / file;
        if (fs::exists(pth)) {
            out.set("code", read_file(pth));
        } else {
            // New file: start from the starter controller, renamed.
            std::string code = read_file(code_dir() / "my_rover.cpp");
            const std::string stem = fs::path(file).stem().string();
            code = std::regex_replace(code, std::regex(R"(AXIMETRY_REGISTER_CONTROLLER\(MyRover, "my_rover")"),
                                      "AXIMETRY_REGISTER_CONTROLLER(MyRover, \"" + stem + "\"");
            out.set("code", code);
        }
        return json_response(out);
    }

    static std::string quote(const fs::path& p) { return "\"" + p.string() + "\""; }

    static int run_command(const std::string& cmd) {
#ifdef _WIN32
        // cmd.exe strips the outer quotes when the command line starts with one.
        return std::system(("\"" + cmd + "\"").c_str());
#else
        return std::system(cmd.c_str());
#endif
    }

    // Compile my_controllers/<file> + the prebuilt libraries into a runner program.
    // Returns {ok, log, controller, seconds}.
    json::Value compile_file(const std::string& file) {
        json::Value out = json::Value::object();
        const fs::path src = code_dir() / file;
        const std::string code = read_file(src);
        const std::string controller = controller_name_in(code);
        out.set("controller", controller);
        if (controller.empty()) {
            out.set("ok", false);
            out.set("log", std::string("No registration line found. End your file with:\n  AXIMETRY_REGISTER_CONTROLLER(MyClass, \"my_name\", \"description\");"));
            return out;
        }
        const std::string problem = compiler_problem();
        if (!problem.empty()) {
            out.set("ok", false);
            out.set("log", "Cannot compile here: " + problem);
            return out;
        }
        std::error_code ec;
        fs::create_directories(runner_dir(), ec);
        const std::string stem = fs::path(file).stem().string();
#ifdef _WIN32
        const fs::path exe = runner_dir() / (stem + ".exe");
#else
        const fs::path exe = runner_dir() / stem;
#endif
        const fs::path log = runner_dir() / (stem + "_compile.log");
        std::string cmd = quote(opt_.compiler) + " -std=c++17 -O2 -Wall -Wextra -I" + quote(root_ / "include") + " " + quote(src) +
                          " " + quote(opt_.cli_lib) + " " + quote(opt_.core_lib) + " -o " + quote(exe);
#ifdef _WIN32
        cmd += " -lws2_32";
#else
        cmd += " -pthread";
#endif
#ifdef __MINGW32__
        cmd += " -static";
#endif
        cmd += " > " + quote(log) + " 2>&1";
        std::printf("  compiling %s ...\n", file.c_str());
        std::fflush(stdout);
        fs::remove(exe, ec);
        const auto t0 = std::chrono::steady_clock::now();
        const int rc = run_command(cmd);
        const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        std::string text = fs::exists(log) ? read_file(log) : "";
        // Show paths relative to the project so errors read "my_controllers/x.cpp:12:5: error ...".
        for (const std::string& full : {src.string(), fs::path(src).make_preferred().string()}) {
            size_t pos;
            while ((pos = text.find(full)) != std::string::npos) text.replace(pos, full.size(), "my_controllers/" + file);
        }
        const bool ok = rc == 0 && fs::exists(exe);
        out.set("ok", ok);
        out.set("log", text.empty() ? std::string(ok ? "Compiled with no warnings." : "Compiler failed with no output.") : text);
        out.set("seconds", secs);
        if (ok) compiled_[file] = fs::last_write_time(src);
        std::printf("  %s in %.1f s\n", ok ? "compiled" : "compile FAILED", secs);
        std::fflush(stdout);
        return out;
    }

    Response code_compile(const std::string& body_text) {
        const json::Value body = json::parse(body_text, "request");
        const std::string file = body.str("file", "");
        if (!safe_name(file) || fs::path(file).extension() != ".cpp") return error_response(400, "file name must look like my_rover.cpp");
        std::error_code ec;
        fs::create_directories(code_dir(), ec);
        {
            std::ofstream outf(code_dir() / file, std::ios::binary);
            if (!outf) return error_response(500, "cannot write my_controllers/" + file);
            outf << body.str("code", "");
        }
        return json_response(compile_file(file));
    }

    Response run_user_code(const json::Value& body) {
        const std::string file = body.str("code_file", "");
        if (!safe_name(file) || fs::path(file).extension() != ".cpp") return error_response(400, "bad code file");
        if (!fs::exists(code_dir() / file)) return error_response(400, "my_controllers/" + file + " not found");
        const std::string stem = fs::path(file).stem().string();
#ifdef _WIN32
        const fs::path exe = runner_dir() / (stem + ".exe");
#else
        const fs::path exe = runner_dir() / stem;
#endif
        auto it = compiled_.find(file);
        if (it == compiled_.end() || it->second != fs::last_write_time(code_dir() / file) || !fs::exists(exe)) {
            json::Value c = compile_file(file);
            if (!c.at("ok").as_bool()) {
                json::Value err = json::Value::object();
                err.set("error", "your code did not compile (see the CODE tab)");
                err.set("compile_log", c.at("log"));
                return json_response(err, 400);
            }
        }
        const std::string controller = controller_name_in(read_file(code_dir() / file));
        const RoverDesign design = rover_design_from_json(body.at("rover"));  // validate first
        const std::string mission_file = body.str("mission", "");
        if (!safe_name(mission_file)) return error_response(400, "bad mission name");

        const fs::path rover_tmp = runner_dir() / "gui_rover.json";
        const fs::path out_json = runner_dir() / "gui_run.json";
        const fs::path log = runner_dir() / "gui_run.log";
        {
            std::ofstream rf(rover_tmp, std::ios::binary);
            rf << json::dump(rover_design_to_json(design), 2);
        }
        std::error_code ec;
        fs::remove(out_json, ec);
        std::string cmd = quote(exe) + " run --rover " + quote(rover_tmp) + " --mission " + quote(root_ / "missions" / mission_file) +
                          " --controller " + controller + " --out " + quote(out_json) + " --no-color --quiet --time-limit 180";
        if (body.has("seed") && body.at("seed").is_number())
            cmd += " --seed " + std::to_string(static_cast<long long>(body.at("seed").as_number()));
        cmd += " > " + quote(log) + " 2>&1";
        std::printf("  launching %s on %s with your controller %s ...\n", design.name.c_str(), mission_file.c_str(), controller.c_str());
        std::fflush(stdout);
        const auto t0 = std::chrono::steady_clock::now();
        run_command(cmd);
        const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        const std::string text = fs::exists(log) ? read_file(log) : "";
        if (!fs::exists(out_json)) {
            json::Value err = json::Value::object();
            err.set("error", "your controller crashed or timed out before the mission finished");
            err.set("run_log", text);
            return json_response(err, 400);
        }
        std::string replay_text = read_file(out_json);
        const json::Value replay = json::parse(replay_text, "replay");
        const json::Value& res = replay.at("result");
        const int id = next_id_++;
        replays_[id] = std::move(replay_text);
        while (replays_.size() > 8) replays_.erase(replays_.begin());

        json::Value out = json::Value::object();
        out.set("id", id);
        out.set("score", res.num("score", 0));
        out.set("science", res.num("science", 0));
        out.set("goal_points", res.num("goal", 0));
        out.set("goal_reached", res.boolean("goal_reached", false));
        out.set("measured", res.integer("measured", 0));
        out.set("received", res.integer("received", 0));
        out.set("distance_m", res.num("distance", 0));
        out.set("failed", res.boolean("failed", false));
        out.set("failure", res.str("failure", ""));
        out.set("lesson", res.str("lesson", ""));
        out.set("wall_time_s", secs);
        out.set("sols", replay.at("trace").at("t").size() ? replay.at("trace").at("t").as_array().back().as_number() / mars::kSolSeconds : 0.0);
        out.set("text", text);
        std::printf("  done in %.1f s: score %.1f\n", secs, res.num("score", 0));
        std::fflush(stdout);
        return json_response(out);
    }

    Response replay(const std::string& tail) {
        const bool raw = tail.size() > 5 && tail.compare(tail.size() - 5, 5, ".json") == 0;
        const int id = std::atoi(tail.c_str());
        auto it = replays_.find(id);
        if (it == replays_.end()) return error_response(404, "replay expired; launch again");
        Response r;
        if (raw) {
            r.body = it->second;
            r.extra_headers = "Content-Disposition: attachment; filename=\"aximetry_run_" + std::to_string(id) + ".json\"\r\n";
            return r;
        }
        std::string page = read_file(root_ / "tools" / "viewer.html");
        const std::string marker = "/*__REPLAY_DATA__*/null";
        const size_t at = page.find(marker);
        if (at == std::string::npos) throw std::runtime_error("viewer.html is missing the replay data marker");
        page.replace(at, marker.size(), it->second);
        r.type = "text/html; charset=utf-8";
        r.body = std::move(page);
        return r;
    }
};

void open_url(const std::string& url) {
#ifdef _WIN32
    const std::string cmd = "start \"\" \"" + url + "\"";
#elif defined(__APPLE__)
    const std::string cmd = "open \"" + url + "\"";
#else
    const std::string cmd = "xdg-open \"" + url + "\" >/dev/null 2>&1 &";
#endif
    if (std::system(cmd.c_str()) != 0) std::printf("  (could not open a browser automatically; open the address above)\n");
}

}  // namespace

int run_gui_server(const GuiOptions& opt) {
#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        std::fprintf(stderr, "error: could not start Windows networking\n");
        return 3;
    }
#else
    std::signal(SIGPIPE, SIG_IGN);
#endif
    if (!fs::exists(fs::path(opt.root) / "tools" / "gui.html")) {
        std::fprintf(stderr, "error: cannot find tools/gui.html under %s (pass --root PATH)\n", opt.root.c_str());
        return 3;
    }

    socket_t server = socket(AF_INET, SOCK_STREAM, 0);
    if (server == kBadSocket) {
        std::fprintf(stderr, "error: could not create socket\n");
        return 3;
    }
#ifndef _WIN32
    int yes = 1;
    setsockopt(server, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&yes), sizeof(yes));
#endif
    int port = opt.port;
    bool bound = false;
    for (int attempt = 0; attempt < 20 && !bound; ++attempt, ++port) {
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);  // localhost only
        addr.sin_port = htons(static_cast<unsigned short>(port));
        bound = bind(server, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0;
    }
    --port;
    if (!bound || listen(server, 16) != 0) {
        std::fprintf(stderr, "error: could not listen on ports %d-%d\n", opt.port, opt.port + 19);
        close_socket(server);
        return 3;
    }

    const std::string url = "http://127.0.0.1:" + std::to_string(port) + "/";
    std::printf("\n  AXIMETRY mission control\n  -> %s\n\n  Keep this window open while you use the GUI. Close it (or press Ctrl+C) to quit.\n\n", url.c_str());
    std::fflush(stdout);
    if (opt.open_browser) open_url(url);

    GuiApp app(opt, port);
    while (true) {
        socket_t client = accept(server, nullptr, nullptr);
        if (client == kBadSocket) continue;
#ifdef _WIN32
        DWORD timeout_ms = 10000;
        setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&timeout_ms), sizeof(timeout_ms));
#else
        timeval tv{10, 0};
        setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&tv), sizeof(tv));
#endif
        Request req;
        if (read_request(client, req)) send_response(client, app.handle(req));
        close_socket(client);
    }
}

}  // namespace aximetry
