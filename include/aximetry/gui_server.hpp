// Local web server behind the Aximetry mission-control GUI.
//
// Serves tools/gui.html on http://127.0.0.1:<port>/ and a small JSON API:
//   GET  /api/catalog   materials, motors, batteries, soils, controllers, missions, rover presets
//   POST /api/check     { rover, mission? }               -> design review
//   POST /api/run       { rover, mission, controller, seed? } -> run summary + replay id
//   POST /api/save      { rover }                         -> writes rovers/custom_<name>.json
//   GET  /replay/<id>   the replay viewer with that run embedded
//   GET  /replay/<id>.json  raw replay data (download)
// Only listens on localhost. No dependencies beyond the OS socket API.
#pragma once

#include <string>

namespace aximetry {

struct GuiOptions {
    std::string root;   // project folder containing tools/, rovers/, missions/
    int port = 8080;
    bool open_browser = true;
    // For compiling user controllers from the GUI (filled in by the build).
    std::string compiler, compiler_id, cli_lib, core_lib, build_dir;
};

int run_gui_server(const GuiOptions& opt);

}  // namespace aximetry
