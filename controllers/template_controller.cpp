// =============================================================================
//  Copy this file to start your own controller.
//
//  1. Copy to controllers/my_rover.cpp and rename the class + registration name.
//  2. cmake --build build
//  3. ./build/aximetry run --rover rovers/mer_class.json
//         --mission missions/01_first_drive.json --controller my_rover --html out.html
//
//  The rover never tells you where it truly is. You only get noisy sensors:
//  wheel encoders, a gyro, a sun sensor, landmark sightings and a hazard scanner.
// =============================================================================
#include <cmath>

#include "aximetry/controller.hpp"

using namespace aximetry;

class TemplateController : public Controller {
public:
    void on_start(const MissionBrief& brief) override {
        brief_ = brief;
        // Ideas: plan a route over brief.map, rank brief.targets, note relay passes.
    }

    Command on_step(const Observation& obs) override {
        Command cmd;

        // Example 1: stop and sleep when the sun is down (saves power).
        if (obs.sun_elevation < 0.0) {
            cmd.sleep_s = 1800.0;
            return cmd;
        }

        // Example 2: stop if the hazard scanner sees something close ahead.
        bool blocked = false;
        for (const auto& ray : obs.hazards)
            if (ray.hit && ray.range < 1.0 && std::fabs(ray.angle) < 0.3) blocked = true;

        // Example 3: drive forward at 3 cm/s, or turn on the spot when blocked.
        if (blocked) cmd.drive(0.0, 0.1, brief_.rover);
        else cmd.drive(0.03, 0.0, brief_.rover);

        // Example 4: always send data home when an orbiter is overhead.
        cmd.transmit = obs.relay_in_view;
        cmd.status = blocked ? "turning" : "cruising";
        return cmd;
    }

private:
    MissionBrief brief_;
};

AXIMETRY_REGISTER_CONTROLLER(TemplateController, "template", "Starter controller: cruise forward, turn away from hazards.");
