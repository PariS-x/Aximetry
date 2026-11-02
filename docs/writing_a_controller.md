# Writing a controller

**Quickest way:** open mission control (`aximetry` with no arguments), go to the
**CODE** tab, edit the starter, and press **Compile & launch**. Files live in
`my_controllers/` and are compiled on demand against the prebuilt simulator,
so there is no full rebuild. The rest of this page explains the API and the
command-line route.

A controller is one C++ class. The simulator calls it; it never sees the true
state of the rover, only noisy sensors. Everything you need is in
[`include/aximetry/controller.hpp`](../include/aximetry/controller.hpp).

## Minimal example

```cpp
#include "aximetry/controller.hpp"
using namespace aximetry;

class Cruise : public Controller {
public:
    void on_start(const MissionBrief& b) override { rover_ = b.rover; }

    Command on_step(const Observation& o) override {
        Command c;
        if (o.sun_elevation < 0.0) { c.sleep_s = 1800; return c; }  // night: sleep 30 min
        c.drive(0.03, 0.0, rover_);                                 // 3 cm/s, straight
        c.transmit = o.relay_in_view;                               // send data home
        return c;
    }

private:
    RoverInfo rover_;
};

AXIMETRY_REGISTER_CONTROLLER(Cruise, "cruise", "Drives straight ahead.");
```

Save it as `controllers/cruise.cpp` and rebuild. It now appears in the
controller list in mission control (run `aximetry` with no arguments). From the
command line:

```
cmake --build build
./build/aximetry run --rover rovers/mer_class.json --mission missions/01_first_drive.json \
    --controller cruise --html cruise.html
```

Every `.cpp` under `controllers/` (including subfolders) is compiled in
automatically. Helper files without a registration line are fine too; the
reference controller is split into `ekf.cpp`, `planner.cpp` and the controller.

## What you get

**`on_start(MissionBrief)`**, once:

* `map` — orbital elevation model (2 m/pixel, with noise) and soil classes guessed
  from orbit (sometimes wrong). `map.height_at(x, y)`, `map.slope_deg_at(x, y)`,
  `map.soil_at(x, y)`.
* `landmarks` — surveyed boulders (id, position, radius).
* `targets` — science targets with the mineralogy orbit *thinks* they are.
* `relay_passes`, `owlt_s`, `product_mbit`, `sample_duration_s`, `sample_range_m`.
* `start_pos`, `start_heading` — the landing estimate (±0.5 m, ±2°).
* `rover` — wheel radius, track, top wheel speed, battery size, tip angle…

**`on_step(Observation)`**, at 5 Hz while awake. See
[physics.md §7](physics.md#7-sensors-what-controllers-see) for the noise on each:
encoders, gyro, pitch/roll, sun heading, landmark sightings, visual odometry,
hazard rays, battery and temperatures, spectrometer progress, relay status,
uplinked messages and events (collisions, wheel failures, brownouts…).

## What you send back

| field | meaning |
|---|---|
| `wheel_speed_left/right` | rad/s per side (or call `drive(speed, yaw_rate, rover)`) |
| `sample_target` | run the spectrometer on a target id (must be parked, within range) |
| `transmit` | downlink stored data if an orbiter is in view |
| `heater_w` | override the thermostat (negative = automatic) |
| `sleep_s` | power down; `on_step` is not called until you wake |
| `end_mission` | stop early (saves simulation time when there is nothing left to do) |
| `has_estimate, est_x/y/heading/sigma` | your pose estimate, drawn in the replay viewer |
| `path_updated, planned_path` | your plan, drawn in the viewer |
| `status`, `log` | text shown in the viewer's HUD and log tab |

## Things that will bite you

* **Encoders lie on soft ground.** They measure wheel rotation; at 40% slip the
  rover moved 60% as far. Use visual odometry or landmarks to estimate slip.
* **Integrate the gyro carefully.** It has a bias. Estimate it while parked
  (zero-velocity update). After a long sleep `o.dt` is large but the rover did
  not move: don't integrate the gyro over it.
* **Motors have no torque at top speed.** `drive()` keeps you at ≤ 85% of the
  no-load speed for that reason.
* **Turning in place scrubs.** Skid-steer turns move the rover sideways a bit,
  more on side slopes.
* **Science only counts on Earth.** Measurements must be downlinked during a
  relay pass, one light-time before the mission ends.
* **Nights are dangerous.** If the battery runs flat, heaters stop. Leave reserve.

## Benchmarking

```
./build/aximetry experiment bench --rover rovers/mer_class.json \
    --mission missions/02_crater_science.json \
    --controllers reference,cruise --seeds 10 --out bench.csv
```

Each seed generates a new terrain, rock field and target set with the same
mission rules, so you are measuring the controller, not one lucky map.
