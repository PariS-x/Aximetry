# Physics model

Everything here is implemented in `src/` and exercised by `tests/test_main.cpp`.
Units are SI. The goal is the fidelity used for early mobility sizing of real
planetary rovers: closed-form terramechanics, quasi-static wheels, rigid-body
planar dynamics, lumped thermal and power models. Where a parameter is
illustrative rather than measured, it says so.

## 1. Terrain

`src/terrain.cpp` builds a heightmap (default 0.5 m cells) from fractal value
noise, explicit and random craters (parabolic bowl + Gaussian rim), and soil
classes:

| soil | role | source of parameters |
|---|---|---|
| `m90_simulant` | default regolith | Bekker–Wong values for the M90 Mars simulant, as compiled in arXiv:2606.06790 (Table 2) |
| `dry_sand` | lab sand | same table |
| `loose_drift` | weak aeolian drift | illustrative, tuned so MER-class rovers bog down on sloped drift |
| `duricrust` | cemented crust | illustrative |
| `bedrock` | outcrop, assigned where slope > 18° | illustrative, near-rigid |

Rocks follow a many-small/few-large size distribution. Rocks with radius ≥ 0.5 m
are "catalogued": resolvable in orbital imagery and given to the rover as
landmarks. Rocks taller than the rover's climb limit are hard obstacles; shorter
ones are driven over and count as wheel impacts.

## 2. Wheel–soil contact (`src/terramechanics.cpp`)

**Static sinkage** of a rigid wheel (Bekker), valid for n < 3:

    z0 = [ 3W / ((3 − n)(kc + b·kφ)·√D) ]^(2/(2n+1))

**Slip sinkage** (empirical, linear): a driving wheel digs itself in,

    z = z0 · (1 + k_ss · max(s, 0))

with k_ss per soil (0 for bedrock up to 0.7 for drift). This is the simplest
model that produces the "dig-in" failure mode; it has no memory of previous
passes.

**Contact length** from the entry angle θ1 = acos(1 − (z + h_g)/r), where h_g is
grouser height, with the exit angle taken as 0.3·θ1:

    L = 1.3 · r · θ1

**Compaction (rolling) resistance:**

    Rc = (kc + b·kφ) · z^(n+1) / (n + 1)

**Thrust** (Janosi–Hanamoto shear, integrated over the contact patch):

    Hmax = A·c + W·tan φ,      A = b·L
    H(s) = Hmax · [1 − K/(sL) · (1 − e^(−sL/K))] · sign(s)

**Slip ratio**, regularised near zero speed so the solver stays smooth:

    s = (rω − v) / max(|rω|, |v|, ε)

## 3. Vehicle dynamics (`src/sim.cpp`)

State: position, heading ψ, forward speed v, yaw rate ω_z.

* **Attitude** from the terrain gradient under the rover: pitch and roll.
* **Normal loads** include weight transfer through the centre of gravity height:
  side-to-side from roll, axle-to-axle from pitch (moment balance over the axle
  positions). Tip-over is declared when pitch or roll exceeds the static tip
  angle `atan((wheelbase/2)/h_cg)` / `atan((track/2)/h_cg)`.
* **Skid steering.** Left and right wheel groups are commanded separately. Each
  side's ground speed is `v ∓ ω_z·B/2`, so each wheel has its own slip, thrust and
  resistance. Lateral scrubbing during turns resists yaw with
  `M_lat = 0.6 · Σ |x_i| · W_i · tan φ_i`.
* **Motors.** Linear DC torque–speed curve at the wheel: `T(ω) = T_stall (1 − |ω|/ω0)`,
  evaluated at the speed the wheels are actually turning. Thrust is clipped to
  `T/r`. When unpowered or commanded to zero, the gearbox holds up to `T_stall`.
* **Integration.** The forward and yaw equations

      m v̇   = Σ(H − Rc) − m g sin(pitch)
      I ω̇_z = (F_R − F_L)·B/2 − M_lat

  are stiff (thrust changes sharply with slip, and slip depends on v). They are
  solved with **backward Euler** using a damped Newton iteration on (v, ω_z)
  with a finite-difference Jacobian. This is stable at 10 Hz while driving.
  A parked rover is held exactly static when locked-wheel traction exceeds the
  downslope force, which lets the simulator take 30 s steps overnight.
* **Encoders** see the wheel speed the motor actually achieves. When torque
  saturates, the achieved slip is found by bisection on H(s) = T/r.
* **Rocks.** Tall rocks are rigid obstacles for the rover's footprint (rectangle
  of wheelbase + wheel diameter by track + wheel width). Contact is resolved by
  pushing the rover back out along the shallower body axis whenever a step would
  increase penetration, so it can slide along or turn away from a rock but never
  drive through it. Short rocks under a wheel are impacts.

## 4. Structures (`src/rover.cpp`)

Every part's mass comes from its material density and geometry. Strength checks:

| part | model | failure |
|---|---|---|
| wheel rim | fixed–fixed curved beam between spokes, point load at mid-span: `M = P·L/8`, `σ = M/(b t²/6)` | yield |
| rim stiffness | `δ = P L³ / (192 E I)` | deflection > 3% of radius warns, > 8% fails |
| suspension leg | axial `P/A` plus bending from traction at the end of the leg `F·L·c/I` | yield |
| suspension leg | Euler buckling, pinned–pinned: `P_cr = π² E I / L²` | buckling |
| body | thin-walled box beam, simply supported, uniform load: `M = W L / 8` | yield |

Rock strikes use an impact factor `1.5 + 2·h_rock/D` and accumulate damage with
a Basquin/Miner rule `Σ (σ/σ_strength)^m`; a wheel fails at 1.0 or immediately
if a single strike exceeds strength. Materials have a brittle-transition
temperature: below it, impact strength drops (plain carbon steel and PLA are the
examples; FCC metals like aluminium and austenitic stainless are unaffected).
Curiosity's wheel damage from sharp, embedded rocks is the real-world example.

The design review (`aximetry check`) applies 2 g dynamic loads with a required
safety factor of 1.5, a 3 g single-wheel rock strike at the coldest hour, and
an "can it stand up in Earth gravity for testing" check.

## 5. Environment (`src/environment.cpp`)

* Mars gravity 3.721 m/s², sol 88 775 s.
* Top-of-atmosphere insolation 586 W/m² at mean distance, scaled by orbital
  eccentricity (e = 0.0934, perihelion at Ls = 251°).
* Sun elevation from latitude, solar declination (obliquity 25.19°) and hour angle.
* Horizontal irradiance: direct beam attenuated by `exp(−τ/μ)` plus a diffuse
  term `0.6 (1 − e^(−τ/μ)) e^(−0.25τ)`. This keeps ~80% of light at τ = 0.5 and a
  few percent in a global storm; for a full treatment see Appelbaum & Flood
  (1990), NASA TM-102299.
* Air temperature: cosine diurnal cycle peaking at 14:00 local, with the swing
  damped by excess dust (storms make nights warmer and days colder).
* Dust storms: optical depth ramps up, plateaus and decays.
* Panel dust: output decays by a fixed fraction per sol.

## 6. Power and thermal

* **Solar** = irradiance × area × cell efficiency × dust factor.
* **Loads:** avionics (awake/asleep), sensors, motors (copper loss + mechanical
  power through gearbox and driver efficiencies, plus controller idle), heater,
  spectrometer (12 W), radio (25 W).
* **Battery:** charge efficiency per chemistry; no charging below the chemistry's
  minimum charge temperature; capacity derated in the cold (the bottom part of
  the capacity becomes unavailable). If the battery cannot deliver, the rover
  **browns out**: everything including heaters turns off, and it reboots only
  once solar power recharges it.
* **Warm electronics box:** one thermal node,

      C dT/dt = P_avionics + P_heater + P_RHU − G (T − T_air)

  If the box falls below the battery's survival temperature, the rover is lost.
  This is the Opportunity failure chain: storm → no power → no heat → cold death.

## 7. Sensors (what controllers see)

| sensor | output | noise / limits |
|---|---|---|
| wheel encoders | cumulative angle, average rate per side | 1e-4 rad, 0.002 rad/s; measure wheel rotation, **not** ground motion |
| gyro | average yaw rate over the step | bias ~2e-4 rad/s with random walk, 0.002 rad/s white noise |
| accelerometer | pitch, roll | 0.5° |
| sun sensor | absolute heading | 1.5°, only when the sun is > 5° up |
| landmark camera | range/bearing to catalogued boulders by id | 15 m, ±60°, 85% detection, 1 Hz, range σ 0.05 + 2% |
| visual odometry | displacement since last frame, body axes | 1 Hz, daylight only, 2% of distance, fails 60% of the time on sand |
| hazard scanner | 21 rays over 100°, 5 m | rocks above climb limit and slopes above the safe limit |
| power/thermal | SOC, solar W, load W, box and air temperature | box temp 0.2 °C |

## 8. Comms and science

Relay passes happen at fixed local times each sol with a data rate. Data
products (40 Mbit per measurement) wait in onboard storage until downlinked
during a pass; a measurement scores only if it reaches Earth (downlink time +
one-way light time) before the mission ends. Uplinked messages from the ground
travel one light-time and are delivered at the next pass.

Each target has a true mineralogy and a value (silica 0.92, clay 0.70, sulfate
0.45, basalt 0.15 mean). Orbit reports a mineralogy that is wrong with a
mission-set probability. The spectrometer needs the rover parked within 1.5 m of
the target for 240 s.

## 9. Known limitations

* Planar (2.5D) vehicle model: no suspension articulation, no wheel lift-off
  dynamics beyond load transfer, no lateral (downhill) side-slip.
* Quasi-static wheel–soil model with no memory: no ruts, no multi-pass effects,
  no bulldozing resistance, no grouser-specific shear model.
* Single-node thermal model; no conduction paths between components.
* Soil parameters for drift, crust and bedrock are illustrative.
* Sensors are measurement models, not rendered images.
