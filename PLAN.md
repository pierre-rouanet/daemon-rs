# daemon-rs — Base Plan

Rust rewrite of the Reachy Mini daemon, targeting the **wireless**
(Pi Zero 2W + CSI `imx708` + USB audio) configuration only. Note: the
production "wireless" Reachy Mini uses a CM4, but the point of this
prototype is to see whether the Rust daemon holds up on the tighter
Pi Zero 2W (448M ARM / 64M GPU). If it does, CM4 is trivially fine.

**Status: prototype.** The point of this project is to see whether a Rust
daemon meaningfully improves **behavior under load** (many concurrent API
calls, heavy multithreading) and **code stability / maintainability**,
compared with the Python daemon. Scope grows only as results justify it.

## Goals

- Mirror the behavior of the `reachy-mini` wireless daemon in Rust.
- Better concurrency behavior under load — stable latency when many
  clients hammer the control API at once, clean shutdown, predictable
  resource use.
- More robust / maintainable code than the Python daemon: typed end-to-end,
  single binary, no interpreter, fewer moving parts.
- Keep the on-device surface small: **one binary, always used remotely**.

We are explicitly **not** trying to make the media pipeline itself faster.
GStreamer does the heavy lifting and the Python daemon already sits on top
of the same native stack; expected media perf delta is negligible.

### Non-goals for this phase

- **Wire-protocol compatibility with existing Python / JS SDK clients** is
  deferred to a later phase. For the prototype we ship a minimum endpoint
  set whose shape doesn't have to match the Python daemon. Compatibility
  (matching paths / JSON / WS shapes from `reachy_mini/scripts/generate_openapi.py`)
  gets added when we commit to a full replacement.

## Non-goals (explicit simplifications vs. `reachy-mini`)

1. **No Lite / USB-camera support.** CSI (`libcamerasrc` + `imx708` via
   `v4l2h264enc`) only. No `v4l2src` / `jpegdec` / `v4l2jpegdec` paths.
2. **No local-access path.** Drop the IPC branch (`unixfdsink` /
   `win32ipcvideosink` at `CAMERA_SOCKET_PATH`). No tee. The daemon's camera
   stream is **only** consumed by remote clients over WebRTC.
3. **No simulation / UDP source.** No `use_sim` path.
4. **No on-device apps.** The daemon does not host or execute user code
   locally. No app store, no HF auth flow, no local Python SDK. All
   interactions are remote.
5. **No web dashboard.** The Python daemon's `daemon/app/dashboard/` is
   deprecated; we don't re-implement it.

## Target pipeline (media)

```
libcamerasrc
  ! video/x-raw,format=YUY2,W,H,F/1,colorimetry=bt709,interlace-mode=progressive
  ! v4l2h264enc (extra-controls: repeat_sps=1, bitrate=5 Mb/s, gop=256, I-frame=60)
  ! video/x-h264,stream-format=byte-stream,alignment=au,profile=constrained-baseline,level=3.1
  ! webrtcsink (run-signalling-server=true, meta={name:reachymini})

alsasrc device=<USB-Audio-card>
  ! webrtcsink
```

Same structure as `reachy_mini/media/media_server.py:_build_libcamera_source`
+ `_build_rpi_encoder_branch`, minus the tee / IPC branch.

## Components to build

### Phase 1 — Media daemon (current)

- [x] GStreamer pipeline: `libcamerasrc` → `v4l2h264enc` → `webrtcsink`.
- [x] Audio branch: `alsasrc` (USB Audio card by name) → `webrtcsink`.
- [x] Built-in signalling server (`run-signalling-server=true`).
- [ ] Validate end-to-end on CM4 + CSI (pending hardware).

### Phase 2 — Minimum control API

The smallest endpoint set that covers ~95% of real robot-control use:

- **Audio / video** — already handled by `webrtcsink`'s signalling.
- **`SetFullTargetCmd`** — single WebSocket message, atomic write of any
  subset of `{head: [f64], antennas: [f64], body_yaw: f64}`. Stateless.
  Schema: `reachy_mini/src/reachy_mini/io/protocol.py:133`.
- **`GotoTargetCmd`** — smooth interpolated goto with the same
  `{head, antennas, body_yaw}` fields plus `duration: f64`. Required by
  `wake_up` / `goto_sleep` flows, so we can't skip it.
  Schema: `reachy_mini/src/reachy_mini/io/protocol.py:147`.
  Implementation: a small interpolation task driving repeated
  `SetFullTargetCmd`-equivalent writes until duration elapses.

  **Concurrency policy:** while a `GotoTargetCmd` is in flight, any
  incoming motor command is **rejected** (error back to the sender) —
  uniformly, no preemption. That includes `SetFullTargetCmd`, a second
  `GotoTargetCmd`, and the `WakeUpCmd` / `GotoSleepCmd` wrappers. The
  Python daemon has no rule and just races; we're deliberately stricter
  in Rust because rejecting is simpler to reason about and easier to
  implement race-free. Error-response shape TBD when we wire up the WS
  layer.
- **`WakeUpCmd` / `GotoSleepCmd`** — thin wrappers over `GotoTargetCmd`
  with hardcoded targets (matches the Python behavior).
- **Sensor reads** (motor state, IMU) exposed via the same WS channel so
  a dashboard / rerun viewer can consume them. Exact read message shape
  TBD; we'll pick the Python equivalent once we wire it up.

That's the prototype surface. No telemetry-stream management, no config
endpoints, no SDK parity beyond these messages. If the test workload
needs more, we add one message at a time.

Prototype coexistence is **not** a concern: one robot runs the Python
daemon, another robot runs the Rust daemon. No need to mix on one host.

Framework candidate: `axum` (tokio-native, matches the async ergonomics we
need for WS endpoints). Not locked in.

### Phase 2b — Wire-compat with Python daemon (later)

If / when the prototype proves its value and we commit to replacing the
Python daemon, we expand the Rust API to match the Python daemon's
OpenAPI schema exactly (paths / JSON / WS message shapes). Schema source
of truth: `reachy_mini/scripts/generate_openapi.py`.

Not a Phase 2 concern.

### Phase 3 — Hardware I/O

Wherever possible, **reuse the existing Rust crates** rather than re-porting:

- **Motors** — Dynamixel via
  [`pollen-robotics/reachy-mini-motor-controller`](https://github.com/pollen-robotics/reachy-mini-motor-controller)
  (built on `rustypot`). Drop-in.
- **Kinematics** —
  [`pollen-robotics/reachy_mini_rust_kinematics`](https://github.com/pollen-robotics/reachy_mini_rust_kinematics).
  Drop-in.
- **IMU** — BMI088 over I2C. Reuse the existing `imu` crate from
  [`pollen-robotics/gstreamer_data_capture`](https://github.com/pollen-robotics/gstreamer_data_capture/blob/main/imu/src/lib.rs):
  self-contained BMI088 driver with Madgwick AHRS, exposes `Bmi088::new(i2c_bus)`
  and `read_all() -> ImuReading { accelerometer, gyroscope, quaternion, temperature }`.
  Drop-in; we only wire it to the control API.

## Architectural direction (guide, not spec)

North-star shape we *lean toward* — not something we implement up-front:

- Two conceptual layers:
  - **Transport / protocol** — HTTP, WebSocket, WebRTC signalling +
    media muxing. Knows nothing about specific hardware.
  - **Devices** — motors, IMU, camera, microphone, speaker. Each is a
    self-contained module exposing a small Rust-level API; knows nothing
    about how its data reaches a client.
- Long-term: a declarative config ("I have an imx708, a ReSpeaker, these
  motors, this IMU — stream all that") wires the devices to the transport
  layer.

**Reusability happens at the crate level, not at the daemon level.**
We're not building a framework for other robots. We *are* picking /
contributing to small, focused crates (motor controller, IMU driver,
kinematics are already Rust crates) so that a future robot can pull the
same building blocks. The daemon itself stays Reachy-Mini-specific until
there's a concrete second user.

### Practical rules while we're still prototyping

- **Don't introduce an abstraction until there are two concrete
  implementations** that would share it. A single-camera daemon with
  `libcamerasrc` hardcoded is fine — we'll extract the trait the day we
  add a second source.
- GStreamer + `webrtcsink` deliberately entangle video/audio capture with
  the transport layer. **Don't fight that.** Treat the GStreamer pipeline
  as one leaf of the tree, not something to cleanly abstract around.
- Motors / IMU are easy wins for the split because they're pure-Rust and
  already have narrow APIs. Put each behind a small module in its own
  file, with a constructor that takes its hardware handle (I2C bus
  number, serial port path, etc.) — that alone makes the config-file
  direction cheap to pursue later.
- Config file (TOML) is probably the right end state, but we start with
  CLI flags / env vars. Introduce the file when we have ≥2 hardware
  combos to switch between.

## Architecture

```
┌─────────────────── Pi (CM4) ───────────────────┐     ┌─ Remote computer ─┐
│                                                │     │                   │
│  daemon-rs  ──────────┬── webrtcsink ──────────┼─────┼→ WebRTC consumer  │
│    │                  │   (signalling :8443)   │     │                   │
│    │                  └── HTTP/WS API :PORT ───┼─────┼→ Control client   │
│    │                                           │     │                   │
│    ├── libcamerasrc (CSI imx708)               │     │                   │
│    ├── alsasrc (USB Audio)                     │     │                   │
│    ├── motors (serial)                         │     │                   │
│    └── IMU (I2C)                               │     │                   │
└────────────────────────────────────────────────┘     └───────────────────┘
```

## Load model (what "under load" means here)

Concrete scenario the prototype has to handle gracefully:

- **One app** sending `SetFullTargetCmd` at a reasonable rate (tens of Hz)
  and occasionally reading sensors.
- **One dashboard** subscribing to all sensor streams (motors + IMU +
  camera + audio).
- **One Rerun viewer** doing the same — logging motors, IMU, camera,
  audio at their native rates.

≈ 3 concurrent clients, mix of high-frequency reads + sporadic writes.
That's what the Rust concurrency model is supposed to handle better than
the Python daemon; if it doesn't, the whole premise is wrong.

## Open questions

- **Exact shape of sensor-read messages** (motor positions, IMU) in the
  prototype — port the Python WS messages as-is, or pick a minimal set?
- **Sample-rate decisions** for sensor publishing (motors at 100 Hz? IMU
  at 100 Hz? camera at native rate via WebRTC?).
- **Load-test harness** — how do we actually drive the 3-client scenario
  to produce numbers we can compare against Python?
- Whether we keep `webrtcsink`'s built-in signalling server or externalize it.
- Config format (env vars only, TOML file, CLI flags?).
- Deployment: cross-compile + scp, or OS image bake-in?

## Reference

- `~/Dev/mini/reachy_mini/src/reachy_mini/media/media_server.py` — Python
  media pipeline we're mirroring.
- `~/Dev/mini/reachy_mini/src/reachy_mini/daemon/app/` — Python control
  server whose scope we'll trim.
- `~/Dev/mini/reachy_mini/scripts/generate_openapi.py` — source of truth
  for the REST/WS schemas the Rust daemon must match.
- [`pollen-robotics/reachy-mini-motor-controller`](https://github.com/pollen-robotics/reachy-mini-motor-controller)
  — existing Rust crate for Dynamixel motor control (via `rustypot`).
- [`pollen-robotics/reachy_mini_rust_kinematics`](https://github.com/pollen-robotics/reachy_mini_rust_kinematics)
  — existing Rust kinematics crate.
- `pollen-robotics/gstreamer_data_capture`
  — pipeline patterns for the media side;
  [`imu/src/lib.rs`](https://github.com/pollen-robotics/gstreamer_data_capture/blob/main/imu/src/lib.rs)
  is the BMI088 driver we'll reuse.
