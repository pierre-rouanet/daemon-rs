# daemon-rs — Open Issues

Reviewed 2026-04-15. Issues #1–8, #10, #12, #13 and #14 are resolved. The rest are pending.

---

## Issue #5 — RESOLVED (2026-04-15)

Goto now uses `linear_pose_interpolation` (SLERP on rotation via
`nalgebra::UnitQuaternion::slerp`, lerp on translation) in a new
`src/interpolation.rs` module, matching Python's `linear_pose_interpolation`.

Also ported Python's four time-shaping techniques
(`linear`/`minjerk`/`ease_in_out`/`cartoon`) as `InterpolationTechnique`. The
`method` field on `GotoTarget` / `GotoTaskRequest` is now live (was dead before)
and defaults to `minjerk`, matching the Python daemon. The REST `move_goto`
endpoint also accepts a `method` key in its JSON body.

---

## Issue #6 — RESOLVED (2026-04-15)

REST `/api/daemon/status` and WS `daemon_status` broadcast now share a single
`DaemonStatus` struct built by `RobotCore::build_daemon_status`. REST returns
real `wlan_ip`; adding a field = editing one struct.

---

## Issue #7 — RESOLVED (2026-04-15)

REST `move_goto` now calls `goto_as_task` (made `pub` on `RobotCore`), which
mints the UUID, tracks completion, and broadcasts `TaskProgress` — matching
the WS task behaviour and the Python daemon.

---

## Issue #8 — RESOLVED (2026-04-15)

`SerialMotors` now holds its own `read_or_last_known` method that returns
`self.last_known` on serial-read failure. Removed the free-function
`read_or_default` which returned zeros.

---

## Issue #9: No graceful shutdown of background tasks

**File:** `src/main.rs:94-110`, `src/robot.rs` (sensor_task, daemon_status_task, motor_loop)

**Problem:** On Ctrl+C, the pipeline is set to Null but the spawned tasks
(sensor_task, daemon_status_task, motor_loop thread) are not explicitly
stopped. They rely on process exit to clean up. The motor_loop thread in
particular holds the serial port — if it's mid-write when the process exits,
the controller may be left in an inconsistent state.

**Fix:** Use a `CancellationToken` (from `tokio_util`) or a shutdown channel.
On Ctrl+C: signal the token, await task handles, disable torque, then exit.

---

## Issue #10 — RESOLVED (2026-04-15)

`GET /api/state/full` now takes the same 10 query params as the Python
daemon (`with_control_mode`, `with_head_pose`, `with_target_head_pose`,
`with_head_joints`, `with_target_head_joints`, `with_body_yaw`,
`with_target_body_yaw`, `with_antenna_positions`,
`with_target_antenna_positions`, `use_pose_matrix`) and emits a dynamic
JSON object with the selected fields + a `timestamp`.

`WS /api/state/ws/full` streams the same payload at a configurable
`frequency` (Hz, default 10, clamped to [0.1, 200]). `GET
/api/state/present_head_pose` also now honors `use_pose_matrix`.

Poses are serialized as either `{m: [16 floats]}` or
`{x, y, z, roll, pitch, yaw}` — the latter using scipy's extrinsic
"xyz" Euler convention so the wire format matches Python exactly.

`RobotStateSnapshot` gained `head_joints`, `target_head_pose`, and
`target_body_yaw` to feed the new flags. `passive_joints` and `doa` are
intentionally left out — the Rust daemon has no data source for them
(Python gets `doa` from the mic array via `reachy_mini_audio`).

---

## Issue #11: Stubs that should be called out

Several features are accepted but no-op. This is fine for now but should be
tracked so they don't silently stay stubs forever:

- **`play_sound`** — logs filename, returns OK. Needs GStreamer `playbin` or
  equivalent to actually play audio files.
- **`set_automatic_body_yaw`** — accepted, no effect. The Python daemon
  implements head-tracking-driven body yaw.
- **`start_recording` / `stop_recording` / `append_record`** — toggles a bool,
  no actual data recording. Python records to JSON files.
- **`play_move`** (task) — immediately reports success. Python replays recorded
  move files.

---

## Issue #12 — RESOLVED (2026-04-15)

`RobotTarget.head` and `head_joints` are now private, with access only
through `head()` / `head_joints()` getters and the invariant-enforcing
`set_head(kin, ...)` / `set_joints(kin, ...)` methods — either setter runs
the FK/IK pass that keeps the two representations in sync. It's a compile
error for outside code to mutate one without going through the pair.

The two free helpers `set_head_and_derive_joints` / `set_joints_and_derive_head`
are gone. All internal call sites (`set_target`, `set_head_joints`, `run_goto`,
`apply_target`, `sync_target_from_positions`, `push_to_motors`, `get_state`,
`sensor_task`) route through the new API. `State` picked up two thin
`set_head` / `set_joints` helpers for call sites that hold a `MutexGuard`
(the guard's `Deref` blocks Rust's field-split borrow checker — a plain
`&mut self` on `State` lets the split through).

Kept `antennas` and `body_yaw` as `pub` on `RobotTarget`: they're
independent scalars with no invariant across them.

---

## Issue #13 — RESOLVED (2026-04-15)

`RealImu` added in `src/devices/imu.rs`, wrapping the `Bmi088` driver from
the workspace `gstreamer_data_capture/imu` crate (Madgwick-fused
orientation, 100 Hz sample rate, ±3 g / ±500 °/s ranges). Dep is
Linux-gated (`[target.'cfg(target_os = "linux")']`) since the driver
uses `i2cdev::linux`; non-Linux hosts still compile with `MockImu`.

`main.rs` now boots `RealImu::new(1)` (i.e. `/dev/i2c-1`, the Pi
default) and falls back to `MockImu` if I2C init fails or if
`REACHYMINI_IMU=none` is set — mirroring the serial-motors selection.
On read failure, `RealImu` holds the last-known sample (same pattern as
Issue #8).

Note: the ISSUES.md text said BNO055; the actual chip on the wireless
Reachy Mini is a BMI088 (what the existing driver targets).

---

## Issue #14 — RESOLVED (2026-04-15)

`motor_loop` now records each read-tick timestamp into a rolling 1-second
window (`STATS_WINDOW`, matching Python's `_stats_record_period`). At the
end of each window, `compute_stats` reduces them to a `ControlLoopStats`
with `mean_control_loop_frequency`, `max_control_loop_interval`, and
`nb_error` — the same keys Python publishes — and `send_replace`s them
onto a `watch::Sender<ControlLoopStats>` on `RobotCore`.

`build_daemon_status` reads the latest snapshot (non-blocking
`stats_tx.borrow()`) and emits the three fields under
`backend_status.control_loop_stats`. Both the 1 Hz WS broadcast and the
REST `/api/daemon/status` endpoint pick this up for free (they share one
builder, per Issue #6).

`Motors` trait gained `fn error_count(&self) -> u64` with a default of 0;
`SerialMotors` overrides it with a counter that bumps on failed
`read_all_positions` / `set_all_goal_positions` (incl. the startup read
being zeroed out so it doesn't inflate steady-state numbers).

Skipped Python's fourth `motor_controller` key — that dumps the C++
controller's internal stats via `get_stats()`, which
`reachy_mini_motor_controller` doesn't expose. Not worth plumbing through
for a free-form debug string.

---

## Issue #15: No media lifecycle management

**Files:** `src/media.rs`, `src/api.rs:167-178`

**Problem:** The media pipeline is started once at boot and never stopped.
The Python daemon supports releasing/reacquiring the camera (important when
another process needs it). The REST `/api/media/status` is hardcoded
(`available: true, released: false`), and there's no way to release/reacquire.

**Fix:** Add pipeline state management (Null ↔ Playing transitions), wire
up to REST endpoints or a `release_media` / `acquire_media` command.
