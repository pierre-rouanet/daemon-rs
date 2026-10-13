//! Motor device trait + implementations (mock and serial).
//!
//! The trait intentionally speaks in robot-semantic units (body yaw,
//! stewart angles, antenna angles) rather than raw motor IDs. Maps 1:1
//! onto `reachy-mini-motor-controller` position arrays so the serial
//! implementation is a thin adapter.

/// Full-robot motor snapshot. 6 Stewart + 2 antennas + body yaw = 9 DOF.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct MotorPositions {
    pub stewart: [f64; 6],
    pub antennas: [f64; 2],
    pub body_yaw: f64,
}

/// Commanded motor targets. Same shape as `MotorPositions` — the
/// distinction is intent (write) vs observation (read).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct MotorTargets {
    pub stewart: [f64; 6],
    pub antennas: [f64; 2],
    pub body_yaw: f64,
}

/// Abstract motor device.
///
/// `Send` bound: the daemon lives behind a `tokio::sync::Mutex`, which
/// requires its contents to be `Send` to cross `.await` points.
pub trait Motors: Send {
    /// Current observed positions. On the mock this is whatever was
    /// last written. On real hardware this comes off the control loop's
    /// last-known-good cache.
    fn read_positions(&mut self) -> MotorPositions;

    /// Write commanded targets. Fire-and-forget; the real driver
    /// pushes to an internal mpsc and returns immediately.
    fn set_targets(&mut self, cmd: MotorTargets);

    /// Enable torque on all motors.
    fn enable_torque(&mut self);

    /// Disable torque on all motors.
    fn disable_torque(&mut self);

    /// Cumulative count of bus errors (failed reads or writes) since
    /// construction. Feeds `control_loop_stats.nb_error` in the daemon
    /// status. Default is 0 — only the serial driver counts.
    fn error_count(&self) -> u64 {
        0
    }
}

/// In-memory mock: round-trips targets as positions verbatim. Lets us
/// unit-test the daemon's command / sensor pipeline without a robot.
#[derive(Debug, Clone, Default)]
pub struct MockMotors {
    state: MotorPositions,
}

impl MockMotors {
    pub fn new() -> Self {
        Self::default()
    }
}

impl Motors for MockMotors {
    fn read_positions(&mut self) -> MotorPositions {
        self.state
    }

    fn set_targets(&mut self, cmd: MotorTargets) {
        self.state = MotorPositions {
            stewart: cmd.stewart,
            antennas: cmd.antennas,
            body_yaw: cmd.body_yaw,
        };
    }

    fn enable_torque(&mut self) {}
    fn disable_torque(&mut self) {}
}

// ---------------------------------------------------------------------------
// Serial motor driver (wraps `ReachyMiniMotorController`)
// ---------------------------------------------------------------------------

use reachy_mini_motor_controller::ReachyMiniMotorController;
use tracing::warn;

/// Real motor driver over a serial port (USB or GPIO UART).
///
/// Position array layout from the controller crate:
///   `[body_rotation, stewart_1..6, antenna_right, antenna_left]` — 9 floats.
///
/// Errors on the serial bus are logged and silently absorbed: `set_targets`
/// is fire-and-forget, and `read_positions` falls back to the last known
/// good reading.
pub struct SerialMotors {
    controller: ReachyMiniMotorController,
    last_known: MotorPositions,
    /// Cumulative read/write error count — surfaced via `error_count()`
    /// and rolled into `control_loop_stats.nb_error` (Issue #14).
    errors: u64,
}

impl SerialMotors {
    /// Open the serial port, enable torque on all motors, and read the
    /// initial position snapshot.
    pub fn new(port: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let mut controller = ReachyMiniMotorController::new(port)?;
        controller.enable_torque()?;
        // No prior reading yet — use zeros as the initial "last known".
        // The first successful read below will overwrite it immediately.
        let mut s = Self {
            controller,
            last_known: MotorPositions::default(),
            errors: 0,
        };
        s.last_known = s.read_positions();
        // Don't let the startup read inflate the error counter — that's
        // a different class of event (init vs steady-state).
        s.errors = 0;
        Ok(s)
    }

    /// Read positions from the bus, falling back to the last successful
    /// reading on a serial error (Issue #8 — returning zeros caused a
    /// one-frame jump to the origin pose whenever a read glitched).
    fn read_or_last_known(&mut self) -> MotorPositions {
        match self.controller.read_all_positions() {
            Ok(p) => MotorPositions {
                body_yaw: p[0],
                stewart: [p[1], p[2], p[3], p[4], p[5], p[6]],
                antennas: [p[7], p[8]],
            },
            Err(e) => {
                self.errors = self.errors.saturating_add(1);
                warn!("serial read_positions: {e} (holding last known)");
                self.last_known
            }
        }
    }
}

impl Motors for SerialMotors {
    fn read_positions(&mut self) -> MotorPositions {
        let pos = self.read_or_last_known();
        self.last_known = pos;
        pos
    }

    fn set_targets(&mut self, cmd: MotorTargets) {
        // Controller order: [body_yaw, stewart_1..6, antenna_right, antenna_left]
        let positions = [
            cmd.body_yaw,
            cmd.stewart[0],
            cmd.stewart[1],
            cmd.stewart[2],
            cmd.stewart[3],
            cmd.stewart[4],
            cmd.stewart[5],
            cmd.antennas[0],
            cmd.antennas[1],
        ];
        if let Err(e) = self.controller.set_all_goal_positions(positions) {
            self.errors = self.errors.saturating_add(1);
            warn!("serial set_targets: {e}");
        }
    }

    fn enable_torque(&mut self) {
        if let Err(e) = self.controller.enable_torque() {
            warn!("serial enable_torque: {e}");
        }
    }

    fn disable_torque(&mut self) {
        if let Err(e) = self.controller.disable_torque() {
            warn!("serial disable_torque: {e}");
        }
    }

    fn error_count(&self) -> u64 {
        self.errors
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mock_round_trips() {
        let mut m = MockMotors::new();
        assert_eq!(m.read_positions(), MotorPositions::default());
        let tgt = MotorTargets {
            stewart: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
            antennas: [-0.2, 0.2],
            body_yaw: 0.7,
        };
        m.set_targets(tgt);
        let got = m.read_positions();
        assert_eq!(got.stewart, tgt.stewart);
        assert_eq!(got.antennas, tgt.antennas);
        assert_eq!(got.body_yaw, tgt.body_yaw);
    }
}
