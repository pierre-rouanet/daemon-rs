//! Robot orchestration: owns devices (motors, IMU), kinematics, and the
//! current commanded target; dispatches WS commands onto them and drives
//! the sensor broadcast.
//!
//! Wire-compatible with the Python daemon's protocol. All broadcast
//! messages (`joint_positions`, `head_pose`, `imu_data`, `daemon_status`,
//! `task_progress`) go through a single `broadcast::Sender<ServerMsg>`.
//!
//! Per-client responses (`get_state`, `get_motor_mode`, command acks)
//! are returned as JSON strings from the dispatch function — the caller
//! (WS handler or data channel handler) decides how to send them back.

use std::sync::Arc;
use std::time::{Duration, Instant};

use reachy_mini_rust_kinematics::Kinematics;
use tokio::sync::{Mutex, broadcast, watch};
use tokio::time;

use crate::devices::{Imu, MotorPositions, MotorTargets, Motors};
use crate::interpolation::{InterpolationTechnique, linear_pose_interpolation, time_trajectory};
use crate::kinematics::{head_wire_to_pose, pose_to_head_wire};
use crate::protocol::{
    AnyMessage, DaemonStatus, GotoTarget, GotoTaskRequest, ServerMsg, SetFullTarget, TaskRequestInner,
    head_to_nested,
};

/// Publish rate for sensor broadcasts (100 Hz).
const SENSOR_HZ: u64 = 100;

/// Number of interpolation steps per second during a goto.
const GOTO_HZ: u64 = 50;

/// Capacity of the sensor broadcast channel.
const SENSOR_CHAN_CAPACITY: usize = 128;

/// Window over which motor_loop recomputes control-loop stats (1 s — matches
/// Python's `_stats_record_period`).
const STATS_WINDOW: Duration = Duration::from_secs(1);

/// Stats for the motor I/O loop, surfaced in `daemon_status.backend_status
/// .control_loop_stats` and kept in sync with Python's key names.
///
/// `Default` = all zeros, which is what gets reported for the first second
/// after boot (before enough ticks have accumulated to compute an interval).
#[derive(Debug, Clone, Copy, Default)]
pub struct ControlLoopStats {
    /// Mean control-loop frequency (Hz) over the last STATS_WINDOW.
    pub mean_control_loop_frequency: f64,
    /// Maximum observed tick-to-tick interval (seconds) over the window —
    /// captures jitter / stalls that the mean smooths out.
    pub max_control_loop_interval: f64,
    /// Cumulative serial-bus errors since boot (not per-window — matches
    /// the semantics of the `errors` counter on `SerialMotors`).
    pub nb_error: u64,
}

/// Current commanded target.
///
/// `head` (4x4 matrix) and `head_joints` (6 stewart angles) are redundant
/// representations of the same pose — one FK/IK call derives either from the
/// other. Keeping them private + going through `set_head` / `set_joints` is
/// how we enforce the invariant that they never drift (Issue #12). Field
/// access is limited to this module so every write routes through the
/// kinematics-aware setters.
///
/// `antennas` and `body_yaw` are independent scalars with no invariant, so
/// they stay `pub` for direct mutation.
#[derive(Debug, Clone)]
pub struct RobotTarget {
    head: [f64; 16],
    head_joints: [f64; 6],
    pub antennas: [f64; 2],
    pub body_yaw: f64,
}

impl Default for RobotTarget {
    fn default() -> Self {
        let mut head = [0.0; 16];
        head[0] = 1.0;
        head[5] = 1.0;
        head[10] = 1.0;
        head[15] = 1.0;
        Self {
            head,
            head_joints: [0.0; 6],
            antennas: [0.0, 0.0],
            body_yaw: 0.0,
        }
    }
}

impl RobotTarget {
    /// Current commanded head pose (4x4, row-major).
    #[inline]
    pub fn head(&self) -> &[f64; 16] {
        &self.head
    }

    /// Current commanded stewart-platform joints (6 floats).
    #[inline]
    pub fn head_joints(&self) -> &[f64; 6] {
        &self.head_joints
    }

    /// Set head pose and derive joints via IK. Keeps the two representations
    /// in sync — the only public way to write `head`.
    pub fn set_head(&mut self, kin: &mut Kinematics, head: &[f64; 16]) {
        self.head.copy_from_slice(head);
        let pose = head_wire_to_pose(head);
        let angles = kin.inverse_kinematics(pose, Some(self.body_yaw));
        if angles.len() == 6 {
            for i in 0..6 {
                self.head_joints[i] = angles[i];
            }
        }
    }

    /// Set joints and derive head pose via FK. Keeps the two representations
    /// in sync — the only public way to write `head_joints`.
    pub fn set_joints(&mut self, kin: &mut Kinematics, joints: &[f64; 6]) {
        self.head_joints.copy_from_slice(joints);
        kin.reset_forward_kinematics(head_wire_to_pose(&self.head));
        let pose = kin.forward_kinematics(joints.to_vec(), Some(self.body_yaw));
        self.head = pose_to_head_wire(&pose);
    }
}

struct State {
    target: RobotTarget,
    goto_active: bool,
    kinematics: Kinematics,
    motor_mode: String,
    is_recording: bool,
}

impl State {
    /// Thin wrapper that hands disjoint field borrows to `RobotTarget::set_head`.
    /// Callers usually hold a `MutexGuard<State>`, through which Rust can't
    /// split borrows — going through `&mut self` here does.
    #[inline]
    fn set_head(&mut self, head: &[f64; 16]) {
        self.target.set_head(&mut self.kinematics, head);
    }

    /// See [`State::set_head`] — same split-borrow trick for `set_joints`.
    #[inline]
    fn set_joints(&mut self, joints: &[f64; 6]) {
        self.target.set_joints(&mut self.kinematics, joints);
    }
}

/// Commands sent to the motor I/O thread.
enum MotorCommand {
    EnableTorque,
    DisableTorque,
}

/// Reason a command was rejected.
#[derive(Debug, Clone, thiserror::Error)]
pub enum RobotError {
    #[error("goto in flight; command rejected")]
    GotoInFlight,
    #[error("bad field length ({field}: expected {expected}, got {got})")]
    BadLength {
        field: &'static str,
        expected: usize,
        got: usize,
    },
    #[error("`head` and `head_joints` cannot both be set in one command")]
    HeadConflict,
}

/// Snapshot of robot state for `get_state` responses.
/// Matches Python daemon's format, plus target_* fields used by clients
/// to measure tracking latency without extending the 100 Hz broadcast.
#[derive(Debug, Clone, serde::Serialize)]
pub struct RobotStateSnapshot {
    /// Present head pose (4x4 nested), from FK on motor readback.
    pub head_pose: Vec<Vec<f64>>,
    /// Present head joints: [body_yaw, stewart_0..5] (7 floats, rad) from readback.
    pub head_joints: Vec<f64>,
    /// Present antenna positions [right, left] (rad).
    pub antennas: Vec<f64>,
    /// Present body yaw (rad).
    pub body_yaw: f64,
    /// Commanded head pose (4x4 nested).
    pub target_head_pose: Vec<Vec<f64>>,
    /// Latest commanded head joints: [body_yaw, stewart_0..5] (7 floats, rad).
    pub target_head_joint_positions: Vec<f64>,
    /// Latest commanded antenna targets [right, left] (rad).
    pub target_antennas_joint_positions: Vec<f64>,
    /// Latest commanded body yaw (rad).
    pub target_body_yaw: f64,
    pub motor_mode: String,
    pub is_recording: bool,
    pub is_move_running: bool,
}

pub struct RobotCore {
    state: Mutex<State>,
    imu: Mutex<Box<dyn Imu>>,
    sensor_tx: broadcast::Sender<ServerMsg>,
    /// Latest commanded target → motor thread picks this up each cycle.
    target_tx: watch::Sender<MotorTargets>,
    /// Latest motor readback — written by motor thread, read by sensor_task / get_state.
    positions_tx: watch::Sender<MotorPositions>,
    /// Torque on/off commands → motor thread.
    motor_cmd_tx: std::sync::mpsc::Sender<MotorCommand>,
    /// Wake signal — nudges the motor thread to write immediately.
    nudge_tx: std::sync::mpsc::Sender<()>,
    /// Rolling control-loop stats, recomputed each STATS_WINDOW by motor_loop.
    /// Read (non-blocking) by `build_daemon_status` to fill
    /// `backend_status.control_loop_stats`. Watch channel instead of Mutex
    /// so the reader never blocks the motor thread.
    stats_tx: watch::Sender<ControlLoopStats>,
}

impl RobotCore {
    pub fn spawn(
        motors: Box<dyn Motors>,
        imu: Box<dyn Imu>,
        kinematics: Kinematics,
    ) -> Arc<Self> {
        let (sensor_tx, _) = broadcast::channel(SENSOR_CHAN_CAPACITY);
        let (target_tx, _) = watch::channel(MotorTargets::default());
        let (positions_tx, _) = watch::channel(MotorPositions::default());
        let (motor_cmd_tx, motor_cmd_rx) = std::sync::mpsc::channel();
        let (nudge_tx, nudge_rx) = std::sync::mpsc::channel();
        let (stats_tx, _) = watch::channel(ControlLoopStats::default());
        let core = Arc::new(Self {
            state: Mutex::new(State {
                target: RobotTarget::default(),
                goto_active: false,
                kinematics,
                motor_mode: "enabled".to_string(),
                is_recording: false,
            }),
            imu: Mutex::new(imu),
            sensor_tx,
            target_tx,
            positions_tx,
            motor_cmd_tx,
            nudge_tx,
            stats_tx,
        });
        // Motor I/O on a dedicated std::thread — keeps blocking serial
        // reads off the tokio runtime so commands never stall behind I/O.
        let core_for_motor = core.clone();
        std::thread::Builder::new()
            .name("motor-io".into())
            .spawn(move || motor_loop(motors, core_for_motor, motor_cmd_rx, nudge_rx))
            .expect("spawn motor-io thread");
        tokio::spawn(sensor_task(core.clone()));
        tokio::spawn(daemon_status_task(core.clone()));
        core
    }

    pub fn subscribe(&self) -> broadcast::Receiver<ServerMsg> {
        self.sensor_tx.subscribe()
    }

    /// Broadcast a message to all clients (used by task completion, etc).
    pub fn broadcast(&self, msg: ServerMsg) {
        let _ = self.sensor_tx.send(msg);
    }

    // -----------------------------------------------------------------
    // State queries
    // -----------------------------------------------------------------

    pub async fn get_state(&self) -> RobotStateSnapshot {
        let positions = *self.positions_tx.borrow();
        let target = *self.target_tx.borrow();
        let mut s = self.state.lock().await;
        let last_head = *s.target.head();
        let target_head = *s.target.head();
        let present_pose = compute_head_pose(&mut s.kinematics, &positions.stewart, positions.body_yaw, &last_head);

        let mut head_joints = Vec::with_capacity(7);
        head_joints.push(positions.body_yaw);
        head_joints.extend_from_slice(&positions.stewart);

        let mut target_head_joint_positions = Vec::with_capacity(7);
        target_head_joint_positions.push(target.body_yaw);
        target_head_joint_positions.extend_from_slice(&target.stewart);

        RobotStateSnapshot {
            head_pose: head_to_nested(&present_pose),
            head_joints,
            antennas: positions.antennas.to_vec(),
            body_yaw: positions.body_yaw,
            target_head_pose: head_to_nested(&target_head),
            target_head_joint_positions,
            target_antennas_joint_positions: target.antennas.to_vec(),
            target_body_yaw: target.body_yaw,
            motor_mode: s.motor_mode.clone(),
            is_recording: s.is_recording,
            is_move_running: s.goto_active,
        }
    }

    pub async fn get_motor_mode(&self) -> String {
        self.state.lock().await.motor_mode.clone()
    }

    /// Build the daemon status payload — shared by the WS 1 Hz broadcast
    /// (`daemon_status_task`) and the REST `GET /api/daemon/status` handler,
    /// so the two never drift (notably `wlan_ip` — REST used to hardcode null).
    pub async fn build_daemon_status(&self) -> DaemonStatus {
        let motor_mode = self.get_motor_mode().await;
        let stats = *self.stats_tx.borrow();
        DaemonStatus {
            robot_name: "reachy_mini".to_string(),
            state: "running".to_string(),
            wireless_version: true,
            desktop_app_daemon: false,
            simulation_enabled: Some(false),
            mockup_sim_enabled: Some(false),
            no_media: false,
            media_released: false,
            camera_specs_name: "wireless".to_string(),
            backend_status: Some(serde_json::json!({
                "ready": true,
                "motor_control_mode": motor_mode,
                "last_alive": null,
                "control_loop_stats": {
                    "mean_control_loop_frequency": stats.mean_control_loop_frequency,
                    "max_control_loop_interval": stats.max_control_loop_interval,
                    "nb_error": stats.nb_error,
                },
                "error": null,
            })),
            error: None,
            wlan_ip: detect_local_ip(),
            version: Some("0.1.0".to_string()),
        }
    }

    // -----------------------------------------------------------------
    // Individual setters (Python protocol parity)
    // -----------------------------------------------------------------

    /// `set_target` — head pose only (4x4 flattened, 16 floats).
    pub async fn set_target(&self, head: &[f64]) -> Result<(), RobotError> {
        if head.len() != 16 {
            return Err(RobotError::BadLength {
                field: "head",
                expected: 16,
                got: head.len(),
            });
        }
        let mut s = self.state.lock().await;
        if s.goto_active {
            return Ok(()); // Python silently ignores during goto
        }
        let mut h = [0.0; 16];
        h.copy_from_slice(head);
        s.set_head(&h);
        self.push_to_motors(&s);
        Ok(())
    }

    /// `set_head_joints` — 7 floats: [body_yaw, stewart_0..5].
    pub async fn set_head_joints(&self, joints: &[f64]) -> Result<(), RobotError> {
        if joints.len() != 7 {
            return Err(RobotError::BadLength {
                field: "joints",
                expected: 7,
                got: joints.len(),
            });
        }
        let mut s = self.state.lock().await;
        if s.goto_active {
            return Ok(());
        }
        s.target.body_yaw = joints[0];
        let mut j = [0.0; 6];
        j.copy_from_slice(&joints[1..7]);
        s.set_joints(&j);
        self.push_to_motors(&s);
        Ok(())
    }

    /// `set_body_yaw`
    pub async fn set_body_yaw(&self, yaw: f64) {
        let mut s = self.state.lock().await;
        if s.goto_active {
            return;
        }
        s.target.body_yaw = yaw;
        self.push_to_motors(&s);
    }

    /// `set_antennas` — [right, left].
    pub async fn set_antennas(&self, antennas: &[f64]) -> Result<(), RobotError> {
        if antennas.len() != 2 {
            return Err(RobotError::BadLength {
                field: "antennas",
                expected: 2,
                got: antennas.len(),
            });
        }
        let mut s = self.state.lock().await;
        if s.goto_active {
            return Ok(());
        }
        s.target.antennas.copy_from_slice(antennas);
        self.push_to_motors(&s);
        Ok(())
    }

    // -----------------------------------------------------------------
    // Full target / goto (existing)
    // -----------------------------------------------------------------

    pub async fn set_full_target(&self, cmd: SetFullTarget) -> Result<(), RobotError> {
        let mut s = self.state.lock().await;
        if s.goto_active {
            return Err(RobotError::GotoInFlight);
        }
        apply_target(&mut s, &cmd)?;
        self.push_to_motors(&s);
        Ok(())
    }

    pub async fn start_goto(self: &Arc<Self>, cmd: GotoTarget) -> Result<(), RobotError> {
        validate_goto(&cmd)?;
        let start = {
            let mut s = self.state.lock().await;
            if s.goto_active {
                return Err(RobotError::GotoInFlight);
            }
            s.goto_active = true;
            s.target.clone()
        };
        tokio::spawn({
            let core = self.clone();
            async move {
                core.run_goto(start, cmd).await;
                core.state.lock().await.goto_active = false;
            }
        });
        Ok(())
    }

    /// Task-based goto: tracks UUID and broadcasts TaskProgress on completion.
    pub async fn start_goto_task(
        self: &Arc<Self>,
        uuid: String,
        req: GotoTaskRequest,
    ) -> Result<(), RobotError> {
        let goto = GotoTarget {
            head: req.head,
            head_joints: None,
            antennas: req.antennas,
            body_yaw: req.body_yaw,
            duration: req.duration,
            method: req.method,
        };
        validate_goto(&goto)?;
        let start = {
            let mut s = self.state.lock().await;
            if s.goto_active {
                return Err(RobotError::GotoInFlight);
            }
            s.goto_active = true;
            s.target.clone()
        };
        tokio::spawn({
            let core = self.clone();
            async move {
                core.run_goto(start, goto).await;
                core.state.lock().await.goto_active = false;
                core.broadcast(ServerMsg::TaskProgress {
                    uuid,
                    finished: true,
                    error: None,
                    timestamp: chrono::Utc::now().to_rfc3339(),
                });
            }
        });
        Ok(())
    }

    // -----------------------------------------------------------------
    // Wake up / goto sleep — matches Python daemon poses
    // -----------------------------------------------------------------

    /// Identity head pose (4x4 flattened row-major).
    const INIT_HEAD: [f64; 16] = [
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    ];
    /// Antennas at ±10 deg (≈ ±0.1745 rad).
    const INIT_ANTENNAS: [f64; 2] = [-0.1745, 0.1745];

    /// Sleep head pose (pitched back, from Python's SLEEP_HEAD_POSE).
    #[rustfmt::skip]
    const SLEEP_HEAD: [f64; 16] = [
         0.911,  0.004,  0.413, -0.021,
        -0.004,  1.0,   -0.001,  0.001,
        -0.413, -0.001,  0.911, -0.044,
         0.0,    0.0,    0.0,    1.0,
    ];
    /// Antennas fully extended (±175 deg ≈ ±3.05 rad).
    const SLEEP_ANTENNAS: [f64; 2] = [-3.05, 3.05];

    pub async fn wake_up(self: &Arc<Self>) -> Result<(), RobotError> {
        self.start_goto(GotoTarget {
            head: Some(Self::INIT_HEAD.to_vec()),
            head_joints: None,
            antennas: Some(Self::INIT_ANTENNAS.to_vec()),
            body_yaw: Some(0.0),
            duration: 2.0,
            method: InterpolationTechnique::default(),
        })
        .await
    }

    pub async fn goto_sleep(self: &Arc<Self>) -> Result<(), RobotError> {
        self.start_goto(GotoTarget {
            head: Some(Self::SLEEP_HEAD.to_vec()),
            head_joints: None,
            antennas: Some(Self::SLEEP_ANTENNAS.to_vec()),
            body_yaw: Some(0.0),
            duration: 2.0,
            method: InterpolationTechnique::default(),
        })
        .await
    }

    /// Wake up as a task (returns UUID, broadcasts TaskProgress on completion).
    pub async fn wake_up_task(self: &Arc<Self>) -> Result<String, RobotError> {
        self.goto_as_task(GotoTarget {
            head: Some(Self::INIT_HEAD.to_vec()),
            head_joints: None,
            antennas: Some(Self::INIT_ANTENNAS.to_vec()),
            body_yaw: Some(0.0),
            duration: 2.0,
            method: InterpolationTechnique::default(),
        })
        .await
    }

    /// Sleep as a task (returns UUID, broadcasts TaskProgress on completion).
    pub async fn goto_sleep_task(self: &Arc<Self>) -> Result<String, RobotError> {
        self.goto_as_task(GotoTarget {
            head: Some(Self::SLEEP_HEAD.to_vec()),
            head_joints: None,
            antennas: Some(Self::SLEEP_ANTENNAS.to_vec()),
            body_yaw: Some(0.0),
            duration: 2.0,
            method: InterpolationTechnique::default(),
        })
        .await
    }

    /// Generic goto-as-task: runs a goto and broadcasts TaskProgress.
    pub async fn goto_as_task(
        self: &Arc<Self>,
        goto: GotoTarget,
    ) -> Result<String, RobotError> {
        validate_goto(&goto)?;
        let uuid = uuid::Uuid::new_v4().to_string();
        let start = {
            let mut s = self.state.lock().await;
            if s.goto_active {
                return Err(RobotError::GotoInFlight);
            }
            s.goto_active = true;
            s.target.clone()
        };
        let uuid_clone = uuid.clone();
        tokio::spawn({
            let core = self.clone();
            async move {
                core.run_goto(start, goto).await;
                core.state.lock().await.goto_active = false;
                core.broadcast(ServerMsg::TaskProgress {
                    uuid: uuid_clone,
                    finished: true,
                    error: None,
                    timestamp: chrono::Utc::now().to_rfc3339(),
                });
            }
        });
        Ok(uuid)
    }

    // -----------------------------------------------------------------
    // Motor mode
    // -----------------------------------------------------------------

    pub async fn set_motor_mode(&self, mode: &str) {
        let mut s = self.state.lock().await;
        match mode {
            "enabled" => {
                self.sync_target_from_positions(&mut s);
                let _ = self.motor_cmd_tx.send(MotorCommand::EnableTorque);
            }
            "disabled" => {
                let _ = self.motor_cmd_tx.send(MotorCommand::DisableTorque);
            }
            "gravity_compensation" => {
                // No real gravity comp — just set the flag.
            }
            _ => {
                tracing::warn!("Unknown motor mode: {mode}");
                return;
            }
        }
        s.motor_mode = mode.to_string();
    }

    pub async fn set_torque(&self, on: bool) {
        let mut s = self.state.lock().await;
        if on {
            self.sync_target_from_positions(&mut s);
            let _ = self.motor_cmd_tx.send(MotorCommand::EnableTorque);
            s.motor_mode = "enabled".to_string();
        } else {
            let _ = self.motor_cmd_tx.send(MotorCommand::DisableTorque);
            s.motor_mode = "disabled".to_string();
        }
    }

    pub async fn set_recording(&self, recording: bool) {
        self.state.lock().await.is_recording = recording;
    }

    // -----------------------------------------------------------------
    // Dispatch: shared entry point for WS and data channel
    // -----------------------------------------------------------------

    /// Process an incoming message (command or task). Returns a list of
    /// per-client JSON responses to send back to the caller. Broadcast
    /// messages (sensor data, task progress) go through the broadcast
    /// channel and are NOT included in the return value.
    pub async fn dispatch(self: &Arc<Self>, msg: AnyMessage) -> Vec<String> {
        match msg {
            // Fire-and-forget motor commands — no per-client ack (matches
            // the Python daemon). Errors are silently absorbed; the client
            // sees the result in the next sensor broadcast.
            AnyMessage::SetTarget(cmd) => {
                let _ = self.set_target(&cmd.head).await;
                vec![]
            }
            AnyMessage::SetHeadJoints(cmd) => {
                let _ = self.set_head_joints(&cmd.joints).await;
                vec![]
            }
            AnyMessage::SetBodyYaw(cmd) => {
                self.set_body_yaw(cmd.body_yaw).await;
                vec![]
            }
            AnyMessage::SetAntennas(cmd) => {
                let _ = self.set_antennas(&cmd.antennas).await;
                vec![]
            }
            AnyMessage::SetFullTarget(cmd) => {
                if let Err(e) = self.set_full_target(cmd).await {
                    vec![format!(r#"{{"error":"{}"}}"#, e)]
                } else {
                    vec![]
                }
            }
            AnyMessage::GotoTarget(cmd) => {
                if let Err(e) = self.start_goto(cmd).await {
                    vec![format!(r#"{{"error":"{}","command":"goto_target"}}"#, e)]
                } else {
                    // Response comes later when goto completes (for fire-and-forget WS)
                    vec![]
                }
            }
            AnyMessage::WakeUp => {
                if let Err(e) = self.wake_up().await {
                    vec![format!(r#"{{"error":"{}","command":"wake_up"}}"#, e)]
                } else {
                    vec![]
                }
            }
            AnyMessage::GotoSleep => {
                if let Err(e) = self.goto_sleep().await {
                    vec![format!(r#"{{"error":"{}","command":"goto_sleep"}}"#, e)]
                } else {
                    vec![]
                }
            }
            AnyMessage::PlaySound(cmd) => {
                tracing::debug!("play_sound stub: {}", cmd.file);
                vec![r#"{"status":"ok","command":"play_sound"}"#.to_string()]
            }
            AnyMessage::SetMotorMode(cmd) => {
                self.set_motor_mode(&cmd.mode).await;
                vec![format!(
                    r#"{{"motor_mode":"{}","status":"ok"}}"#,
                    cmd.mode
                )]
            }
            AnyMessage::SetTorque(cmd) => {
                self.set_torque(cmd.on).await;
                vec![r#"{"status":"ok","command":"set_torque"}"#.to_string()]
            }
            AnyMessage::GetMotorMode => {
                let mode = self.get_motor_mode().await;
                vec![format!(r#"{{"motor_mode":"{}"}}"#, mode)]
            }
            AnyMessage::SetGravityCompensation(cmd) => {
                if cmd.enabled {
                    self.set_motor_mode("gravity_compensation").await;
                } else {
                    self.set_motor_mode("enabled").await;
                }
                vec![r#"{"status":"ok","command":"set_gravity_compensation"}"#.to_string()]
            }
            AnyMessage::SetAutomaticBodyYaw(_cmd) => {
                // Stub
                vec![r#"{"status":"ok","command":"set_automatic_body_yaw"}"#.to_string()]
            }
            AnyMessage::GetState => {
                let state = self.get_state().await;
                let resp = serde_json::json!({ "state": state });
                vec![resp.to_string()]
            }
            AnyMessage::StartRecording => {
                self.set_recording(true).await;
                vec![r#"{"status":"ok","command":"start_recording","is_recording":true}"#
                    .to_string()]
            }
            AnyMessage::StopRecording => {
                self.set_recording(false).await;
                vec![r#"{"status":"ok","command":"stop_recording","is_recording":false}"#
                    .to_string()]
            }
            AnyMessage::AppendRecord(_cmd) => {
                vec![r#"{"status":"ok","command":"append_record"}"#.to_string()]
            }
            AnyMessage::Task(task_req) => {
                match task_req.req {
                    TaskRequestInner::Goto(goto_req) => {
                        if let Err(e) =
                            self.start_goto_task(task_req.uuid.clone(), goto_req).await
                        {
                            // Immediately broadcast failure
                            self.broadcast(ServerMsg::TaskProgress {
                                uuid: task_req.uuid,
                                finished: true,
                                error: Some(e.to_string()),
                                timestamp: chrono::Utc::now().to_rfc3339(),
                            });
                        }
                    }
                    TaskRequestInner::PlayMove(pm) => {
                        tracing::debug!("play_move stub: {}", pm.move_name);
                        self.broadcast(ServerMsg::TaskProgress {
                            uuid: task_req.uuid,
                            finished: true,
                            error: None,
                            timestamp: chrono::Utc::now().to_rfc3339(),
                        });
                    }
                }
                vec![]
            }
        }
    }

    #[cfg(test)]
    pub async fn snapshot(&self) -> RobotTarget {
        self.state.lock().await.target.clone()
    }

    async fn run_goto(&self, start: RobotTarget, cmd: GotoTarget) {
        let total_steps = ((cmd.duration * GOTO_HZ as f64).max(1.0)) as u64;
        let mut interval = time::interval(Duration::from_millis(1000 / GOTO_HZ));
        interval.set_missed_tick_behavior(time::MissedTickBehavior::Delay);

        for step in 1..=total_steps {
            interval.tick().await;
            // Raw progress along wall-clock time, then reshape through the
            // chosen interpolation technique so the blend weight `s` rises
            // as Python does (e.g. min-jerk: slow-fast-slow).
            let t = (step as f64 / total_steps as f64).clamp(0.0, 1.0);
            let s_blend = time_trajectory(t, cmd.method);
            let mut st = self.state.lock().await;

            if let Some(end_h) = cmd.head.as_deref()
                && end_h.len() == 16
            {
                // Stack-allocated [f64;16] copy of the target for SLERP.
                let mut end_arr = [0.0; 16];
                end_arr.copy_from_slice(end_h);
                // SLERP rotation, lerp translation — matches Python's
                // `linear_pose_interpolation`. Fixes Issue #5: elementwise
                // lerp of the 4x4 was skewing the rotation block.
                let pose = linear_pose_interpolation(start.head(), &end_arr, s_blend);
                st.set_head(&pose);
            } else if let Some(end_hj) = cmd.head_joints.as_deref()
                && end_hj.len() == 6
            {
                let mut lerped = [0.0; 6];
                let start_joints = start.head_joints();
                for i in 0..6 {
                    lerped[i] = lerp(start_joints[i], end_hj[i], s_blend);
                }
                st.set_joints(&lerped);
            }
            if let Some(end_a) = cmd.antennas.as_deref()
                && end_a.len() == 2
            {
                st.target.antennas[0] = lerp(start.antennas[0], end_a[0], s_blend);
                st.target.antennas[1] = lerp(start.antennas[1], end_a[1], s_blend);
            }
            if let Some(end_y) = cmd.body_yaw {
                st.target.body_yaw = lerp(start.body_yaw, end_y, s_blend);
            }

            self.push_to_motors(&st);
        }
    }

    /// Send the current target to the motor thread via the watch channel
    /// and nudge it to write immediately (no waiting for next periodic tick).
    ///
    /// Uses `send_replace` (not `send`) because the motor thread reads via
    /// `target_tx.borrow()` rather than subscribing — so the channel has zero
    /// receivers, and plain `send` would silently fail and discard the value.
    fn push_to_motors(&self, s: &State) {
        self.target_tx.send_replace(MotorTargets {
            stewart: *s.target.head_joints(),
            antennas: s.target.antennas,
            body_yaw: s.target.body_yaw,
        });
        let _ = self.nudge_tx.send(());
    }

    /// Read latest motor positions from the watch channel and update the
    /// internal target so that re-enabling torque doesn't snap to a stale pose.
    fn sync_target_from_positions(&self, s: &mut State) {
        let pos = *self.positions_tx.borrow();
        s.target.antennas = pos.antennas;
        s.target.body_yaw = pos.body_yaw;
        // set_joints runs FK and keeps head ↔ head_joints in sync.
        s.set_joints(&pos.stewart);
    }
}

fn apply_target(s: &mut State, cmd: &SetFullTarget) -> Result<(), RobotError> {
    if let Some(h) = cmd.head.as_deref() {
        if h.len() != 16 {
            return Err(RobotError::BadLength {
                field: "head",
                expected: 16,
                got: h.len(),
            });
        }
        let mut head = [0.0; 16];
        head.copy_from_slice(h);
        s.set_head(&head);
    }
    if let Some(a) = cmd.antennas.as_deref() {
        if a.len() != 2 {
            return Err(RobotError::BadLength {
                field: "antennas",
                expected: 2,
                got: a.len(),
            });
        }
        s.target.antennas.copy_from_slice(a);
    }
    if let Some(y) = cmd.body_yaw {
        s.target.body_yaw = y;
    }
    Ok(())
}

fn validate_goto(cmd: &GotoTarget) -> Result<(), RobotError> {
    if cmd.head.is_some() && cmd.head_joints.is_some() {
        return Err(RobotError::HeadConflict);
    }
    if let Some(h) = cmd.head.as_deref() {
        if h.len() != 16 {
            return Err(RobotError::BadLength {
                field: "head",
                expected: 16,
                got: h.len(),
            });
        }
    }
    if let Some(hj) = cmd.head_joints.as_deref() {
        if hj.len() != 6 {
            return Err(RobotError::BadLength {
                field: "head_joints",
                expected: 6,
                got: hj.len(),
            });
        }
    }
    if let Some(a) = cmd.antennas.as_deref() {
        if a.len() != 2 {
            return Err(RobotError::BadLength {
                field: "antennas",
                expected: 2,
                got: a.len(),
            });
        }
    }
    Ok(())
}

/// Compute head pose from present motor positions via forward kinematics.
fn compute_head_pose(
    kin: &mut reachy_mini_rust_kinematics::Kinematics,
    stewart: &[f64; 6],
    body_yaw: f64,
    last_head: &[f64; 16],
) -> [f64; 16] {
    kin.reset_forward_kinematics(crate::kinematics::head_wire_to_pose(last_head));
    let pose = kin.forward_kinematics(stewart.to_vec(), Some(body_yaw));
    pose_to_head_wire(&pose)
}

#[inline]
fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

/// Broadcast sensor data whenever the motor thread publishes new positions.
///
/// JointPositions: every tick (SENSOR_HZ).
/// HeadPose + ImuData: every other tick (~50 Hz), matching the Python daemon.
async fn sensor_task(core: Arc<RobotCore>) {
    let mut positions_rx: watch::Receiver<MotorPositions> = core.positions_tx.subscribe();
    let mut tick: u64 = 0;
    loop {
        if positions_rx.changed().await.is_err() {
            break;
        }
        let positions = *positions_rx.borrow_and_update();

        let mut head_joint_positions = Vec::with_capacity(7);
        head_joint_positions.push(positions.body_yaw);
        head_joint_positions.extend_from_slice(&positions.stewart);

        let _ = core.sensor_tx.send(ServerMsg::JointPositions {
            head_joint_positions,
            antennas_joint_positions: positions.antennas.to_vec(),
        });

        // HeadPose + ImuData at ~50 Hz (every other tick).
        if tick % 2 == 0 {
            let present_pose = {
                let mut s = core.state.lock().await;
                let last_head = *s.target.head();
                compute_head_pose(&mut s.kinematics, &positions.stewart, positions.body_yaw, &last_head)
            };
            let _ = core.sensor_tx.send(ServerMsg::HeadPose {
                head_pose: head_to_nested(&present_pose),
            });

            let imu = core.imu.lock().await.read();
            let _ = core.sensor_tx.send(ServerMsg::ImuData {
                accelerometer: imu.accelerometer,
                gyroscope: imu.gyroscope,
                quaternion: imu.quaternion,
                temperature: imu.temperature,
            });
        }

        tick = tick.wrapping_add(1);
    }
}

/// Motor I/O loop — runs on a dedicated std::thread.
///
/// Writes happen immediately when a nudge arrives (new target from a
/// command). Reads happen at SENSOR_HZ regardless. This gives near-zero
/// latency for command writes while keeping a steady sensor broadcast rate.
///
/// Also tracks control-loop stats (Issue #14): each read tick's timestamp
/// feeds a rolling 1-second window, from which we compute mean frequency
/// and max tick-to-tick interval. Published via `core.stats_tx` for
/// `build_daemon_status` to pick up.
fn motor_loop(
    mut motors: Box<dyn Motors>,
    core: Arc<RobotCore>,
    cmd_rx: std::sync::mpsc::Receiver<MotorCommand>,
    nudge_rx: std::sync::mpsc::Receiver<()>,
) {
    let period = Duration::from_millis(1000 / SENSOR_HZ);
    let mut next_read = Instant::now();

    // Stats: tick timestamps (read cadence) within the current window.
    // Pre-allocate for ~1 s at SENSOR_HZ so we don't reallocate each tick.
    let mut tick_times: Vec<Instant> = Vec::with_capacity(SENSOR_HZ as usize + 8);
    let mut window_start = Instant::now();

    loop {
        // Wait for a nudge (new target) or the next periodic read.
        let now = Instant::now();
        let wait = next_read.saturating_duration_since(now);
        let _ = nudge_rx.recv_timeout(wait);
        while nudge_rx.try_recv().is_ok() {} // drain extra nudges

        // Drain pending torque commands.
        while let Ok(cmd) = cmd_rx.try_recv() {
            match cmd {
                MotorCommand::EnableTorque => motors.enable_torque(),
                MotorCommand::DisableTorque => motors.disable_torque(),
            }
        }

        // Always write the latest target.
        let target = *core.target_tx.borrow();
        motors.set_targets(target);

        // Read positions at SENSOR_HZ.
        let now = Instant::now();
        if now >= next_read {
            let positions = motors.read_positions();
            let _ = core.positions_tx.send(positions);
            next_read += period;

            tick_times.push(now);

            // Recompute stats once per window.
            if now.duration_since(window_start) >= STATS_WINDOW {
                let stats = compute_stats(&tick_times, motors.error_count());
                core.stats_tx.send_replace(stats);
                tick_times.clear();
                window_start = now;
            }
        }
    }
}

/// Reduce a window of read timestamps into a `ControlLoopStats`.
///
/// Mean frequency = ticks / wall-seconds. Max interval = largest gap
/// between successive ticks (in seconds). With fewer than two ticks the
/// window is effectively silent — return zeros rather than fake values.
fn compute_stats(ticks: &[Instant], nb_error: u64) -> ControlLoopStats {
    if ticks.len() < 2 {
        return ControlLoopStats {
            mean_control_loop_frequency: 0.0,
            max_control_loop_interval: 0.0,
            nb_error,
        };
    }
    let first = ticks[0];
    let last = *ticks.last().unwrap();
    let wall = last.duration_since(first).as_secs_f64();
    let mean_hz = if wall > 0.0 {
        (ticks.len() - 1) as f64 / wall
    } else {
        0.0
    };
    let mut max_interval = 0.0f64;
    for pair in ticks.windows(2) {
        let dt = pair[1].duration_since(pair[0]).as_secs_f64();
        if dt > max_interval {
            max_interval = dt;
        }
    }
    ControlLoopStats {
        mean_control_loop_frequency: mean_hz,
        max_control_loop_interval: max_interval,
        nb_error,
    }
}

/// Publish daemon status at 1 Hz.
async fn daemon_status_task(core: Arc<RobotCore>) {
    let mut interval = time::interval(Duration::from_secs(1));
    interval.set_missed_tick_behavior(time::MissedTickBehavior::Delay);
    loop {
        interval.tick().await;
        let status = core.build_daemon_status().await;
        let _ = core.sensor_tx.send(ServerMsg::DaemonStatus(status));
    }
}

/// Detect the local IP by briefly "connecting" a UDP socket to a public address.
/// No packets are sent — the OS just picks the outbound interface.
fn detect_local_ip() -> Option<String> {
    let sock = std::net::UdpSocket::bind("0.0.0.0:0").ok()?;
    sock.connect("8.8.8.8:80").ok()?;
    Some(sock.local_addr().ok()?.ip().to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::devices::{MockImu, MockMotors};
    use crate::kinematics::build_kinematics;

    fn spawn_mock() -> Arc<RobotCore> {
        RobotCore::spawn(
            Box::new(MockMotors::new()),
            Box::new(MockImu::new()),
            build_kinematics().unwrap(),
        )
    }

    #[tokio::test]
    async fn set_full_target_updates_antennas_and_body_yaw() {
        let core = spawn_mock();
        core.set_full_target(SetFullTarget {
            head: None,
            antennas: Some(vec![0.5, -0.5]),
            body_yaw: Some(0.2),
        })
        .await
        .unwrap();
        let snap = core.snapshot().await;
        assert_eq!(snap.antennas, [0.5, -0.5]);
        assert!((snap.body_yaw - 0.2).abs() < 1e-9);
    }

    #[tokio::test]
    async fn bad_length_rejected() {
        let core = spawn_mock();
        let err = core
            .set_head_joints(&[0.0; 5])
            .await
            .unwrap_err();
        assert!(matches!(err, RobotError::BadLength { field: "joints", .. }));
    }

    #[tokio::test]
    async fn setting_head_joints_derives_head() {
        let core = spawn_mock();
        // 7 floats: [body_yaw, stewart_0..5]
        core.set_head_joints(&[0.0, 0.1, -0.05, 0.2, 0.0, -0.15, 0.1])
            .await
            .unwrap();
        let snap = core.snapshot().await;
        let any_translation_change =
            (snap.head[3]).abs() > 1e-6 || (snap.head[7]).abs() > 1e-6 || (snap.head[11]).abs() > 1e-6;
        let any_rotation_change = (snap.head[0] - 1.0).abs() > 1e-6
            || (snap.head[5] - 1.0).abs() > 1e-6
            || (snap.head[10] - 1.0).abs() > 1e-6;
        assert!(any_translation_change || any_rotation_change);
    }

    #[tokio::test]
    async fn goto_rejects_concurrent_commands() {
        let core = spawn_mock();
        core.start_goto(GotoTarget {
            head: None,
            head_joints: None,
            antennas: Some(vec![0.5, 0.5]),
            body_yaw: None,
            duration: 0.2,
            method: InterpolationTechnique::default(),
        })
        .await
        .unwrap();
        let err = core
            .set_full_target(SetFullTarget {
                head: None,
                antennas: Some(vec![0.0, 0.0]),
                body_yaw: None,
            })
            .await
            .unwrap_err();
        assert!(matches!(err, RobotError::GotoInFlight));
    }

    #[test]
    fn compute_stats_zero_when_fewer_than_two_ticks() {
        let now = Instant::now();
        let s = compute_stats(&[], 3);
        assert_eq!(s.mean_control_loop_frequency, 0.0);
        assert_eq!(s.max_control_loop_interval, 0.0);
        assert_eq!(s.nb_error, 3);

        let s = compute_stats(&[now], 0);
        assert_eq!(s.mean_control_loop_frequency, 0.0);
        assert_eq!(s.max_control_loop_interval, 0.0);
    }

    #[test]
    fn compute_stats_uniform_ticks_give_expected_frequency() {
        let t0 = Instant::now();
        // 11 ticks at exactly 10 ms apart → 100 Hz, max interval 0.010 s.
        let ticks: Vec<Instant> = (0..11)
            .map(|i| t0 + Duration::from_millis(i * 10))
            .collect();
        let s = compute_stats(&ticks, 0);
        assert!((s.mean_control_loop_frequency - 100.0).abs() < 1e-6);
        assert!((s.max_control_loop_interval - 0.010).abs() < 1e-9);
    }

    #[test]
    fn compute_stats_catches_max_interval_jitter() {
        // 5 evenly spaced 10-ms ticks, then a 50-ms gap.
        let t0 = Instant::now();
        let mut ticks: Vec<Instant> = (0..5)
            .map(|i| t0 + Duration::from_millis(i * 10))
            .collect();
        ticks.push(t0 + Duration::from_millis(40 + 50));
        let s = compute_stats(&ticks, 7);
        assert!(s.max_control_loop_interval >= 0.049);
        assert_eq!(s.nb_error, 7);
    }

    #[tokio::test]
    async fn daemon_status_control_loop_stats_has_expected_keys() {
        let core = spawn_mock();
        let status = core.build_daemon_status().await;
        let backend = status.backend_status.expect("backend_status");
        let stats = &backend["control_loop_stats"];
        assert!(stats.get("mean_control_loop_frequency").is_some());
        assert!(stats.get("max_control_loop_interval").is_some());
        assert!(stats.get("nb_error").is_some());
    }

    #[tokio::test]
    async fn goto_reaches_endpoint_and_releases_lock() {
        let core = spawn_mock();
        core.start_goto(GotoTarget {
            head: None,
            head_joints: None,
            antennas: Some(vec![0.7, -0.7]),
            body_yaw: Some(0.3),
            duration: 0.1,
            method: InterpolationTechnique::default(),
        })
        .await
        .unwrap();
        tokio::time::sleep(Duration::from_millis(300)).await;
        let snap = core.snapshot().await;
        assert!((snap.antennas[0] - 0.7).abs() < 1e-6);
        assert!((snap.antennas[1] - (-0.7)).abs() < 1e-6);
        assert!((snap.body_yaw - 0.3).abs() < 1e-6);
        core.set_full_target(SetFullTarget {
            head: None,
            antennas: Some(vec![0.0, 0.0]),
            body_yaw: None,
        })
        .await
        .expect("lock should be free");
    }
}
