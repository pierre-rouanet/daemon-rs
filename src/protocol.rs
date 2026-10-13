//! WebSocket message types for the control API.
//!
//! Wire-compatible with the Python daemon's protocol
//! (`reachy_mini/io/protocol.py`). Discriminator is the `type` field.
//!
//! Two message families arrive on `/ws/sdk`:
//!   - `Command` — fire-and-forget motor/state commands
//!   - `TaskRequest` — UUID-tracked async operations (goto, play_move)
//!
//! `AnyMessage` is the top-level parse target (Command | TaskRequest).

use serde::{Deserialize, Serialize};

use crate::interpolation::InterpolationTechnique;

// ------------------------------------------------------------------
// Client -> Server commands
// ------------------------------------------------------------------

// Note: `AnyMessage` below is the top-level parse target. It duplicates the
// variant list but also includes `Task(TaskRequest)`. We don't need a
// separate `Command` enum.

#[derive(Debug, Clone, Deserialize)]
pub struct SetTargetCmd {
    pub head: Vec<f64>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct SetHeadJointsCmd {
    pub joints: Vec<f64>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct SetBodyYawCmd {
    pub body_yaw: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct SetAntennasCmd {
    pub antennas: Vec<f64>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct SetFullTarget {
    #[serde(default)]
    pub head: Option<Vec<f64>>,
    #[serde(default)]
    pub antennas: Option<Vec<f64>>,
    #[serde(default)]
    pub body_yaw: Option<f64>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct GotoTarget {
    #[serde(default)]
    pub head: Option<Vec<f64>>,
    /// Joint-space interpolation target. Not part of the wire protocol
    /// (Python SDK never sends this). Used internally by `run_goto`.
    #[serde(skip)]
    pub head_joints: Option<Vec<f64>>,
    #[serde(default)]
    pub antennas: Option<Vec<f64>>,
    #[serde(default)]
    pub body_yaw: Option<f64>,
    #[serde(default = "default_goto_duration")]
    pub duration: f64,
    /// Time-shaping curve applied to the goto (linear/minjerk/ease_in_out/cartoon).
    /// Matches Python's `reachy_mini.utils.interpolation.InterpolationTechnique`.
    #[serde(default)]
    pub method: InterpolationTechnique,
}

fn default_goto_duration() -> f64 {
    0.5
}

#[derive(Debug, Clone, Deserialize)]
pub struct PlaySoundCmd {
    pub file: String,
}

#[derive(Debug, Clone, Deserialize)]
pub struct SetMotorModeCmd {
    pub mode: String,
}

#[derive(Debug, Clone, Deserialize)]
pub struct SetTorqueCmd {
    pub on: bool,
    #[serde(default)]
    pub ids: Option<Vec<String>>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct SetGravityCompensationCmd {
    pub enabled: bool,
}

#[derive(Debug, Clone, Deserialize)]
pub struct SetAutomaticBodyYawCmd {
    pub enabled: bool,
}

#[derive(Debug, Clone, Deserialize)]
pub struct AppendRecordCmd {
    pub record: serde_json::Value,
}

// ------------------------------------------------------------------
// Task protocol (UUID-tracked async operations)
// ------------------------------------------------------------------

#[derive(Debug, Clone, Deserialize)]
pub struct TaskRequest {
    pub uuid: String,
    pub req: TaskRequestInner,
    #[serde(default)]
    pub timestamp: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(untagged)]
pub enum TaskRequestInner {
    Goto(GotoTaskRequest),
    PlayMove(PlayMoveTaskRequest),
}

#[derive(Debug, Clone, Deserialize)]
pub struct GotoTaskRequest {
    #[serde(default)]
    pub head: Option<Vec<f64>>,
    #[serde(default)]
    pub antennas: Option<Vec<f64>>,
    #[serde(default = "default_goto_duration")]
    pub duration: f64,
    #[serde(default)]
    pub method: InterpolationTechnique,
    #[serde(default)]
    pub body_yaw: Option<f64>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct PlayMoveTaskRequest {
    pub move_name: String,
}

/// Top-level inbound message: either a Command or a TaskRequest.
#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "type")]
pub enum AnyMessage {
    // Commands (same discriminator values as Command)
    #[serde(rename = "set_target")]
    SetTarget(SetTargetCmd),
    #[serde(rename = "set_head_joints")]
    SetHeadJoints(SetHeadJointsCmd),
    #[serde(rename = "set_body_yaw")]
    SetBodyYaw(SetBodyYawCmd),
    #[serde(rename = "set_antennas")]
    SetAntennas(SetAntennasCmd),
    #[serde(rename = "set_full_target")]
    SetFullTarget(SetFullTarget),
    #[serde(rename = "goto_target")]
    GotoTarget(GotoTarget),
    #[serde(rename = "wake_up")]
    WakeUp,
    #[serde(rename = "goto_sleep")]
    GotoSleep,
    #[serde(rename = "play_sound")]
    PlaySound(PlaySoundCmd),
    #[serde(rename = "set_motor_mode")]
    SetMotorMode(SetMotorModeCmd),
    #[serde(rename = "set_torque")]
    SetTorque(SetTorqueCmd),
    #[serde(rename = "get_motor_mode")]
    GetMotorMode,
    #[serde(rename = "set_gravity_compensation")]
    SetGravityCompensation(SetGravityCompensationCmd),
    #[serde(rename = "set_automatic_body_yaw")]
    SetAutomaticBodyYaw(SetAutomaticBodyYawCmd),
    #[serde(rename = "get_state")]
    GetState,
    #[serde(rename = "start_recording")]
    StartRecording,
    #[serde(rename = "stop_recording")]
    StopRecording,
    #[serde(rename = "append_record")]
    AppendRecord(AppendRecordCmd),
    // Task request
    #[serde(rename = "task")]
    Task(TaskRequest),
}

// ------------------------------------------------------------------
// Server -> Client (broadcast)
// ------------------------------------------------------------------

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ServerMsg {
    /// Head + antenna joint positions, published at 100 Hz.
    /// `head_joint_positions` is 7 floats: [body_yaw, stewart_0..5] (present).
    JointPositions {
        head_joint_positions: Vec<f64>,
        antennas_joint_positions: Vec<f64>,
    },
    /// Head pose as nested 4x4 matrix, published at 50 Hz.
    HeadPose {
        head_pose: Vec<Vec<f64>>,
    },
    /// IMU sample, published at 50 Hz.
    ImuData {
        accelerometer: [f64; 3],
        gyroscope: [f64; 3],
        quaternion: [f64; 4],
        temperature: f64,
    },
    /// Daemon status, published at 1 Hz.
    /// Newtype around `DaemonStatus` so the same struct can be reused by
    /// the REST `/api/daemon/status` endpoint — single source of truth.
    DaemonStatus(DaemonStatus),
    /// Task completion, broadcast to all clients.
    TaskProgress {
        uuid: String,
        finished: bool,
        error: Option<String>,
        timestamp: String,
    },
    /// Per-client error reply.
    Error {
        message: String,
    },
}

/// Daemon status payload — shared by the WS broadcast (`ServerMsg::DaemonStatus`)
/// and the REST `GET /api/daemon/status` endpoint so the two never drift.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DaemonStatus {
    pub robot_name: String,
    pub state: String,
    pub wireless_version: bool,
    pub desktop_app_daemon: bool,
    pub simulation_enabled: Option<bool>,
    pub mockup_sim_enabled: Option<bool>,
    pub no_media: bool,
    pub media_released: bool,
    pub camera_specs_name: String,
    pub backend_status: Option<serde_json::Value>,
    pub error: Option<String>,
    pub wlan_ip: Option<String>,
    pub version: Option<String>,
}

/// Convert a row-major 16-float head matrix to a nested 4x4 for the wire.
pub fn head_to_nested(head: &[f64; 16]) -> Vec<Vec<f64>> {
    head.chunks(4).map(|row| row.to_vec()).collect()
}
