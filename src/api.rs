//! REST API endpoints for Python daemon compatibility.
//!
//! All routes are mounted under `/api/` in the main router.

use std::sync::Arc;
use std::time::Duration;

use axum::Router;
use axum::extract::ws::{Message, WebSocket, WebSocketUpgrade};
use axum::extract::{Path, Query, State};
use axum::response::IntoResponse;
use axum::routing::{get, post};
use futures_util::{SinkExt, StreamExt};
use serde::Deserialize;
use serde_json::{Value, json};

use crate::control::AppState;
use crate::protocol::GotoTarget;
use crate::robot::{RobotCore, RobotStateSnapshot};

pub fn build_router(core: Arc<RobotCore>) -> Router {
    Router::new()
        // Daemon
        .route("/daemon/status", get(daemon_status))
        .route("/daemon/start", post(daemon_start))
        .route("/daemon/stop", post(daemon_stop))
        // Motors
        .route("/motors/status", get(motors_status))
        .route("/motors/set_mode/{mode}", post(motors_set_mode))
        // State
        .route("/state/present_head_pose", get(state_head_pose))
        .route("/state/present_body_yaw", get(state_body_yaw))
        .route(
            "/state/present_antenna_joint_positions",
            get(state_antennas),
        )
        .route("/state/full", get(state_full))
        .route("/state/ws/full", get(state_full_ws))
        // Move
        .route("/move/goto", post(move_goto))
        .route("/move/play/wake_up", post(move_wake_up))
        .route("/move/play/goto_sleep", post(move_goto_sleep))
        // Media
        .route("/media/play_sound", post(media_play_sound))
        .route("/media/stop_sound", post(media_stop_sound))
        .route("/media/status", get(media_status))
        // Camera
        .route("/camera/specs", get(camera_specs))
        .with_state(AppState { core })
}

// ---- Daemon ----

async fn daemon_status(State(state): State<AppState>) -> axum::Json<crate::protocol::DaemonStatus> {
    axum::Json(state.core.build_daemon_status().await)
}

async fn daemon_start() -> axum::Json<serde_json::Value> {
    axum::Json(json!({ "job_id": uuid::Uuid::new_v4().to_string() }))
}

async fn daemon_stop() -> axum::Json<serde_json::Value> {
    axum::Json(json!({ "job_id": uuid::Uuid::new_v4().to_string() }))
}

// ---- Motors ----

async fn motors_status(State(state): State<AppState>) -> axum::Json<serde_json::Value> {
    let mode = state.core.get_motor_mode().await;
    axum::Json(json!({ "mode": mode }))
}

async fn motors_set_mode(
    State(state): State<AppState>,
    Path(mode): Path<String>,
) -> axum::Json<serde_json::Value> {
    state.core.set_motor_mode(&mode).await;
    axum::Json(json!({ "status": "ok", "mode": mode }))
}

// ---- State ----
//
// The `/state/full` endpoint and its WS streaming sibling mirror the
// Python daemon: a bag of optional flags selects which fields to
// include, plus `use_pose_matrix` controls whether poses come back as
// a flat 16-float matrix or as `{x, y, z, roll, pitch, yaw}`.

#[inline]
fn default_true() -> bool {
    true
}

/// Query params for `GET /state/full` and `WS /state/ws/full`.
/// Defaults match Python's `get_full_state` — present pose + body yaw +
/// antennas + control mode on; everything else off.
#[derive(Debug, Deserialize)]
pub struct FullStateQuery {
    #[serde(default = "default_true")]
    pub with_control_mode: bool,
    #[serde(default = "default_true")]
    pub with_head_pose: bool,
    #[serde(default)]
    pub with_target_head_pose: bool,
    #[serde(default)]
    pub with_head_joints: bool,
    #[serde(default)]
    pub with_target_head_joints: bool,
    #[serde(default = "default_true")]
    pub with_body_yaw: bool,
    #[serde(default)]
    pub with_target_body_yaw: bool,
    #[serde(default = "default_true")]
    pub with_antenna_positions: bool,
    #[serde(default)]
    pub with_target_antenna_positions: bool,
    #[serde(default)]
    pub use_pose_matrix: bool,
}

/// `/state/ws/full` accepts `FullStateQuery` plus a `frequency` (Hz).
#[derive(Debug, Deserialize)]
struct FullStateWsQuery {
    #[serde(flatten)]
    full: FullStateQuery,
    #[serde(default = "default_ws_frequency")]
    frequency: f64,
}

fn default_ws_frequency() -> f64 {
    10.0
}

#[derive(Debug, Deserialize)]
pub struct HeadPoseQuery {
    #[serde(default)]
    pub use_pose_matrix: bool,
}

async fn state_head_pose(
    State(state): State<AppState>,
    Query(q): Query<HeadPoseQuery>,
) -> axum::Json<Value> {
    let snap = state.core.get_state().await;
    axum::Json(pose_to_json(&snap.head_pose, q.use_pose_matrix))
}

async fn state_body_yaw(State(state): State<AppState>) -> axum::Json<Value> {
    let snap = state.core.get_state().await;
    axum::Json(json!(snap.body_yaw))
}

async fn state_antennas(State(state): State<AppState>) -> axum::Json<Value> {
    let snap = state.core.get_state().await;
    axum::Json(json!(snap.antennas))
}

async fn state_full(
    State(state): State<AppState>,
    Query(q): Query<FullStateQuery>,
) -> axum::Json<Value> {
    let snap = state.core.get_state().await;
    axum::Json(build_full_state(&snap, &q))
}

async fn state_full_ws(
    ws: WebSocketUpgrade,
    State(state): State<AppState>,
    Query(q): Query<FullStateWsQuery>,
) -> impl IntoResponse {
    ws.on_upgrade(move |socket| run_full_state_ws(socket, state.core, q))
}

async fn run_full_state_ws(socket: WebSocket, core: Arc<RobotCore>, q: FullStateWsQuery) {
    // Clamp the frequency to something sane; avoids a pathological
    // 0-period interval if the client requests `frequency=0`.
    let hz = q.frequency.clamp(0.1, 200.0);
    let period = Duration::from_secs_f64(1.0 / hz);
    let mut interval = tokio::time::interval(period);
    interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);

    let (mut sink, mut stream) = socket.split();
    // Drain any inbound frames (the client shouldn't be sending, but close
    // frames and pings still need to be observed so we exit cleanly).
    let closer = tokio::spawn(async move {
        while let Some(Ok(m)) = stream.next().await {
            if matches!(m, Message::Close(_)) {
                break;
            }
        }
    });

    loop {
        interval.tick().await;
        let snap = core.get_state().await;
        let payload = build_full_state(&snap, &q.full);
        let text = match serde_json::to_string(&payload) {
            Ok(s) => s,
            Err(_) => continue,
        };
        if sink.send(Message::Text(text.into())).await.is_err() {
            break;
        }
    }
    let _ = closer.await;
}

/// Assemble the JSON response for `/state/full` by selecting fields
/// from `snap` according to `q`. Adds a UTC ISO-8601 `timestamp` last,
/// matching Python.
fn build_full_state(snap: &RobotStateSnapshot, q: &FullStateQuery) -> Value {
    let mut out = serde_json::Map::new();

    if q.with_control_mode {
        out.insert("control_mode".into(), json!(snap.motor_mode));
    }
    if q.with_head_pose {
        out.insert(
            "head_pose".into(),
            pose_to_json(&snap.head_pose, q.use_pose_matrix),
        );
    }
    if q.with_target_head_pose {
        out.insert(
            "target_head_pose".into(),
            pose_to_json(&snap.target_head_pose, q.use_pose_matrix),
        );
    }
    if q.with_head_joints {
        out.insert("head_joints".into(), json!(snap.head_joints));
    }
    if q.with_target_head_joints {
        out.insert(
            "target_head_joints".into(),
            json!(snap.target_head_joint_positions),
        );
    }
    if q.with_body_yaw {
        out.insert("body_yaw".into(), json!(snap.body_yaw));
    }
    if q.with_target_body_yaw {
        out.insert("target_body_yaw".into(), json!(snap.target_body_yaw));
    }
    if q.with_antenna_positions {
        out.insert("antennas_position".into(), json!(snap.antennas));
    }
    if q.with_target_antenna_positions {
        out.insert(
            "target_antennas_position".into(),
            json!(snap.target_antennas_joint_positions),
        );
    }
    out.insert(
        "timestamp".into(),
        json!(chrono::Utc::now().to_rfc3339()),
    );
    Value::Object(out)
}

/// Serialize a nested-4x4 pose as either a flat 16-float `Matrix4x4Pose`
/// (`use_pose_matrix=true`) or the six-field `XYZRPYPose` (default).
/// Euler convention matches scipy's `as_euler("xyz")` — extrinsic XYZ,
/// i.e. `R = Rz(yaw) · Ry(pitch) · Rx(roll)`.
fn pose_to_json(nested: &[Vec<f64>], use_matrix: bool) -> Value {
    if use_matrix {
        let mut flat = Vec::with_capacity(16);
        for row in nested {
            flat.extend(row.iter().copied());
        }
        return json!({ "m": flat });
    }
    let (x, y, z, roll, pitch, yaw) = pose_to_xyzrpy(nested);
    json!({
        "x": x, "y": y, "z": z,
        "roll": roll, "pitch": pitch, "yaw": yaw
    })
}

/// Decompose a 4x4 pose into `(x, y, z, roll, pitch, yaw)` with the
/// scipy "xyz" extrinsic convention: `R = Rz(yaw) · Ry(pitch) · Rx(roll)`.
/// Gimbal-lock case (|r31|≈1) falls back to yaw=0.
fn pose_to_xyzrpy(m: &[Vec<f64>]) -> (f64, f64, f64, f64, f64, f64) {
    let x = m[0][3];
    let y = m[1][3];
    let z = m[2][3];
    let r11 = m[0][0];
    let r21 = m[1][0];
    let r31 = m[2][0];
    let r32 = m[2][1];
    let r33 = m[2][2];

    let pitch = (-r31).clamp(-1.0, 1.0).asin();
    let (roll, yaw) = if (r31.abs() - 1.0).abs() < 1e-6 {
        // Gimbal lock: pitch = ±π/2. Fix yaw=0, derive roll from r12/r22.
        let r12 = m[0][1];
        let r22 = m[1][1];
        (r12.atan2(r22), 0.0)
    } else {
        (r32.atan2(r33), r21.atan2(r11))
    };
    (x, y, z, roll, pitch, yaw)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_4;

    fn identity_nested() -> Vec<Vec<f64>> {
        vec![
            vec![1.0, 0.0, 0.0, 0.0],
            vec![0.0, 1.0, 0.0, 0.0],
            vec![0.0, 0.0, 1.0, 0.0],
            vec![0.0, 0.0, 0.0, 1.0],
        ]
    }

    fn rot_z_nested(angle: f64) -> Vec<Vec<f64>> {
        let c = angle.cos();
        let s = angle.sin();
        vec![
            vec![c, -s, 0.0, 0.0],
            vec![s, c, 0.0, 0.0],
            vec![0.0, 0.0, 1.0, 0.0],
            vec![0.0, 0.0, 0.0, 1.0],
        ]
    }

    #[test]
    fn xyzrpy_identity() {
        let (x, y, z, r, p, w) = pose_to_xyzrpy(&identity_nested());
        for v in [x, y, z, r, p, w] {
            assert!(v.abs() < 1e-9, "non-zero component: {v}");
        }
    }

    #[test]
    fn xyzrpy_yaw_only() {
        // 45° about Z should give yaw=π/4, roll=pitch=0.
        let (_, _, _, r, p, y) = pose_to_xyzrpy(&rot_z_nested(FRAC_PI_4));
        assert!(r.abs() < 1e-9);
        assert!(p.abs() < 1e-9);
        assert!((y - FRAC_PI_4).abs() < 1e-9, "yaw: {y}");
    }

    #[test]
    fn xyzrpy_translation() {
        let mut m = identity_nested();
        m[0][3] = 0.1;
        m[1][3] = -0.2;
        m[2][3] = 0.3;
        let (x, y, z, _, _, _) = pose_to_xyzrpy(&m);
        assert!((x - 0.1).abs() < 1e-12);
        assert!((y + 0.2).abs() < 1e-12);
        assert!((z - 0.3).abs() < 1e-12);
    }

    #[test]
    fn full_state_respects_flags() {
        let snap = RobotStateSnapshot {
            head_pose: identity_nested(),
            head_joints: vec![0.0; 7],
            antennas: vec![0.0, 0.0],
            body_yaw: 0.1,
            target_head_pose: identity_nested(),
            target_head_joint_positions: vec![0.0; 7],
            target_antennas_joint_positions: vec![0.0, 0.0],
            target_body_yaw: 0.2,
            motor_mode: "enabled".into(),
            is_recording: false,
            is_move_running: false,
        };
        let q = FullStateQuery {
            with_control_mode: false,
            with_head_pose: false,
            with_target_head_pose: false,
            with_head_joints: false,
            with_target_head_joints: false,
            with_body_yaw: true,
            with_target_body_yaw: true,
            with_antenna_positions: false,
            with_target_antenna_positions: false,
            use_pose_matrix: false,
        };
        let v = build_full_state(&snap, &q);
        let obj = v.as_object().unwrap();
        // Exactly these two fields + the always-on timestamp.
        assert!(obj.contains_key("body_yaw"));
        assert!(obj.contains_key("target_body_yaw"));
        assert!(obj.contains_key("timestamp"));
        assert!(!obj.contains_key("head_pose"));
        assert!(!obj.contains_key("control_mode"));
        assert_eq!(obj.len(), 3);
        assert_eq!(obj["body_yaw"], json!(0.1));
        assert_eq!(obj["target_body_yaw"], json!(0.2));
    }

    #[test]
    fn full_state_pose_matrix_flag() {
        let snap = RobotStateSnapshot {
            head_pose: rot_z_nested(FRAC_PI_4),
            head_joints: vec![0.0; 7],
            antennas: vec![0.0, 0.0],
            body_yaw: 0.0,
            target_head_pose: identity_nested(),
            target_head_joint_positions: vec![0.0; 7],
            target_antennas_joint_positions: vec![0.0, 0.0],
            target_body_yaw: 0.0,
            motor_mode: "enabled".into(),
            is_recording: false,
            is_move_running: false,
        };
        let mut q = FullStateQuery {
            with_control_mode: false,
            with_head_pose: true,
            with_target_head_pose: false,
            with_head_joints: false,
            with_target_head_joints: false,
            with_body_yaw: false,
            with_target_body_yaw: false,
            with_antenna_positions: false,
            with_target_antenna_positions: false,
            use_pose_matrix: false,
        };
        // XYZRPY: object with six named fields.
        let v = build_full_state(&snap, &q);
        let head = &v["head_pose"];
        assert!(head.get("yaw").is_some());
        assert!(head.get("m").is_none());
        // Matrix: object with a single "m" array of 16 floats.
        q.use_pose_matrix = true;
        let v = build_full_state(&snap, &q);
        let head = &v["head_pose"];
        assert!(head.get("m").is_some());
        assert_eq!(head["m"].as_array().unwrap().len(), 16);
    }
}

// ---- Move ----

async fn move_goto(
    State(state): State<AppState>,
    axum::Json(body): axum::Json<serde_json::Value>,
) -> axum::Json<serde_json::Value> {
    let head = body.get("head").and_then(|v| {
        v.as_array()
            .map(|a| a.iter().filter_map(|x| x.as_f64()).collect::<Vec<_>>())
    });
    let antennas = body.get("antennas").and_then(|v| {
        v.as_array()
            .map(|a| a.iter().filter_map(|x| x.as_f64()).collect::<Vec<_>>())
    });
    let body_yaw = body.get("body_yaw").and_then(|v| v.as_f64());
    let duration = body
        .get("duration")
        .and_then(|v| v.as_f64())
        .unwrap_or(0.5);

    // Optional `method` (linear | minjerk | ease_in_out | cartoon); falls
    // back to the Python-compatible default (minjerk) if absent or invalid.
    let method = body
        .get("method")
        .and_then(|v| serde_json::from_value(v.clone()).ok())
        .unwrap_or_default();

    let goto = GotoTarget {
        head,
        head_joints: None,
        antennas,
        body_yaw,
        duration,
        method,
    };
    // `goto_as_task` mints the UUID, tracks completion, and broadcasts
    // `TaskProgress` — so the UUID we return is actually observable by
    // clients subscribed to the WS (was fire-and-forget before: Issue #7).
    match state.core.goto_as_task(goto).await {
        Ok(uuid) => axum::Json(json!({ "uuid": uuid })),
        Err(e) => axum::Json(json!({ "error": e.to_string() })),
    }
}

async fn move_wake_up(State(state): State<AppState>) -> axum::Json<serde_json::Value> {
    match state.core.wake_up_task().await {
        Ok(uuid) => axum::Json(json!({ "uuid": uuid })),
        Err(e) => axum::Json(json!({ "error": e.to_string() })),
    }
}

async fn move_goto_sleep(State(state): State<AppState>) -> axum::Json<serde_json::Value> {
    match state.core.goto_sleep_task().await {
        Ok(uuid) => axum::Json(json!({ "uuid": uuid })),
        Err(e) => axum::Json(json!({ "error": e.to_string() })),
    }
}

// ---- Media ----

async fn media_play_sound() -> axum::Json<serde_json::Value> {
    axum::Json(json!({ "status": "ok" }))
}

async fn media_stop_sound() -> axum::Json<serde_json::Value> {
    axum::Json(json!({ "status": "ok" }))
}

async fn media_status() -> axum::Json<serde_json::Value> {
    axum::Json(json!({ "available": true, "released": false, "no_media": false }))
}

// ---- Camera ----

async fn camera_specs() -> axum::Json<serde_json::Value> {
    axum::Json(json!({
        "name": "wireless",
        "width": 1536,
        "height": 864,
        "fps": 30,
        "fov_h": 120.0,
        "fov_v": 66.0
    }))
}
