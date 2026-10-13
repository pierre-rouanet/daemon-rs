//! Thin wrapper around `reachy_mini_rust_kinematics::Kinematics` —
//! constructs the 6-branch Stewart platform from the bundled
//! `motors.json` and exposes the head pose ↔ joint angles conversions
//! the daemon needs. Pure math, no I/O, no feature flags.
//!
//! The numeric constants (`MOTOR_ARM_LENGTH`, `ROD_LENGTH`, `HEAD_Z_OFFSET`)
//! come from `reachy_mini_rust_kinematics/src/main.rs` and describe the
//! physical Reachy Mini geometry. Changing them is a hardware change.

use anyhow::{Context, Result};
use nalgebra::{Matrix4, Vector3};
use reachy_mini_rust_kinematics::Kinematics as InnerKinematics;
use serde::Deserialize;

/// `motors.json` pulled in at build time — no runtime file access needed.
const MOTORS_JSON: &str = include_str!("../assets/motors.json");

const MOTOR_ARM_LENGTH: f64 = 0.038;
const ROD_LENGTH: f64 = 0.09;
/// Head platform sits 0.177 m above the world origin at neutral pose.
/// The client's `head` matrix is treated as a transform _relative_ to
/// this neutral pose, so identity-head = neutral-robot.
pub const HEAD_Z_OFFSET: f64 = 0.177;

#[allow(non_snake_case)]
#[derive(Deserialize)]
struct MotorJson {
    branch_position: Vec<f64>,
    T_motor_world: Vec<Vec<f64>>,
    solution: f64,
}

/// Construct + configure a Stewart-platform `Kinematics` instance.
pub fn build_kinematics() -> Result<InnerKinematics> {
    let motors: Vec<MotorJson> =
        serde_json::from_str(MOTORS_JSON).context("parse motors.json")?;
    if motors.len() != 6 {
        anyhow::bail!("motors.json expected 6 branches, got {}", motors.len());
    }

    let mut k = InnerKinematics::new(MOTOR_ARM_LENGTH, ROD_LENGTH);
    for m in motors {
        let branch_position = Vector3::new(
            m.branch_position[0],
            m.branch_position[1],
            m.branch_position[2],
        );
        let t_motor_world = mat4_from_rows(&m.T_motor_world)?;
        let t_world_motor = t_motor_world
            .try_inverse()
            .context("T_motor_world is singular")?;
        let solution = if m.solution != 0.0 { 1.0 } else { -1.0 };
        k.add_branch(branch_position, t_world_motor, solution);
    }

    // Prime forward kinematics at the neutral pose.
    k.reset_forward_kinematics(neutral_pose());
    Ok(k)
}

/// World-frame pose of the head platform at the zero-joint neutral stance.
pub fn neutral_pose() -> Matrix4<f64> {
    Matrix4::new_translation(&Vector3::new(0.0, 0.0, HEAD_Z_OFFSET))
}

/// Convert the 16-float row-major head matrix we get over the wire into
/// the `Matrix4<f64>` the kinematics crate wants, with the neutral-pose
/// z-offset added (client identity = robot neutral).
pub fn head_wire_to_pose(head: &[f64; 16]) -> Matrix4<f64> {
    let client = Matrix4::from_row_slice(head);
    let mut pose = client;
    pose[(2, 3)] += HEAD_Z_OFFSET;
    pose
}

/// Inverse of `head_wire_to_pose`.
pub fn pose_to_head_wire(pose: &Matrix4<f64>) -> [f64; 16] {
    let mut out = [0.0; 16];
    for i in 0..4 {
        for j in 0..4 {
            out[i * 4 + j] = pose[(i, j)];
        }
    }
    out[2 * 4 + 3] -= HEAD_Z_OFFSET;
    out
}

fn mat4_from_rows(rows: &[Vec<f64>]) -> Result<Matrix4<f64>> {
    if rows.len() != 4 || rows.iter().any(|r| r.len() != 4) {
        anyhow::bail!("expected 4x4 matrix");
    }
    Ok(Matrix4::new(
        rows[0][0], rows[0][1], rows[0][2], rows[0][3], //
        rows[1][0], rows[1][1], rows[1][2], rows[1][3], //
        rows[2][0], rows[2][1], rows[2][2], rows[2][3], //
        rows[3][0], rows[3][1], rows[3][2], rows[3][3], //
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ik_at_neutral_returns_six_angles() {
        // The joint angles at the neutral pose are _not_ zero — they're
        // whatever the physical assembly dictates. Just check that IK
        // runs and produces 6 values.
        let mut k = build_kinematics().unwrap();
        let angles = k.inverse_kinematics(neutral_pose(), None);
        assert_eq!(angles.len(), 6);
    }

    #[test]
    fn fk_ik_roundtrip_preserves_neutral_pose() {
        let mut k = build_kinematics().unwrap();
        let pose = neutral_pose();
        let angles = k.inverse_kinematics(pose, None);
        k.reset_forward_kinematics(pose);
        let back = k.forward_kinematics(angles, None);
        for i in 0..4 {
            for j in 0..4 {
                assert!(
                    (pose[(i, j)] - back[(i, j)]).abs() < 1e-4,
                    "FK(IK(neutral)) drift at [{i},{j}]: {} vs {}",
                    pose[(i, j)],
                    back[(i, j)]
                );
            }
        }
    }

    #[test]
    fn wire_roundtrip_identity() {
        let identity = {
            let mut h = [0.0; 16];
            h[0] = 1.0;
            h[5] = 1.0;
            h[10] = 1.0;
            h[15] = 1.0;
            h
        };
        let pose = head_wire_to_pose(&identity);
        let back = pose_to_head_wire(&pose);
        for (a, b) in identity.iter().zip(back.iter()) {
            assert!((a - b).abs() < 1e-9);
        }
    }
}
