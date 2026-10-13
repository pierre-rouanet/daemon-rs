//! Goto time-trajectory shaping and SO(3)-correct pose interpolation.
//!
//! Mirrors `reachy_mini/utils/interpolation.py`:
//!   - `InterpolationTechnique` enum (same wire strings: "linear",
//!     "minjerk", "ease_in_out", "cartoon")
//!   - `time_trajectory(t, method)` — reshape normalized time t ∈ [0, 1]
//!   - `linear_pose_interpolation(start, end, s)` — SLERP the rotation,
//!     lerp the translation. Fixes Issue #5: interpolating the 4x4 matrix
//!     elementwise skews the rotation (not an SO(3) geodesic).
//!
//! The two together reproduce the Python daemon's goto exactly:
//! compute `s = time_trajectory(t, method)` once per tick, then pass
//! `s` as the blend weight to everything (head pose, antennas, body_yaw,
//! joint-space head_joints fallback).
//!
//! Row-major convention: head matrices cross the wire as 16 floats in
//! row-major order. nalgebra's `Matrix4::from_row_slice` handles that.

use nalgebra::{Matrix3, Matrix4, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum InterpolationTechnique {
    #[serde(rename = "linear")]
    Linear,
    #[serde(rename = "minjerk")]
    MinJerk,
    #[serde(rename = "ease_in_out")]
    EaseInOut,
    #[serde(rename = "cartoon")]
    Cartoon,
}

impl Default for InterpolationTechnique {
    fn default() -> Self {
        // Python's default: `InterpolationTechnique.MIN_JERK`.
        Self::MinJerk
    }
}

/// Reshape normalized time. Input and output are both in [0, 1] at the
/// endpoints; the curve in between is what distinguishes the methods.
/// Ported verbatim from `reachy_mini/utils/interpolation.py:time_trajectory`.
pub fn time_trajectory(t: f64, method: InterpolationTechnique) -> f64 {
    let t = t.clamp(0.0, 1.0);
    match method {
        InterpolationTechnique::Linear => t,
        InterpolationTechnique::MinJerk => {
            10.0 * t.powi(3) - 15.0 * t.powi(4) + 6.0 * t.powi(5)
        }
        InterpolationTechnique::EaseInOut => {
            if t < 0.5 {
                2.0 * t * t
            } else {
                1.0 - (-2.0 * t + 2.0).powi(2) / 2.0
            }
        }
        InterpolationTechnique::Cartoon => {
            let c1 = 1.70158_f64;
            let c2 = c1 * 1.525;
            if t < 0.5 {
                ((2.0 * t).powi(2) * ((c2 + 1.0) * 2.0 * t - c2)) / 2.0
            } else {
                ((2.0 * t - 2.0).powi(2) * ((c2 + 1.0) * (2.0 * t - 2.0) + c2) + 2.0) / 2.0
            }
        }
    }
}

/// SO(3)-correct pose interpolation between two row-major 4x4 matrices
/// at blend weight `s` ∈ [0, 1]. Rotation via quaternion SLERP, translation
/// via plain lerp — matches Python's `linear_pose_interpolation` which
/// uses `R.from_rotvec(rotvec_rel * t)` (axis-angle scaling = SLERP).
pub fn linear_pose_interpolation(start: &[f64; 16], end: &[f64; 16], s: f64) -> [f64; 16] {
    let start_m = Matrix4::from_row_slice(start);
    let end_m = Matrix4::from_row_slice(end);

    let r_start = rotation_part(&start_m);
    let r_end = rotation_part(&end_m);
    let q_start = UnitQuaternion::from_rotation_matrix(&nalgebra::Rotation3::from_matrix(&r_start));
    let q_end = UnitQuaternion::from_rotation_matrix(&nalgebra::Rotation3::from_matrix(&r_end));
    let q = q_start.slerp(&q_end, s);
    let r: Matrix3<f64> = q.to_rotation_matrix().into_inner();

    let t_start = Vector3::new(start_m[(0, 3)], start_m[(1, 3)], start_m[(2, 3)]);
    let t_end = Vector3::new(end_m[(0, 3)], end_m[(1, 3)], end_m[(2, 3)]);
    let t = t_start.lerp(&t_end, s);

    let mut out = [0.0; 16];
    for i in 0..3 {
        for j in 0..3 {
            out[i * 4 + j] = r[(i, j)];
        }
        out[i * 4 + 3] = t[i];
    }
    out[15] = 1.0;
    out
}

fn rotation_part(m: &Matrix4<f64>) -> Matrix3<f64> {
    Matrix3::new(
        m[(0, 0)], m[(0, 1)], m[(0, 2)],
        m[(1, 0)], m[(1, 1)], m[(1, 2)],
        m[(2, 0)], m[(2, 1)], m[(2, 2)],
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn time_trajectory_endpoints() {
        for m in [
            InterpolationTechnique::Linear,
            InterpolationTechnique::MinJerk,
            InterpolationTechnique::EaseInOut,
            InterpolationTechnique::Cartoon,
        ] {
            assert!((time_trajectory(0.0, m) - 0.0).abs() < 1e-9, "{m:?} @ 0");
            assert!((time_trajectory(1.0, m) - 1.0).abs() < 1e-9, "{m:?} @ 1");
        }
    }

    #[test]
    fn time_trajectory_midpoint_python_parity() {
        // Values computed directly from the Python polynomials at t=0.5.
        assert!((time_trajectory(0.5, InterpolationTechnique::Linear) - 0.5).abs() < 1e-9);
        // 10*.125 − 15*.0625 + 6*.03125 = 1.25 − 0.9375 + 0.1875 = 0.5
        assert!((time_trajectory(0.5, InterpolationTechnique::MinJerk) - 0.5).abs() < 1e-9);
        // EaseInOut: the t<0.5 branch at exactly 0.5 falls to the else branch,
        // 1 − ((-1+2)^2)/2 = 1 − 0.5 = 0.5. The t<0.5 branch would give 0.5 too.
        assert!((time_trajectory(0.5, InterpolationTechnique::EaseInOut) - 0.5).abs() < 1e-9);
        // Cartoon passes through 0.5 at t=0.5 (symmetric overshoot).
        assert!((time_trajectory(0.5, InterpolationTechnique::Cartoon) - 0.5).abs() < 1e-9);
    }

    #[test]
    fn minjerk_is_slow_at_start_and_end() {
        // Min-jerk's distinguishing feature vs linear: slower near endpoints,
        // faster in the middle.
        assert!(time_trajectory(0.1, InterpolationTechnique::MinJerk) < 0.1);
        assert!(time_trajectory(0.9, InterpolationTechnique::MinJerk) > 0.9);
    }

    #[test]
    fn pose_interp_identity_endpoints() {
        let a = identity_head();
        let b = rot_z_head(std::f64::consts::FRAC_PI_2);
        let at0 = linear_pose_interpolation(&a, &b, 0.0);
        let at1 = linear_pose_interpolation(&a, &b, 1.0);
        for i in 0..16 {
            assert!((at0[i] - a[i]).abs() < 1e-9, "start idx {i}: {} vs {}", at0[i], a[i]);
            assert!((at1[i] - b[i]).abs() < 1e-9, "end idx {i}: {} vs {}", at1[i], b[i]);
        }
    }

    #[test]
    fn pose_interp_90deg_yaw_midpoint_is_45deg_and_orthonormal() {
        let a = identity_head();
        let b = rot_z_head(std::f64::consts::FRAC_PI_2);
        let mid = linear_pose_interpolation(&a, &b, 0.5);
        // Midpoint should be rot_z(45°).
        let expect = rot_z_head(std::f64::consts::FRAC_PI_4);
        for i in 0..16 {
            assert!(
                (mid[i] - expect[i]).abs() < 1e-9,
                "idx {i}: got {}, expected {}",
                mid[i],
                expect[i]
            );
        }
        // And the rotation block must be orthonormal — the whole point
        // of switching to SLERP. `R * R^T == I`.
        let r = Matrix3::new(
            mid[0], mid[1], mid[2],
            mid[4], mid[5], mid[6],
            mid[8], mid[9], mid[10],
        );
        let drift = (r * r.transpose() - Matrix3::identity()).norm();
        assert!(drift < 1e-9, "rotation not orthonormal: drift {drift}");
    }

    #[test]
    fn pose_interp_translates_linearly() {
        let mut a = identity_head();
        let mut b = identity_head();
        a[3] = 0.0;
        a[7] = 0.0;
        a[11] = 0.0;
        b[3] = 1.0;
        b[7] = 2.0;
        b[11] = 3.0;
        let mid = linear_pose_interpolation(&a, &b, 0.25);
        assert!((mid[3] - 0.25).abs() < 1e-9);
        assert!((mid[7] - 0.5).abs() < 1e-9);
        assert!((mid[11] - 0.75).abs() < 1e-9);
    }

    #[test]
    fn method_serde_wire_strings() {
        // These strings must match Python exactly — clients send them.
        let s = serde_json::to_string(&InterpolationTechnique::Linear).unwrap();
        assert_eq!(s, "\"linear\"");
        let s = serde_json::to_string(&InterpolationTechnique::MinJerk).unwrap();
        assert_eq!(s, "\"minjerk\"");
        let s = serde_json::to_string(&InterpolationTechnique::EaseInOut).unwrap();
        assert_eq!(s, "\"ease_in_out\"");
        let s = serde_json::to_string(&InterpolationTechnique::Cartoon).unwrap();
        assert_eq!(s, "\"cartoon\"");

        let m: InterpolationTechnique = serde_json::from_str("\"minjerk\"").unwrap();
        assert_eq!(m, InterpolationTechnique::MinJerk);
    }

    fn identity_head() -> [f64; 16] {
        let mut h = [0.0; 16];
        h[0] = 1.0;
        h[5] = 1.0;
        h[10] = 1.0;
        h[15] = 1.0;
        h
    }

    /// Row-major 4x4 of a pure rotation about Z by `angle` rad.
    fn rot_z_head(angle: f64) -> [f64; 16] {
        let c = angle.cos();
        let s = angle.sin();
        #[rustfmt::skip]
        let m = [
             c,  -s, 0.0, 0.0,
             s,   c, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        ];
        m
    }
}
