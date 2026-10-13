//! Hardware abstraction: motors and IMU behind traits.
//!
//! The trait boundary exists so the daemon can run against mocks on a
//! dev laptop and against real drivers on the robot without changes to
//! `robot.rs`. Only kinematics is a direct dependency — it's pure math,
//! has no hardware, and there's nothing useful a mock would add.
//!
//! The traits are deliberately narrow: read a snapshot, write a target.
//! Anything more structured (trajectories, velocity control, torque
//! modes) is a Phase 3 concern.

pub mod imu;
pub mod motors;

pub use imu::{Imu, MockImu};
#[cfg(target_os = "linux")]
pub use imu::RealImu;
pub use motors::{MockMotors, MotorPositions, MotorTargets, Motors, SerialMotors};
