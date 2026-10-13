//! IMU device trait + mock implementation.
//!
//! Fields mirror the BMI088 driver from `gstreamer_data_capture/imu`
//! (`ImuReading { accelerometer, gyroscope, quaternion, temperature }`),
//! so a `RealImu` wrapper can forward `read()` directly once Phase 3 lands.

/// Single IMU sample.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuReading {
    pub accelerometer: [f64; 3],
    pub gyroscope: [f64; 3],
    /// Madgwick-fused orientation (w, x, y, z).
    pub quaternion: [f64; 4],
    pub temperature: f64,
}

impl Default for ImuReading {
    fn default() -> Self {
        Self {
            accelerometer: [0.0, 0.0, 9.81],
            gyroscope: [0.0; 3],
            quaternion: [1.0, 0.0, 0.0, 0.0],
            temperature: 25.0,
        }
    }
}

pub trait Imu: Send {
    fn read(&mut self) -> ImuReading;
}

/// Returns a fixed reading (gravity-down, zero gyro, identity quat, 25 °C).
/// Enough for the sensor-publishing pipeline to move bytes; not a physics sim.
#[derive(Debug, Clone, Default)]
pub struct MockImu;

impl MockImu {
    pub fn new() -> Self {
        Self
    }
}

impl Imu for MockImu {
    fn read(&mut self) -> ImuReading {
        ImuReading::default()
    }
}

// ---------------------------------------------------------------------------
// Real IMU (BMI088 over I2C) — Linux only
// ---------------------------------------------------------------------------
//
// The low-level driver + Madgwick fusion lives in the
// `gstreamer_data_capture/imu` workspace crate. It binds `i2cdev::linux`
// so is compiled in only for Linux targets (the Pi). On other hosts
// `RealImu` is absent; `main.rs` picks `MockImu` automatically.

#[cfg(target_os = "linux")]
pub use linux_impl::RealImu;

#[cfg(target_os = "linux")]
mod linux_impl {
    use super::{Imu, ImuReading};
    use tracing::warn;

    /// BMI088 IMU on I2C, with Madgwick-fused orientation.
    ///
    /// On read failure, holds the last successful sample (same strategy as
    /// `SerialMotors`: hardware glitches shouldn't flash zeros through the
    /// broadcast).
    pub struct RealImu {
        inner: imu::Bmi088,
        last_known: ImuReading,
    }

    impl RealImu {
        /// Open the BMI088 on the given I2C bus (e.g. `1` for `/dev/i2c-1`).
        pub fn new(i2c_bus: u8) -> anyhow::Result<Self> {
            let mut inner = imu::Bmi088::new(i2c_bus)?;
            // Seed `last_known` with a first read so the initial broadcast
            // isn't the default sample.
            let first = match inner.read_all() {
                Ok(r) => from_driver(&r),
                Err(e) => {
                    warn!("initial IMU read failed: {e} — starting with default");
                    ImuReading::default()
                }
            };
            Ok(Self {
                inner,
                last_known: first,
            })
        }
    }

    impl Imu for RealImu {
        fn read(&mut self) -> ImuReading {
            match self.inner.read_all() {
                Ok(r) => {
                    let r = from_driver(&r);
                    self.last_known = r;
                    r
                }
                Err(e) => {
                    warn!("IMU read failed: {e} (holding last known)");
                    self.last_known
                }
            }
        }
    }

    fn from_driver(r: &imu::ImuReading) -> ImuReading {
        ImuReading {
            accelerometer: r.accelerometer,
            gyroscope: r.gyroscope,
            quaternion: r.quaternion,
            temperature: r.temperature,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mock_reads_default() {
        let mut imu = MockImu::new();
        let r = imu.read();
        assert_eq!(r.accelerometer, [0.0, 0.0, 9.81]);
        assert_eq!(r.quaternion, [1.0, 0.0, 0.0, 0.0]);
    }
}
