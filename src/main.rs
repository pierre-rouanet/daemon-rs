use std::net::SocketAddr;

use anyhow::{Context, Result};
use gstreamer as gst;
use gstreamer::prelude::*;
use tracing::{error, info, warn};

mod api;
mod audio;
mod control;
mod devices;
mod interpolation;
mod kinematics;
mod media;
mod protocol;
mod relay;
mod robot;
mod video;

const CONTROL_ADDR: &str = "0.0.0.0:8080";

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "daemon_rs=info".parse().unwrap()),
        )
        .init();

    gst::init()?;

    // -- Control API on tokio --
    let motors: Box<dyn devices::Motors> =
        match std::env::var("REACHYMINI_SERIAL").ok().as_deref() {
            Some("none") | None if cfg!(target_os = "linux") => {
                // Auto-detect: try /dev/ttyUSB0 on Linux, fall back to mock.
                match devices::SerialMotors::new("/dev/ttyUSB0") {
                    Ok(m) => {
                        info!("Motors: serial /dev/ttyUSB0");
                        Box::new(m)
                    }
                    Err(e) => {
                        warn!("Motors: mock (serial failed: {e})");
                        Box::new(devices::MockMotors::new())
                    }
                }
            }
            Some("none") => {
                info!("Motors: mock (REACHYMINI_SERIAL=none)");
                Box::new(devices::MockMotors::new())
            }
            Some(port) => {
                let m = devices::SerialMotors::new(port)
                    .expect(&format!("Failed to open serial port {port}"));
                info!("Motors: serial {port}");
                Box::new(m)
            }
            None => {
                info!("Motors: mock (not Linux, no REACHYMINI_SERIAL)");
                Box::new(devices::MockMotors::new())
            }
        };
    // IMU: real BMI088 on I2C bus 1 (Pi default) if available, mock otherwise.
    // Can be forced to mock with REACHYMINI_IMU=none.
    let imu: Box<dyn devices::Imu> = pick_imu();
    let kin = kinematics::build_kinematics().context("build kinematics")?;
    let core = robot::RobotCore::spawn(motors, imu, kin);

    // -- Media pipeline on a dedicated std thread --
    //
    // The GStreamer bus loop is a blocking iterator; keeping it off the
    // tokio runtime avoids accidentally starving axum tasks if the bus
    // stalls. The pipeline is built here (after core exists) so data
    // channel bridging can dispatch commands through RobotCore.
    let pipeline = media::build_pipeline(Some(core.clone()))?;
    pipeline
        .set_state(gst::State::Playing)
        .context("Failed to set media pipeline to Playing")?;
    info!("Media: WebRTC signalling on ws://0.0.0.0:8443");

    // Central signaling relay: connect to HF central server if HF_TOKEN is set.
    if let Some(hf_token) = std::env::var("HF_TOKEN").ok().filter(|s| !s.is_empty()) {
        let robot_name = std::env::var("ROBOT_NAME").unwrap_or_else(|_| "reachymini".into());
        relay::spawn(hf_token, robot_name);
    }

    let pipeline_for_thread = pipeline.clone();
    std::thread::spawn(move || {
        if let Err(e) = media::run_bus_loop(&pipeline_for_thread) {
            error!("Media bus loop exited with error: {e:?}");
        }
    });

    // Merge WS + REST routers, with permissive CORS for cross-origin clients.
    let cors = tower_http::cors::CorsLayer::permissive();
    let app = control::build_router(core.clone())
        .nest("/api", api::build_router(core))
        .layer(cors);

    let addr: SocketAddr = CONTROL_ADDR.parse().expect("invalid CONTROL_ADDR");
    let listener = tokio::net::TcpListener::bind(addr).await?;
    info!("Control API listening on http://{addr}  (WS: /ws/sdk, REST: /api/*)");

    let ctrl_c = tokio::signal::ctrl_c();
    let server = axum::serve(listener, app);

    tokio::select! {
        res = server => {
            if let Err(e) = res {
                error!("Control server exited with error: {e:?}");
            }
        }
        _ = ctrl_c => {
            info!("Ctrl+C — shutting down (Ctrl-C again to force exit)");
        }
    }

    // GStreamer set_state(Null) can take seconds to walk Playing→Paused→Ready→Null
    // while WebRTC peers tear down. Arm a second Ctrl-C as an escape hatch.
    tokio::spawn(async {
        let _ = tokio::signal::ctrl_c().await;
        error!("Force exit");
        std::process::exit(130);
    });

    pipeline.set_state(gst::State::Null)?;
    Ok(())
}

#[cfg(target_os = "linux")]
fn pick_imu() -> Box<dyn devices::Imu> {
    if matches!(std::env::var("REACHYMINI_IMU").ok().as_deref(), Some("none")) {
        info!("IMU: mock (REACHYMINI_IMU=none)");
        return Box::new(devices::MockImu::new());
    }
    match devices::RealImu::new(1) {
        Ok(imu) => {
            info!("IMU: BMI088 on /dev/i2c-1");
            Box::new(imu)
        }
        Err(e) => {
            warn!("IMU: mock (BMI088 init failed: {e})");
            Box::new(devices::MockImu::new())
        }
    }
}

#[cfg(not(target_os = "linux"))]
fn pick_imu() -> Box<dyn devices::Imu> {
    info!("IMU: mock (real BMI088 driver is Linux-only)");
    Box::new(devices::MockImu::new())
}
