//! Video device probe for Reachy Mini wireless.
//!
//! Mirrors the pattern in `audio.rs`: walk a `Gst.DeviceMonitor` filtered
//! on `Video/Source` and check whether anything shows up.  Used to decide
//! whether to build the video branch at all, so the daemon can run on a
//! host without a camera (dev laptop, CM4 with the CSI cable unplugged).
//!
//! We don't bother narrowing by provider (libcamera vs v4l2) — PLAN.md §1
//! explicitly scopes the pipeline to libcamera/imx708, so the only
//! realistic "Video/Source" on a reachy-mini-os host is the CSI camera.

use anyhow::{Context, Result};
use gstreamer as gst;
use gstreamer::prelude::*;
use tracing::info;

/// Return `true` if at least one video capture device is currently visible.
pub fn has_video_source() -> Result<bool> {
    let monitor = gst::DeviceMonitor::new();
    monitor
        .add_filter(Some("Video/Source"), None)
        .context("Failed to add Video/Source filter to DeviceMonitor")?;
    monitor.start().context("Failed to start DeviceMonitor")?;
    let devices = monitor.devices();
    monitor.stop();

    if let Some(first) = devices.iter().next() {
        info!("Video source found: {}", first.display_name());
        Ok(true)
    } else {
        Ok(false)
    }
}
