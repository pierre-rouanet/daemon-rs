//! Audio source detection for Reachy Mini.
//!
//! Mirrors `reachy_mini/media/device_detection.py::find_audio_device` for the
//! wireless case: walk a `Gst.DeviceMonitor` filtered on `Audio/Source`, pick
//! the first device whose display name contains "Reachy Mini Audio" (and isn't
//! a PulseAudio monitor source), and return a ready-to-link source element.
//!
//! Unlike the Python code we don't have to stringify a platform-specific ID:
//! `gst::Device::create_element` gives us an element already bound to the
//! right device regardless of backend (PipeWire, PulseAudio, raw ALSA).
//!
//! `REACHYMINI_AUDIO_DEVICE=none` disables the audio branch entirely (useful
//! when the mic is physically broken, which is why this function returns
//! `Option` rather than erroring out).

use anyhow::{Context, Result};
use gstreamer as gst;
use gstreamer::prelude::*;
use tracing::{info, warn};

/// Substring matched against `Gst.Device::display_name`. The wireless
/// Reachy Mini's USB Audio Class device advertises itself under this name.
const AUDIO_TARGET: &str = "Reachy Mini Audio";

/// Find the Reachy Mini microphone and build a source element for it.
///
/// Returns `Ok(None)` when:
/// - `REACHYMINI_AUDIO_DEVICE=none` is set, or
/// - no matching device is present (e.g. broken mic, robot not plugged in).
///
/// Returns `Err` only when the monitor itself fails to start.
pub fn make_audio_source() -> Result<Option<gst::Element>> {
    if std::env::var("REACHYMINI_AUDIO_DEVICE").as_deref() == Ok("none") {
        info!("Audio disabled via REACHYMINI_AUDIO_DEVICE=none");
        return Ok(None);
    }

    let monitor = gst::DeviceMonitor::new();
    monitor
        .add_filter(Some("Audio/Source"), None)
        .context("Failed to add Audio/Source filter to DeviceMonitor")?;
    monitor.start().context("Failed to start DeviceMonitor")?;
    let devices = monitor.devices();
    monitor.stop();

    for device in devices {
        let name = device.display_name();
        if !name.as_str().contains(AUDIO_TARGET) {
            continue;
        }

        // Skip PulseAudio monitor sources (loopback of a sink).
        if let Some(props) = device.properties() {
            if matches!(
                props.get_optional::<String>("device.class"),
                Ok(Some(ref c)) if c == "monitor"
            ) {
                continue;
            }
        }

        let element = device
            .create_element(None)
            .context("Failed to create element from matched audio device")?;
        let factory = element
            .factory()
            .map(|f| f.name().to_string())
            .unwrap_or_default();
        info!("Audio source: {} ({})", name, factory);
        return Ok(Some(element));
    }

    warn!("No '{}' Audio/Source found — audio branch disabled", AUDIO_TARGET);
    Ok(None)
}
