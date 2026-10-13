//! GStreamer media pipeline for Reachy Mini wireless.
//!
//! Mirrors `reachy_mini/media/media_server.py::_build_libcamera_source` +
//! `::_build_rpi_encoder_branch`, minus the tee / IPC branch (remote-only).
//!
//! Pipeline shape:
//!
//! ```text
//!   libcamerasrc → YUY2 caps ┐
//!                            ├─ webrtcsink ──[data channel per peer]
//!      Reachy Mini USB Audio ┘
//! ```
//!
//! webrtcsink owns the H.264 encoder internally: with the patched
//! `gstvideo4linux2` + `libgstpbutils` on the Pi, webrtcsink's encoder
//! discovery picks `v4l2h264enc` and drives its `bitrate` property for
//! congestion-controlled adaptive bitrate. We pin `video-caps` to
//! `video/x-h264` so webrtcsink doesn't fall back to VP8.
//!
//! The audio branch is skipped when no "Reachy Mini Audio" device is
//! present (see `audio::make_audio_source`).
//!
//! **Data channel bridging**: when a WebRTC peer connects, webrtcsink emits
//! `consumer-added`. We create a data channel named `"data"` on the
//! per-peer `webrtcbin`, then wire `on-message-string` → `RobotCore::dispatch`
//! → `send-string` to bridge JSON commands identically to the WebSocket path.

use std::sync::Arc;

use anyhow::{Context, Result};
use gstreamer as gst;
use gstreamer::glib;
use gstreamer::prelude::*;
use tracing::{debug, error, info, warn};

use crate::protocol::AnyMessage;
use crate::robot::RobotCore;
use crate::{audio, video};

const WIDTH: i32 = 1280;
const HEIGHT: i32 = 720;
const FRAMERATE: i32 = 30;

/// Build the full media pipeline: (optional) video + (optional) audio → webrtcsink.
/// Pipeline is returned in `Null` state — call `set_state(Playing)` to start.
///
/// When a `RobotCore` is provided, data channel bridging is set up so that
/// WebRTC peers can send JSON commands (same protocol as the WebSocket API).
///
/// Both branches are optional so the daemon can run on a host with no
/// camera (dev laptop, CM4 with the CSI cable unplugged) or no mic
/// (wireless robot with a broken mic).
pub fn build_pipeline(core: Option<Arc<RobotCore>>) -> Result<gst::Pipeline> {
    let pipeline = gst::Pipeline::with_name("reachymini_webrtc_sender");

    let webrtcsink = build_webrtcsink()?;
    pipeline.add(&webrtcsink)?;

    spawn_stats_logger(&webrtcsink);
    spawn_keyframe_pusher(&pipeline, &webrtcsink);

    // Data channel bridging: if we have a RobotCore, wire up the signals.
    if let Some(core) = core {
        setup_data_channel_bridge(&webrtcsink, core);
    }

    // Video branch: skip gracefully when no camera is attached, or when
    // construction of the libcamera/v4l2 elements fails (missing plugins).
    match video::has_video_source() {
        Ok(true) => {
            if let Err(e) = build_video_branch(&pipeline, &webrtcsink) {
                warn!("Video branch disabled (build failed): {e:#}");
            }
        }
        Ok(false) => warn!("Video branch disabled: no Video/Source devices"),
        Err(e) => warn!("Video branch disabled (probe failed): {e:#}"),
    }

    // Audio branch: skip gracefully when the mic isn't found.
    if let Some(audiosrc) = audio::make_audio_source()? {
        pipeline.add(&audiosrc)?;
        audiosrc.link(&webrtcsink)?;
    }

    Ok(pipeline)
}

/// Drive the pipeline's bus until EOS or error. Blocks the current thread.
pub fn run_bus_loop(pipeline: &gst::Pipeline) -> Result<()> {
    let bus = pipeline.bus().context("Pipeline has no bus")?;
    for msg in bus.iter_timed(gst::ClockTime::NONE) {
        match msg.view() {
            gst::MessageView::Eos(..) => {
                info!("End of stream");
                break;
            }
            gst::MessageView::Error(err) => {
                error!(
                    "Error from {:?}: {} ({:?})",
                    err.src().map(|s| s.path_string()),
                    err.error(),
                    err.debug()
                );
                break;
            }
            gst::MessageView::StateChanged(state_changed) => {
                if state_changed
                    .src()
                    .is_some_and(|s| s == pipeline.upcast_ref::<gst::Object>())
                {
                    debug!(
                        "Pipeline state: {:?} -> {:?}",
                        state_changed.old(),
                        state_changed.current()
                    );
                }
            }
            _ => {}
        }
    }
    Ok(())
}

fn build_webrtcsink() -> Result<gst::Element> {
    let webrtcsink = gst::ElementFactory::make("webrtcsink")
        .build()
        .context("Failed to create webrtcsink. Is gst-plugins-rs installed?")?;

    let meta = gst::Structure::builder("meta")
        .field("name", "reachymini")
        .build();
    webrtcsink.set_property("meta", meta);
    webrtcsink.set_property("run-signalling-server", true);

    // Pin the output codec to H.264 — the Pi has v4l2h264enc for hardware
    // offload; VP8 would silently fall back to a software encoder.
    let video_caps = gst::Caps::builder("video/x-h264").build();
    webrtcsink.set_property("video-caps", &video_caps);

    // Initial bitrate: 5 Mbps. webrtcsink's congestion controller will
    // ramp this up/down via the encoder's `bitrate` property at runtime.
    webrtcsink.set_property("start-bitrate", 5_000_000u32);
    // Force GCC on — defaults vary across webrtcsink versions and we've
    // observed `mitigation-mode = "No mitigation applied"` despite heavy
    // packet loss, which is what `congestion-control = disabled` looks like.
    webrtcsink.set_property_from_str("congestion-control", "homegrown");

    Ok(webrtcsink)
}

/// Poll `webrtcsink.stats` once per second and log per-session bitrate +
/// loss counters. Confirms whether GCC is actually adapting the encoder
/// bitrate in response to TWCC feedback (vs. sitting pinned at start-bitrate).
fn spawn_stats_logger(webrtcsink: &gst::Element) {
    let sink = webrtcsink.clone();
    tokio::spawn(async move {
        let mut tick = tokio::time::interval(std::time::Duration::from_secs(1));
        let mut prev_bytes_sent: std::collections::HashMap<String, u64> =
            std::collections::HashMap::new();
        loop {
            tick.tick().await;
            let stats: gst::Structure = sink.property("stats");
            for (session_id, value) in stats.iter() {
                let Ok(session) = value.get::<gst::Structure>() else { continue };
                log_session_summary(session_id, &session, &mut prev_bytes_sent);
            }
        }
    });
}

fn log_session_summary(
    session_id: &str,
    session: &gst::Structure,
    prev_bytes_sent: &mut std::collections::HashMap<String, u64>,
) {
    let mut enc_kbps: Option<i32> = None;
    let mut mitigation = String::from("?");
    let mut twcc_sent_kbps: Option<u32> = None;
    let mut loss_pct: Option<f64> = None;
    let mut rb_fraction: Option<f64> = None;
    let mut rtt_ms: Option<f64> = None;
    let mut nacks: Option<u32> = None;
    let mut plis: Option<u32> = None;
    let mut firs: Option<u32> = None;
    let mut bytes_sent: Option<u64> = None;

    for (key, value) in session.iter() {
        let Ok(s) = value.get::<gst::Structure>() else { continue };

        if let Ok(twcc) = s.get::<gst::Structure>("gst-twcc-stats") {
            twcc_sent_kbps = twcc.get::<u32>("bitrate-sent").ok().map(|b| b / 1000);
            loss_pct = twcc.get::<f64>("packet-loss-pct").ok();
        }
        if key.starts_with("rtp-remote-inbound-stream-stats") {
            rb_fraction = s.get::<f64>("fraction-lost").ok();
            rtt_ms = s.get::<f64>("round-trip-time").ok().map(|v| v * 1000.0);
        }
        if key.starts_with("rtp-outbound-stream-stats") {
            nacks = s.get::<u32>("nack-count").ok();
            plis = s.get::<u32>("pli-count").ok();
            firs = s.get::<u32>("fir-count").ok();
            bytes_sent = s.get::<u64>("bytes-sent").ok();
        }
        if key == "consumer-stats" {
            // `video-encoders` is a GstValueList of Structures. The Rust
            // FromValue bridge for it is fragile across gst-plugins-rs
            // versions, so we fall back to scanning the serialized form.
            // GstStructure escapes special chars with `\` inside nested
            // strings; stripping every `\` is safe for our ASCII fields.
            let raw = s.to_string().replace('\\', "");
            if let Some(after) = raw.split("bitrate=(int)").nth(1) {
                let digits: String =
                    after.chars().take_while(|c| c.is_ascii_digit()).collect();
                enc_kbps = digits.parse::<i32>().ok().map(|b| b / 1000);
            }
            if let Some(after) =
                raw.split("mitigation-mode=(GstWebRTCSinkMitigationMode)").nth(1)
            {
                let m: String = after
                    .chars()
                    .take_while(|&c| c != ',' && c != ';' && c != '"')
                    .collect();
                mitigation = m.trim().to_string();
            }
        }
    }

    fn fmt_i<T: std::fmt::Display>(v: Option<T>) -> String {
        v.map(|x| x.to_string()).unwrap_or_else(|| "?".into())
    }
    fn fmt_f(v: Option<f64>, decimals: usize) -> String {
        match v {
            Some(x) if x.is_finite() => format!("{:.*}", decimals, x),
            _ => "?".into(),
        }
    }
    // Compute actual outbound rate by diffing cumulative bytes-sent against
    // the previous tick (~1 s apart). When this drops to ~0 while enc>0,
    // packets are being encoded but not leaving the socket — strong signal
    // that webrtcsink's per-consumer pipeline or the ICE path is stuck.
    let actual_kbps = bytes_sent.map(|b| {
        let prev = prev_bytes_sent
            .insert(session_id.to_string(), b)
            .unwrap_or(b);
        ((b.saturating_sub(prev)) * 8 / 1000) as u32
    });

    info!(
        "GCC: enc={:>5} kbps wire={:>5} kbps mit={:<25} twcc_sent={:>5} kbps loss={:>5}% rb_lost={:>4} rtt={:>4}ms pli={} fir={} nacks={}",
        fmt_i(enc_kbps),
        fmt_i(actual_kbps),
        mitigation,
        fmt_i(twcc_sent_kbps),
        fmt_f(loss_pct, 1),
        fmt_f(rb_fraction, 2),
        fmt_f(rtt_ms, 0),
        fmt_i(plis),
        fmt_i(firs),
        fmt_i(nacks),
    );
}

/// Push a downstream `force-key-unit` event every 2 seconds so the encoder
/// produces a keyframe on a predictable wall-clock cadence.
///
/// Why: webrtcsink's adaptive controller drops *framerate* under sustained
/// loss. With a frame-counted GOP (e.g. v4l2h264enc gop=60 at 30 fps = 2 s
/// keyframe interval), lowered framerate stretches the keyframe interval
/// in wall-time — at 5 fps that GOP becomes 12 s. If a keyframe is lost,
/// the decoder freezes for that long.
///
/// We try two injection points and accept whichever the encoder honors:
///  1. `pipeline.send_event` — routes through source elements (camerasrc).
///  2. `webrtcsink.<video sink pad>.send_event` — lands directly at
///     webrtcsink's entry point and propagates into its internal pipeline.
///
/// Empirically the pipeline-level route returned `false` in this build, so
/// we keep the sink-pad fallback as the primary working path.
fn spawn_keyframe_pusher(pipeline: &gst::Pipeline, webrtcsink: &gst::Element) {
    let pipeline = pipeline.clone();
    let sink = webrtcsink.clone();
    tokio::spawn(async move {
        let mut tick = tokio::time::interval(std::time::Duration::from_secs(2));
        tick.tick().await; // skip the immediate first fire
        loop {
            tick.tick().await;
            push_force_key_unit(&pipeline, &sink);
        }
    });
}

fn push_force_key_unit(pipeline: &gst::Pipeline, webrtcsink: &gst::Element) {
    let make_event = || {
        let s = gst::Structure::builder("GstForceKeyUnit")
            .field("all-headers", true)
            .field("count", 0u32)
            .build();
        gst::event::CustomDownstream::new(s)
    };

    let via_pipeline = pipeline.send_event(make_event());

    let mut via_sink_pad = false;
    for pad in webrtcsink.sink_pads() {
        if pad.name().starts_with("video") && pad.send_event(make_event()) {
            via_sink_pad = true;
        }
    }

    if !via_pipeline && !via_sink_pad {
        debug!("force-key-unit not handled (no peer connected yet?)");
    }
}

// ---------------------------------------------------------------------------
// Data channel bridging
// ---------------------------------------------------------------------------

/// Thin wrapper to move `glib::Object` across thread boundaries.
///
/// GStreamer objects are internally thread-safe (atomic refcount, signal
/// emission is protected by mutexes). The Rust bindings don't mark them
/// `Send`/`Sync` because of the inner raw pointer, but crossing threads
/// with `emit_by_name("send-string", …)` is safe in practice — the
/// Python daemon does the equivalent via `asyncio.call_soon_threadsafe`.
struct SendChannel(glib::Object);
unsafe impl Send for SendChannel {}
unsafe impl Sync for SendChannel {}

impl SendChannel {
    fn send_string(&self, msg: &str) {
        self.0.emit_by_name::<()>("send-string", &[&msg]);
    }
}

/// Log every transition of webrtcbin's `connection-state` and
/// `ice-connection-state`. When a stream goes silent without webrtcsink
/// removing the consumer, these states are usually the smoking gun:
/// `ice-connection-state` flipping to `disconnected`/`failed` while no
/// consumer-removed signal fires means webrtcsink is not noticing the
/// dead path. State enums are read via `glib::Value::serialize` which
/// produces the enum nick (e.g. "connected", "failed").
fn watch_connection_state(webrtcbin: &gst::Element, peer_id: &str) {
    fn log_state(bin: &gst::Element, peer_id: &str, name: &str) {
        let s = bin
            .property_value(name)
            .serialize()
            .map(|gs| gs.to_string())
            .unwrap_or_else(|_| "?".into());
        info!("WebRTC peer {peer_id}: {name}={s}");
    }

    log_state(webrtcbin, peer_id, "connection-state");
    log_state(webrtcbin, peer_id, "ice-connection-state");

    let peer_for_conn = peer_id.to_string();
    webrtcbin.connect_notify(Some("connection-state"), move |bin, _| {
        log_state(bin, &peer_for_conn, "connection-state");
    });

    let peer_for_ice = peer_id.to_string();
    webrtcbin.connect_notify(Some("ice-connection-state"), move |bin, _| {
        log_state(bin, &peer_for_ice, "ice-connection-state");
    });
}

/// Connect to `consumer-added` on webrtcsink to create a data channel for
/// each WebRTC peer. Incoming messages are dispatched through `RobotCore`,
/// and per-client responses are sent back on the data channel.
///
/// Note: sensor broadcasts (50 Hz) are NOT relayed through data channels.
/// The JS SDK polls `get_state` instead — matching the Python daemon's
/// behaviour.
fn setup_data_channel_bridge(webrtcsink: &gst::Element, core: Arc<RobotCore>) {
    let rt = tokio::runtime::Handle::current();

    webrtcsink.connect("consumer-added", false, move |values| {
        // values: [webrtcsink: Element, peer_id: String, webrtcbin: Element]
        let peer_id: String = values[1].get().expect("peer_id is a String");
        let webrtcbin: gst::Element = values[2].get().expect("webrtcbin is an Element");

        info!("Data channel: peer {peer_id} connected");

        // Watch ICE/DTLS state transitions so we can see a stuck connection
        // (e.g. NAT mapping expiry, ICE consent failure) that webrtcsink
        // doesn't surface as a consumer-removed event.
        watch_connection_state(&webrtcbin, &peer_id);

        // Create an ordered data channel named "data" on this peer.
        let options = gst::Structure::builder("options")
            .field("ordered", true)
            .build();
        let channel: Option<glib::Object> = webrtcbin
            .emit_by_name("create-data-channel", &[&"data", &options]);

        let Some(channel) = channel else {
            error!("Data channel: failed to create for peer {peer_id}");
            return None;
        };

        // Channel for incoming messages: GStreamer thread → tokio task.
        let (in_tx, mut in_rx) =
            tokio::sync::mpsc::unbounded_channel::<String>();

        // Spawn a tokio task that dispatches messages and sends replies
        // back on the data channel.
        let send_ch = Arc::new(SendChannel(channel.clone()));
        let core_clone = core.clone();
        let peer_id_for_task = peer_id.clone();
        rt.spawn(async move {
            while let Some(text) = in_rx.recv().await {
                let msg: AnyMessage = match serde_json::from_str(&text) {
                    Ok(m) => m,
                    Err(e) => {
                        let err = serde_json::json!({
                            "type": "error",
                            "message": format!("bad command: {e}")
                        })
                        .to_string();
                        send_ch.send_string(&err);
                        continue;
                    }
                };
                let replies = core_clone.dispatch(msg).await;
                for reply in replies {
                    send_ch.send_string(&reply);
                }
            }
            debug!("Data channel: dispatch task for {peer_id_for_task} ended");
        });

        // on-message-string: forward to the tokio dispatch task.
        let in_tx_clone = in_tx.clone();
        channel.connect("on-message-string", false, move |values| {
            let message: String = values[1].get().expect("message is a String");
            let _ = in_tx_clone.send(message);
            None
        });

        // on-open / on-close / on-error: log.
        let peer_id_for_open = peer_id.clone();
        channel.connect("on-open", false, move |_| {
            debug!("Data channel: peer {peer_id_for_open} open");
            None
        });

        let peer_id_for_close = peer_id.clone();
        channel.connect("on-close", false, move |_| {
            debug!("Data channel: peer {peer_id_for_close} closed");
            None
        });

        channel.connect("on-error", false, move |values| {
            let error: glib::Error = values[1].get().unwrap_or_else(|_| {
                glib::Error::new(glib::FileError::Failed, "unknown")
            });
            error!("Data channel: peer {peer_id} error: {error}");
            None
        });

        None
    });
}

/// libcamerasrc → YUY2 caps → webrtcsink (webrtcsink owns the encoder).
fn build_video_branch(pipeline: &gst::Pipeline, sink: &gst::Element) -> Result<()> {
    let camerasrc = gst::ElementFactory::make("libcamerasrc")
        .build()
        .context("Failed to create libcamerasrc")?;

    let caps_yuy2 = gst::Caps::builder("video/x-raw")
        .field("format", "YUY2")
        .field("width", WIDTH)
        .field("height", HEIGHT)
        .field("framerate", gst::Fraction::new(FRAMERATE, 1))
        .field("colorimetry", "bt709")
        .field("interlace-mode", "progressive")
        .build();
    let capsfilter_src = gst::ElementFactory::make("capsfilter")
        .property("caps", &caps_yuy2)
        .build()?;

    pipeline.add_many([&camerasrc, &capsfilter_src])?;
    gst::Element::link_many([&camerasrc, &capsfilter_src, sink])?;

    Ok(())
}
