# daemon-rs

Rust rewrite of the Reachy Mini daemon, wireless config only. See
[PLAN.md](PLAN.md) for scope and architectural direction.

## On-device dependencies (Raspberry Pi OS / Debian Trixie)

`daemon-rs` speaks to hardware through GStreamer + libcamera + PipeWire.
The package list below is lifted from
[`pollen-robotics/reachy-mini-os`](https://github.com/pollen-robotics/reachy-mini-os)
(`stage2/01-sys-tweaks/00-packages-nr` + `stage2/05-reachy-mini/00-run.sh`)
so the Pi's runtime matches what the Python daemon sits on.

### Runtime (media stack)

```sh
sudo apt install \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libcamera \
    gstreamer1.0-alsa \
    gstreamer1.0-nice \
    libnice10 \
    libspa-0.2-libcamera \
    rpicam-apps-lite \
    librpicam-app1 \
    alsa-utils \
    v4l-utils \
    i2c-tools
```

What each one gives us:

| Package | Provides |
| --- | --- |
| `gstreamer1.0-tools` | `gst-launch-1.0`, `gst-inspect-1.0`, `gst-device-monitor-1.0` |
| `gstreamer1.0-plugins-good` | `v4l2src` / `v4l2h264enc` / `v4l2jpegdec`, `alsasrc`, `audioconvert` … |
| `gstreamer1.0-plugins-bad` | extra codecs, `h264parse`, RTP helpers |
| `gstreamer1.0-libcamera` | `libcamerasrc` (the CSI imx708 source) |
| `gstreamer1.0-alsa` | ALSA sink/src |
| `gstreamer1.0-nice` + `libnice10` | ICE transport for WebRTC |
| `libspa-0.2-libcamera` | SPA plugin for libcamera device detection |
| `rpicam-apps-lite`, `librpicam-app1` | `rpicam-hello`, camera tuning files |
| `alsa-utils` | `aplay`, `arecord`, `amixer` for debugging |
| `v4l-utils` | `v4l2-ctl` for debugging camera pipelines |
| `i2c-tools` | `i2cdetect` — needed for the BMI088 IMU (Phase 3) |

### WebRTC sink (`webrtcsink`)

`webrtcsink` and `webrtcsrc` come from **`gst-plugins-rs`**, which is **not
packaged on Debian Trixie**. `reachy-mini-os` ships a prebuilt
`gst-plugins-rs-rpi.tar.gz` extracted at `/opt/gst-plugins-rs/` and exposed
via `GST_PLUGIN_PATH`:

```sh
export GST_PLUGIN_PATH=/opt/gst-plugins-rs/lib/aarch64-linux-gnu/
```

On a fresh Raspberry Pi OS image, grab the same tarball from the
reachy-mini-os repo (`stage2/05-reachy-mini/files/gst-plugins-rs-rpi.tar.gz`)
or build `gst-plugins-rs` from source (`net/webrtc` workspace crate).

Verify the plugin is visible:

```sh
gst-inspect-1.0 webrtcsink
gst-inspect-1.0 webrtcsrc
```

### Build-time (only if compiling gst-plugins-rs or similar on the Pi itself)

Not needed for running the prebuilt daemon — we cross-compile
`daemon-rs` from the laptop. Listed for completeness:

```sh
sudo apt install \
    build-essential pkg-config \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    libglib2.0-dev \
    libssl-dev
```

## Running

```sh
./daemon-rs
```

- WebRTC signalling: `ws://<pi-ip>:8443`
- `REACHYMINI_AUDIO_DEVICE=none` skips the audio branch.

## Test client

`client/index.html` — open in a browser, point it at the daemon's
signalling URL, shows live resolution / fps / bitrate / rtt / jitter-buffer
delay.
