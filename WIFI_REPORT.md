# Pi Zero 2W Wi-Fi — Field Notes

Notes from looking at Wi-Fi on the Pi Zero 2W as a streaming platform
for our robots — the built-in radio is **2.4 GHz only** (Cypress
CYW43438), sharing a single antenna with Bluetooth. The point: keep
the gotchas worth remembering next time someone says "Wi-Fi is broken
on the robot."

## Gotchas to remember

1. **Modern iPhone hotspots default to 5 GHz** — Pi can't see them. Fix: *Settings → Personal Hotspot → Maximize Compatibility = ON*. Same trap on newer Android (look for the AP Band toggle).
2. **Wi-Fi and Bluetooth share one antenna.** Using both at once costs you on both — they trade airtime, not just bandwidth.
3. **"Wi-Fi is broken" is often power, not Wi-Fi** — undervoltage on the 5 V rail makes the radio the first thing to fail. Run `vcgencmd get_throttled` first: it returns a firmware bitfield where `0x0` is healthy and anything non-zero means undervoltage or thermal throttling has happened (bit 0/16 = currently / since-boot undervolted).
4. **Captive portals on public Wi-Fi.** Don't try to solve them on the Pi — phone tether (phone has already authenticated) or a travel router (GL.iNet) in front.
5. **USB 3.0 ports near the Pi tank Wi-Fi.** USB 3 emits broadband noise across 2.4–2.5 GHz that degrades the radio's SNR and can't be filtered out — keep distance. Source: [Intel/USB-IF whitepaper](https://www.usb.org/sites/default/files/327216.pdf).
6. **High CPU load drags down Wi-Fi.** Built-in Wi-Fi is processed by a kernel workqueue on the CPU, with no offload engine — heavy CPU load competes with it, so throughput and latency degrade under pressure even when signal is fine. More: [raspberrypi/linux#4166](https://github.com/raspberrypi/linux/issues/4166).
7. **WPA3-only networks can fail to associate.** CYW43438 firmware has been historically flaky on WPA3-only; mixed WPA2/WPA3 transitional usually works. Modern routers increasingly default to WPA3 — relevant when a customer buys new gear.

## How we looked at it

Light-touch: `wavemon` for live link quality on the Pi, plus subjective
WebRTC stream stability while moving the robot around.

## Quick diagnostics

Power health (run first when "Wi-Fi is broken" — see gotcha 3):

```bash
vcgencmd get_throttled
```

Install `wavemon`:

```bash
sudo apt install wavemon
```

Live link-quality monitor (signal, noise, bitrate, frame stats):

```bash
wavemon -i wlan0
```

Per-station link stats from the Pi (signal dBm, TX/RX bitrate, MCS, retries):

```bash
iw dev wlan0 station dump
```

Recent kernel events from the Wi-Fi driver, BT stack, and firmware loader (crashes, assoc failures, undervoltage):

```bash
dmesg | grep -iE 'brcmfmac|bluetooth|firmware'
```

## Links

- `WIFI_2_4GHZ_ISSUES.md` — full failure-mode catalog.
- `USB_WIFI_DONGLE_SETUP.md` — TP-Link AX1800 setup procedure.
