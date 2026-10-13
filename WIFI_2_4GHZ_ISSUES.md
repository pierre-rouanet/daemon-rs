# Pi Zero 2 W — 2.4 GHz-only WiFi: Connectivity Issues

The Pi Zero 2 W's CYW43438 radio supports **only 2.4 GHz 802.11 b/g/n** (no 5 GHz, no WiFi 6). This document summarizes every realistic failure mode we should plan for.

---

## 1. Bandwidth & throughput limits

- Real-world throughput caps around **40–50 Mbps** (theoretical 72 Mbps single-stream).
- ~5–10× slower than 5 GHz WiFi on a Pi 4/5.
- **Fine for**: control, telemetry, low-bitrate audio, ≤1080p compressed video, OTA updates.
- **Problematic for**: 4K video, multi-stream, high-frame-rate raw video, large file sync.

## 2. Latency, jitter & congestion

- 2.4 GHz has only **3 non-overlapping channels** (1/6/11) vs ~25 on 5 GHz.
- Shared with Bluetooth, microwaves, baby monitors, Zigbee, neighboring routers.
- Expect **occasional 50–500 ms latency spikes** even with strong signal.
- Closed-loop teleop must tolerate jitter; design timeouts accordingly.

---

## 3. Router compatibility

### Generally safe
- Virtually every consumer router supports 2.4 GHz.
- Dual-band (2.4 + 5) is the norm; tri-band keeps 2.4.
- WiFi 6E / WiFi 7 routers still broadcast 2.4 GHz.

### Real failure modes
- **Single-SSID band steering**: dual-band routers exposing one SSID may misroute or drop 2.4-only clients. Fix: split SSIDs (`MyNet` / `MyNet-5G`).
- **Enterprise APs with 2.4 GHz disabled** (some corporate / dense-deployment setups).
- **Cheap APs with low concurrent client caps** (10–20 clients) — late arrivals at demos can't associate.

---

## 4. Phone hotspot pitfalls (very common)

- **iPhone 12+** defaults hotspot to **5 GHz** — Pi can't see it.
  Fix: *Settings → Personal Hotspot → Maximize Compatibility = ON*.
- **Android (Pixel, newer Samsung)** may also default to 5 GHz.
  Fix: *Settings → Hotspot → AP Band = 2.4 GHz*.
- Some carrier-locked Android phones hide the band toggle entirely.
- 5G/mmWave phones sometimes force 5 GHz hotspot for thermal reasons.

---

## 5. Captive portals & enterprise auth

- **Hotel / airport / café WiFi** with browser login: headless Pi has no easy way through.
- **WPA2-Enterprise / 802.1X** (corporate WiFi): works but `wpa_supplicant` config is fiddly.
- **eduroam**: similar, plus per-institution quirks.
- **Guest networks with client isolation**: Pi connects but isn't reachable from your laptop on the same SSID — breaks SSH, mDNS, local HTTP control.

### Quick wins for captive portals

**Phone tether (recommended default)**
The phone has already authenticated to the portal in its browser, so a USB-tether or hotspot connection inherits that auth — the Pi sees a normal open network with internet. No portal login needed on the Pi side. Works at 90% of conferences, hotels, and airports.

- iPhone: USB tether or hotspot (Maximize Compatibility ON).
- Android: USB tethering is most reliable and bypasses the Pi's WiFi entirely.

**Travel router as a buffer**
Carry a small travel router (e.g. GL.iNet Mango / Beryl). Log in to the captive portal **once on the router's admin page**, and every device behind it — including the Pi — gets clean internet. Bonus benefits:

- Stable known SSID for the Pi — no reprovisioning per venue.
- 5 GHz uplink, 2.4 GHz downlink to the Pi (also fixes the band issue).
- Built-in VPN client if needed.

This is the **production answer for demo robots**: one config, works everywhere.

**Recommendation for the daemon**: build for tether-first, travel-router-second. Don't try to solve captive portals on the Pi itself — every venue breaks differently.

## 6. Modern WiFi security gotchas

- **WPA3-only networks**: CYW43438 firmware support has been historically flaky. Mixed WPA2/WPA3 transitional mode usually works.
- **PMF (Protected Management Frames) required**: newer WiFi 6 setups may mandate it; older Pi firmware sometimes fails the handshake.
- **MAC randomization** breaks router-side MAC filtering if enabled.

---

## 7. Regulatory / channel issues

- **Channels 12, 13, 14**: used in EU/JP, **disabled in US** regulatory domain.
  If the router is on channel 13 and your Pi has `country=US` in `wpa_supplicant.conf`, it won't see the network. Bites international shipments hard.
- **40 MHz channel width on 2.4 GHz**: works on the Pi but cheap APs misbehave.

---

## 8. Bluetooth coexistence (hardware-level)

The Pi Zero 2 W shares **one antenna and one 2.4 GHz radio** between WiFi and BT. They actively fight.

### Symptoms
- WiFi throughput drops **30–60%** during A2DP audio.
- Latency spikes from ~2 ms to 5–50 ms with heavy BT.
- BLE connection events get missed under WiFi load → sensor gaps, gamepad lag.
- A2DP audio stutters in crowded WiFi environments.
- In severe cases: full WiFi disassociation.

### Fixes (effective)
- **External USB BT dongle** + `dtoverlay=disable-bt` — most common clean fix.
- **External USB WiFi dongle** (5 GHz) — frees internal radio for BT.
- **Wired Ethernet via USB OTG** — eliminates WiFi contention.
- **Disable whichever radio you don't need.**

### Mitigations (partial)
- Set router to channel 1 or 11 (BLE adv channels 37/38/39 sit in the gaps).
- Prefer BLE over Classic Bluetooth.
- Avoid A2DP / SCO over internal BT.
- Don't run BT scans during heavy WiFi traffic.

### Decision matrix
| Use case | Outcome on Pi Zero 2 W |
|---|---|
| WiFi only | Disable BT, fine |
| BT only | Disable WiFi, fine |
| WiFi + light BLE | Works, expect ~50 ms jitter spikes |
| WiFi + BT audio (A2DP) | **Plan for external dongle** |
| WiFi video streaming + BT audio | **Wrong board — use Pi 4 / CM4** |

---

## 9. Power / electrical (often misdiagnosed as WiFi)

- **Undervoltage** on the 5V rail (cheap cable, weak power bank, motor inrush) makes the WiFi radio the **first thing to fail**: dropouts, slow reconnects, dead radio until reboot.
- High CPU load can also destabilize WiFi on this SoC.
- **Diagnostic**: `vcgencmd get_throttled`, watch the red power LED.

## 10. Physical / RF environment

- **USB 3.0 emits broadband noise around 2.4 GHz** — a USB 3 hub next to the Pi tanks WiFi throughput. Documented and significant.
- Microwaves, badly-shielded HDMI cables, and metal enclosures all degrade signal.
- 2.4 GHz penetrates walls *better* than 5 GHz — actually a plus in some deployments.
- The Zero 2 W's antenna is on-PCB; keep it clear of metal and high-speed cables.

---

## 11. IP-layer / network-layer issues (band-agnostic but still bite)

- **DHCP lease exhaustion** on busy networks → no IP, looks like WiFi failure.
- **mDNS / `.local`** broken on networks blocking multicast → `raspberrypi.local` won't resolve.
- **IPv6-only networks** (some carriers, modern enterprise): fails if daemon assumes IPv4.

## 12. Deployment / fleet issues

- **Provisioning**: hardcoded SSID won't match customer networks. Need BLE / AP-mode / USB-config flow.
- **Hotspot routing**: phone-hotspot subnets sometimes block client-to-client traffic; control app may not reach the Pi.
- **Multiple Pis on one SSID**: cheap routers cap concurrent clients.

---

## Diagnostic quick-reference

```bash
iperf3 -c <server> -t 30
```

```bash
dmesg | grep -iE 'brcmfmac|bluetooth|firmware'
```

```bash
iw dev wlan0 station dump
```

```bash
vcgencmd get_throttled
```

---

## Summary: top failure modes by likelihood

1. **iPhone hotspot stuck in 5 GHz mode** (Maximize Compatibility off).
2. **Captive portals** on public WiFi.
3. **Power undervoltage** masquerading as WiFi failure.
4. **Country-code mismatch** on channels 12–14 abroad.
5. **Bluetooth coexistence** when both radios are heavily loaded.
6. **Enterprise / WPA2-Enterprise** auth complexity.
7. **Band-steering routers** misrouting 2.4-only clients.
8. **USB 3.0 noise** on hubs near the Pi.

## Recommended fallback paths to design in

- **USB Ethernet** (most reliable).
- **USB tether to phone** (USB networking, not WiFi hotspot — bypasses band issues).
- **AP mode for provisioning** (Pi exposes its own WiFi for first-time config).
- **External USB WiFi/BT dongle slot** in hardware design if BT + WiFi are both critical.
