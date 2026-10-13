# Notion Report Plan — Pi Zero 2W + Wi-Fi Investigation

Outline for a team-facing decision document on Pi Zero 2W Wi-Fi options
(built-in radio vs. antennas vs. USB dongles). Audience: dev team and future-self.
Optimize for **decision-time skim** with a support runbook tail.

## Sections

### 1. TL;DR (3–5 lines)

The recommendation, and the one sentence each developer needs to remember.
Example shape: *"Pi Zero 2W ships with 2.4 GHz-only Wi-Fi that's contended
with BT. For demo robots: USB AX1800 dongle (or tether). For dev: built-in
is fine. Never promise 5 GHz to a customer without a dongle in the BOM."*

### 2. Why we investigated

The specific incident(s) or design question that triggered the work. Anchors
the doc — without it, future-you won't remember why any of this mattered.

### 3. The hardware constraint

What the built-in radio actually is (CYW43438, 2.4 GHz b/g/n, shared antenna
with BT). One paragraph. Link to `WIFI_2_4GHZ_ISSUES.md` rather than re-paste.

### 4. What we tested *(table)*

One row per option. Columns: option, chipset, band(s), connector/form-factor,
cost, where to buy, kernel/driver story (in-tree vs. out-of-tree DKMS).

### 5. Measured results *(table)*

One row per option × test scenario. Columns: throughput (iperf3), latency
p50/p99, range fall-off, behavior with BT-A2DP under load, power draw,
brownout behavior on Pi Zero 2W's USB. The numerical comparison is the heart
of the doc.

### 6. Decision matrix *(table)*

Rows = deployment scenario (home dev, indoor demo, conference floor,
customer install). Columns = recommended option, why, gotchas. **This is
what the team will actually read.**

### 7. Support playbook

When a user reports "Wi-Fi is broken" — the diagnostic order, canned answers
for the top 3 failure modes (iPhone hotspot stuck in 5 GHz, undervoltage,
captive portal). Lifted from `WIFI_2_4GHZ_ISSUES.md` §summary but condensed
to a runbook.

### 8. Open questions / revisit triggers

e.g., "Re-test if Pi 5 / new CM variant lands in our BOM" or "Re-test if
morrownr drops support for our chipset". Keeps the doc honest about its
half-life.

### 9. Appendix

- Link to repo docs: `WIFI_2_4GHZ_ISSUES.md`, `USB_WIFI_DONGLE_SETUP.md`.
- Driver install commands for the AX1800 dongle (lift from the setup doc,
  don't re-derive).

## Inputs needed before drafting

1. **Trigger**: was there a specific incident/customer report, or proactive
   design work? (Sets the tone of section 2.)
2. **Antennas/dongles tested**: which exact options? Built-in only? External
   IPEX? AX1800 dongle? Others?
3. **Test methodology**: what was measured, how, where? (iperf3? distance /
   walls? BT load on/off? phone hotspot?)
4. **Audience priority**: optimize for "team picks options at design time"
   or "on-call diagnoses a flaky robot"? Default lean: decision-first with
   runbook as a clear later section.

## Source material already in repo

- `WIFI_2_4GHZ_ISSUES.md` — full failure-mode catalog for 2.4 GHz on
  Pi Zero 2W (channels, BT coexistence, captive portals, USB 3 noise, etc.).
- `USB_WIFI_DONGLE_SETUP.md` — TP-Link AX1800 (RTL8852BU, USB ID 35bc:0108)
  setup procedure on Raspberry Pi OS.
