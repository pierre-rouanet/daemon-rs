# USB Wi-Fi Dongle Setup on Raspberry Pi (Zero 2W)

How to bring up a **TP-Link AX1800 USB dongle** (Archer TX20U / TX20U Plus / TX20U Nano family,
USB ID `35bc:0108`, chipset **RTL8852BU**) on a fresh Raspberry Pi OS install.

Verified on:
- Pi Zero 2W
- Raspberry Pi OS, Debian Trixie 13
- Kernel `6.12.75+rpt-rpi-v8` (64-bit, `aarch64` / `rpi-v8`)
- Driver: `morrownr/rtl8852bu-20250826` (version `1.19.21-86`)

The same procedure works for any Pi running a recent 64-bit Pi OS. There are
**two non-obvious traps** specific to small-RAM Pis with USB 2.0 ports (Pi Zero
family, Pi 3B/B+) — both burned us during this setup. They're called out as
critical steps below.

For other Realtek USB Wi-Fi 6 dongles, the chipset and repo will differ — see
the "Other chipsets" section at the bottom.

---

## 1. Verify the dongle is detected on USB

Plug the dongle in, then on the Pi:

```bash
lsusb
```

You should see an entry like:

```
Bus 001 Device 002: ID 35bc:0108 Realtek 802.11ac WLAN Adapter
```

The string says "802.11ac" but it is actually a **Wi-Fi 6 (802.11ax) RTL8852BU** —
the descriptor is just generic firmware text. Don't let it mislead you. Once
the driver loads, the device re-enumerates as "802.11ax WLAN Adapter".

If `lsusb` does NOT show the dongle, it is a power / cable problem, not a driver
problem. Try a different cable or a powered USB hub (see "Power" below).

## 2. Confirm no driver is bound yet

```bash
ip link show
```

```bash
iw dev
```

On a fresh install you will only see `lo` and `wlan0` (the Pi's built-in Wi-Fi).
The dongle does NOT appear because no in-tree driver covers RTL8852BU. We need
to install an out-of-tree DKMS driver.

```bash
dmesg | grep -iE "usb|rtl|wlan" | tail -40
```

You'll see the device enumerate (`New USB device found, idVendor=35bc, idProduct=0108`)
but no `rtw...` or `8852bu` driver-bind line. Expected.

## 3. Install kernel headers and build tools

```bash
sudo apt update
```

```bash
sudo apt install -y raspberrypi-kernel-headers dkms git build-essential bc
```

If `raspberrypi-kernel-headers` is reported missing on a newer Pi OS, use:

```bash
sudo apt install -y linux-headers-rpi-v8 dkms git build-essential bc
```

Verify the headers match your running kernel:

```bash
ls /lib/modules/$(uname -r)/build
```

You should see a populated directory (Makefile, `include/`, etc.). If it's missing,
DKMS will fail to build. Reboot, re-check, and only continue once this directory
is populated.

## 4. CRITICAL — Add disk-backed swap on small-RAM Pis

**Skip this section only on Pi 4 (≥1 GB) or Pi 5.** On Pi Zero 2W (512 MB),
Pi 3B / 3B+ (1 GB), and any other low-RAM host, the build will OOM-kill `cc1`
when it hits `phl/hal_g6/mac/fw_ax/rtl8852b/hal8852b_fw.o`. That source file
is a giant auto-generated C array containing the chip's firmware blob; cc1's
working set on it exceeds available RAM. **zram alone won't save you** — zram
swap is in RAM, so it can't give you more memory than you physically have.
You need real disk-backed swap.

Check what swap you currently have:

```bash
swapon --show
```

Pi OS Trixie ships with a 416 MB zram swap and **no disk swap**. Add a 2 GB
file-backed swap on the SD card:

```bash
sudo fallocate -l 2G /swapfile
```

```bash
sudo chmod 600 /swapfile
```

```bash
sudo mkswap /swapfile
```

```bash
sudo swapon /swapfile
```

Confirm both swaps are active:

```bash
swapon --show
```

Persist across reboots (recommended — DKMS rebuilds on every kernel upgrade,
so you'll need swap again):

```bash
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

Note: even with swap, the build takes 30–60 minutes on a Pi Zero 2W because
cc1 thrashes through SD-backed swap on the big firmware file. That's normal.
Don't kill it.

## 5. Clone and install the driver

```bash
cd ~
```

```bash
git clone https://github.com/morrownr/rtl8852bu-20250826.git
```

```bash
cd rtl8852bu-20250826
```

Check whether your USB ID is already in the supported list:

```bash
grep -rn "35bc" .
```

If there are no matches, add it before installing:

```bash
sudo ./add-driver-id.sh 35bc 0108
```

(If that script is not present, follow the README section on "adding a new USB ID"
and edit `os_dep/linux/usb_intf.c` directly, or use `edit-options.sh`.)

Build and install via DKMS:

```bash
sudo ./install-driver.sh
```

The script will:
- Build the module against your running kernel.
- Register it with DKMS so it auto-rebuilds across kernel upgrades.
- Prompt for a reboot — say **no** for now; we have a config tweak first.

## 6. CRITICAL — Disable USB 3.0 mode switch on USB-2.0-only hosts

The morrownr install script writes `/etc/modprobe.d/8852bu.conf` with a default
of `rtw_switch_usb_mode=1`, which tells the driver to **negotiate the dongle
from USB 2.0 to USB 3.0** during init. The Pi Zero 2W (and Pi Zero W, Pi 3B/B+,
and any other USB-2.0-only host) physically cannot negotiate SuperSpeed — the
result is a continuous reset loop, with `dmesg` showing:

```
rtl8852bu 1-1:1.0: Runtime PM usage count underflow!
usb 1-1: USB disconnect, device number N
usb 1-1: new high-speed USB device number N+1 using dwc_otg
```

Repeating every ~4 seconds, forever. The "Runtime PM underflow" is a
side-effect of the abrupt disconnect during init, not the root cause.

Fix: set `rtw_switch_usb_mode=0` before booting into the new driver.

```bash
sudo sed -i 's/rtw_switch_usb_mode=1/rtw_switch_usb_mode=0/' /etc/modprobe.d/8852bu.conf
```

Verify:

```bash
grep rtw_switch_usb_mode /etc/modprobe.d/8852bu.conf
```

Should print `options 8852bu rtw_switch_usb_mode=0`.

If you're on a Pi 4 / Pi 5 (USB 3.0), leave the default `=1` and the driver
will switch the dongle to SuperSpeed on bring-up. Worth the 5–10 Mbps faster
USB headroom.

## 7. Reboot and verify

```bash
sudo reboot
```

After reboot:

```bash
ip link show
```

You should now see a second wireless interface, typically `wlan1` (or a
`wlxXXXXXXXXXXXX` predictable name).

```bash
iw dev
```

```bash
dmesg | grep -iE "8852|8832|rtw|wlan"
```

Look for a successful module load line naming `8852bu` and a quiet `dmesg`
(no disconnect storm). If you still see the disconnect/reconnect loop, double
check section 6 — `rtw_switch_usb_mode` must be 0.

## 8. Connect to a network

Easiest path on Raspberry Pi OS Bookworm/Trixie (NetworkManager):

```bash
nmcli device wifi list ifname wlan1
```

```bash
sudo nmcli device wifi connect "YOUR_SSID" password "YOUR_PASSWORD" ifname wlan1
```

If `nmcli` errors with `802-11-wireless-security.key-mgmt: property is missing`,
the AP is WPA3-only or WPA2/WPA3 transitional and NM can't infer the type.
Make it explicit:

```bash
sudo nmcli device wifi connect "YOUR_SSID" password "YOUR_PASSWORD" ifname wlan1 wifi-sec.key-mgmt wpa-psk
```

(or `wifi-sec.key-mgmt sae` for WPA3-only).

For older Pi OS releases (`wpa_supplicant` based), add a block to
`/etc/wpa_supplicant/wpa_supplicant-wlan1.conf` and enable
`wpa_supplicant@wlan1.service`.

## 9. (Optional) Make the dongle the default route

If you want all traffic to prefer the dongle over built-in Wi-Fi:

```bash
sudo nmcli connection modify "YOUR_SSID" ipv4.route-metric 50 connection.interface-name wlan1
```

Lower metric wins. Built-in `wlan0` typically sits at 600 by default. Pinning
`connection.interface-name=wlan1` is important — without it, NM can move the
profile to wlan0 on reboot, especially if netplan is in the mix (see
"netplan caveat" in Troubleshooting).

---

## 10. Deploying to another Pi

Once you have a working Pi, you can avoid the 30–60-minute build on every
other Pi by copying artifacts. Two strategies:

### Path A — Binary copy (fast, kernel-pinned)

Ships only the compiled module + config. Works only if the target runs the
**exact same kernel version**. After any `apt full-upgrade` that bumps the
kernel on either Pi, the binary won't load.

On the source Pi:

```bash
sudo tar czf /tmp/rtl8852bu-bin-$(uname -r).tar.gz /lib/modules/$(uname -r)/updates/dkms/8852bu.ko /etc/modprobe.d/8852bu.conf
```

```bash
scp /tmp/rtl8852bu-bin-*.tar.gz pi@target.local:/tmp/
```

On the target (confirm `uname -r` matches the source first):

```bash
sudo tar xzf /tmp/rtl8852bu-bin-*.tar.gz -C /
```

```bash
sudo depmod -a
```

```bash
sudo modprobe 8852bu
```

Verify with `ip link show` — `wlan1` should appear. The `.ko` carries a USB
device alias for `35bc:0108`, so once `depmod` registers it, the next dongle
hotplug auto-loads it. No `/etc/modules-load.d/` entry needed.

### Path B — DKMS source (resilient, slow first build)

Ships the source tree so DKMS rebuilds on every kernel upgrade.

On the source:

```bash
sudo tar czf /tmp/rtl8852bu-dkms-src.tar.gz /usr/src/rtl8852bu-1.19.21-86 /etc/modprobe.d/8852bu.conf
```

```bash
scp /tmp/rtl8852bu-dkms-src.tar.gz pi@target.local:/tmp/
```

On the target (also needs swap from section 4, headers from section 3):

```bash
sudo apt install -y linux-headers-rpi-v8 dkms build-essential bc
```

```bash
sudo tar xzf /tmp/rtl8852bu-dkms-src.tar.gz -C /
```

```bash
sudo dkms add rtl8852bu/1.19.21-86
```

```bash
sudo dkms install rtl8852bu/1.19.21-86
```

Replace `1.19.21-86` with whatever version the source tree carries (look in
`/usr/src/rtl8852bu-*/dkms.conf` for `PACKAGE_VERSION`).

### Choosing between paths

For a small fleet on a stable kernel: **Path A** — copy the binary, accept
that kernel upgrades require redoing it.

For a fleet that auto-updates: **Path B** — pay the build cost once per Pi,
then DKMS keeps it working across upgrades indefinitely.

---

## Power (important on Pi Zero 2W)

The dongle's USB descriptor declares `MaxPower 500mA`, which is the maximum a
USB 2.0 port is allowed to deliver. The Pi Zero 2W's single USB port is
power-limited and will brown out the dongle under sustained transmit load.

Symptoms of insufficient power:
- Dongle works for a few seconds then disappears from `ip link`.
- `dmesg` shows repeated `usb 1-1: USB disconnect` followed by re-enumeration
  (different from the `rtw_switch_usb_mode` reset loop — the power version
  doesn't show "Runtime PM usage count underflow").
- Throughput collapses or the link drops every few minutes under iperf3 load.

Diagnostic:

```bash
vcgencmd get_throttled
```

Anything other than `throttled=0x0` indicates the 5V rail dropped at some point.

Fix: use a **powered USB hub** (or a Y-cable that injects 5V from a separate
supply) between the Pi and the dongle. Don't waste hours debugging the driver
before checking this.

## Troubleshooting

### Continuous disconnect/reconnect loop with "Runtime PM usage count underflow"

Symptom in `dmesg` (every ~4 seconds, forever):

```
rtl8852bu 1-1:1.0: Runtime PM usage count underflow!
usb 1-1: USB disconnect
usb 1-1: new high-speed USB device number N
```

Cause: `rtw_switch_usb_mode=1` on a USB-2.0-only host. See section 6.

### Continuous disconnect/reconnect, NO "Runtime PM" message

Cause: power brownout. See "Power" above.

### `wlan1` never shows up after reboot

Check the module actually loaded:

```bash
lsmod | grep 8852
```

```bash
sudo modprobe 8852bu
```

```bash
dmesg | tail -50
```

If `modprobe` fails with "module not found", DKMS didn't install the module for
your running kernel. Re-run:

```bash
sudo dkms status
```

```bash
sudo dkms install rtl8852bu/<version>
```

(replace `<version>` with what `dkms status` prints).

### DKMS shows the wrong driver

If `dkms status` shows `rtl8821cu` or any other driver, you installed the
wrong one. RTL8852BU and RTL8821CU are different chipsets despite both being
"Realtek USB Wi-Fi" — `35bc:0108` is RTL8852BU. Remove the wrong one:

```bash
sudo dkms remove rtl8821cu/<version> --all
```

Then re-do section 5 with the correct repo (`morrownr/rtl8852bu-20250826`).

### Headers don't match running kernel after `apt install`

This happens when `apt` upgraded the kernel package but you haven't rebooted.
Reboot, then re-run `ls /lib/modules/$(uname -r)/build`.

### Module loads but `wlan1` is missing

Possibly the wrong chipset variant. Check the dmesg load line — if it complains
about an unrecognized device, your USB ID is not registered. Re-run the
add-driver-id step from section 5.

### netplan caveat (advanced)

If your Pi has netplan installed (Ubuntu-style network config — uncommon on
default Pi OS but possible if cloud-init or a custom image set it up), NM
connections are auto-imported into `/etc/netplan/90-NM-*.yaml` files with
`match: {}` blocks that prevent pinning a profile to a specific interface.
Symptoms: `connection.interface-name wlan1` set via `nmcli` is silently
overwritten by netplan on reboot, so wlan0 keeps grabbing the network.

Fix: bypass netplan for that connection by writing the NM profile directly:

```bash
sudo mv /etc/netplan/90-NM-*.yaml /etc/netplan/disabled/   # if those files exist
```

Then create the profile in `/etc/NetworkManager/system-connections/<SSID>.nmconnection`
manually with `[connection] interface-name=wlan1` set explicitly. NM's native
location takes precedence and isn't touched by netplan regen.

## Other chipsets

The morrownr GitHub org maintains drivers for most Realtek USB Wi-Fi adapters.
Match your chipset to the right repo:

- **RTL8811CU / RTL8821CU** (older 802.11ac): `morrownr/8821cu-20210916`
- **RTL8812AU / RTL8821AU** (older 802.11ac): `morrownr/8812au-20210820`
- **RTL88x2BU** (802.11ac): `morrownr/88x2bu-20210702`
- **RTL8832BU / RTL8852BU** (Wi-Fi 6, this guide): `morrownr/rtl8852bu-20250826`

Identify your chipset by running `sudo lsusb -v -d <vid>:<pid>` and cross-referencing
the VID:PID against the supported-IDs file in each repo, or by searching
`morrownr/USB-WiFi` (a documentation-only repo with a chipset-to-dongle table).

The two CRITICAL traps in this guide (swap-during-build, `rtw_switch_usb_mode=0`
on USB 2.0 hosts) likely apply to the other modern AX-series morrownr drivers
too — the firmware-blob OOM and the USB 3.0 mode-switch logic are shared
across the codebase.
