//! Motor controller benchmark for Pi Zero 2W.
//!
//! Measures:
//!   1. Single motor position read (sync_read with 1 ID)
//!   2. All 9 motors sync_read
//!   3. Subset reads (6 stewart, 7 head)
//!   4. Read + Write cycle (one control loop tick)
//!   5. Sustained control loop at target Hz with CPU usage
//!
//! Usage:
//!   ./motor_bench [serial_port]
//!   Default serial port: /dev/ttyUSB0

use std::time::{Duration, Instant};

use reachy_mini_motor_controller::ReachyMiniMotorController;

const N: usize = 1000;

fn main() {
    let port = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "/dev/ttyUSB0".to_string());

    println!("Motor Controller Benchmark");
    println!("==========================");
    println!("Port: {port}");
    println!("Iterations: {N}");
    println!();

    // Single serial handle — controller owns the port exclusively.
    let mut controller = ReachyMiniMotorController::new(&port)
        .expect("Failed to open motor controller");

    // ---- 1. Single motor read (body_yaw, ID 10) ----
    // Use read_raw_bytes to read position register (addr 132, len 4) for one motor.
    {
        let tic = Instant::now();
        for _ in 0..N {
            let _ = controller.read_raw_bytes(10, 132, 4);
        }
        let elapsed = tic.elapsed();
        println!(
            "Single motor read (ID 10):  {:>7.1} us/read  ({:.1} ms total for {N})",
            elapsed.as_micros() as f64 / N as f64,
            elapsed.as_millis(),
        );
    }

    // ---- 2. All 9 motors sync_read ----
    {
        let tic = Instant::now();
        for _ in 0..N {
            let _ = controller.read_all_positions();
        }
        let elapsed = tic.elapsed();
        println!(
            "Sync read all 9 motors:     {:>7.1} us/read  ({:.1} ms total for {N})",
            elapsed.as_micros() as f64 / N as f64,
            elapsed.as_millis(),
        );
    }

    // ---- 3. Read + Write cycle (one control loop tick) ----
    {
        let positions = controller.read_all_positions().expect("initial read");

        let tic = Instant::now();
        for _ in 0..N {
            let _ = controller.read_all_positions();
            let _ = controller.set_all_goal_positions(positions);
        }
        let elapsed = tic.elapsed();
        println!(
            "Read + Write cycle (all 9): {:>7.1} us/cycle ({:.1} ms total for {N})",
            elapsed.as_micros() as f64 / N as f64,
            elapsed.as_millis(),
        );
    }

    println!();

    // ---- 4. Sustained control loop at various Hz ----
    bench_control_loop(&mut controller, 100);
    bench_control_loop(&mut controller, 200);
    bench_control_loop(&mut controller, 500);
}

/// Read process CPU time (user + system) from /proc/self/stat.
/// Returns seconds of actual CPU time consumed (excludes I/O wait).
#[cfg(target_os = "linux")]
fn process_cpu_secs() -> Option<f64> {
    let stat = std::fs::read_to_string("/proc/self/stat").ok()?;
    let after_comm = stat.find(')')? + 2;
    let rest: Vec<&str> = stat[after_comm..].split_whitespace().collect();
    // rest[11] = utime (field 14), rest[12] = stime (field 15)
    let utime: u64 = rest.get(11)?.parse().ok()?;
    let stime: u64 = rest.get(12)?.parse().ok()?;
    let ticks_per_sec = 100_u64; // _SC_CLK_TCK, always 100 on Linux
    Some((utime + stime) as f64 / ticks_per_sec as f64)
}

fn bench_control_loop(controller: &mut ReachyMiniMotorController, target_hz: u64) {
    let period = Duration::from_micros(1_000_000 / target_hz);
    let test_duration = Duration::from_secs(5);
    let positions = controller.read_all_positions().expect("initial read");

    println!(
        "Control loop @ {} Hz (period: {:.1} ms, 5s run):",
        target_hz,
        period.as_micros() as f64 / 1000.0,
    );

    #[cfg(target_os = "linux")]
    let cpu_before = process_cpu_secs();

    let start = Instant::now();
    let mut ticks: u64 = 0;
    let mut overruns: u64 = 0;
    let mut max_tick_us: u64 = 0;
    let mut total_work_us: u64 = 0;
    let mut next_tick = start;

    while start.elapsed() < test_duration {
        next_tick += period;

        let work_start = Instant::now();
        let _ = controller.read_all_positions();
        let _ = controller.set_all_goal_positions(positions);
        let work_us = work_start.elapsed().as_micros() as u64;

        total_work_us += work_us;
        if work_us > max_tick_us {
            max_tick_us = work_us;
        }
        ticks += 1;

        let now = Instant::now();
        if now < next_tick {
            std::thread::sleep(next_tick - now);
        } else {
            overruns += 1;
        }
    }

    let wall_secs = start.elapsed().as_secs_f64();
    let actual_hz = ticks as f64 / wall_secs;
    let avg_work_us = total_work_us as f64 / ticks as f64;
    let busy_pct = (total_work_us as f64 / (wall_secs * 1_000_000.0)) * 100.0;

    println!("  Ticks:     {ticks}");
    println!("  Actual Hz: {actual_hz:.1}");
    println!("  Avg work:  {avg_work_us:.0} us ({:.1} ms)", avg_work_us / 1000.0);
    println!("  Max work:  {max_tick_us} us ({:.1} ms)", max_tick_us as f64 / 1000.0);
    println!("  Overruns:  {overruns} ({:.1}%)", overruns as f64 / ticks as f64 * 100.0);
    println!("  Busy time: {busy_pct:.1}% (wall-clock in serial I/O)");

    #[cfg(target_os = "linux")]
    if let (Some(before), Some(after)) = (cpu_before, process_cpu_secs()) {
        let cpu_pct = (after - before) / wall_secs * 100.0;
        println!("  CPU usage: {cpu_pct:.1}% (actual process CPU)");
    }

    println!();
}
