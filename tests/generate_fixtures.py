#!/usr/bin/env python3
"""Generate JSON test fixtures from the Python Pydantic protocol models.

Run once to produce tests/fixtures/*.json — Rust tests parse these
to catch wire-protocol drift between Python and Rust.
"""

import json
import os
import sys
from datetime import datetime, timezone
from pathlib import Path
from uuid import UUID

# Add reachy_mini to path
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "reachy_mini" / "src"))

from reachy_mini.io.protocol import (
    # Client -> Server
    SetTargetCmd,
    SetHeadJointsCmd,
    SetBodyYawCmd,
    SetAntennasCmd,
    SetFullTargetCmd,
    GotoTargetCmd,
    WakeUpCmd,
    GotoSleepCmd,
    PlaySoundCmd,
    SetMotorModeCmd,
    SetTorqueCmd,
    GetMotorModeCmd,
    SetGravityCompensationCmd,
    SetAutomaticBodyYawCmd,
    GetStateCmd,
    StartRecordingCmd,
    StopRecordingCmd,
    AppendRecordCmd,
    # Task protocol
    TaskRequest,
    GotoTaskRequest,
    PlayMoveTaskRequest,
    # Server -> Client
    JointPositionsMsg,
    HeadPoseMsg,
    ImuDataMsg,
    DaemonStatus,
    TaskProgress,
    # Enums
    MotorControlMode,
    DaemonState,
    RobotBackendStatus,
)
from reachy_mini.utils.interpolation import InterpolationTechnique

FIXTURES_DIR = Path(__file__).parent / "fixtures"
FIXTURES_DIR.mkdir(exist_ok=True)

# Example 4x4 identity matrix flattened
IDENTITY_16 = [
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
]

IDENTITY_4x4 = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
]

TIMESTAMP = datetime(2025, 6, 1, 12, 0, 0, tzinfo=timezone.utc)
FIXED_UUID = UUID("12345678-1234-5678-1234-567812345678")


def write(name: str, obj):
    """Write a Pydantic model to a fixture JSON file."""
    path = FIXTURES_DIR / f"{name}.json"
    data = obj.model_dump(mode="json")
    path.write_text(json.dumps(data, indent=2) + "\n")
    print(f"  {path.name}")


# ── Client → Server ──────────────────────────────────────────────

print("Client -> Server commands:")

write("set_target", SetTargetCmd(head=IDENTITY_16))

write("set_head_joints", SetHeadJointsCmd(joints=[0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6]))

write("set_body_yaw", SetBodyYawCmd(body_yaw=0.5))

write("set_antennas", SetAntennasCmd(antennas=[-0.17, 0.17]))

write("set_full_target", SetFullTargetCmd(
    head=IDENTITY_16,
    antennas=[-0.17, 0.17],
    body_yaw=0.1,
))

write("set_full_target_partial", SetFullTargetCmd(
    antennas=[-0.5, 0.5],
))

write("goto_target", GotoTargetCmd(
    head=IDENTITY_16,
    antennas=[-0.17, 0.17],
    body_yaw=0.0,
    duration=2.0,
))

write("goto_target_minimal", GotoTargetCmd(duration=1.0))

write("wake_up", WakeUpCmd())
write("goto_sleep", GotoSleepCmd())

write("play_sound", PlaySoundCmd(file="hello.wav"))

write("set_motor_mode", SetMotorModeCmd(mode="enabled"))

write("set_torque", SetTorqueCmd(on=True))
write("set_torque_with_ids", SetTorqueCmd(on=False, ids=["stewart_0", "body_yaw"]))

write("get_motor_mode", GetMotorModeCmd())

write("set_gravity_compensation", SetGravityCompensationCmd(enabled=True))

write("set_automatic_body_yaw", SetAutomaticBodyYawCmd(enabled=False))

write("get_state", GetStateCmd())

write("start_recording", StartRecordingCmd())
write("stop_recording", StopRecordingCmd())
write("append_record", AppendRecordCmd(record={"position": [1.0, 2.0], "timestamp": 123.4}))

# ── Task protocol ────────────────────────────────────────────────

print("\nTask protocol:")

write("task_goto", TaskRequest(
    uuid=FIXED_UUID,
    req=GotoTaskRequest(
        head=IDENTITY_16,
        antennas=[-0.17, 0.17],
        duration=2.0,
        method=InterpolationTechnique.MIN_JERK,
        body_yaw=0.0,
    ),
    timestamp=TIMESTAMP,
))

write("task_play_move", TaskRequest(
    uuid=FIXED_UUID,
    req=PlayMoveTaskRequest(move_name="wave"),
    timestamp=TIMESTAMP,
))

# ── Server → Client ──────────────────────────────────────────────

print("\nServer -> Client messages:")

write("joint_positions", JointPositionsMsg(
    head_joint_positions=[0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
    antennas_joint_positions=[-0.17, 0.17],
))

write("head_pose", HeadPoseMsg(head_pose=IDENTITY_4x4))

write("imu_data", ImuDataMsg(
    accelerometer=[0.0, 0.0, 9.81],
    gyroscope=[0.0, 0.0, 0.0],
    quaternion=[1.0, 0.0, 0.0, 0.0],
    temperature=25.0,
))

write("daemon_status", DaemonStatus(
    robot_name="reachy_mini",
    state=DaemonState.RUNNING,
    wireless_version=True,
    desktop_app_daemon=False,
    simulation_enabled=False,
    mockup_sim_enabled=False,
    no_media=False,
    media_released=False,
    camera_specs_name="wireless",
    backend_status=RobotBackendStatus(
        ready=True,
        motor_control_mode=MotorControlMode.Enabled,
        last_alive=None,
        control_loop_stats={},
        error=None,
    ),
    error=None,
    wlan_ip="192.168.1.42",
    version="0.1.0",
))

write("task_progress", TaskProgress(
    uuid=FIXED_UUID,
    finished=True,
    error=None,
    timestamp=TIMESTAMP,
))

print(f"\nDone. {len(list(FIXTURES_DIR.glob('*.json')))} fixtures in {FIXTURES_DIR}")
