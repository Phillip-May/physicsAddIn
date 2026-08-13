#!/usr/bin/env python3
"""Convert AR4 STL link geometry into a self-contained robot package."""

from __future__ import annotations

import argparse
import json
import math
import re
import struct
from pathlib import Path
from typing import Iterable

from pkglib import ROOT, slug
from pkglib.archive import write_zip
from pkglib.hulls import convex_hull
from pkglib.meshbin import bounds_of as compute_bounds, merge_bounds, write_meshbin
from pkglib.nodes import (TYPE_MESH_GEOMETRY, TYPE_OPW6_ROBOT, TYPE_ROBOT_LINK,
                          TYPE_ROBOT_TOOL, node)
DEFAULT_PACKAGE_DIR = ROOT / "library" / "sources" / "ar4_6dof_robot"
DEFAULT_ZIP = ROOT / "library" / "packages" / "ar4_6dof_robot.zip"

# AR4 planner defaults. Joint velocity is derived from the AR4 HMI step timing
# and steps/degree. Acceleration is derived from the public AR4 motor/drivetrain
# specs plus explicit package inertia/derating assumptions.
AR4_REASONABLE_LIMIT_PROFILE = {
    "name": "reasonable",
    "description": "Torque-derived AR4 planner limits before stress-test scaling.",
    "motorTorqueDerate": 0.25,
    "jerkRiseTimeSec": 0.12,
    "defaultLinearAccelerationMmSec2": 1000.0,
    "defaultLinearJerkMmSec3": 10000.0,
    "defaultToolAngularSpeedDegSec": 180.0,
    "defaultToolAngularAccelerationDegSec2": 720.0,
    "defaultToolAngularJerkDegSec3": 7200.0,
}
AR4_STRESS_HIGH_LIMIT_PROFILE = {
    "name": "stress-high",
    "description": "Intentionally high blend-stress limits for planner experimentation.",
    "motorTorqueDerate": 100000.0,
    "jerkRiseTimeSec": 0.08,
    "defaultLinearAccelerationMmSec2": 1000000000.0,
    "defaultLinearJerkMmSec3": 1000000000000.0,
    "defaultToolAngularSpeedDegSec": 1000000.0,
    "defaultToolAngularAccelerationDegSec2": 100000000.0,
    "defaultToolAngularJerkDegSec3": 10000000000.0,
}
AR4_LIMIT_PROFILES = {
    AR4_REASONABLE_LIMIT_PROFILE["name"]: AR4_REASONABLE_LIMIT_PROFILE,
    AR4_STRESS_HIGH_LIMIT_PROFILE["name"]: AR4_STRESS_HIGH_LIMIT_PROFILE,
}
AR4_JOINT_EQUIVALENT_INERTIA_KG_M2 = [1.80, 3.20, 1.20, 0.35, 0.15, 0.12]
AR4_DRIVETRAINS = [
    {
        "joint": "J1",
        "sku": "17HS15-1684D-EG10-AR4",
        "source": "StepperOnline AR4-MK4/5 J1 motor page",
        "motorHoldingTorqueNm": 0.39,
        "ratedCurrentA": 1.68,
        "motorStepAngleDeg": 1.8,
        "encoderPpr": 1000,
        "gearboxType": "EG planetary",
        "gearRatio": 10.0,
        "gearboxEfficiency": 0.96,
        "gearboxMaxPermissibleTorqueNm": 5.0,
    },
    {
        "joint": "J2",
        "sku": "23HS22-2804D-YGS50-AR4",
        "source": "StepperOnline AR4-MK4/5 J2 motor page",
        "motorHoldingTorqueNm": 1.02,
        "ratedCurrentA": 2.8,
        "motorStepAngleDeg": 1.8,
        "encoderPpr": 1000,
        "gearboxType": "YGS planetary",
        "gearRatio": 50.0,
        "gearboxEfficiency": 0.94,
        "gearboxMaxPermissibleTorqueNm": 25.0,
    },
    {
        "joint": "J3",
        "sku": "17HS15-1684D-EG50-AR4",
        "source": "StepperOnline AR4-MK4/5 J3 motor page",
        "motorHoldingTorqueNm": 0.39,
        "ratedCurrentA": 1.68,
        "motorStepAngleDeg": 1.8,
        "encoderPpr": 1000,
        "gearboxType": "EG planetary",
        "gearRatio": 50.0,
        "gearboxEfficiency": 0.94,
        "gearboxMaxPermissibleTorqueNm": 10.0,
    },
    {
        "joint": "J4",
        "sku": "11HS20-0674D-EGS16-AR4",
        "source": "StepperOnline AR4-MK4/5 J4 motor page",
        "motorHoldingTorqueNm": 0.14,
        "ratedCurrentA": 0.70,
        "motorStepAngleDeg": 1.8,
        "encoderPpr": 1000,
        "gearboxType": "EGS planetary",
        "gearRatio": 16.0,
        "gearboxEfficiency": 0.94,
        "gearboxMaxPermissibleTorqueNm": 4.0,
    },
    {
        "joint": "J5",
        "sku": "17E19S1684MB4-200RS-AR4",
        "source": "StepperOnline AR4-MK4/5 J5 linear stepper page",
        "motorHoldingTorqueNm": 0.44,
        "ratedCurrentA": 1.68,
        "motorStepAngleDeg": 1.8,
        "encoderPpr": 1000,
        "transmissionType": "external_trapezoidal_lead_screw",
        "leadScrewTravelPerRevMm": 8.0,
        "leadScrewEfficiencyEstimate": 0.35,
    },
    {
        "joint": "J6",
        "sku": "14HS11-1004D-EGS20-AR4",
        "source": "StepperOnline AR4-MK4/5 J6 motor page",
        "motorHoldingTorqueNm": 0.125,
        "ratedCurrentA": 1.0,
        "motorStepAngleDeg": 1.8,
        "encoderPpr": 1000,
        "gearboxType": "EGS planetary",
        "gearRatio": 20.0,
        "gearboxEfficiency": 0.94,
        "gearboxMaxPermissibleTorqueNm": 8.5,
    },
]
AR4_DEFAULT_LINEAR_SPEED_MM_S = 25.0
AR4_DEFAULT_SINGULARITY_THRESHOLD_DEG = 5.0
AR4_DEFAULT_CONTROL_PERIOD_S = 0.005

DEFAULT_COLOR = [0.78, 0.78, 0.76, 1.0]
GEOMETRY_COLOR = [0.75, 0.75, 0.75, 1.0]
TOOL_COLOR = [0.50, 0.54, 0.58, 1.0]
VTK_COLORS = {
    "Silver": [0.752941, 0.752941, 0.752941, 1.0],
    "RoyalBlue": [0.254902, 0.411765, 0.882353, 1.0],
    "DimGray": [0.411765, 0.411765, 0.411765, 1.0],
    "Gold": [1.0, 0.843137, 0.0, 1.0],
    "Tomato": [1.0, 0.388235, 0.278431, 1.0],
}

LINK_STLS = [
    ("Link 0 Base", ["Link Base-1.STL", "Link Base-2.STL", "Link Base-3.STL"]),
    ("Link 1", ["Link 1-1.STL", "Link 1-2.STL"]),
    ("Link 2", ["Link 2-1.STL", "Link 2-2.STL", "Link 2-3.STL"]),
    ("Link 3", ["Link 3-1.STL", "Link 3-2.STL"]),
    ("Link 4", ["Link 4-1.STL", "Link 4-2.STL", "Link 4-3.STL"]),
    ("Link 5", ["Link 5-1.STL", "Link 5-2.STL"]),
    ("Link 6", ["Link 6-1.STL", "Link 6-2.STL"]),
]

TOOLS = []

AR4_DHM = [
    0.0, -1.5707963267948966, 0.0, 0.0, 0.0, 3.141592653589793,
    0.0, 64.2, 305.0, 0.0, 0.0, 0.0,
    0.0, -1.5707963267948966, 0.0, -1.5707963267948966, 1.5707963267948966, -1.5707963267948966,
    169.77, 0.0, 0.0, 222.63, 0.0, 41.0,
]


def _required_number(mapping: dict, key: str, source_file: Path) -> float:
    if key not in mapping:
        raise KeyError(f"{source_file}: missing required AR4 value '{key}'")
    try:
        value = float(mapping[key])
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{source_file}: AR4 value '{key}' is not numeric: {mapping[key]!r}") from exc
    if not math.isfinite(value):
        raise ValueError(f"{source_file}: AR4 value '{key}' is not finite")
    return value


def _round_list(values: Iterable[float], digits: int = 6) -> list[float]:
    return [round(float(value), digits) for value in values]


def derive_ar4_joint_limits(step_deg: list[float], drive_microsteps: list[float], limit_profile: dict) -> dict:
    effective_ratios: list[float] = []
    output_torque_nm: list[float] = []
    derated_torque_nm: list[float] = []
    accel_deg_s2: list[float] = []
    jerk_deg_s3: list[float] = []
    drivetrain_rows: list[dict] = []

    for index, drivetrain in enumerate(AR4_DRIVETRAINS):
        microsteps_per_rev = drive_microsteps[index]
        if microsteps_per_rev <= 0.0:
            raise ValueError(f"{drivetrain['joint']}: J*DriveMS must be positive")

        effective_ratio = step_deg[index] * 360.0 / microsteps_per_rev
        if not math.isfinite(effective_ratio) or effective_ratio <= 0.0:
            raise ValueError(f"{drivetrain['joint']}: derived effective joint reduction is invalid")

        motor_torque_nm = drivetrain["motorHoldingTorqueNm"]
        if "gearRatio" in drivetrain:
            gearbox_efficiency = drivetrain.get("gearboxEfficiency", 1.0)
            gear_ratio = drivetrain["gearRatio"]
            post_gear_ratio = effective_ratio / gear_ratio
            gearbox_input_torque = motor_torque_nm * gear_ratio * gearbox_efficiency
            gearbox_output_torque = min(
                gearbox_input_torque,
                drivetrain["gearboxMaxPermissibleTorqueNm"],
            )
            joint_torque = gearbox_output_torque * post_gear_ratio
        else:
            # J5 is a lead-screw linkage. The HMI steps/degree calibration gives
            # the effective motor-to-joint ratio; use the public screw lead and a
            # conservative trapezoidal screw efficiency estimate in the metadata.
            joint_torque = motor_torque_nm * effective_ratio * drivetrain["leadScrewEfficiencyEstimate"]

        inertia = AR4_JOINT_EQUIVALENT_INERTIA_KG_M2[index]
        if inertia <= 0.0:
            raise ValueError(f"{drivetrain['joint']}: equivalent inertia must be positive")

        derated_torque = joint_torque * limit_profile["motorTorqueDerate"]
        accel_rad_s2 = derated_torque / inertia
        accel = math.degrees(accel_rad_s2)
        jerk = accel / limit_profile["jerkRiseTimeSec"]

        effective_ratios.append(effective_ratio)
        output_torque_nm.append(joint_torque)
        derated_torque_nm.append(derated_torque)
        accel_deg_s2.append(accel)
        jerk_deg_s3.append(jerk)

        row = dict(drivetrain)
        row["driveMicrostepsPerRev"] = microsteps_per_rev
        row["stepsPerOutputDegree"] = step_deg[index]
        row["effectiveJointReduction"] = round(effective_ratio, 6)
        row["jointOutputTorqueNm"] = round(joint_torque, 6)
        row["deratedPlannerTorqueNm"] = round(derated_torque, 6)
        row["equivalentJointInertiaKgM2"] = inertia
        row["derivedAccelerationDegS2"] = round(accel, 6)
        row["derivedJerkDegS3"] = round(jerk, 6)
        drivetrain_rows.append(row)

    return {
        "jointAccelerationMaxRadS2": [math.radians(v) for v in accel_deg_s2],
        "jointJerkMaxRadS3": [math.radians(v) for v in jerk_deg_s3],
        "motionLimitDerivation": {
            "schema": "ar4_public_drivetrain_v1",
            "notes": [
                "Velocity limits come from AR4 HMI minSpeedDelay, virtual_speed_scale, and steps/degree.",
                "Acceleration limits are torque/inertia estimates derived from public AR4 motor and transmission specs.",
                "Stepper holding torque is a zero-speed value, so the planner uses motorTorqueDerate for torque-speed margin.",
                "Equivalent joint inertia is a package assumption and should be replaced by measured/CAD mass properties when available.",
            ],
            "limitProfile": limit_profile["name"],
            "limitProfileDescription": limit_profile["description"],
            "motorTorqueDerate": limit_profile["motorTorqueDerate"],
            "jerkRiseTimeSec": limit_profile["jerkRiseTimeSec"],
            "effectiveJointReduction": _round_list(effective_ratios),
            "jointOutputTorqueNm": _round_list(output_torque_nm),
            "deratedPlannerTorqueNm": _round_list(derated_torque_nm),
            "equivalentJointInertiaKgM2": _round_list(AR4_JOINT_EQUIVALENT_INERTIA_KG_M2),
            "jointAccelerationMaxDegS2": _round_list(accel_deg_s2),
            "jointJerkMaxDegS3": _round_list(jerk_deg_s3),
            "drivetrains": drivetrain_rows,
        },
    }


def load_ar4_dynamics(source_dir: Path, limit_profile: dict) -> dict:
    """Load planner dynamics from local AR4 HMI source files."""
    defaults_path = source_dir / "defaults.json"
    ar4_py_path = source_dir / "AR4.py"
    if not defaults_path.exists():
        raise FileNotFoundError(f"Missing AR4 dynamics source file: {defaults_path}")
    if not ar4_py_path.exists():
        raise FileNotFoundError(f"Missing AR4 dynamics source file: {ar4_py_path}")

    defaults = json.loads(defaults_path.read_text(encoding="utf-8"))
    ar4_text = ar4_py_path.read_text(encoding="utf-8", errors="ignore")
    match = re.search(r"RUN\['minSpeedDelay'\]\s*=\s*([0-9.]+)", ar4_text)
    if not match:
        raise ValueError(f"{ar4_py_path}: missing required AR4 value RUN['minSpeedDelay']")
    min_delay_us = float(match.group(1))
    if not math.isfinite(min_delay_us) or min_delay_us <= 0.0:
        raise ValueError(f"{ar4_py_path}: RUN['minSpeedDelay'] must be positive")

    # AR4.py applies this virtual timing scale in driveMotorsJ after clamping to
    # RUN['minSpeedDelay'], so the effective pulse gap is minSpeedDelay * scale.
    scale_match = re.search(r"virtual_speed_scale\s*=\s*([0-9.]+)", ar4_text)
    if not scale_match:
        raise ValueError(f"{ar4_py_path}: missing required AR4 value virtual_speed_scale")
    virtual_speed_scale = float(scale_match.group(1))
    if not math.isfinite(virtual_speed_scale) or virtual_speed_scale <= 0.0:
        raise ValueError(f"{ar4_py_path}: virtual_speed_scale must be positive")

    step_deg = [_required_number(defaults, f"J{i}StepDeg", defaults_path) for i in range(1, 7)]
    drive_microsteps = [_required_number(defaults, f"J{i}DriveMS", defaults_path) for i in range(1, 7)]
    neg_lim = [_required_number(defaults, f"J{i}NegLim", defaults_path) for i in range(1, 7)]
    pos_lim = [_required_number(defaults, f"J{i}PosLim", defaults_path) for i in range(1, 7)]
    zero_offsets = [_required_number(defaults, f"J{i}calOff", defaults_path) for i in range(1, 7)]

    effective_delay_us = min_delay_us * virtual_speed_scale
    effective_delay_s = effective_delay_us / 1_000_000.0
    velocity_rad_s: list[float] = []
    for steps_per_degree in step_deg:
        if steps_per_degree <= 0.0:
            raise ValueError(f"{defaults_path}: J*StepDeg values must be positive")
        max_deg_s = 1.0 / (effective_delay_s * steps_per_degree)
        velocity_rad_s.append(math.radians(max_deg_s))

    derived_limits = derive_ar4_joint_limits(step_deg, drive_microsteps, limit_profile)

    return {
        "stepsPerDegreeNominal": step_deg,
        "driveMicrostepsPerRev": drive_microsteps,
        "hmiMinSpeedDelayUs": min_delay_us,
        "hmiVirtualSpeedScale": virtual_speed_scale,
        "controllerMinTickGapUs": effective_delay_us,
        "controlPeriodSec": AR4_DEFAULT_CONTROL_PERIOD_S,
        "defaultJointSpeedDegPerSec": 10.0,
        "defaultLinearSpeedMmPerSec": AR4_DEFAULT_LINEAR_SPEED_MM_S,
        "defaultLinearAccelerationMmSec2": limit_profile["defaultLinearAccelerationMmSec2"],
        "defaultLinearJerkMmSec3": limit_profile["defaultLinearJerkMmSec3"],
        "defaultToolAngularSpeedRadSec": math.radians(limit_profile["defaultToolAngularSpeedDegSec"]),
        "defaultToolAngularAccelerationRadSec2": math.radians(limit_profile["defaultToolAngularAccelerationDegSec2"]),
        "defaultToolAngularJerkRadSec3": math.radians(limit_profile["defaultToolAngularJerkDegSec3"]),
        "singularityThresholdRad": math.radians(AR4_DEFAULT_SINGULARITY_THRESHOLD_DEG),
        "jointVelocityMaxRadS": velocity_rad_s,
        "jointAccelerationMaxRadS2": derived_limits["jointAccelerationMaxRadS2"],
        "jointJerkMaxRadS3": derived_limits["jointJerkMaxRadS3"],
        "motionLimitDerivation": derived_limits["motionLimitDerivation"],
        "jointZeroOffsetRad": [math.radians(v) for v in zero_offsets],
        "qMin": [-math.radians(v) for v in neg_lim],
        "qMax": [math.radians(v) for v in pos_lim],
        "dhmCorrection": [0.0] * 24,
        "toolCalibration": [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0],
        "baseCalibration": [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0],
    }

STL_PARENT = {
    "Link Base-2.STL": "Link Base-1.STL",
    "Link Base-3.STL": "Link Base-2.STL",
    "Link 1-1.STL": "Link Base-3.STL",
    "Link 1-2.STL": "Link 1-1.STL",
    "Link 2-1.STL": "Link 1-2.STL",
    "Link 2-2.STL": "Link 2-1.STL",
    "Link 2-3.STL": "Link 2-2.STL",
    "Link 3-1.STL": "Link 2-3.STL",
    "Link 3-2.STL": "Link 3-1.STL",
    "Link 4-1.STL": "Link 3-2.STL",
    "Link 4-2.STL": "Link 4-1.STL",
    "Link 4-3.STL": "Link 4-2.STL",
    "Link 5-1.STL": "Link 4-3.STL",
    "Link 5-2.STL": "Link 5-1.STL",
    "Link 6-1.STL": "Link 5-2.STL",
    "Link 6-2.STL": "Link 6-1.STL",
}

PART_COLORS = {stl: VTK_COLORS["Silver"] for _link, stls in LINK_STLS for stl in stls}
PART_COLORS.update({
    "Link Base-2.STL": VTK_COLORS["RoyalBlue"],
    "Link Base-3.STL": VTK_COLORS["DimGray"],
    "Link 1-2.STL": VTK_COLORS["DimGray"],
    "Link 2-2.STL": VTK_COLORS["RoyalBlue"],
    "Link 2-3.STL": VTK_COLORS["DimGray"],
    "Link 3-2.STL": VTK_COLORS["DimGray"],
    "Link 4-2.STL": VTK_COLORS["RoyalBlue"],
    "Link 4-3.STL": VTK_COLORS["DimGray"],
    "Link 5-2.STL": VTK_COLORS["DimGray"],
    "Link 6-2.STL": VTK_COLORS["DimGray"],
})

JOINT_BASE_TRANSFORMS = {
    "Link 1-1.STL": [("rx", 180), ("rz", -90), ("t", (0, 0, -92))],
    "Link 2-1.STL": [("rz", -90), ("rx", 270), ("t", (-64.15, 77.78, 8.87))],
    "Link 3-1.STL": [("rz", 180), ("rx", 180), ("t", (0, 305, -27.84))],
    "Link 4-1.STL": [("ry", 90), ("rx", 180), ("t", (-36.7, 0, -75.94))],
    "Link 5-1.STL": [("rz", 180), ("ry", 90), ("t", (147, 0, 44.88))],
    "Link 6-1.STL": [("ry", 90), ("t", (43.3, 0, 25))],
}

JOINT_AXIS_LOCAL = {
    "Link 1-1.STL": (0, 0, 1),
    "Link 2-1.STL": (0, 0, 1),
    "Link 3-1.STL": (0, 0, 1),
    "Link 4-1.STL": (0, 0, 1),
    "Link 5-1.STL": (0, 0, 1),
    "Link 6-1.STL": (0, 0, 1),
}

def identity_node(name: str, node_type: int, data: dict, color=None, children=None) -> dict:
    return node(name, node_type, data, color or DEFAULT_COLOR, children)


def mat_identity() -> list[list[float]]:
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def mat_mul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [[sum(a[r][k] * b[k][c] for k in range(4)) for c in range(4)] for r in range(4)]


def mat_translate(x: float, y: float, z: float) -> list[list[float]]:
    m = mat_identity()
    m[0][3] = x
    m[1][3] = y
    m[2][3] = z
    return m


def mat_rot_x(deg: float) -> list[list[float]]:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]]


def mat_rot_y(deg: float) -> list[list[float]]:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]]


def mat_rot_z(deg: float) -> list[list[float]]:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


def mat_inverse_rigid(m: list[list[float]]) -> list[list[float]]:
    out = mat_identity()
    for r in range(3):
        for c in range(3):
            out[r][c] = m[c][r]
    for r in range(3):
        out[r][3] = -sum(out[r][k] * m[k][3] for k in range(3))
    return out


def vtk_style_transform(ops: Iterable[tuple[str, object]]) -> list[list[float]]:
    # vtkTransform defaults to PreMultiply, where each new operation occurs
    # before the current transform. For column vectors this is current @ op.
    m = mat_identity()
    for op, value in ops:
        if op == "rx":
            t = mat_rot_x(float(value))
        elif op == "ry":
            t = mat_rot_y(float(value))
        elif op == "rz":
            t = mat_rot_z(float(value))
        elif op == "t":
            x, y, z = value
            t = mat_translate(float(x), float(y), float(z))
        else:
            raise ValueError(op)
        m = mat_mul(m, t)
    return m


def transform_point(m: list[list[float]], p: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = p
    return (
        m[0][0] * x + m[0][1] * y + m[0][2] * z + m[0][3],
        m[1][0] * x + m[1][1] * y + m[1][2] * z + m[1][3],
        m[2][0] * x + m[2][1] * y + m[2][2] * z + m[2][3],
    )


def transform_vector(m: list[list[float]], v: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = v
    return (
        m[0][0] * x + m[0][1] * y + m[0][2] * z,
        m[1][0] * x + m[1][1] * y + m[1][2] * z,
        m[2][0] * x + m[2][1] * y + m[2][2] * z,
    )


def normalize(v: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(sum(c * c for c in v))
    if length <= 1e-9:
        return (1.0, 0.0, 0.0)
    return tuple(c / length for c in v)


def cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def add_vec(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def scale_vec(v, s):
    return (v[0] * s, v[1] * s, v[2] * s)


LOCAL_TRANSFORMS = {
    stl: vtk_style_transform(JOINT_BASE_TRANSFORMS.get(stl, []))
    for _link, stls in LINK_STLS + TOOLS
    for stl in stls
}


def world_transform_for(stl_name: str) -> list[list[float]]:
    chain = []
    current = stl_name
    while current:
        chain.append(current)
        current = STL_PARENT.get(current)
    world = mat_identity()
    for stl in reversed(chain):
        world = mat_mul(world, LOCAL_TRANSFORMS.get(stl, mat_identity()))
    return world


def read_stl(path: Path) -> tuple[list[float], list[int], list[float]]:
    data = path.read_bytes()
    if len(data) >= 84:
        tri_count = struct.unpack_from("<I", data, 80)[0]
        expected_size = 84 + tri_count * 50
        if expected_size == len(data):
            return read_binary_stl(data, tri_count)
    return read_ascii_stl(data.decode("utf-8", errors="ignore"))


def read_binary_stl(data: bytes, tri_count: int) -> tuple[list[float], list[int], list[float]]:
    vertices: list[float] = []
    indices: list[int] = []
    offset = 84
    for _ in range(tri_count):
        offset += 12
        for _vertex in range(3):
            x, y, z = struct.unpack_from("<fff", data, offset)
            vertices.extend([x, y, z])
            indices.append(len(indices))
            offset += 12
        offset += 2
    return vertices, indices, compute_bounds(vertices)


def read_ascii_stl(text: str) -> tuple[list[float], list[int], list[float]]:
    vertices: list[float] = []
    indices: list[int] = []
    for line in text.splitlines():
        parts = line.strip().split()
        if len(parts) == 4 and parts[0].lower() == "vertex":
            vertices.extend([float(parts[1]), float(parts[2]), float(parts[3])])
            indices.append(len(indices))
    if len(indices) % 3 != 0:
        raise ValueError("ASCII STL did not contain complete triangles")
    return vertices, indices, compute_bounds(vertices)


def stl_z_up_to_viewer_y_up(vertices: list[float]) -> list[float]:
    converted: list[float] = []
    for i in range(0, len(vertices), 3):
        x = vertices[i]
        y = vertices[i + 1]
        z = vertices[i + 2]
        converted.extend([x, z, -y])
    return converted


def vtk_point_to_viewer(p: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = p
    return (x, z, -y)


AR4_TO_VIEWER = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, -1.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
]


def ar4_transform_to_viewer(m: list[list[float]]) -> list[list[float]]:
    return mat_mul(mat_mul(AR4_TO_VIEWER, m), mat_inverse_rigid(AR4_TO_VIEWER))


def dhm_link_transform(joint_index: int, q: float = 0.0) -> list[list[float]]:
    theta = AR4_DHM[joint_index] + q
    a = AR4_DHM[6 + joint_index]
    alpha = AR4_DHM[12 + joint_index]
    d = AR4_DHM[18 + joint_index]
    crx, srx = math.cos(alpha), math.sin(alpha)
    crz, srz = math.cos(theta), math.sin(theta)
    return [
        [crz, -srz, 0.0, a],
        [crx * srz, crx * crz, -srx, -d * srx],
        [srx * srz, srx * crz, crx, d * crx],
        [0.0, 0.0, 0.0, 1.0],
    ]


def dhm_joint_frame_transform(joint_index: int) -> list[list[float]]:
    a = AR4_DHM[6 + joint_index]
    alpha = AR4_DHM[12 + joint_index]
    crx, srx = math.cos(alpha), math.sin(alpha)
    return [
        [1.0, 0.0, 0.0, a],
        [0.0, crx, -srx, 0.0],
        [0.0, srx, crx, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


# Home flange pose in viewer coordinates. Derive it from DHM, not mesh bounds, and rotate the
# viewer-frame approach axis from Y to the Z axis expected by tool-pose consumers.
def dhm_flange_loc() -> list[float]:
    ar4_pose = mat_identity()
    for i in range(6):
        ar4_pose = mat_mul(ar4_pose, dhm_link_transform(i))
    viewer_flange = ar4_transform_to_viewer(ar4_pose)
    approach_to_z = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    flange = mat_mul(viewer_flange, approach_to_z)
    # Snapped, because the quarter turns leave cos(pi/2) as 6.1e-17 in nine of the twelve slots.
    # Those spell out in full in the load_robot_model command the firmware parses from a fixed
    # 1536-byte buffer, and they cost about a hundred bytes of it to say zero.
    return [snap_axis_value(flange[row][col]) for row in range(3) for col in range(4)]


def snap_axis_value(value: float) -> float:
    for exact in (0.0, 1.0, -1.0):
        if abs(value - exact) < 1.0e-12:
            return exact
    return value


def dhm_joint_axis_pose(joint_index: int) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    ar4_pose = mat_identity()
    for i in range(joint_index):
        ar4_pose = mat_mul(ar4_pose, dhm_link_transform(i))
    viewer_joint_frame = ar4_transform_to_viewer(mat_mul(ar4_pose, dhm_joint_frame_transform(joint_index)))
    origin = transform_point(viewer_joint_frame, (0.0, 0.0, 0.0))
    direction = transform_vector(viewer_joint_frame, (0.0, 1.0, 0.0))
    return origin, normalize(direction)


def apply_world_and_viewer_transform(vertices: list[float], world: list[list[float]]) -> list[float]:
    converted: list[float] = []
    for i in range(0, len(vertices), 3):
        p = transform_point(world, (vertices[i], vertices[i + 1], vertices[i + 2]))
        converted.extend(vtk_point_to_viewer(p))
    return converted



# Display reduction is ordered from lossless welding through culling to silhouette-changing
# decimation. Collision hulls retain the full-fidelity vertices. Reduction requires numpy.
try:
    import numpy as _np
except ImportError:
    _np = None


MESH_WELD_TOLERANCE_MM = 1.0e-4

# Set from the command line before the meshes are converted.
cull_hidden_geometry = True
max_display_faces = 0


def weld_mesh(vertices: list[float], indices: list[int]):
    flat = _np.asarray(vertices, dtype=_np.float64).reshape(-1, 3)
    faces = _np.asarray(indices, dtype=_np.int64).reshape(-1, 3)
    key = _np.round(flat / MESH_WELD_TOLERANCE_MM).astype(_np.int64)
    _, first, inverse = _np.unique(key, axis=0, return_index=True, return_inverse=True)
    verts = flat[first]
    # reshape(-1): numpy 2 returns the inverse with the reduced axis kept, older versions flatten it.
    faces = inverse.reshape(-1)[faces]
    keep = (faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])
    return verts, faces[keep]


def visible_face_mask(verts, faces, directions: int = 128, resolution: int = 512):
    visible_vertex = _np.zeros(len(verts), dtype=bool)
    visible_face = _np.zeros(len(faces), dtype=bool)
    centre = 0.5 * (verts.min(0) + verts.max(0))
    radius = float(_np.linalg.norm(verts.max(0) - centre)) + 1.0e-6
    tolerance = radius * 1.0e-4

    i = _np.arange(directions) + 0.5
    polar = _np.arccos(1.0 - 2.0 * i / directions)
    azimuth = _np.pi * (1.0 + 5.0 ** 0.5) * i
    dirs = _np.stack([_np.cos(azimuth) * _np.sin(polar),
                      _np.sin(azimuth) * _np.sin(polar),
                      _np.cos(polar)], axis=1)

    corners = [faces[:, 0], faces[:, 1], faces[:, 2]]
    for d in dirs:
        up = _np.array([0.0, 0.0, 1.0]) if abs(d[2]) < 0.9 else _np.array([1.0, 0.0, 0.0])
        right = _np.cross(up, d); right /= _np.linalg.norm(right)
        down = _np.cross(d, right)
        camera = (verts - centre) @ _np.stack([right, down, d], axis=1)
        scale = (resolution - 2) / (2.0 * radius)
        px = _np.clip(((camera[:, 0] + radius) * scale).astype(_np.int32) + 1, 0, resolution - 1)
        py = _np.clip(((camera[:, 1] + radius) * scale).astype(_np.int32) + 1, 0, resolution - 1)
        depth = camera[:, 2]
        pixel = py * resolution + px

        centroid_depth = depth[faces].mean(1)
        centroid_pixel = (py[faces].mean(1).astype(_np.int32) * resolution +
                          px[faces].mean(1).astype(_np.int32))

        zbuf = _np.full(resolution * resolution, _np.inf)
        _np.minimum.at(zbuf, pixel, depth)
        _np.minimum.at(zbuf, centroid_pixel, centroid_depth)

        visible_vertex |= depth <= zbuf[pixel] + tolerance
        visible_face |= centroid_depth <= zbuf[centroid_pixel] + tolerance

    # Keep faces visible at their centroid or at two or more corners to preserve silhouette slivers.
    corner_votes = _np.zeros(len(faces), dtype=_np.int8)
    for corner in corners:
        corner_votes += visible_vertex[corner].astype(_np.int8)
    return visible_face | (corner_votes >= 2)


def decimate_mesh(verts, faces, target_faces: int):
    if len(faces) <= target_faces:
        return verts, faces
    extent = float(_np.linalg.norm(verts.max(0) - verts.min(0)))
    low, high = extent * 1.0e-5, extent
    best = (verts, faces)
    for _ in range(24):
        cell = 0.5 * (low + high)
        key = _np.floor((verts - verts.min(0)) / cell).astype(_np.int64)
        _, first, inverse = _np.unique(key, axis=0, return_index=True, return_inverse=True)
        merged = inverse.reshape(-1)[faces]
        keep = ((merged[:, 0] != merged[:, 1]) & (merged[:, 1] != merged[:, 2]) &
                (merged[:, 0] != merged[:, 2]))
        count = int(keep.sum())
        if count > target_faces:
            low = cell
        else:
            high = cell
            best = (verts[first], merged[keep])
        if abs(count - target_faces) <= max(16, target_faces // 100):
            best = (verts[first], merged[keep])
            break
    return best


def reduce_display_mesh(vertices: list[float], indices: list[int], stl_name: str,
                        cull: bool, max_faces: int):
    """Weld, then drop what cannot be seen, then decimate if it is still over budget."""
    if _np is None:
        return vertices, indices, ""
    soup_faces = len(indices) // 3
    verts, faces = weld_mesh(vertices, indices)
    welded_faces = len(faces)
    culled = 0
    if cull and welded_faces > 0:
        mask = visible_face_mask(verts, faces)
        culled = welded_faces - int(mask.sum())
        faces = faces[mask]
        used, faces = _np.unique(faces, return_inverse=True)
        verts, faces = verts[used], faces.reshape(-1, 3)
    decimated = 0
    if max_faces > 0 and len(faces) > max_faces:
        before = len(faces)
        verts, faces = decimate_mesh(verts, faces, max_faces)
        decimated = before - len(faces)
    note = ("%-20s %6d tris -> %6d  (welded, %d internal dropped, %d decimated), %d verts"
            % (stl_name, soup_faces, len(faces), culled, decimated, len(verts)))
    return verts, faces, note


def mesh_node(name: str, source: str, color=None) -> dict:
    return node(name, TYPE_MESH_GEOMETRY, {"meshSource": source.replace("\\", "/")},
                color or GEOMETRY_COLOR)


def convert_stl(source_dir: Path, package_dir: Path, stl_name: str, prefix: str) -> tuple[str, list[float], list[float], list[int]]:
    stl_path = source_dir / stl_name
    if not stl_path.exists():
        raise FileNotFoundError(stl_path)
    vertices, indices, bounds = read_stl(stl_path)
    vertices = apply_world_and_viewer_transform(vertices, world_transform_for(stl_name))
    bounds = compute_bounds(vertices)
    mesh_rel = f"meshes/{prefix}_{slug(stl_name)}.meshbin"
    # Bounds stay off the full input, so a culling mistake cannot quietly shrink the part's extent.
    display_vertices, display_indices, note = reduce_display_mesh(
        vertices, indices, stl_name, cull_hidden_geometry, max_display_faces)
    if note:
        print("  " + note)
    write_meshbin(package_dir / mesh_rel, display_vertices, display_indices, bounds)
    # The hull gets the original: a convex hull is a safety surface, not a picture.
    return mesh_rel, bounds, vertices, indices


def make_axis_marker(origin: tuple[float, float, float], axis: tuple[float, float, float], length=100.0, radius=4.0) -> tuple[list[float], list[int], list[float]]:
    axis = normalize(axis)
    helper = (0.0, 1.0, 0.0) if abs(axis[1]) < 0.9 else (1.0, 0.0, 0.0)
    u = normalize(cross(axis, helper))
    v = normalize(cross(axis, u))
    start = add_vec(origin, scale_vec(axis, -length * 0.5))
    end = add_vec(origin, scale_vec(axis, length * 0.5))
    segments = 16
    verts: list[float] = []
    inds: list[int] = []
    for center in (start, end):
        for i in range(segments):
            angle = 2.0 * math.pi * i / segments
            point = add_vec(center, add_vec(scale_vec(u, math.cos(angle) * radius), scale_vec(v, math.sin(angle) * radius)))
            verts.extend(point)
    for i in range(segments):
        j = (i + 1) % segments
        inds.extend([i, j, segments + j])
        inds.extend([i, segments + j, segments + i])

    origin_index = len(verts) // 3
    pivot_radius = radius * 2.6
    for p in (
        add_vec(origin, scale_vec(axis, pivot_radius)),
        add_vec(origin, scale_vec(axis, -pivot_radius)),
        add_vec(origin, scale_vec(u, pivot_radius)),
        add_vec(origin, scale_vec(u, -pivot_radius)),
        add_vec(origin, scale_vec(v, pivot_radius)),
        add_vec(origin, scale_vec(v, -pivot_radius)),
    ):
        verts.extend(p)
    inds.extend([
        origin_index + 0, origin_index + 2, origin_index + 4,
        origin_index + 2, origin_index + 1, origin_index + 4,
        origin_index + 1, origin_index + 3, origin_index + 4,
        origin_index + 3, origin_index + 0, origin_index + 4,
        origin_index + 2, origin_index + 0, origin_index + 5,
        origin_index + 1, origin_index + 2, origin_index + 5,
        origin_index + 3, origin_index + 1, origin_index + 5,
        origin_index + 0, origin_index + 3, origin_index + 5,
    ])
    bounds = compute_bounds(verts)
    return verts, inds, bounds


def write_axis_marker(package_dir: Path, mesh_rel: str, joint_index: int) -> list[float]:
    origin, axis = dhm_joint_axis_pose(joint_index)
    vertices, indices, bounds = make_axis_marker(origin, axis)
    write_meshbin(package_dir / mesh_rel, vertices, indices, bounds)
    return bounds


def build_robot_json(source_dir: Path, package_dir: Path, limit_profile: dict) -> dict:
    children: list[dict] = []
    active_tool_path: list[int] = []
    dynamics = load_ar4_dynamics(source_dir, limit_profile)

    for link_number, (link_name, stl_names) in enumerate(LINK_STLS):
        link_node = identity_node(
            link_name,
            TYPE_ROBOT_LINK,
            {"geometryPaths": []},
            color=DEFAULT_COLOR,
        )
        if link_number == 0:
            # The four external through-holes in the pedestal floor interface of Link Base-2.STL.
            # The 2x6 hole pattern under the blue cover is internal and is not a snap location.
            link_node["mountingHoles"] = {
                "comment": (
                    "Four external AR4 pedestal mounting-hole centers fitted from complete cylindrical "
                    "rings in the full-resolution Link Base-2 STL. The internal 2 by 6 enclosure pattern "
                    "and sidewall holes are excluded."
                ),
                "placementSource": True,
                "grids": [{
                    "originMm": [-60.0, 0.0, -50.0],
                    "uStepMm": [100.0, 0.0, 0.0],
                    "vStepMm": [0.0, 0.0, 100.0],
                    "uCount": 2,
                    "vCount": 2,
                }],
            }
        children.append(link_node)

        geometry_paths = []
        link_bounds: list[float] | None = None
        collision_hulls: list[dict] = []
        for stl_name in stl_names:
            mesh_rel, bounds, vertices, indices = convert_stl(source_dir, package_dir, stl_name, f"link{link_number}")
            link_bounds = merge_bounds(link_bounds, bounds)
            collision_hulls.append(convex_hull(vertices, stl_name, dedup_decimals=6))
            geometry_paths.append([len(children)])
            children.append(mesh_node(stl_name.replace(".STL", ""), mesh_rel, color=PART_COLORS.get(stl_name, GEOMETRY_COLOR)))
        link_node["data"]["geometryPaths"] = geometry_paths
        if link_bounds is not None:
            link_node["data"]["collisionHulls"] = collision_hulls

    tool_node = identity_node(
        "J6 TCP",
        TYPE_ROBOT_TOOL,
        {
            "geometryPaths": [],
            "tcps": [{
                "name": "TCP",
                "loc": dhm_flange_loc(),
            }],
        },
        color=TOOL_COLOR,
    )
    active_tool_path = [len(children)]
    children.append(tool_node)

    for joint_index, _joint_stl in enumerate(JOINT_AXIS_LOCAL, start=1):
        mesh_rel = f"meshes/joint_axis_{joint_index}.meshbin"
        write_axis_marker(package_dir, mesh_rel, joint_index - 1)
        color = VTK_COLORS["Gold"] if joint_index % 2 else VTK_COLORS["Tomato"]
        children.append(mesh_node(f"Joint {joint_index} Axis", mesh_rel, color=color))

    robot_name = (
        "AR4 6DOF Robot (Stress High Limits)"
        if limit_profile["name"] == AR4_STRESS_HIGH_LIMIT_PROFILE["name"]
        else "AR4 6DOF Robot"
    )
    return identity_node(
        robot_name,
        TYPE_OPW6_ROBOT,
        {
            "dhm": AR4_DHM,
            "qHome": [0, 0, 0, 0, 0, 0],
            "qMin": dynamics["qMin"],
            "qMax": dynamics["qMax"],
            "stepsPerDegreeNominal": dynamics["stepsPerDegreeNominal"],
            "driveMicrostepsPerRev": dynamics["driveMicrostepsPerRev"],
            "hmiMinSpeedDelayUs": dynamics["hmiMinSpeedDelayUs"],
            "hmiVirtualSpeedScale": dynamics["hmiVirtualSpeedScale"],
            "controllerMinTickGapUs": dynamics["controllerMinTickGapUs"],
            "controlPeriodSec": dynamics["controlPeriodSec"],
            "defaultJointSpeedDegPerSec": dynamics["defaultJointSpeedDegPerSec"],
            "defaultLinearSpeedMmPerSec": dynamics["defaultLinearSpeedMmPerSec"],
            "defaultLinearAccelerationMmSec2": dynamics["defaultLinearAccelerationMmSec2"],
            "defaultLinearJerkMmSec3": dynamics["defaultLinearJerkMmSec3"],
            "defaultToolAngularSpeedRadSec": dynamics["defaultToolAngularSpeedRadSec"],
            "defaultToolAngularAccelerationRadSec2": dynamics["defaultToolAngularAccelerationRadSec2"],
            "defaultToolAngularJerkRadSec3": dynamics["defaultToolAngularJerkRadSec3"],
            "singularityThresholdRad": dynamics["singularityThresholdRad"],
            "jointVelocityMaxRadS": dynamics["jointVelocityMaxRadS"],
            "jointAccelerationMaxRadS2": dynamics["jointAccelerationMaxRadS2"],
            "jointJerkMaxRadS3": dynamics["jointJerkMaxRadS3"],
            "motionLimitDerivation": dynamics["motionLimitDerivation"],
            "jointZeroOffsetRad": dynamics["jointZeroOffsetRad"],
            "dhmCorrection": dynamics["dhmCorrection"],
            "toolCalibration": dynamics["toolCalibration"],
            "baseCalibration": dynamics["baseCalibration"],
            "activeToolPath": active_tool_path,
            "collisionIgnores": [[0, 1], [1, 2], [2, 3], [3, 4], [4, 5], [5, 6], [2, 4], [4, 6], [0, 2]],
        },
        children=children,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True,
                        help="AR4 HMI source folder containing the STL files (vendor data, not in the repo)")
    parser.add_argument("--package-dir", type=Path, default=DEFAULT_PACKAGE_DIR, help="Extracted package output folder")
    parser.add_argument("--zip", type=Path, default=DEFAULT_ZIP, help="Zip package output path")
    parser.add_argument("--keep-hidden-geometry", action="store_true",
                        help="Keep faces no outside view can reach (skips the visibility cull)")
    parser.add_argument("--max-faces", type=int, default=25000,
                        help="Decimate any mesh above this many triangles (0 disables decimation). "
                             "The default lands the zip near a megabyte; the base castings are the "
                             "only meshes it touches.")
    parser.add_argument(
        "--limit-profile",
        choices=sorted(AR4_LIMIT_PROFILES),
        default=AR4_REASONABLE_LIMIT_PROFILE["name"],
        help="Planner limit profile to embed in the robot package",
    )
    args = parser.parse_args()
    limit_profile = AR4_LIMIT_PROFILES[args.limit_profile]
    global cull_hidden_geometry, max_display_faces
    cull_hidden_geometry = not args.keep_hidden_geometry
    max_display_faces = max(0, args.max_faces)

    package_dir = args.package_dir
    if package_dir.exists():
        for existing in sorted(package_dir.rglob("*"), reverse=True):
            if existing.is_file():
                existing.unlink()
            elif existing.is_dir():
                existing.rmdir()
    package_dir.mkdir(parents=True, exist_ok=True)

    robot = build_robot_json(args.source, package_dir, limit_profile)
    robot_json = package_dir / "robot.json"
    robot_json.write_text(json.dumps(robot, indent=2), encoding="utf-8")
    write_zip(package_dir, args.zip)

    print(f"Wrote package directory: {package_dir}")
    print(f"Wrote zip package: {args.zip}")
    print(f"Limit profile: {limit_profile['name']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
