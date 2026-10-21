#!/usr/bin/env python3
"""Validate gesture candidates offline. No ROS initialization or publishers."""

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin
import yaml


ROOT = Path(__file__).resolve().parents[3]
PACKAGE = Path(__file__).resolve().parents[1]
URDF = ROOT / "src/qi_robot_description/urdf/s4_dual_arm.urdf"
GESTURES = PACKAGE / "config/gesture_actions.yaml"
IK_CONFIG = PACKAGE / "config/differential_ik.yaml"
SIDES = ("left", "right")
STEP = 0.02  # The planned real control loop runs at 50 Hz.


def vector(value, length, label):
    result = np.asarray(value, dtype=float)
    if result.shape != (length,) or not np.isfinite(result).all():
        raise ValueError(f"{label} must be {length} finite numbers")
    return result


def joint_mapping(model, order):
    if len(order) != 7 or len(set(order)) != 7:
        raise ValueError("joint_order must contain seven unique entries")
    mapping = {}
    for side in SIDES:
        names = [f"{side}_{part}_joint" for part in order]
        ids = [model.getJointId(name) for name in names]
        if any(joint_id == 0 for joint_id in ids):
            raise ValueError(f"URDF missing joint in {names}")
        mapping[side] = [model.idx_qs[joint_id] for joint_id in ids]
        if any(model.nqs[joint_id] != 1 for joint_id in ids):
            raise ValueError(f"non-scalar joint in {names}")
    return mapping


def build_collision_model(model):
    # Full URDF meshes are used for the torso/legs, arm links, and palm bases.
    # The fixed finger meshes are omitted because their URDF pose does not
    # reflect the O6 command. Attached neighbours are also omitted.
    geom = pin.buildGeomFromUrdf(
        model, str(URDF), pin.GeometryType.COLLISION,
        package_dirs=[str(ROOT / "src")])
    arm_links = {}
    fixed_links = []
    for index, obj in enumerate(geom.geometryObjects):
        name = obj.name.removesuffix("_0")
        if name == "base_link" or any(
            name.startswith(f"{side}_{part}")
            for side in SIDES for part in ("hip_", "knee_", "foot_")):
            fixed_links.append(index)
        for side in SIDES:
            if name in [f"{side}_{part}_link" for part in (
                "shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
                "wrist_roll", "wrist_pitch", "wrist_yaw")]:
                arm_links[index] = (side, name)
            elif name == ("LH_hand_base_link" if side == "left" else "RH_hand_base_link"):
                arm_links[index] = (side, name)
    expected = 2 * 8
    if len(arm_links) != expected or not fixed_links:
        raise ValueError("unexpected URDF collision geometry; review pair selection")
    arm_indices = list(arm_links)
    for i, first in enumerate(arm_indices):
        side_a, _ = arm_links[first]
        for second in arm_indices[i + 1:]:
            side_b, _ = arm_links[second]
            if side_a == side_b:
                side_order = [k for k in arm_indices if arm_links[k][0] == side_a]
                if abs(side_order.index(first) - side_order.index(second)) < 3:
                    continue
            geom.addCollisionPair(pin.CollisionPair(first, second))
        for fixed in fixed_links:
            # The shoulder pitch meshes meet the base shell at the mounting
            # points even at home, so this attached pair is excluded.
            if geom.geometryObjects[fixed].name == "base_link_0" and \
                    arm_links[first][1].endswith("shoulder_pitch_link"):
                continue
            geom.addCollisionPair(pin.CollisionPair(fixed, first))
    return geom


def quintic(s):
    return (10 * s**3 - 15 * s**4 + 6 * s**5,
            30 * s**2 - 60 * s**3 + 30 * s**4,
            60 * s - 180 * s**2 + 120 * s**3)


def oscillation(s, cycles, terms):
    sine, cosine = math.sin(math.pi * s), math.cos(math.pi * s)
    envelope = sine**4
    envelope_d = 4 * math.pi * sine**3 * cosine
    envelope_dd = 4 * math.pi**2 * (3 * sine**2 * cosine**2 - sine**4)
    angle_rate = 2 * math.pi * cycles
    value = speed = acceleration = 0.0
    for amplitude, phase in terms:
        angle = angle_rate * s + phase
        sine, cosine = math.sin(angle), math.cos(angle)
        value += amplitude * envelope * sine
        speed += amplitude * (envelope_d * sine + envelope * angle_rate * cosine)
        acceleration += amplitude * (
            envelope_dd * sine + 2 * envelope_d * angle_rate * cosine
            - envelope * angle_rate**2 * sine)
    return value, speed, acceleration


def sample_segment(kind, start, target, spec, order, s):
    pose = {side: start[side].copy() for side in SIDES}
    speed = {side: np.zeros(7) for side in SIDES}
    acceleration = {side: np.zeros(7) for side in SIDES}
    if kind != "motion":
        blend, blend_d, blend_dd = quintic(s)
        for side in SIDES:
            delta = target[side] - start[side]
            pose[side] += blend * delta
            speed[side] = blend_d * delta
            acceleration[side] = blend_dd * delta
    else:
        cycles = float(spec["motion_cycles"])
        for side in SIDES:
            for joint, terms in spec.get(f"{side}_oscillation", {}).items():
                index = order.index(joint)
                value, derivative, second = oscillation(s, cycles, terms)
                pose[side][index] += value
                speed[side][index] = derivative
                acceleration[side][index] = second
    return pose, speed, acceleration


def choose_duration(kind, start, target, spec, order, velocity, acceleration, factor):
    minimum = float(spec[f"{kind}_min_sec"])
    if minimum <= 0 or factor < 1:
        raise ValueError("segment durations must be positive; safety factor >= 1")
    required = minimum
    # Dense parameter-space sweep determines time scaling; the final path is
    # checked again at the controller period with analytic derivatives.
    for s in np.linspace(0, 1, 2001):
        _, dq_ds, ddq_ds2 = sample_segment(kind, start, target, spec, order, float(s))
        for side in SIDES:
            required = max(required,
                           float(np.max(np.abs(dq_ds[side]) / velocity)),
                           float(np.max(np.sqrt(np.abs(ddq_ds2[side]) / acceleration))))
    return math.ceil(required * factor / STEP) * STEP


def parse_start(values, home):
    if values is None:
        return {side: home[side].copy() for side in SIDES}
    raw = vector([float(item) for item in values.split(",")], 26, "motor positions")
    return {"left": raw[12:19], "right": raw[19:26]}


def validate_action(name, spec, start, home, order, mapping, model, geom,
                    lower, upper, velocity, acceleration, margin, factor):
    target = {side: vector(spec.get(f"{side}_target", start[side]), 7,
                           f"{name} {side} target") for side in SIDES}
    segments = (("entry", start, target),
                ("motion", target, target),
                ("return", target, home))
    data = model.createData()
    geom_data = geom.createData()
    q = pin.neutral(model)
    min_clearance = math.inf
    peak_speed = peak_acceleration = 0.0
    total_time = 0.0
    durations = []
    for kind, initial, final in segments:
        duration = choose_duration(kind, initial, final, spec, order,
                                   velocity, acceleration, factor)
        durations.append(duration)
        for frame in range(round(duration / STEP) + 1):
            s = min(frame * STEP / duration, 1.0)
            pose, dq_ds, ddq_ds2 = sample_segment(kind, initial, final, spec, order, s)
            for side in SIDES:
                values = pose[side]
                clearance = np.minimum(values - lower[side], upper[side] - values)
                smallest = float(np.min(clearance))
                min_clearance = min(min_clearance, smallest)
                if smallest < margin - 1e-9:
                    joint = order[int(np.argmin(clearance))]
                    raise ValueError(f"{name} {kind} t={total_time + frame * STEP:.2f}s: "
                                     f"{side} {joint} limit clearance {smallest:.4f} < {margin:.4f}")
                speed = float(np.max(np.abs(dq_ds[side]) / duration / velocity))
                accel = float(np.max(np.abs(ddq_ds2[side]) / duration**2 / acceleration))
                peak_speed = max(peak_speed, speed)
                peak_acceleration = max(peak_acceleration, accel)
                if speed > 1 + 1e-9 or accel > 1 + 1e-9:
                    raise ValueError(f"{name} {kind} t={total_time + frame * STEP:.2f}s: "
                                     "velocity or acceleration exceeds configured limit")
                q[mapping[side]] = values
            pin.computeCollisions(model, data, geom, geom_data, q, False)
            for pair, result in zip(geom.collisionPairs, geom_data.collisionResults):
                if result.isCollision():
                    first = geom.geometryObjects[pair.first].name
                    second = geom.geometryObjects[pair.second].name
                    raise ValueError(f"{name} {kind} t={total_time + frame * STEP:.2f}s: "
                                     f"collision {first} / {second}")
        total_time += duration
        if kind == "entry":
            # The grip wait holds the already checked target configuration.
            total_time += float(spec.get("grip_wait_sec", 0.0))
    print(f"{name}: PASS  entry={durations[0]:.2f}s "
          f"grip_wait={float(spec.get('grip_wait_sec', 0.0)):.2f}s "
          f"motion={durations[1]:.2f}s "
          f"return={durations[2]:.2f}s total={total_time:.2f}s "
          f"clearance={min_clearance:.3f}rad "
          f"speed={peak_speed:.2f}x acceleration={peak_acceleration:.2f}x")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--action", choices=("wave", "shake", "baoquan"))
    parser.add_argument("--motor-positions", help="26 comma-separated read-only motor positions; default: configured home")
    args = parser.parse_args()
    settings = yaml.safe_load(GESTURES.read_text())
    ik = yaml.safe_load(IK_CONFIG.read_text())["qiling_differential_ik_real"]["ros__parameters"]
    model = pin.buildModelFromUrdf(str(URDF))
    order = settings["joint_order"]
    mapping = joint_mapping(model, order)
    if settings["motor_offsets"] != {"left": 12, "right": 19}:
        raise ValueError("expected left/right arm motor offsets 12/19")
    lower = {side: np.asarray(model.lowerPositionLimit[mapping[side]]) for side in SIDES}
    upper = {side: np.asarray(model.upperPositionLimit[mapping[side]]) for side in SIDES}
    home = {side: vector(ik[f"{side}_q_rest"], 7, f"{side} home") for side in SIDES}
    start = parse_start(args.motor_positions, home)
    velocity = vector(ik["max_joint_velocity_rps"], 7, "velocity limits")
    acceleration = vector(ik["max_joint_acceleration_rps2"], 7, "acceleration limits")
    if np.any(velocity <= 0) or np.any(acceleration <= 0):
        raise ValueError("velocity and acceleration limits must be positive")
    geom = build_collision_model(model)
    print("Offline URDF/Coal check; no ROS nodes or command publishers")
    print("Arm motor mapping:")
    for side in SIDES:
        print(f"  {side}: " + ", ".join(
            f"{settings['motor_offsets'][side] + i}={side}_{name}_joint"
            for i, name in enumerate(order)))
    print(f"Collision pairs: {len(geom.collisionPairs)}; finger pose, scene objects, "
          "and live leg pose are outside this model")
    names = (args.action,) if args.action else settings["actions"].keys()
    failures = 0
    for name in names:
        try:
            validate_action(name, settings["actions"][name], start, home, order,
                            mapping, model, geom, lower, upper, velocity,
                            acceleration, settings["minimum_limit_clearance_rad"],
                            settings["duration_safety_factor"])
        except ValueError as exc:
            failures += 1
            print(f"{name}: FAIL  {exc}", file=sys.stderr)
    return 1 if failures else 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError) as error:
        print(f"offline check unavailable: {error}", file=sys.stderr)
        sys.exit(2)
