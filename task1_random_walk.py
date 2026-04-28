#!/usr/bin/env python3
import json
import math
import random
import time

import paho.mqtt.client as mqtt
from pipuck.pipuck import PiPuck


# ======================
# CONFIG
# ======================

BROKER = "192.168.178.43"
PORT = 1883
POSITION_TOPIC = "robot_pos/all"

ROBOT_ID = "33"

# Safer arena bounds for the robot CENTER.
# Your previous code allowed y=1.0, but the robot can physically go over the border.
ARENA_X_MIN = 0.05
ARENA_X_MAX = 1.95
ARENA_Y_MIN = 0.05
ARENA_Y_MAX = 0.95

BOUNDARY_MARGIN = 0.18

# Based on your log, the robot got stuck around top-left.
# Change/delete this if your forbidden zone is somewhere else.
FORBIDDEN_ZONES = [
    # name, x_min, x_max, y_min, y_max
    ("top_left_forbidden", 0.00, 0.25, 0.75, 1.00),
]

FORBIDDEN_MARGIN = 0.03

ROBOT_AVOID_DISTANCE = 0.23

FORWARD_SPEED = 220
AVOID_SPEED = 170
TURN_SPEED = 190
MAX_SPEED = 320

CONTROL_PERIOD = 0.10

# From your logs, angle behaves like degrees, and world heading is roughly:
# heading = -camera_angle + 95 degrees
ANGLE_SIGN = -1
INITIAL_HEADING_OFFSET_DEG = 95.0

# If the robot turns the wrong way when avoiding, change this to -1.
TURN_SIGN = 1


# ======================
# GLOBAL STATE
# ======================

latest_positions = {}
latest_raw = {}

random_target_heading = 0.0
next_random_change = 0.0

heading_offset = math.radians(INITIAL_HEADING_OFFSET_DEG)
last_pose_for_calib = None
last_cmd = (0, 0)

last_print_time = 0.0


# ======================
# SMALL HELPERS
# ======================

def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def norm_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def rad_to_deg(a):
    return math.degrees(norm_angle(a))


def robot_key(robot_id):
    return str(robot_id).replace("pi-puck", "").replace("robot", "").strip()


def raw_angle_to_rad(raw_angle):
    # In your log, the angle is printed like 338.86, 307.54, etc.
    # So treat it as degrees.
    return math.radians(float(raw_angle))


def pose_from_record(record):
    """
    Simple parser for likely MQTT formats:

    {"position": [x, y], "angle": a}
    {"x": x, "y": y, "angle": a}
    [x, y, angle]
    """

    if isinstance(record, (list, tuple)):
        if len(record) >= 2:
            x = float(record[0])
            y = float(record[1])
            angle = float(record[2]) if len(record) >= 3 else 0.0
            return x, y, angle

    if isinstance(record, dict):
        if "position" in record:
            pos = record["position"]

            if isinstance(pos, (list, tuple)):
                x = float(pos[0])
                y = float(pos[1])
            elif isinstance(pos, dict):
                x = float(pos["x"])
                y = float(pos["y"])
            else:
                return None

            angle = float(record.get("angle", record.get("theta", 0.0)))
            return x, y, angle

        if "x" in record and "y" in record:
            x = float(record["x"])
            y = float(record["y"])
            angle = float(record.get("angle", record.get("theta", 0.0)))
            return x, y, angle

    return None


def parse_positions(data):
    positions = {}

    if not isinstance(data, dict):
        return positions

    # Format: {"robots": [{"id": "33", "position": [...], "angle": ...}, ...]}
    if "robots" in data and isinstance(data["robots"], list):
        for record in data["robots"]:
            if not isinstance(record, dict):
                continue

            rid = record.get("id", record.get("robot_id"))
            pose = pose_from_record(record)

            if rid is not None and pose is not None:
                positions[robot_key(rid)] = pose

        return positions

    # Format: {"33": {"position": [...], "angle": ...}, "34": ...}
    for rid, record in data.items():
        pose = pose_from_record(record)

        if pose is not None:
            positions[robot_key(rid)] = pose

    return positions


def get_my_pose():
    return latest_positions.get(robot_key(ROBOT_ID))


def current_heading(my_pose):
    raw_angle = raw_angle_to_rad(my_pose[2])
    return norm_angle(ANGLE_SIGN * raw_angle + heading_offset)


def smooth_angle(old_angle, new_angle, alpha):
    x = (1.0 - alpha) * math.cos(old_angle) + alpha * math.cos(new_angle)
    y = (1.0 - alpha) * math.sin(old_angle) + alpha * math.sin(new_angle)
    return math.atan2(y, x)


# ======================
# HEADING CALIBRATION
# ======================

def update_heading_calibration(my_pose):
    """
    Learns the camera-angle offset from actual movement.

    Only calibrates when the previous command was almost straight forward.
    """
    global last_pose_for_calib, heading_offset

    if my_pose is None:
        return

    if last_pose_for_calib is not None:
        old_x, old_y, old_angle = last_pose_for_calib
        x, y, raw_angle = my_pose

        dx = x - old_x
        dy = y - old_y
        dist = math.sqrt(dx * dx + dy * dy)

        left, right = last_cmd
        went_straight = left > 130 and right > 130 and abs(left - right) < 60

        if went_straight and dist > 0.01:
            observed_heading = math.atan2(dy, dx)
            raw_rad = raw_angle_to_rad(raw_angle)

            sample_offset = norm_angle(observed_heading - ANGLE_SIGN * raw_rad)
            heading_offset = smooth_angle(heading_offset, sample_offset, 0.20)

    last_pose_for_calib = my_pose


# ======================
# AVOIDANCE LOGIC
# ======================

def inside_rect(x, y, zone, margin=0.0):
    _name, x_min, x_max, y_min, y_max = zone

    return (
        x_min - margin <= x <= x_max + margin and
        y_min - margin <= y <= y_max + margin
    )


def add_vector_towards(vec, x, y, tx, ty, strength):
    dx = tx - x
    dy = ty - y
    dist = math.sqrt(dx * dx + dy * dy)

    if dist < 0.001:
        return

    vec[0] += strength * dx / dist
    vec[1] += strength * dy / dist


def avoidance_vector(my_pose):
    """
    Returns a vector pointing toward safety.
    If vector is near zero, random walk is allowed.
    """
    x, y, _angle = my_pose
    vec = [0.0, 0.0]
    reasons = []

    # Boundary avoidance.
    if x < ARENA_X_MIN + BOUNDARY_MARGIN:
        strength = (ARENA_X_MIN + BOUNDARY_MARGIN - x) / BOUNDARY_MARGIN
        vec[0] += 2.5 * strength
        reasons.append("left boundary")

    if x > ARENA_X_MAX - BOUNDARY_MARGIN:
        strength = (x - (ARENA_X_MAX - BOUNDARY_MARGIN)) / BOUNDARY_MARGIN
        vec[0] -= 2.5 * strength
        reasons.append("right boundary")

    if y < ARENA_Y_MIN + BOUNDARY_MARGIN:
        strength = (ARENA_Y_MIN + BOUNDARY_MARGIN - y) / BOUNDARY_MARGIN
        vec[1] += 2.5 * strength
        reasons.append("bottom boundary")

    if y > ARENA_Y_MAX - BOUNDARY_MARGIN:
        strength = (y - (ARENA_Y_MAX - BOUNDARY_MARGIN)) / BOUNDARY_MARGIN
        vec[1] -= 2.5 * strength
        reasons.append("top boundary")

    # Forbidden-zone avoidance.
    for zone in FORBIDDEN_ZONES:
        name, _x_min, _x_max, _y_min, _y_max = zone

        if inside_rect(x, y, zone, FORBIDDEN_MARGIN):
            # Strongly drive toward arena center.
            cx = (ARENA_X_MIN + ARENA_X_MAX) / 2.0
            cy = (ARENA_Y_MIN + ARENA_Y_MAX) / 2.0
            add_vector_towards(vec, x, y, cx, cy, 4.0)
            reasons.append("forbidden zone: " + name)

    # Robot collision avoidance.
    for rid, pose in latest_positions.items():
        if robot_key(rid) == robot_key(ROBOT_ID):
            continue

        ox, oy, _ = pose
        dx = x - ox
        dy = y - oy
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < 0.001:
            continue

        if dist < ROBOT_AVOID_DISTANCE:
            strength = (ROBOT_AVOID_DISTANCE - dist) / ROBOT_AVOID_DISTANCE
            vec[0] += 2.0 * strength * dx / dist
            vec[1] += 2.0 * strength * dy / dist
            reasons.append("robot {} too close".format(rid))

    return vec, reasons


def choose_random_target():
    global random_target_heading, next_random_change

    now = time.time()

    if now >= next_random_change:
        random_target_heading = random.uniform(-math.pi, math.pi)
        next_random_change = now + random.uniform(1.5, 3.5)

    return random_target_heading


def steer_to_heading(my_pose, target_heading, base_speed):
    heading = current_heading(my_pose)
    error = norm_angle(target_heading - heading)

    # Large error: turn in place first.
    if abs(error) > math.radians(70):
        turn = TURN_SPEED if error > 0 else -TURN_SPEED
        turn *= TURN_SIGN

        left = -turn
        right = turn

        return int(left), int(right), error

    # Smaller error: drive forward while steering.
    turn = int(clamp(260.0 * error, -TURN_SPEED, TURN_SPEED))
    turn *= TURN_SIGN

    left = base_speed - turn
    right = base_speed + turn

    left = int(clamp(left, -MAX_SPEED, MAX_SPEED))
    right = int(clamp(right, -MAX_SPEED, MAX_SPEED))

    return left, right, error


def choose_command(my_pose):
    if my_pose is None:
        return 0, 0, "waiting for own position", None, None

    vec, reasons = avoidance_vector(my_pose)

    mag = math.sqrt(vec[0] * vec[0] + vec[1] * vec[1])

    if mag > 0.05:
        target = math.atan2(vec[1], vec[0])
        left, right, error = steer_to_heading(my_pose, target, AVOID_SPEED)
        return left, right, "avoid: " + ", ".join(reasons), target, error

    target = choose_random_target()
    left, right, error = steer_to_heading(my_pose, target, FORWARD_SPEED)
    return left, right, "random walk", target, error


# ======================
# MQTT
# ======================

def on_connect(client, userdata, flags, rc):
    print("Connected with result code", rc)
    client.subscribe(POSITION_TOPIC)


def on_message(client, userdata, msg):
    global latest_positions, latest_raw

    try:
        data = json.loads(msg.payload.decode("utf-8", "replace"))
        latest_raw = data
        latest_positions = parse_positions(data)
    except Exception as e:
        print("Bad MQTT message:", e)


# ======================
# MAIN
# ======================

def main():
    global last_cmd, last_print_time

    print("Task 1 v2: random walk with safer boundary/forbidden-zone avoidance")
    print("Robot ID:", ROBOT_ID)
    print("Broker:", BROKER)
    print("Arena:", ARENA_X_MIN, ARENA_X_MAX, ARENA_Y_MIN, ARENA_Y_MAX)
    print("Forbidden zones:", FORBIDDEN_ZONES)

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(BROKER, PORT, 60)
    client.loop_start()

    pipuck = PiPuck(epuck_version=2)

    try:
        pipuck.epuck.set_motor_speeds(0, 0)
        time.sleep(0.5)

        while True:
            my_pose = get_my_pose()
            update_heading_calibration(my_pose)

            left, right, reason, target, error = choose_command(my_pose)
            pipuck.epuck.set_motor_speeds(left, right)
            last_cmd = (left, right)

            now = time.time()

            if now - last_print_time > 1.5:
                if my_pose is None:
                    print("cmd=({}, {}) reason={} raw={}".format(
                        left, right, reason, latest_raw
                    ))
                else:
                    heading = current_heading(my_pose)

                    print(
                        "cmd=({}, {}) reason={} pose=({:.3f}, {:.3f}, raw_angle={:.1f}) "
                        "heading={:.1f} target={} err={} offset={:.1f} ids={}".format(
                            left,
                            right,
                            reason,
                            my_pose[0],
                            my_pose[1],
                            my_pose[2],
                            rad_to_deg(heading),
                            None if target is None else round(rad_to_deg(target), 1),
                            None if error is None else round(rad_to_deg(error), 1),
                            rad_to_deg(heading_offset),
                            sorted(latest_positions.keys())
                        )
                    )

                last_print_time = now

            time.sleep(CONTROL_PERIOD)

    except KeyboardInterrupt:
        print("\nCtrl+C received")

    finally:
        print("Stopping robot")
        try:
            pipuck.epuck.set_motor_speeds(0, 0)
        except Exception as e:
            print("Could not stop motors:", e)

        client.loop_stop()
        client.disconnect()
        print("Done")


if __name__ == "__main__":
    main()