#!/usr/bin/env python3
import json
import math
import random
import time

import paho.mqtt.client as mqtt
from pipuck.pipuck import PiPuck


BROKER = "192.168.178.43"
PORT = 1883
POSITION_TOPIC = "robot_pos/all"
ROBOT_ID = "33"

# Overall arena limits for robot center
ARENA_X_MIN = 0.05
ARENA_X_MAX = 1.95
ARENA_Y_MIN = 0.05
ARENA_Y_MAX = 0.95

# Random targets are chosen safely away from the boundary
TARGET_X_MIN = 0.35
TARGET_X_MAX = 1.65
TARGET_Y_MIN = 0.20
TARGET_Y_MAX = 0.80

BOUNDARY_MARGIN = 0.16
ROBOT_AVOID_DISTANCE = 0.20

# Adjust if forbidden zone is different
FORBIDDEN_ZONES = [
    ("top_left_forbidden", 0.00, 0.25, 0.75, 1.00),
]

FORWARD_SPEED = 260
AVOID_SPEED = 210
TURN_LIMIT = 170
MAX_SPEED = 330

# Your camera angle looked like degrees in the logs.
# If it turns the wrong way, first change TURN_SIGN to -1.
ANGLE_SIGN = -1
HEADING_OFFSET_DEG = 95.0
TURN_SIGN = 1

CONTROL_PERIOD = 0.10

latest_positions = {}
latest_raw = {}

current_target = None
target_until = 0.0
last_print = 0.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def norm_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def robot_key(rid):
    return str(rid).replace("pi-puck", "").replace("robot", "").strip()


def parse_pose(record):
    if isinstance(record, dict):
        if "position" in record:
            pos = record["position"]
            x = float(pos[0])
            y = float(pos[1])
            angle = float(record.get("angle", record.get("theta", 0.0)))
            return x, y, angle

        if "x" in record and "y" in record:
            x = float(record["x"])
            y = float(record["y"])
            angle = float(record.get("angle", record.get("theta", 0.0)))
            return x, y, angle

    if isinstance(record, (list, tuple)) and len(record) >= 2:
        x = float(record[0])
        y = float(record[1])
        angle = float(record[2]) if len(record) >= 3 else 0.0
        return x, y, angle

    return None


def parse_positions(data):
    positions = {}

    if not isinstance(data, dict):
        return positions

    if "robots" in data and isinstance(data["robots"], list):
        for record in data["robots"]:
            rid = record.get("id", record.get("robot_id"))
            pose = parse_pose(record)
            if rid is not None and pose is not None:
                positions[robot_key(rid)] = pose
        return positions

    for rid, record in data.items():
        pose = parse_pose(record)
        if pose is not None:
            positions[robot_key(rid)] = pose

    return positions


def get_my_pose():
    return latest_positions.get(robot_key(ROBOT_ID))


def current_heading(pose):
    raw_angle_deg = pose[2]
    raw_angle_rad = math.radians(raw_angle_deg)
    offset = math.radians(HEADING_OFFSET_DEG)
    return norm_angle(ANGLE_SIGN * raw_angle_rad + offset)


def choose_random_target():
    global current_target, target_until

    current_target = (
        random.uniform(TARGET_X_MIN, TARGET_X_MAX),
        random.uniform(TARGET_Y_MIN, TARGET_Y_MAX),
    )

    # Longer straight movement
    target_until = time.time() + random.uniform(7.0, 12.0)


def inside_zone(x, y, zone):
    _name, x_min, x_max, y_min, y_max = zone
    return x_min <= x <= x_max and y_min <= y <= y_max


def get_safety_target(pose):
    x, y, _ = pose

    # Forbidden zone: escape toward center
    for zone in FORBIDDEN_ZONES:
        name, *_ = zone
        if inside_zone(x, y, zone):
            return (1.00, 0.50), "escape forbidden zone: " + name

    # Boundary zones: target the inside of the arena, not backwards
    if x < ARENA_X_MIN + BOUNDARY_MARGIN:
        return (0.75, clamp(y, 0.25, 0.75)), "avoid left boundary"

    if x > ARENA_X_MAX - BOUNDARY_MARGIN:
        return (1.25, clamp(y, 0.25, 0.75)), "avoid right boundary"

    if y < ARENA_Y_MIN + BOUNDARY_MARGIN:
        return (clamp(x, 0.45, 1.55), 0.45), "avoid bottom boundary"

    if y > ARENA_Y_MAX - BOUNDARY_MARGIN:
        return (clamp(x, 0.45, 1.55), 0.55), "avoid top boundary"

    # Other robot avoidance
    for rid, other in latest_positions.items():
        if robot_key(rid) == robot_key(ROBOT_ID):
            continue

        ox, oy, _ = other
        dx = x - ox
        dy = y - oy
        d = math.sqrt(dx * dx + dy * dy)

        if 0.001 < d < ROBOT_AVOID_DISTANCE:
            tx = clamp(x + 0.45 * dx / d, TARGET_X_MIN, TARGET_X_MAX)
            ty = clamp(y + 0.45 * dy / d, TARGET_Y_MIN, TARGET_Y_MAX)
            return (tx, ty), "avoid robot " + str(rid)

    return None, "clear"


def steer_to_target(pose, target, speed):
    x, y, _ = pose
    tx, ty = target

    desired_heading = math.atan2(ty - y, tx - x)
    heading = current_heading(pose)
    error = norm_angle(desired_heading - heading)

    # Always keep some forward movement.
    # This prevents it from spinning in place near the boundary.
    abs_error = abs(error)
    if abs_error > math.radians(110):
        base = 80
    elif abs_error > math.radians(60):
        base = 140
    else:
        base = speed

    turn = TURN_SIGN * int(clamp(250 * error, -TURN_LIMIT, TURN_LIMIT))

    left = int(clamp(base - turn, -MAX_SPEED, MAX_SPEED))
    right = int(clamp(base + turn, -MAX_SPEED, MAX_SPEED))

    return left, right, math.degrees(error)


def choose_command():
    global current_target

    pose = get_my_pose()

    if pose is None:
        return 0, 0, "waiting for position", None, None

    safety_target, safety_reason = get_safety_target(pose)

    if safety_target is not None:
        left, right, err = steer_to_target(pose, safety_target, AVOID_SPEED)
        return left, right, safety_reason, safety_target, err

    if current_target is None or time.time() > target_until:
        choose_random_target()

    x, y, _ = pose
    tx, ty = current_target
    dist_to_target = math.sqrt((tx - x) ** 2 + (ty - y) ** 2)

    if dist_to_target < 0.12:
        choose_random_target()

    left, right, err = steer_to_target(pose, current_target, FORWARD_SPEED)
    return left, right, "long random target", current_target, err


def on_connect(client, userdata, flags, rc):
    print("Connected with result code", rc)
    client.subscribe(POSITION_TOPIC)


def on_message(client, userdata, msg):
    global latest_positions, latest_raw

    try:
        latest_raw = json.loads(msg.payload.decode("utf-8", "replace"))
        latest_positions = parse_positions(latest_raw)
    except Exception as e:
        print("Bad MQTT:", e)


def main():
    global last_print

    print("Task 1 v3: long random straight-ish movement")
    print("Robot:", ROBOT_ID)
    print("Broker:", BROKER)
    print("If it turns the wrong way, change TURN_SIGN from 1 to -1.")

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(BROKER, PORT, 60)
    client.loop_start()

    pipuck = PiPuck(epuck_version=2)

    try:
        pipuck.epuck.set_motor_speeds(0, 0)
        time.sleep(0.5)

        choose_random_target()

        while True:
            left, right, reason, target, err = choose_command()
            pipuck.epuck.set_motor_speeds(left, right)

            now = time.time()
            if now - last_print > 1.5:
                pose = get_my_pose()

                print(
                    "cmd=({}, {}) reason={} pose={} target={} err={}".format(
                        left,
                        right,
                        reason,
                        pose,
                        target,
                        None if err is None else round(err, 1),
                    )
                )

                last_print = now

            time.sleep(CONTROL_PERIOD)

    except KeyboardInterrupt:
        print("\nCtrl+C received")

    finally:
        print("Stopping robot")
        pipuck.epuck.set_motor_speeds(0, 0)
        client.loop_stop()
        client.disconnect()
        print("Done")


if __name__ == "__main__":
    main()