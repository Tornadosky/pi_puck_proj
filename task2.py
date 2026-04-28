#!/usr/bin/env python3
import json
import math
import random
import time

import paho.mqtt.client as mqtt
from pipuck.pipuck import PiPuck


# ============================================================
# TASK 2 CONFIG
# ============================================================

BROKER = "192.168.178.43"
PORT = 1883

POSITION_TOPIC = "robot_pos/all"

ROBOT_ID = "33"
OWN_TOPIC = "robot/{}".format(ROBOT_ID)

COMMUNICATION_RADIUS = 0.30      # 50 cm, as requested in Task 2
PROXIMITY_CHECK_INTERVAL = 2.0    # check nearby robots every few seconds
HELLO_COOLDOWN_SECONDS = 8.0      # avoid spamming the same robot

LOG_FILE = "task2_messages.log"


# ============================================================
# TASK 1 MOVEMENT CONFIG
# ============================================================

# Overall arena limits for robot center.
ARENA_X_MIN = 0.05
ARENA_X_MAX = 1.95
ARENA_Y_MIN = 0.05
ARENA_Y_MAX = 0.95

# Random targets are chosen away from the boundary.
TARGET_X_MIN = 0.35
TARGET_X_MAX = 1.65
TARGET_Y_MIN = 0.20
TARGET_Y_MAX = 0.80

BOUNDARY_MARGIN = 0.16

# Close collision avoidance. This is smaller than communication radius.
ROBOT_AVOID_DISTANCE = 0.15

# Change/delete this if your forbidden zone is different.
FORBIDDEN_ZONES = [
    ("top_left_forbidden", 0.00, 0.25, 0.75, 1.00),
]

FORWARD_SPEED = 340
AVOID_SPEED = 210
TURN_LIMIT = 170
MAX_SPEED = 330

# Based on your earlier logs, the tracking angle looked like degrees.
# If the robot turns the wrong way, change TURN_SIGN to -1.
ANGLE_SIGN = -1
HEADING_OFFSET_DEG = 95.0
TURN_SIGN = 1

CONTROL_PERIOD = 0.10


# ============================================================
# GLOBAL STATE
# ============================================================

latest_positions = {}
latest_raw = {}

current_target = None
target_until = 0.0

last_hello_sent = {}
next_proximity_check = 0.0

blink_until = 0.0
led_state = None
led_available = True

last_print = 0.0
sent_count = 0
received_count = 0


# ============================================================
# HELPERS
# ============================================================

def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def norm_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def robot_key(rid):
    return str(rid).replace("pi-puck", "").replace("robot", "").strip()


def raw_angle_to_rad(raw_angle):
    raw = float(raw_angle)

    # If it is already radians, keep it. Otherwise treat it as degrees.
    if abs(raw) <= 2.0 * math.pi + 0.1:
        return raw

    return math.radians(raw)


def parse_pose(record):
    """
    Supports likely MQTT formats:
      {"position": [x, y], "angle": a}
      {"x": x, "y": y, "angle": a}
      [x, y, angle]
    """
    try:
        if isinstance(record, dict):
            if "position" in record:
                pos = record["position"]

                if isinstance(pos, dict):
                    x = float(pos["x"])
                    y = float(pos["y"])
                else:
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

    except Exception:
        return None

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
            pose = parse_pose(record)

            if rid is not None and pose is not None:
                positions[robot_key(rid)] = pose

        return positions

    # Format: {"33": {"position": [...], "angle": ...}, "34": ...}
    for rid, record in data.items():
        pose = parse_pose(record)

        if pose is not None:
            positions[robot_key(rid)] = pose

    return positions


def get_my_pose():
    return latest_positions.get(robot_key(ROBOT_ID))


def distance(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return math.sqrt(dx * dx + dy * dy)


def current_heading(pose):
    raw_angle_rad = raw_angle_to_rad(pose[2])
    offset = math.radians(HEADING_OFFSET_DEG)

    return norm_angle(ANGLE_SIGN * raw_angle_rad + offset)


def log_line(text):
    stamp = time.strftime("%H:%M:%S")
    line = "[{}] {}".format(stamp, text)

    print(line)

    try:
        with open(LOG_FILE, "a") as f:
            f.write(line + "\n")
    except Exception as e:
        print("Could not write log:", e)


# ============================================================
# TASK 1 RANDOM WALK / AVOIDANCE
# ============================================================

def choose_random_target():
    global current_target, target_until

    current_target = (
        random.uniform(TARGET_X_MIN, TARGET_X_MAX),
        random.uniform(TARGET_Y_MIN, TARGET_Y_MAX),
    )

    # Longer straight-ish movements.
    target_until = time.time() + random.uniform(7.0, 12.0)


def inside_zone(x, y, zone):
    _name, x_min, x_max, y_min, y_max = zone
    return x_min <= x <= x_max and y_min <= y <= y_max


def get_safety_target(pose):
    x, y, _angle = pose

    # Forbidden zone: escape toward center.
    for zone in FORBIDDEN_ZONES:
        name, _x_min, _x_max, _y_min, _y_max = zone

        if inside_zone(x, y, zone):
            return (1.00, 0.50), "escape forbidden zone: " + name

    # Boundary zones: target the inside of the arena.
    if x < ARENA_X_MIN + BOUNDARY_MARGIN:
        return (0.75, clamp(y, 0.25, 0.75)), "avoid left boundary"

    if x > ARENA_X_MAX - BOUNDARY_MARGIN:
        return (1.25, clamp(y, 0.25, 0.75)), "avoid right boundary"

    if y < ARENA_Y_MIN + BOUNDARY_MARGIN:
        return (clamp(x, 0.45, 1.55), 0.45), "avoid bottom boundary"

    if y > ARENA_Y_MAX - BOUNDARY_MARGIN:
        return (clamp(x, 0.45, 1.55), 0.55), "avoid top boundary"

    # Collision avoidance with other robots.
    for rid, other in latest_positions.items():
        if robot_key(rid) == robot_key(ROBOT_ID):
            continue

        d = distance(pose, other)

        if 0.001 < d < ROBOT_AVOID_DISTANCE:
            # Move away from the other robot.
            dx = pose[0] - other[0]
            dy = pose[1] - other[1]

            tx = clamp(pose[0] + 0.45 * dx / d, TARGET_X_MIN, TARGET_X_MAX)
            ty = clamp(pose[1] + 0.45 * dy / d, TARGET_Y_MIN, TARGET_Y_MAX)

            return (tx, ty), "avoid robot {} at {:.2f}m".format(rid, d)

    return None, "clear"


def steer_to_target(pose, target, speed):
    x, y, _angle = pose
    tx, ty = target

    desired_heading = math.atan2(ty - y, tx - x)
    heading = current_heading(pose)
    error = norm_angle(desired_heading - heading)

    # Keep some forward movement even when turning, so it does not spin in place.
    abs_error = abs(error)

    if abs_error > math.radians(110):
        base = 80
    elif abs_error > math.radians(60):
        base = 140
    else:
        base = speed

    turn = TURN_SIGN * int(clamp(250.0 * error, -TURN_LIMIT, TURN_LIMIT))

    left = int(clamp(base - turn, -MAX_SPEED, MAX_SPEED))
    right = int(clamp(base + turn, -MAX_SPEED, MAX_SPEED))

    return left, right, math.degrees(error)


def choose_movement_command():
    global current_target

    pose = get_my_pose()

    if pose is None:
        return 0, 0, "waiting for own position", None, None

    safety_target, safety_reason = get_safety_target(pose)

    if safety_target is not None:
        left, right, err = steer_to_target(pose, safety_target, AVOID_SPEED)
        return left, right, safety_reason, safety_target, err

    if current_target is None or time.time() > target_until:
        choose_random_target()

    tx, ty = current_target
    dist_to_target = math.sqrt((tx - pose[0]) ** 2 + (ty - pose[1]) ** 2)

    if dist_to_target < 0.12:
        choose_random_target()

    left, right, err = steer_to_target(pose, current_target, FORWARD_SPEED)
    return left, right, "long random target", current_target, err


# ============================================================
# TASK 2 COMMUNICATION
# ============================================================

def send_hello_to_close_robots(client):
    """
    Every few seconds, look at latest robot_pos/all positions.
    If another robot is within COMMUNICATION_RADIUS, send Hello to robot/<id>.
    """
    global next_proximity_check, sent_count

    now = time.time()

    if now < next_proximity_check:
        return

    next_proximity_check = now + PROXIMITY_CHECK_INTERVAL

    my_pose = get_my_pose()

    if my_pose is None:
        return

    for rid, other_pose in latest_positions.items():
        rid = robot_key(rid)

        if rid == robot_key(ROBOT_ID):
            continue

        d = distance(my_pose, other_pose)

        if d <= COMMUNICATION_RADIUS:
            last_sent = last_hello_sent.get(rid, 0.0)

            if now - last_sent < HELLO_COOLDOWN_SECONDS:
                continue

            topic = "robot/{}".format(rid)

            payload = {
                "type": "hello",
                "from": ROBOT_ID,
                "to": rid,
                "text": "Hello from robot {}".format(ROBOT_ID),
                "distance_m": round(d, 3),
                "time": now,
            }

            client.publish(topic, json.dumps(payload))
            last_hello_sent[rid] = now
            sent_count += 1

            log_line("TX to {} on {}: {}".format(rid, topic, payload))


def handle_robot_message(payload_text):
    global blink_until, received_count

    try:
        message = json.loads(payload_text)
    except Exception:
        message = {
            "type": "raw",
            "text": payload_text,
        }

    sender = message.get("from", "unknown")
    text = message.get("text", str(message))

    received_count += 1
    blink_until = time.time() + 2.0

    log_line("RX from {} on {}: {}".format(sender, OWN_TOPIC, text))


def update_leds(pipuck):
    """
    Blink blue when a message has been received.
    If LEDs fail for any reason, the program continues and just logs messages.
    """
    global led_state, led_available

    if not led_available:
        return

    now = time.time()

    if now < blink_until:
        # Blink: blue/off/blue/off...
        desired = "blue" if int(now * 6) % 2 == 0 else "off"
    else:
        desired = "off"

    if desired == led_state:
        return

    try:
        pipuck.set_leds_colour(desired)
        led_state = desired
    except Exception as e:
        led_available = False
        print("LED control failed, continuing without LEDs:", e)


# ============================================================
# MQTT CALLBACKS
# ============================================================

def on_connect(client, userdata, flags, rc):
    print("Connected with result code", rc)

    client.subscribe(POSITION_TOPIC)
    client.subscribe(OWN_TOPIC)

    print("Subscribed to:", POSITION_TOPIC)
    print("Subscribed to:", OWN_TOPIC)


def on_message(client, userdata, msg):
    global latest_positions, latest_raw

    payload_text = msg.payload.decode("utf-8", "replace")

    if msg.topic == POSITION_TOPIC:
        try:
            latest_raw = json.loads(payload_text)
            latest_positions = parse_positions(latest_raw)
        except Exception as e:
            print("Bad position MQTT message:", e)

    elif msg.topic == OWN_TOPIC:
        handle_robot_message(payload_text)


# ============================================================
# MAIN
# ============================================================

def main():
    global last_print

    print("Task 2: random walk + talk to close robots")
    print("Robot ID:", ROBOT_ID)
    print("Own topic:", OWN_TOPIC)
    print("Position topic:", POSITION_TOPIC)
    print("Communication radius:", COMMUNICATION_RADIUS, "m")
    print("Broker:", BROKER, PORT)
    print("If movement turns the wrong way, change TURN_SIGN from 1 to -1.")

    client = mqtt.Client(client_id="task2_robot_{}".format(ROBOT_ID))
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
            left, right, reason, target, err = choose_movement_command()
            pipuck.epuck.set_motor_speeds(left, right)

            send_hello_to_close_robots(client)
            update_leds(pipuck)

            now = time.time()

            if now - last_print > 1.5:
                pose = get_my_pose()

                print(
                    "cmd=({}, {}) reason={} pose={} target={} err={} sent={} received={} ids={}".format(
                        left,
                        right,
                        reason,
                        pose,
                        target,
                        None if err is None else round(err, 1),
                        sent_count,
                        received_count,
                        sorted(latest_positions.keys()),
                    )
                )

                last_print = now

            time.sleep(CONTROL_PERIOD)

    except KeyboardInterrupt:
        print("\nCtrl+C received")

    finally:
        print("Stopping robot")
        try:
            pipuck.epuck.set_motor_speeds(0, 0)
        except Exception as e:
            print("Could not stop motors:", e)

        try:
            pipuck.set_leds_colour("off")
        except Exception:
            pass

        client.loop_stop()
        client.disconnect()

        print("Done")


if __name__ == "__main__":
    main()
