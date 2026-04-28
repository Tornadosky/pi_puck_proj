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

COMMUNICATION_RADIUS = 0.50
PROXIMITY_CHECK_INTERVAL = 2.0
HELLO_COOLDOWN_SECONDS = 8.0

LOG_FILE = "task2_messages.log"
BLINK_DURATION_SECONDS = 7.0


# ============================================================
# ARENA / DANGER CONFIG
# ============================================================

ARENA_X_MIN = 0.05
ARENA_X_MAX = 1.95
ARENA_Y_MIN = 0.05
ARENA_Y_MAX = 0.95

# Only avoid when ACTUALLY near the boundary.
# No predictive boundary logic.
BOUNDARY_TRIGGER = 0.14
BOUNDARY_CRITICAL = 0.08

FORBIDDEN_ZONES = [
    # name, x_min, x_max, y_min, y_max
    ("top_left_forbidden", 0.00, 0.25, 0.75, 1.00),
]

# Small buffer around forbidden zone.
FORBIDDEN_BUFFER = 0.06

# Robot collision avoidance threshold.
# Communication radius can be 50 cm, but collision avoidance should be much smaller.
ROBOT_AVOID_DISTANCE = 0.18
ROBOT_CRITICAL_DISTANCE = 0.13

SAFE_CENTER_X = 1.00
SAFE_CENTER_Y = 0.50


# ============================================================
# MOVEMENT CONFIG
# ============================================================

# Normal behavior: straight.
CRUISE_SPEED = 390

# Avoidance behavior: turn away, then straight escape.
TURN_SPEED = 180
MIN_TURN_TIME = 0.35
MAX_TURN_TIME = 0.95

ESCAPE_SPEED = 330
ESCAPE_TIME = 2.1

# Only backup for very close robot collisions, not for boundaries.
BACK_SPEED = 180
BACK_TIME = 0.25

MAX_SPEED = 520
CONTROL_PERIOD = 0.10

# If it turns the wrong direction during avoidance, change this to -1.
TURN_SIGN = 1

# Fallback camera-heading conversion.
# The actual turn decision mostly uses recent movement direction, not the marker angle.
ANGLE_SIGN = -1
HEADING_OFFSET_DEG = 95.0


# ============================================================
# GLOBAL STATE
# ============================================================

latest_positions = {}
latest_raw = {}
last_position_time = 0.0

motion_history = []

phase = "straight"
phase_until = 0.0
turn_dir = 1
avoid_reason = ""
avoid_started = 0.0
avoid_vector = (0.0, 0.0)

last_hello_sent = {}
next_proximity_check = 0.0

blink_until = 0.0
led_state = None
led_available = True

last_print = 0.0
sent_count = 0
received_count = 0


# ============================================================
# BASIC HELPERS
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

    if abs(raw) <= 2.0 * math.pi + 0.1:
        return raw

    return math.radians(raw)


def parse_pose(record):
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

    if "robots" in data and isinstance(data["robots"], list):
        for record in data["robots"]:
            if not isinstance(record, dict):
                continue

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


def distance(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return math.sqrt(dx * dx + dy * dy)


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
# HEADING / MOTION ESTIMATE
# ============================================================

def camera_heading(pose):
    raw_angle_rad = raw_angle_to_rad(pose[2])
    offset = math.radians(HEADING_OFFSET_DEG)
    return norm_angle(ANGLE_SIGN * raw_angle_rad + offset)


def update_motion_history(pose):
    global motion_history

    if pose is None:
        return

    now = time.time()
    x, y, _angle = pose

    motion_history.append((now, x, y))
    motion_history = [p for p in motion_history if now - p[0] <= 5.0]


def recent_motion_heading(pose):
    """
    Prefer actual recent movement direction.
    This is more reliable for deciding which way to turn away from danger.
    """
    if pose is None or len(motion_history) < 2:
        return camera_heading(pose) if pose is not None else 0.0

    now = time.time()
    x, y, _angle = pose

    old_sample = None

    for sample in motion_history:
        t, sx, sy = sample
        if now - t >= 0.8:
            old_sample = sample
            break

    if old_sample is None:
        old_sample = motion_history[0]

    _t, old_x, old_y = old_sample

    dx = x - old_x
    dy = y - old_y
    d = math.sqrt(dx * dx + dy * dy)

    if d > 0.025:
        return math.atan2(dy, dx)

    return camera_heading(pose)


# ============================================================
# DANGER DETECTION
# ============================================================

def inside_rect(x, y, zone, buffer_amount):
    _name, x_min, x_max, y_min, y_max = zone

    return (
        x_min - buffer_amount <= x <= x_max + buffer_amount and
        y_min - buffer_amount <= y <= y_max + buffer_amount
    )


def add_vector(vec, dx, dy, weight):
    length = math.sqrt(dx * dx + dy * dy)

    if length < 0.0001:
        return

    vec[0] += weight * dx / length
    vec[1] += weight * dy / length


def detect_danger(pose):
    """
    Returns None if there is no real danger.

    Danger only triggers when:
      - close to a boundary,
      - close to/inside forbidden zone,
      - very close to another robot.
    """
    x, y, _angle = pose

    vec = [0.0, 0.0]
    reasons = []
    critical = False
    robot_critical = False

    # ---------- Boundary ----------
    if x < ARENA_X_MIN + BOUNDARY_TRIGGER:
        vec[0] += 1.8
        reasons.append("left boundary")
        if x < ARENA_X_MIN + BOUNDARY_CRITICAL:
            critical = True

    if x > ARENA_X_MAX - BOUNDARY_TRIGGER:
        vec[0] -= 1.8
        reasons.append("right boundary")
        if x > ARENA_X_MAX - BOUNDARY_CRITICAL:
            critical = True

    if y < ARENA_Y_MIN + BOUNDARY_TRIGGER:
        vec[1] += 1.8
        reasons.append("bottom boundary")
        if y < ARENA_Y_MIN + BOUNDARY_CRITICAL:
            critical = True

    if y > ARENA_Y_MAX - BOUNDARY_TRIGGER:
        vec[1] -= 1.8
        reasons.append("top boundary")
        if y > ARENA_Y_MAX - BOUNDARY_CRITICAL:
            critical = True

    # ---------- Forbidden zone ----------
    for zone in FORBIDDEN_ZONES:
        name = zone[0]

        if inside_rect(x, y, zone, 0.0):
            add_vector(vec, SAFE_CENTER_X - x, SAFE_CENTER_Y - y, 3.0)
            reasons.append("inside forbidden zone: " + name)
            critical = True

        elif inside_rect(x, y, zone, FORBIDDEN_BUFFER):
            add_vector(vec, SAFE_CENTER_X - x, SAFE_CENTER_Y - y, 2.0)
            reasons.append("near forbidden zone: " + name)

    # ---------- Robot collision ----------
    closest_id = None
    closest_pose = None
    closest_d = None

    for rid, other in latest_positions.items():
        rid = robot_key(rid)

        if rid == robot_key(ROBOT_ID):
            continue

        d = distance(pose, other)

        if closest_d is None or d < closest_d:
            closest_id = rid
            closest_pose = other
            closest_d = d

    if closest_pose is not None and closest_d is not None:
        if 0.001 < closest_d < ROBOT_AVOID_DISTANCE:
            dx = x - closest_pose[0]
            dy = y - closest_pose[1]

            add_vector(vec, dx, dy, 2.4)
            reasons.append("robot {} at {:.2f}m".format(closest_id, closest_d))

            if closest_d < ROBOT_CRITICAL_DISTANCE:
                critical = True
                robot_critical = True

    mag = math.sqrt(vec[0] * vec[0] + vec[1] * vec[1])

    if mag < 0.001:
        return None

    safe_heading = math.atan2(vec[1], vec[0])
    current = recent_motion_heading(pose)
    error = norm_angle(safe_heading - current)

    # If the target direction is almost straight ahead, skip turning and just escape.
    needs_turn = abs(error) > math.radians(30)

    turn_time = clamp(abs(error) / math.pi * 1.1, MIN_TURN_TIME, MAX_TURN_TIME)

    return {
        "reason": ", ".join(reasons),
        "critical": critical,
        "robot_critical": robot_critical,
        "safe_heading": safe_heading,
        "heading_error": error,
        "turn_dir": 1 if error > 0 else -1,
        "turn_time": turn_time,
        "needs_turn": needs_turn,
        "vector": (vec[0], vec[1]),
    }


# ============================================================
# MOVEMENT STATE MACHINE
# ============================================================

def start_avoidance(danger):
    global phase, phase_until, turn_dir, avoid_reason, avoid_started, avoid_vector

    now = time.time()

    avoid_reason = danger["reason"]
    avoid_started = now
    avoid_vector = danger["vector"]
    turn_dir = danger["turn_dir"]

    # Only back up for very close robot collision.
    # Do NOT back up for boundaries, because backing can push it outside.
    if danger["robot_critical"]:
        phase = "back"
        phase_until = now + BACK_TIME
        return

    if danger["needs_turn"]:
        phase = "turn"
        phase_until = now + danger["turn_time"]
        return

    phase = "escape"
    phase_until = now + ESCAPE_TIME


def avoidance_command(pose):
    global phase, phase_until

    now = time.time()

    if phase == "straight":
        return None

    # Safety limit: do not get stuck in avoidance forever.
    if now - avoid_started > 4.5:
        phase = "straight"
        return None

    if phase == "back":
        if now >= phase_until:
            phase = "turn"
            phase_until = now + 0.45
        else:
            return -BACK_SPEED, -BACK_SPEED, "back away: " + avoid_reason

    if phase == "turn":
        if now >= phase_until:
            phase = "escape"
            phase_until = now + ESCAPE_TIME
        else:
            signed = TURN_SIGN * turn_dir * TURN_SPEED
            return -signed, signed, "turn away: " + avoid_reason

    if phase == "escape":
        if now >= phase_until:
            phase = "straight"
            return None

        return ESCAPE_SPEED, ESCAPE_SPEED, "escape straight: " + avoid_reason

    phase = "straight"
    return None


def choose_movement_command():
    pose = get_my_pose()

    if pose is None:
        return 0, 0, "waiting for own position"

    update_motion_history(pose)

    cmd = avoidance_command(pose)
    if cmd is not None:
        return cmd

    danger = detect_danger(pose)

    if danger is not None:
        start_avoidance(danger)

        cmd = avoidance_command(pose)
        if cmd is not None:
            return cmd

    # Main behavior: just go straight.
    return CRUISE_SPEED, CRUISE_SPEED, "straight"


# ============================================================
# TASK 2 COMMUNICATION
# ============================================================

def send_hello_to_close_robots(client):
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
    blink_until = time.time() + BLINK_DURATION_SECONDS

    log_line("RX from {} on {}: {}".format(sender, OWN_TOPIC, text))


def update_leds(pipuck):
    global led_state, led_available

    if not led_available:
        return

    now = time.time()

    if now < blink_until:
        desired = "blue" if int(now * 5) % 2 == 0 else "off"
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
    global latest_positions, latest_raw, last_position_time

    payload_text = msg.payload.decode("utf-8", "replace")

    if msg.topic == POSITION_TOPIC:
        try:
            latest_raw = json.loads(payload_text)
            latest_positions = parse_positions(latest_raw)
            last_position_time = time.time()
        except Exception as e:
            print("Bad position MQTT message:", e)

    elif msg.topic == OWN_TOPIC:
        handle_robot_message(payload_text)


# ============================================================
# MAIN
# ============================================================

def main():
    global last_print

    print("Task 2 straight-reactive")
    print("Robot ID:", ROBOT_ID)
    print("Own topic:", OWN_TOPIC)
    print("Position topic:", POSITION_TOPIC)
    print("Broker:", BROKER, PORT)
    print("Behavior: straight unless actually close to boundary / forbidden zone / robot")
    print("Boundary trigger:", BOUNDARY_TRIGGER)
    print("Forbidden buffer:", FORBIDDEN_BUFFER)
    print("Robot avoid distance:", ROBOT_AVOID_DISTANCE)
    print("If it turns the wrong way during avoidance, set TURN_SIGN = -1.")

    client = mqtt.Client(client_id="task2_straight_reactive_robot_{}".format(ROBOT_ID))
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(BROKER, PORT, 60)
    client.loop_start()

    pipuck = PiPuck(epuck_version=2)

    try:
        pipuck.epuck.set_motor_speeds(0, 0)
        time.sleep(0.5)

        while True:
            # If tracking is lost, stop instead of blindly driving.
            if last_position_time > 0 and time.time() - last_position_time > 2.0:
                left, right, reason = 0, 0, "position stale, stopping"
            else:
                left, right, reason = choose_movement_command()

            pipuck.epuck.set_motor_speeds(left, right)

            send_hello_to_close_robots(client)
            update_leds(pipuck)

            now = time.time()

            if now - last_print > 1.5:
                pose = get_my_pose()

                print(
                    "cmd=({}, {}) phase={} reason={} pose={} sent={} received={} ids={}".format(
                        left,
                        right,
                        phase,
                        reason,
                        pose,
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