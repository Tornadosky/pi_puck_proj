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

# Longer LED blinking after receiving "Hello"
BLINK_DURATION_SECONDS = 7.0


# ============================================================
# ARENA / SAFETY CONFIG
# ============================================================

ARENA_X_MIN = 0.05
ARENA_X_MAX = 1.95
ARENA_Y_MIN = 0.05
ARENA_Y_MAX = 0.95

# Normal random movement can use most of the arena,
# but targets are kept away from the very edge.
TARGET_X_MIN = 0.20
TARGET_X_MAX = 1.80
TARGET_Y_MIN = 0.14
TARGET_Y_MAX = 0.86

# Earlier boundary avoidance.
# Warning = start preparing to escape.
# Emergency = stop confident movement and pivot/escape carefully.
BOUNDARY_WARN = 0.28
BOUNDARY_EMERGENCY = 0.13

# Forbidden zone avoidance.
# The robot should avoid the inflated forbidden zone, not only the real one.
FORBIDDEN_ZONES = [
    # name, x_min, x_max, y_min, y_max
    ("top_left_forbidden", 0.00, 0.25, 0.75, 1.00),
]

FORBIDDEN_WARN_BUFFER = 0.18
FORBIDDEN_EMERGENCY_BUFFER = 0.04

# More active robot collision avoidance.
ROBOT_WARN_DISTANCE = 0.18
ROBOT_EMERGENCY_DISTANCE = 0.23

# Look ahead in the current movement direction.
# This is what prevents entering forbidden zones in the first place.
LOOKAHEAD_DISTANCES = [0.12, 0.22, 0.34, 0.46]


# ============================================================
# MOVEMENT CONFIG
# ============================================================

# Keep the confident movement you liked.
CRUISE_SPEED = 410
MAX_SPEED = 520

# Normal movement is mostly straight.
STRAIGHT_MIN_TIME = 7.0
STRAIGHT_MAX_TIME = 14.0
CURVE_MIN_TIME = 2.0
CURVE_MAX_TIME = 4.5

# Safety movement is slower and more controlled.
SAFETY_SPEED = 300
EMERGENCY_ESCAPE_SPEED = 260
SAFETY_MIN_FORWARD_SPEED = 90

SAFETY_TURN_GAIN = 175.0
SAFETY_TURN_LIMIT = 145

# Pivot speed is used only when danger is high or the required correction is large.
PIVOT_SPEED = 185
WARNING_PIVOT_SPEED = 150

CONTROL_PERIOD = 0.10

# Tracking angle from your logs looked like degrees.
# If avoidance turns the wrong way, change TURN_SIGN to -1.
ANGLE_SIGN = -1
HEADING_OFFSET_DEG = 95.0
TURN_SIGN = 1


# ============================================================
# GLOBAL STATE
# ============================================================

latest_positions = {}
latest_raw = {}

drive_mode = "straight"
drive_mode_until = 0.0

motion_history = []
motion_heading = None
motion_heading_time = 0.0

unstick_until = 0.0
unstick_dir = 1

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


def add_reason(reasons, reason):
    if reason not in reasons:
        reasons.append(reason)


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


def current_heading(pose):
    """
    Prefer the actual movement direction from position updates.
    If that is stale, fall back to camera marker angle.
    """
    if motion_heading is not None and time.time() - motion_heading_time < 2.0:
        return motion_heading

    return camera_heading(pose)


def update_motion_estimate(pose):
    global motion_history, motion_heading, motion_heading_time

    if pose is None:
        return

    now = time.time()
    x, y, _angle = pose

    motion_history.append((now, x, y))

    # Keep about 6 seconds of history.
    motion_history = [p for p in motion_history if now - p[0] <= 6.0]

    if len(motion_history) < 2:
        return

    old_t, old_x, old_y = motion_history[-2]
    dx = x - old_x
    dy = y - old_y
    d = math.sqrt(dx * dx + dy * dy)

    if d > 0.006:
        new_heading = math.atan2(dy, dx)

        if motion_heading is None:
            motion_heading = new_heading
        else:
            sx = 0.75 * math.cos(motion_heading) + 0.25 * math.cos(new_heading)
            sy = 0.75 * math.sin(motion_heading) + 0.25 * math.sin(new_heading)
            motion_heading = math.atan2(sy, sx)

        motion_heading_time = now


def moved_recently_enough():
    if len(motion_history) < 2:
        return True

    now = time.time()
    old_samples = [p for p in motion_history if now - p[0] >= 3.5]

    if not old_samples:
        return True

    old_t, old_x, old_y = old_samples[0]
    new_t, new_x, new_y = motion_history[-1]

    moved = math.sqrt((new_x - old_x) ** 2 + (new_y - old_y) ** 2)

    return moved >= 0.05


# ============================================================
# FORBIDDEN ZONE / SAFETY HELPERS
# ============================================================

def point_inside_zone(x, y, zone, buffer_amount):
    _name, x_min, x_max, y_min, y_max = zone

    return (
        x_min - buffer_amount <= x <= x_max + buffer_amount and
        y_min - buffer_amount <= y <= y_max + buffer_amount
    )


def point_inside_any_forbidden(x, y, buffer_amount):
    for zone in FORBIDDEN_ZONES:
        if point_inside_zone(x, y, zone, buffer_amount):
            return True

    return False


def zone_center(zone):
    _name, x_min, x_max, y_min, y_max = zone
    return (0.5 * (x_min + x_max), 0.5 * (y_min + y_max))


def add_vector(vec, dx, dy, weight):
    length = math.sqrt(dx * dx + dy * dy)

    if length < 0.0001:
        return

    vec[0] += weight * dx / length
    vec[1] += weight * dy / length


def add_forbidden_repulsion(vec, px, py, zone, weight):
    cx, cy = zone_center(zone)

    dx = px - cx
    dy = py - cy

    # If exactly in the middle, escape toward arena center.
    if abs(dx) < 0.001 and abs(dy) < 0.001:
        dx = 1.00 - cx
        dy = 0.50 - cy

    add_vector(vec, dx, dy, weight)


def apply_wall_repulsion(vec, px, py, weight, reasons, prefix):
    triggered = False

    if px < ARENA_X_MIN + BOUNDARY_WARN:
        strength = (ARENA_X_MIN + BOUNDARY_WARN - px) / BOUNDARY_WARN
        vec[0] += weight * strength
        triggered = True

    if px > ARENA_X_MAX - BOUNDARY_WARN:
        strength = (px - (ARENA_X_MAX - BOUNDARY_WARN)) / BOUNDARY_WARN
        vec[0] -= weight * strength
        triggered = True

    if py < ARENA_Y_MIN + BOUNDARY_WARN:
        strength = (ARENA_Y_MIN + BOUNDARY_WARN - py) / BOUNDARY_WARN
        vec[1] += weight * strength
        triggered = True

    if py > ARENA_Y_MAX - BOUNDARY_WARN:
        strength = (py - (ARENA_Y_MAX - BOUNDARY_WARN)) / BOUNDARY_WARN
        vec[1] -= weight * strength
        triggered = True

    if triggered:
        add_reason(reasons, prefix + "boundary")


def ensure_safe_target(tx, ty):
    tx = clamp(tx, TARGET_X_MIN, TARGET_X_MAX)
    ty = clamp(ty, TARGET_Y_MIN, TARGET_Y_MAX)

    # If target is still too close to forbidden zone, use arena center.
    if point_inside_any_forbidden(tx, ty, FORBIDDEN_WARN_BUFFER):
        tx, ty = 1.00, 0.50

    return tx, ty


def compute_safety_target(pose):
    """
    Returns:
      target, reason, severity

    severity:
      0 = no danger
      1 = warning / predicted danger
      2 = emergency / already very close or inside
    """
    x, y, _angle = pose

    vec = [0.0, 0.0]
    reasons = []
    severity = 0

    heading = current_heading(pose)

    # Current boundary danger.
    apply_wall_repulsion(vec, x, y, 2.5, reasons, "near ")

    if (
        x < ARENA_X_MIN + BOUNDARY_EMERGENCY or
        x > ARENA_X_MAX - BOUNDARY_EMERGENCY or
        y < ARENA_Y_MIN + BOUNDARY_EMERGENCY or
        y > ARENA_Y_MAX - BOUNDARY_EMERGENCY
    ):
        severity = max(severity, 2)
        add_reason(reasons, "emergency boundary")
    elif reasons:
        severity = max(severity, 1)

    # Predict future boundary danger.
    for d in LOOKAHEAD_DISTANCES:
        px = x + d * math.cos(heading)
        py = y + d * math.sin(heading)

        before = len(reasons)
        apply_wall_repulsion(vec, px, py, 1.6, reasons, "predicted ")

        if len(reasons) > before:
            severity = max(severity, 1)

    # Current and predicted forbidden-zone danger.
    for zone in FORBIDDEN_ZONES:
        name = zone[0]

        if point_inside_zone(x, y, zone, FORBIDDEN_EMERGENCY_BUFFER):
            add_forbidden_repulsion(vec, x, y, zone, 6.0)
            add_reason(reasons, "emergency forbidden zone: " + name)
            severity = max(severity, 2)

        elif point_inside_zone(x, y, zone, FORBIDDEN_WARN_BUFFER):
            add_forbidden_repulsion(vec, x, y, zone, 4.0)
            add_reason(reasons, "near forbidden zone: " + name)
            severity = max(severity, 1)

        for d in LOOKAHEAD_DISTANCES:
            px = x + d * math.cos(heading)
            py = y + d * math.sin(heading)

            if point_inside_zone(px, py, zone, FORBIDDEN_WARN_BUFFER):
                add_forbidden_repulsion(vec, px, py, zone, 3.5)
                add_reason(reasons, "predicted forbidden zone: " + name)
                severity = max(severity, 1)

    # Robot collision avoidance.
    closest_id = None
    closest_pose = None
    closest_d = None

    for rid, other in latest_positions.items():
        if robot_key(rid) == robot_key(ROBOT_ID):
            continue

        d = distance(pose, other)

        if closest_d is None or d < closest_d:
            closest_id = rid
            closest_pose = other
            closest_d = d

    if closest_pose is not None and closest_d is not None:
        if 0.001 < closest_d < ROBOT_WARN_DISTANCE:
            ox, oy, _ = closest_pose
            dx = x - ox
            dy = y - oy

            if closest_d < ROBOT_EMERGENCY_DISTANCE:
                weight = 5.0
                severity = max(severity, 2)
                add_reason(reasons, "emergency robot {} at {:.2f}m".format(closest_id, closest_d))
            else:
                weight = 3.2
                severity = max(severity, 1)
                add_reason(reasons, "near robot {} at {:.2f}m".format(closest_id, closest_d))

            add_vector(vec, dx, dy, weight)

    mag = math.sqrt(vec[0] * vec[0] + vec[1] * vec[1])

    if severity == 0 or mag < 0.001:
        return None, "clear", 0

    # Choose a target in the safe direction.
    # Emergency target is a bit farther so it commits to escaping.
    escape_distance = 0.65 if severity >= 2 else 0.48

    tx = x + escape_distance * vec[0] / mag
    ty = y + escape_distance * vec[1] / mag

    tx, ty = ensure_safe_target(tx, ty)

    return (tx, ty), ", ".join(reasons), severity


# ============================================================
# MOVEMENT LOGIC
# ============================================================

def choose_new_drive_mode():
    global drive_mode, drive_mode_until

    r = random.random()
    now = time.time()

    if r < 0.82:
        drive_mode = "straight"
        drive_mode_until = now + random.uniform(STRAIGHT_MIN_TIME, STRAIGHT_MAX_TIME)

    elif r < 0.91:
        drive_mode = "curve_left"
        drive_mode_until = now + random.uniform(CURVE_MIN_TIME, CURVE_MAX_TIME)

    else:
        drive_mode = "curve_right"
        drive_mode_until = now + random.uniform(CURVE_MIN_TIME, CURVE_MAX_TIME)


def free_drive_command():
    if time.time() >= drive_mode_until:
        choose_new_drive_mode()

    if drive_mode == "straight":
        return CRUISE_SPEED, CRUISE_SPEED, "confident straight"

    if drive_mode == "curve_left":
        return CRUISE_SPEED - 75, CRUISE_SPEED + 25, "confident gentle curve left"

    if drive_mode == "curve_right":
        return CRUISE_SPEED + 25, CRUISE_SPEED - 75, "confident gentle curve right"

    return CRUISE_SPEED, CRUISE_SPEED, "confident straight"


def steer_safety(pose, target, severity):
    x, y, _angle = pose
    tx, ty = target

    desired_heading = math.atan2(ty - y, tx - x)
    heading = current_heading(pose)

    error = norm_angle(desired_heading - heading)
    abs_error = abs(error)

    # Important: in danger, do not make a huge forward curve.
    # Pivot first, then escape straight-ish.
    if severity >= 2 and abs_error > math.radians(32):
        turn = PIVOT_SPEED if error > 0 else -PIVOT_SPEED
        turn *= TURN_SIGN

        return -turn, turn, math.degrees(error), "emergency pivot"

    if severity == 1 and abs_error > math.radians(78):
        turn = WARNING_PIVOT_SPEED if error > 0 else -WARNING_PIVOT_SPEED
        turn *= TURN_SIGN

        return -turn, turn, math.degrees(error), "warning pre-turn"

    speed = EMERGENCY_ESCAPE_SPEED if severity >= 2 else SAFETY_SPEED

    turn = TURN_SIGN * int(clamp(SAFETY_TURN_GAIN * error, -SAFETY_TURN_LIMIT, SAFETY_TURN_LIMIT))

    left = int(clamp(speed - turn, SAFETY_MIN_FORWARD_SPEED, MAX_SPEED))
    right = int(clamp(speed + turn, SAFETY_MIN_FORWARD_SPEED, MAX_SPEED))

    return left, right, math.degrees(error), "controlled escape"


def maybe_start_unstick():
    global unstick_until, unstick_dir

    if time.time() < unstick_until:
        return

    if moved_recently_enough():
        return

    unstick_dir = random.choice([-1, 1])
    unstick_until = time.time() + 1.4


def unstick_command():
    # Forward curve, not spin.
    if unstick_dir > 0:
        return CRUISE_SPEED - 140, CRUISE_SPEED + 55, "unstick forward curve left"

    return CRUISE_SPEED + 55, CRUISE_SPEED - 140, "unstick forward curve right"


def choose_movement_command():
    pose = get_my_pose()

    if pose is None:
        return 0, 0, "waiting for own position", None, None, 0

    safety_target, safety_reason, severity = compute_safety_target(pose)

    if safety_target is not None:
        left, right, err, steering_mode = steer_safety(pose, safety_target, severity)

        reason = "{}; {}".format(steering_mode, safety_reason)

        return left, right, reason, safety_target, err, severity

    maybe_start_unstick()

    if time.time() < unstick_until:
        left, right, reason = unstick_command()
        return left, right, reason, None, None, 0

    left, right, reason = free_drive_command()
    return left, right, reason, None, None, 0


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

    print("Task 2 confident-safe: confident random walk + earlier collision/zone avoidance")
    print("Robot ID:", ROBOT_ID)
    print("Own topic:", OWN_TOPIC)
    print("Position topic:", POSITION_TOPIC)
    print("Communication radius:", COMMUNICATION_RADIUS, "m")
    print("Broker:", BROKER, PORT)
    print("Forbidden warning buffer:", FORBIDDEN_WARN_BUFFER)
    print("Boundary warning:", BOUNDARY_WARN)
    print("Robot warning distance:", ROBOT_WARN_DISTANCE)
    print("LED blink duration:", BLINK_DURATION_SECONDS, "seconds")
    print("If avoidance turns the wrong way, change TURN_SIGN from 1 to -1.")

    client = mqtt.Client(client_id="task2_confident_safe_robot_{}".format(ROBOT_ID))
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(BROKER, PORT, 60)
    client.loop_start()

    pipuck = PiPuck(epuck_version=2)

    try:
        pipuck.epuck.set_motor_speeds(0, 0)
        time.sleep(0.5)

        choose_new_drive_mode()

        while True:
            pose = get_my_pose()
            update_motion_estimate(pose)

            left, right, reason, target, err, severity = choose_movement_command()
            pipuck.epuck.set_motor_speeds(left, right)

            send_hello_to_close_robots(client)
            update_leds(pipuck)

            now = time.time()

            if now - last_print > 1.2:
                print(
                    "cmd=({}, {}) severity={} reason={} pose={} target={} err={} sent={} received={} ids={}".format(
                        left,
                        right,
                        severity,
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