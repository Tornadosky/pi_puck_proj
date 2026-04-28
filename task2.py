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

# Task 2 communication distance. This is separate from collision avoidance.
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

# Less sensitive than before: no lookahead / predicted boundary.
BOUNDARY_MARGIN = 0.13
BOUNDARY_CRITICAL_MARGIN = 0.08

FORBIDDEN_ZONES = [
    # name, x_min, x_max, y_min, y_max
    ("top_left_forbidden", 0.00, 0.25, 0.75, 1.00),
]

# Small buffer only, so it avoids forbidden zones but does not obsess.
FORBIDDEN_BUFFER = 0.07

# Robot collision avoidance threshold. Not huge.
ROBOT_AVOID_DISTANCE = 0.20
ROBOT_CRITICAL_DISTANCE = 0.15

TARGET_X_MIN = 0.18
TARGET_X_MAX = 1.82
TARGET_Y_MIN = 0.12
TARGET_Y_MAX = 0.88


# ============================================================
# MOVEMENT CONFIG
# ============================================================

# Confident normal movement.
CRUISE_SPEED = 410
MAX_SPEED = 520

STRAIGHT_MIN_TIME = 7.0
STRAIGHT_MAX_TIME = 14.0
CURVE_MIN_TIME = 2.0
CURVE_MAX_TIME = 4.5

# Avoidance: maybe back up, turn, then escape forward.
BACK_SPEED = 190
BACK_TIME = 0.35

TURN_SPEED = 175
TURN_TIME = 0.50

ESCAPE_SPEED = 320
ESCAPE_TIME = 1.15

ESCAPE_TURN_GAIN = 160.0
ESCAPE_TURN_LIMIT = 120
ESCAPE_MIN_FORWARD_SPEED = 110

CONTROL_PERIOD = 0.10

# If avoidance turns the wrong way, change this to -1.
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
last_motor_cmd = (0, 0)

avoid_phase = "idle"
avoid_until = 0.0
avoid_target = None
avoid_reason = ""
avoid_turn_dir = 1
avoid_started_at = 0.0

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
# HEADING ESTIMATE
# ============================================================

def camera_heading(pose):
    raw_angle_rad = raw_angle_to_rad(pose[2])
    offset = math.radians(HEADING_OFFSET_DEG)
    return norm_angle(ANGLE_SIGN * raw_angle_rad + offset)


def current_heading(pose):
    """
    Prefer real movement direction only when the previous command was forward.
    This avoids treating backwards movement as the robot's forward heading.
    """
    left, right = last_motor_cmd
    previous_cmd_was_forward = left > 100 and right > 100

    if previous_cmd_was_forward and motion_heading is not None and time.time() - motion_heading_time < 2.0:
        return motion_heading

    return camera_heading(pose)


def update_motion_estimate(pose):
    global motion_history, motion_heading, motion_heading_time

    if pose is None:
        return

    now = time.time()
    x, y, _angle = pose

    motion_history.append((now, x, y))
    motion_history = [p for p in motion_history if now - p[0] <= 6.0]

    if len(motion_history) < 2:
        return

    left, right = last_motor_cmd
    if not (left > 100 and right > 100):
        return

    _old_t, old_x, old_y = motion_history[-2]
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


# ============================================================
# DANGER DETECTION
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
    return 0.5 * (x_min + x_max), 0.5 * (y_min + y_max)


def add_unit_vector(vec, dx, dy, weight):
    length = math.sqrt(dx * dx + dy * dy)

    if length < 0.0001:
        return

    vec[0] += weight * dx / length
    vec[1] += weight * dy / length


def clamp_safe_target(tx, ty):
    tx = clamp(tx, TARGET_X_MIN, TARGET_X_MAX)
    ty = clamp(ty, TARGET_Y_MIN, TARGET_Y_MAX)

    if point_inside_any_forbidden(tx, ty, FORBIDDEN_BUFFER):
        tx, ty = 1.00, 0.50

    return tx, ty


def detect_danger(pose):
    """
    Moderate sensitivity:
      - no predicted-boundary lookahead
      - small forbidden-zone buffer
      - small robot collision threshold
    """
    x, y, _angle = pose
    vec = [0.0, 0.0]
    reasons = []
    critical = False

    # Boundary danger.
    if x < ARENA_X_MIN + BOUNDARY_MARGIN:
        strength = (ARENA_X_MIN + BOUNDARY_MARGIN - x) / BOUNDARY_MARGIN
        vec[0] += 2.0 * strength
        reasons.append("left boundary")
        if x < ARENA_X_MIN + BOUNDARY_CRITICAL_MARGIN:
            critical = True

    if x > ARENA_X_MAX - BOUNDARY_MARGIN:
        strength = (x - (ARENA_X_MAX - BOUNDARY_MARGIN)) / BOUNDARY_MARGIN
        vec[0] -= 2.0 * strength
        reasons.append("right boundary")
        if x > ARENA_X_MAX - BOUNDARY_CRITICAL_MARGIN:
            critical = True

    if y < ARENA_Y_MIN + BOUNDARY_MARGIN:
        strength = (ARENA_Y_MIN + BOUNDARY_MARGIN - y) / BOUNDARY_MARGIN
        vec[1] += 2.0 * strength
        reasons.append("bottom boundary")
        if y < ARENA_Y_MIN + BOUNDARY_CRITICAL_MARGIN:
            critical = True

    if y > ARENA_Y_MAX - BOUNDARY_MARGIN:
        strength = (y - (ARENA_Y_MAX - BOUNDARY_MARGIN)) / BOUNDARY_MARGIN
        vec[1] -= 2.0 * strength
        reasons.append("top boundary")
        if y > ARENA_Y_MAX - BOUNDARY_CRITICAL_MARGIN:
            critical = True

    # Forbidden-zone danger.
    for zone in FORBIDDEN_ZONES:
        name = zone[0]
        zcx, zcy = zone_center(zone)

        if point_inside_zone(x, y, zone, 0.0):
            dx = x - zcx
            dy = y - zcy

            if abs(dx) < 0.001 and abs(dy) < 0.001:
                dx = 1.0 - x
                dy = 0.5 - y

            add_unit_vector(vec, dx, dy, 3.5)
            reasons.append("inside forbidden zone: " + name)
            critical = True

        elif point_inside_zone(x, y, zone, FORBIDDEN_BUFFER):
            dx = x - zcx
            dy = y - zcy

            add_unit_vector(vec, dx, dy, 2.3)
            reasons.append("near forbidden zone: " + name)

    # Robot collision danger.
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

    if closest_pose is not None and closest_d is not None and 0.001 < closest_d < ROBOT_AVOID_DISTANCE:
        dx = x - closest_pose[0]
        dy = y - closest_pose[1]

        strength = 2.2 + (ROBOT_AVOID_DISTANCE - closest_d) / ROBOT_AVOID_DISTANCE
        add_unit_vector(vec, dx, dy, strength)

        reasons.append("robot {} at {:.2f}m".format(closest_id, closest_d))

        if closest_d < ROBOT_CRITICAL_DISTANCE:
            critical = True

    mag = math.sqrt(vec[0] * vec[0] + vec[1] * vec[1])

    if mag < 0.001:
        return None

    escape_distance = 0.58 if critical else 0.42

    tx = x + escape_distance * vec[0] / mag
    ty = y + escape_distance * vec[1] / mag
    tx, ty = clamp_safe_target(tx, ty)

    # Backup only helps if robot is facing toward danger.
    heading = current_heading(pose)
    safe_x = vec[0] / mag
    safe_y = vec[1] / mag

    heading_dot_safe = math.cos(heading) * safe_x + math.sin(heading) * safe_y
    should_backup = heading_dot_safe < -0.10

    return {
        "target": (tx, ty),
        "reason": ", ".join(reasons),
        "critical": critical,
        "should_backup": should_backup,
    }


# ============================================================
# MOVEMENT LOGIC
# ============================================================

def choose_new_drive_mode():
    global drive_mode, drive_mode_until

    r = random.random()
    now = time.time()

    if r < 0.80:
        drive_mode = "straight"
        drive_mode_until = now + random.uniform(STRAIGHT_MIN_TIME, STRAIGHT_MAX_TIME)

    elif r < 0.90:
        drive_mode = "curve_left"
        drive_mode_until = now + random.uniform(CURVE_MIN_TIME, CURVE_MAX_TIME)

    else:
        drive_mode = "curve_right"
        drive_mode_until = now + random.uniform(CURVE_MIN_TIME, CURVE_MAX_TIME)


def free_drive_command():
    if time.time() >= drive_mode_until:
        choose_new_drive_mode()

    if drive_mode == "straight":
        return CRUISE_SPEED, CRUISE_SPEED, "confident straight", None, None

    if drive_mode == "curve_left":
        return CRUISE_SPEED - 80, CRUISE_SPEED + 25, "confident gentle curve left", None, None

    if drive_mode == "curve_right":
        return CRUISE_SPEED + 25, CRUISE_SPEED - 80, "confident gentle curve right", None, None

    return CRUISE_SPEED, CRUISE_SPEED, "confident straight", None, None


def steer_to_target_forward(pose, target):
    x, y, _angle = pose
    tx, ty = target

    desired_heading = math.atan2(ty - y, tx - x)
    heading = current_heading(pose)
    error = norm_angle(desired_heading - heading)

    turn = TURN_SIGN * int(clamp(ESCAPE_TURN_GAIN * error, -ESCAPE_TURN_LIMIT, ESCAPE_TURN_LIMIT))

    left = int(clamp(ESCAPE_SPEED - turn, ESCAPE_MIN_FORWARD_SPEED, MAX_SPEED))
    right = int(clamp(ESCAPE_SPEED + turn, ESCAPE_MIN_FORWARD_SPEED, MAX_SPEED))

    return left, right, math.degrees(error)


def set_avoid_turn_or_escape(pose):
    global avoid_phase, avoid_until, avoid_turn_dir

    if avoid_target is None:
        avoid_phase = "idle"
        return

    x, y, _angle = pose
    tx, ty = avoid_target

    desired_heading = math.atan2(ty - y, tx - x)
    error = norm_angle(desired_heading - current_heading(pose))

    if abs(error) < math.radians(45):
        avoid_phase = "escape"
        avoid_until = time.time() + ESCAPE_TIME
    else:
        avoid_turn_dir = 1 if error > 0 else -1
        avoid_phase = "turn"
        avoid_until = time.time() + TURN_TIME


def start_avoidance(pose, danger):
    global avoid_phase, avoid_until, avoid_target, avoid_reason, avoid_started_at

    avoid_started_at = time.time()
    avoid_target = danger["target"]
    avoid_reason = danger["reason"]

    if danger["should_backup"]:
        avoid_phase = "back"
        avoid_until = time.time() + BACK_TIME
    else:
        set_avoid_turn_or_escape(pose)


def avoidance_command(pose):
    global avoid_phase, avoid_until

    now = time.time()

    if avoid_phase == "idle":
        return None

    # Never stay in avoidance forever.
    if now - avoid_started_at > 4.0:
        avoid_phase = "idle"
        choose_new_drive_mode()
        return None

    if avoid_phase == "back":
        if now >= avoid_until:
            set_avoid_turn_or_escape(pose)
        else:
            return -BACK_SPEED, -BACK_SPEED, "avoid backup: " + avoid_reason, avoid_target, None

    if avoid_phase == "turn":
        if avoid_target is None:
            avoid_phase = "idle"
            return None

        tx, ty = avoid_target
        desired_heading = math.atan2(ty - pose[1], tx - pose[0])
        error = norm_angle(desired_heading - current_heading(pose))

        if now >= avoid_until or abs(error) < math.radians(38):
            avoid_phase = "escape"
            avoid_until = now + ESCAPE_TIME
        else:
            signed_turn = TURN_SIGN * avoid_turn_dir * TURN_SPEED
            return -signed_turn, signed_turn, "avoid turn: " + avoid_reason, avoid_target, math.degrees(error)

    if avoid_phase == "escape":
        if avoid_target is None:
            avoid_phase = "idle"
            return None

        left, right, err = steer_to_target_forward(pose, avoid_target)

        if now >= avoid_until:
            danger = detect_danger(pose)

            if danger is not None:
                start_avoidance(pose, danger)
                return avoidance_command(pose)

            avoid_phase = "idle"
            choose_new_drive_mode()
            return None

        return left, right, "avoid escape: " + avoid_reason, avoid_target, err

    avoid_phase = "idle"
    return None


def choose_movement_command():
    pose = get_my_pose()

    if pose is None:
        return 0, 0, "waiting for own position", None, None

    cmd = avoidance_command(pose)
    if cmd is not None:
        return cmd

    danger = detect_danger(pose)

    if danger is not None:
        start_avoidance(pose, danger)

        cmd = avoidance_command(pose)
        if cmd is not None:
            return cmd

    return free_drive_command()


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
    global last_print, last_motor_cmd

    print("Task 2 final: confident movement + moderate backup-based avoidance")
    print("Robot ID:", ROBOT_ID)
    print("Own topic:", OWN_TOPIC)
    print("Position topic:", POSITION_TOPIC)
    print("Communication radius:", COMMUNICATION_RADIUS, "m")
    print("Collision avoid distance:", ROBOT_AVOID_DISTANCE, "m")
    print("Boundary margin:", BOUNDARY_MARGIN, "m")
    print("Forbidden buffer:", FORBIDDEN_BUFFER, "m")
    print("LED blink duration:", BLINK_DURATION_SECONDS, "s")
    print("Broker:", BROKER, PORT)
    print("If avoidance turns the wrong way, change TURN_SIGN from 1 to -1.")

    client = mqtt.Client(client_id="task2_final_robot_{}".format(ROBOT_ID))
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(BROKER, PORT, 60)
    client.loop_start()

    pipuck = PiPuck(epuck_version=2)

    try:
        pipuck.epuck.set_motor_speeds(0, 0)
        last_motor_cmd = (0, 0)
        time.sleep(0.5)

        choose_new_drive_mode()

        while True:
            pose = get_my_pose()
            update_motion_estimate(pose)

            left, right, reason, target, err = choose_movement_command()
            pipuck.epuck.set_motor_speeds(left, right)
            last_motor_cmd = (left, right)

            send_hello_to_close_robots(client)
            update_leds(pipuck)

            now = time.time()

            if now - last_print > 1.5:
                print(
                    "cmd=({}, {}) phase={} reason={} pose={} target={} err={} sent={} received={} ids={}".format(
                        left,
                        right,
                        avoid_phase,
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