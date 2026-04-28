cd ~/pi_puck_proj
cat > task2_confident.py <<'PY'
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

# Task says e.g. 50 cm. Reduce to 0.30 if it spams too much.
COMMUNICATION_RADIUS = 0.50
PROXIMITY_CHECK_INTERVAL = 2.0
HELLO_COOLDOWN_SECONDS = 8.0

LOG_FILE = "task2_messages.log"


# ============================================================
# CONFIDENT MOVEMENT CONFIG
# ============================================================

ARENA_X_MIN = 0.05
ARENA_X_MAX = 1.95
ARENA_Y_MIN = 0.05
ARENA_Y_MAX = 0.95

# It starts avoiding when it is this close to the boundary.
BOUNDARY_MARGIN = 0.18

# Wider target range than before, so it explores more of the arena.
TARGET_X_MIN = 0.18
TARGET_X_MAX = 1.82
TARGET_Y_MIN = 0.12
TARGET_Y_MAX = 0.88

# Collision avoidance is separate from communication.
ROBOT_AVOID_DISTANCE = 0.18

# Change this if the forbidden zone is different.
FORBIDDEN_ZONES = [
    ("top_left_forbidden", 0.00, 0.25, 0.75, 1.00),
]

# Faster and more confident.
CRUISE_SPEED = 420
AVOID_SPEED = 340

# No wheel should go below this during steering.
# This is the main anti-spinning parameter.
MIN_FORWARD_SPEED = 150

MAX_SPEED = 520

# Lower turn limit = less rotating, more straight movement.
TURN_LIMIT = 90
TURN_GAIN = 145.0

# Random movement is mostly long straight lines.
STRAIGHT_MIN_TIME = 7.0
STRAIGHT_MAX_TIME = 14.0
CURVE_MIN_TIME = 2.0
CURVE_MAX_TIME = 5.0

CONTROL_PERIOD = 0.10

# Tracking angle from your logs looked like degrees.
# If boundary avoidance turns the wrong way, change TURN_SIGN to -1.
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
    Prefer actual movement direction from position updates.
    This is more reliable than trusting the marker angle alone.
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

    # Only update heading if the robot really moved.
    if d > 0.006:
        new_heading = math.atan2(dy, dx)

        if motion_heading is None:
            motion_heading = new_heading
        else:
            # Smooth angle.
            sx = 0.75 * math.cos(motion_heading) + 0.25 * math.cos(new_heading)
            sy = 0.75 * math.sin(motion_heading) + 0.25 * math.sin(new_heading)
            motion_heading = math.atan2(sy, sx)

        motion_heading_time = now


def moved_recently_enough():
    """
    Detects mumbling/spinning around one place.
    """
    if len(motion_history) < 2:
        return True

    now = time.time()
    old_samples = [p for p in motion_history if now - p[0] >= 3.5]

    if not old_samples:
        return True

    old_t, old_x, old_y = old_samples[0]
    new_t, new_x, new_y = motion_history[-1]

    moved = math.sqrt((new_x - old_x) ** 2 + (new_y - old_y) ** 2)

    # If it moved less than 5 cm in ~3.5+ seconds, it is probably mumbling.
    return moved >= 0.05


# ============================================================
# MOVEMENT LOGIC
# ============================================================

def inside_zone(x, y, zone):
    _name, x_min, x_max, y_min, y_max = zone
    return x_min <= x <= x_max and y_min <= y <= y_max


def choose_new_drive_mode():
    """
    Mostly long straight movement.
    Curves happen sometimes, but no spin-in-place.
    """
    global drive_mode, drive_mode_until

    r = random.random()
    now = time.time()

    if r < 0.78:
        drive_mode = "straight"
        drive_mode_until = now + random.uniform(STRAIGHT_MIN_TIME, STRAIGHT_MAX_TIME)

    elif r < 0.89:
        drive_mode = "curve_left"
        drive_mode_until = now + random.uniform(CURVE_MIN_TIME, CURVE_MAX_TIME)

    else:
        drive_mode = "curve_right"
        drive_mode_until = now + random.uniform(CURVE_MIN_TIME, CURVE_MAX_TIME)


def free_drive_command():
    """
    Normal random walk: mostly straight, sometimes gentle curves.
    """
    global drive_mode

    if time.time() >= drive_mode_until:
        choose_new_drive_mode()

    if drive_mode == "straight":
        return CRUISE_SPEED, CRUISE_SPEED, "confident straight"

    if drive_mode == "curve_left":
        return CRUISE_SPEED - 90, CRUISE_SPEED + 30, "confident curve left"

    if drive_mode == "curve_right":
        return CRUISE_SPEED + 30, CRUISE_SPEED - 90, "confident curve right"

    choose_new_drive_mode()
    return CRUISE_SPEED, CRUISE_SPEED, "confident straight"


def get_safety_target(pose):
    x, y, _angle = pose

    # Forbidden zone: escape toward center.
    for zone in FORBIDDEN_ZONES:
        name, _x_min, _x_max, _y_min, _y_max = zone

        if inside_zone(x, y, zone):
            return (1.00, 0.50), "escape forbidden zone: " + name

    # Boundary avoidance: combine x/y corrections, so corners are handled better.
    target_x = x
    target_y = y
    reasons = []

    if x < ARENA_X_MIN + BOUNDARY_MARGIN:
        target_x = 0.80
        reasons.append("left boundary")

    if x > ARENA_X_MAX - BOUNDARY_MARGIN:
        target_x = 1.20
        reasons.append("right boundary")

    if y < ARENA_Y_MIN + BOUNDARY_MARGIN:
        target_y = 0.45
        reasons.append("bottom boundary")

    if y > ARENA_Y_MAX - BOUNDARY_MARGIN:
        target_y = 0.55
        reasons.append("top boundary")

    if reasons:
        target_x = clamp(target_x, TARGET_X_MIN, TARGET_X_MAX)
        target_y = clamp(target_y, TARGET_Y_MIN, TARGET_Y_MAX)
        return (target_x, target_y), "avoid " + " + ".join(reasons)

    # Robot collision avoidance.
    closest_id = None
    closest_d = None
    closest_pose = None

    for rid, other in latest_positions.items():
        if robot_key(rid) == robot_key(ROBOT_ID):
            continue

        d = distance(pose, other)

        if closest_d is None or d < closest_d:
            closest_id = rid
            closest_d = d
            closest_pose = other

    if closest_pose is not None and 0.001 < closest_d < ROBOT_AVOID_DISTANCE:
        dx = pose[0] - closest_pose[0]
        dy = pose[1] - closest_pose[1]

        tx = clamp(pose[0] + 0.55 * dx / closest_d, TARGET_X_MIN, TARGET_X_MAX)
        ty = clamp(pose[1] + 0.55 * dy / closest_d, TARGET_Y_MIN, TARGET_Y_MAX)

        return (tx, ty), "avoid robot {} at {:.2f}m".format(closest_id, closest_d)

    return None, "clear"


def steer_to_target_forward_only(pose, target, speed):
    """
    Steering without spinning:
    both wheels remain positive, even for large heading errors.
    """
    x, y, _angle = pose
    tx, ty = target

    desired_heading = math.atan2(ty - y, tx - x)
    heading = current_heading(pose)

    error = norm_angle(desired_heading - heading)

    turn = TURN_SIGN * int(clamp(TURN_GAIN * error, -TURN_LIMIT, TURN_LIMIT))

    # Slightly reduce speed for huge error, but never spin in place.
    abs_error = abs(error)

    if abs_error > math.radians(120):
        base = int(speed * 0.78)
    elif abs_error > math.radians(70):
        base = int(speed * 0.88)
    else:
        base = speed

    left = int(clamp(base - turn, MIN_FORWARD_SPEED, MAX_SPEED))
    right = int(clamp(base + turn, MIN_FORWARD_SPEED, MAX_SPEED))

    return left, right, math.degrees(error)


def maybe_start_unstick():
    global unstick_until, unstick_dir

    if time.time() < unstick_until:
        return

    if moved_recently_enough():
        return

    unstick_dir = random.choice([-1, 1])
    unstick_until = time.time() + 1.7


def unstick_command():
    """
    Strong forward curve, not spin.
    """
    if unstick_dir > 0:
        return CRUISE_SPEED - 150, CRUISE_SPEED + 70, "unstick strong forward curve left"

    return CRUISE_SPEED + 70, CRUISE_SPEED - 150, "unstick strong forward curve right"


def choose_movement_command():
    pose = get_my_pose()

    if pose is None:
        return 0, 0, "waiting for own position", None, None

    safety_target, safety_reason = get_safety_target(pose)

    if safety_target is not None:
        left, right, err = steer_to_target_forward_only(pose, safety_target, AVOID_SPEED)
        return left, right, safety_reason, safety_target, err

    maybe_start_unstick()

    if time.time() < unstick_until:
        left, right, reason = unstick_command()
        return left, right, reason, None, None

    left, right, reason = free_drive_command()
    return left, right, reason, None, None


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
    blink_until = time.time() + 2.0

    log_line("RX from {} on {}: {}".format(sender, OWN_TOPIC, text))


def update_leds(pipuck):
    global led_state, led_available

    if not led_available:
        return

    now = time.time()

    if now < blink_until:
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

    print("Task 2 confident: long straight random walk + talk to close robots")
    print("Robot ID:", ROBOT_ID)
    print("Own topic:", OWN_TOPIC)
    print("Position topic:", POSITION_TOPIC)
    print("Communication radius:", COMMUNICATION_RADIUS, "m")
    print("Broker:", BROKER, PORT)
    print("Movement: mostly long straight lines, no spin-in-place steering")
    print("If boundary avoidance turns the wrong way, change TURN_SIGN from 1 to -1.")

    client = mqtt.Client(client_id="task2_confident_robot_{}".format(ROBOT_ID))
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

            left, right, reason, target, err = choose_movement_command()
            pipuck.epuck.set_motor_speeds(left, right)

            send_hello_to_close_robots(client)
            update_leds(pipuck)

            now = time.time()

            if now - last_print > 1.5:
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
PY