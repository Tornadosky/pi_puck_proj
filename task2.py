#!/usr/bin/env python3
import json
import math
import time

import paho.mqtt.client as mqtt
from pipuck.pipuck import PiPuck

try:
    from smbus2 import SMBus, i2c_msg
    SMBUS_AVAILABLE = True
except Exception:
    SMBUS_AVAILABLE = False


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

ARENA_X_MIN = 0.03
ARENA_X_MAX = 1.97
ARENA_Y_MIN = 0.03
ARENA_Y_MAX = 0.97

BOUNDARY_TRIGGER = 0.20
BOUNDARY_CRITICAL = 0.11

# Approximate red zones from your screenshot.
POLYGON_DANGER_ZONES = [
    (
        "right_red_zone",
        [
            (1.70, 0.00),
            (2.00, 0.00),
            (2.00, 0.39),
            (1.95, 0.35),
            (1.90, 0.34),
            (1.84, 0.30),
            (1.78, 0.29),
            (1.73, 0.25),
            (1.75, 0.20),
            (1.78, 0.15),
            (1.74, 0.08),
            (1.66, 0.02),
        ],
    ),
    (
        "bottom_red_zone",
        [
            (0.00, 0.00),
            (1.70, 0.00),
            (1.62, 0.05),
            (1.50, 0.09),
            (1.25, 0.13),
            (1.05, 0.09),
            (0.95, 0.05),
            (0.72, 0.04),
            (0.62, 0.075),
            (0.44, 0.085),
            (0.30, 0.06),
            (0.25, 0.11),
            (0.14, 0.15),
            (0.00, 0.08),
        ],
    ),
    (
        "left_red_zone",
        [
            (0.00, 0.00),
            (0.26, 0.00),
            (0.22, 0.07),
            (0.14, 0.15),
            (0.25, 0.33),
            (0.14, 0.43),
            (0.25, 0.52),
            (0.13, 0.70),
            (0.23, 0.87),
            (0.00, 1.00),
        ],
    ),
]

POLYGON_BUFFER = 0.055
POLYGON_CRITICAL_BUFFER = 0.015

CIRCLE_OBSTACLES = [
    ("yellow_circle", 0.89, 0.53, 0.13),
]
CIRCLE_BUFFER = 0.06

ROBOT_AVOID_DISTANCE = 0.20
ROBOT_CRITICAL_DISTANCE = 0.14

SAFE_CENTER_X = 1.00
SAFE_CENTER_Y = 0.50


# ============================================================
# IR CONFIG
# ============================================================

# Your log had normal IR values around 40-130 and edge noise up to about 395.
# These are lower than before, but still above normal noise.
IR_FRONT_TRIGGER = 480
IR_FRONT_CRITICAL = 800
IR_SIDE_TRIGGER = 560
IR_SIDE_CRITICAL = 900


# ============================================================
# MOVEMENT CONFIG
# ============================================================

CRUISE_SPEED = 350

TURN_SPEED = 190
HEADING_OK_DEG = 25.0
HEADING_EMERGENCY_OK_DEG = 18.0
MAX_TURN_SECONDS = 4.5

ESCAPE_SPEED = 300
ESCAPE_TIME = 1.25

BACK_SPEED = 170
BACK_TIME = 0.28

CONTROL_PERIOD = 0.10

# If rotate-away goes the wrong direction, change this to -1.
TURN_SIGN = 1

ANGLE_SIGN = -1
HEADING_OFFSET_DEG = 95.0


class EPuck2DirectIO:
    I2C_CHANNEL = 12
    LEGACY_I2C_CHANNEL = 4
    ROB_ADDR = 0x1F

    ACTUATORS_SIZE = 20
    SENSORS_SIZE = 47

    def __init__(self):
        self.available = False
        self.bus = None
        self.fail_count = 0
        self.last_prox = None

        if not SMBUS_AVAILABLE:
            print("smbus2 is not available; IR fallback disabled")
            return

        for channel in [self.I2C_CHANNEL, self.LEGACY_I2C_CHANNEL]:
            try:
                self.bus = SMBus(channel)
                self.available = True
                print("Direct e-puck2 I2C opened on bus", channel)
                return
            except Exception:
                pass

        print("Could not open direct e-puck2 I2C; IR disabled")

    @staticmethod
    def _put_int16_le(buf, index, value):
        value = int(max(-2000, min(2000, value)))
        value &= 0xFFFF
        buf[index] = value & 0xFF
        buf[index + 1] = (value >> 8) & 0xFF

    @staticmethod
    def _checksum(buf, n):
        c = 0
        for i in range(n):
            c ^= buf[i]
        return c

    def exchange(self, left_speed, right_speed):
        if not self.available:
            return None

        actuators = bytearray([0] * self.ACTUATORS_SIZE)
        self._put_int16_le(actuators, 0, left_speed)
        self._put_int16_le(actuators, 2, right_speed)

        actuators[4] = 0
        actuators[5] = 0

        for i in range(6, 18):
            actuators[i] = 0

        actuators[18] = 0
        actuators[19] = self._checksum(actuators, self.ACTUATORS_SIZE - 1)

        try:
            write = i2c_msg.write(self.ROB_ADDR, actuators)
            read = i2c_msg.read(self.ROB_ADDR, self.SENSORS_SIZE)
            self.bus.i2c_rdwr(write, read)

            data = list(read)

            checksum = self._checksum(data, self.SENSORS_SIZE - 1)
            if checksum != data[self.SENSORS_SIZE - 1]:
                self.fail_count += 1
                return None

            prox = []
            for i in range(8):
                prox.append(data[i * 2 + 1] * 256 + data[i * 2])

            self.fail_count = 0
            self.last_prox = prox
            return prox

        except Exception as e:
            self.fail_count += 1

            if self.fail_count == 1:
                print("Direct I2C exchange failed:", e)

            if self.fail_count > 8:
                print("Too many direct I2C failures; disabling IR/direct motor mode")
                self.available = False

            return None

    def stop(self):
        if not self.available:
            return

        for _ in range(3):
            self.exchange(0, 0)
            time.sleep(0.05)


latest_positions = {}
latest_raw = {}
last_position_time = 0.0
latest_ir = None

phase = "straight"
phase_until = 0.0
phase_started = 0.0
turn_dir = 1
avoid_reason = ""
safe_heading = None
critical_danger = False

last_hello_sent = {}
next_proximity_check = 0.0

blink_until = 0.0
led_state = None
led_available = True

last_print = 0.0
sent_count = 0
received_count = 0


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


def heading_from_marker(pose):
    raw_angle_rad = raw_angle_to_rad(pose[2])
    offset = math.radians(HEADING_OFFSET_DEG)
    return norm_angle(ANGLE_SIGN * raw_angle_rad + offset)


def heading_error_deg(pose):
    if pose is None or safe_heading is None:
        return None

    return math.degrees(norm_angle(safe_heading - heading_from_marker(pose)))


def format_pose(pose):
    if pose is None:
        return "None"

    return "({:.3f}, {:.3f}, {:.3f})".format(
        float(pose[0]),
        float(pose[1]),
        float(pose[2]),
    )


def format_ir(ir):
    if ir is None:
        return "None"

    return "[" + ", ".join(str(int(v)) for v in ir) + "]"


def log_line(text):
    stamp = time.strftime("%H:%M:%S")
    line = "[{}] {}".format(stamp, text)

    print(line)

    try:
        with open(LOG_FILE, "a") as f:
            f.write(line + "\n")
    except Exception as e:
        print("Could not write log:", e)


def point_in_polygon(x, y, poly):
    inside = False
    j = len(poly) - 1

    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]

        intersects = ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-12) + xi
        )

        if intersects:
            inside = not inside

        j = i

    return inside


def nearest_point_on_segment(px, py, ax, ay, bx, by):
    abx = bx - ax
    aby = by - ay
    apx = px - ax
    apy = py - ay

    denom = abx * abx + aby * aby

    if denom <= 1e-12:
        return ax, ay

    t = clamp((apx * abx + apy * aby) / denom, 0.0, 1.0)

    return ax + t * abx, ay + t * aby


def polygon_distance_and_vector(x, y, poly):
    best_d = None
    best_vec = (0.0, 0.0)

    for i in range(len(poly)):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % len(poly)]

        qx, qy = nearest_point_on_segment(x, y, ax, ay, bx, by)
        dx = x - qx
        dy = y - qy
        d = math.sqrt(dx * dx + dy * dy)

        if best_d is None or d < best_d:
            best_d = d
            best_vec = (dx, dy)

    return best_d, best_vec


def add_unit(vec, dx, dy, weight):
    length = math.sqrt(dx * dx + dy * dy)

    if length < 0.0001:
        return

    vec[0] += weight * dx / length
    vec[1] += weight * dy / length


def detect_ir_danger():
    if latest_ir is None or len(latest_ir) < 8:
        return None

    ps0, ps1, ps2, ps3, ps4, ps5, ps6, ps7 = latest_ir[:8]

    front = max(ps0, ps7, int(0.75 * ps1), int(0.75 * ps6))
    left_side = max(ps5, ps6, ps7)
    right_side = max(ps0, ps1, ps2)

    if front < IR_FRONT_TRIGGER and max(left_side, right_side) < IR_SIDE_TRIGGER:
        return None

    left_strength = ps5 + ps6 + ps7
    right_strength = ps0 + ps1 + ps2

    # Obstacle stronger on left -> turn right.
    # Obstacle stronger on right -> turn left.
    chosen_turn_dir = -1 if left_strength > right_strength else 1

    critical = (
        front >= IR_FRONT_CRITICAL or
        max(left_side, right_side) >= IR_SIDE_CRITICAL
    )

    reasons = []

    if front >= IR_FRONT_TRIGGER:
        reasons.append("IR front {}".format(front))

    if left_side >= IR_SIDE_TRIGGER:
        reasons.append("IR left {}".format(left_side))

    if right_side >= IR_SIDE_TRIGGER:
        reasons.append("IR right {}".format(right_side))

    return {
        "reason": ", ".join(reasons),
        "critical": critical,
        "safe_heading": None,
        "turn_dir": chosen_turn_dir,
        "back_first": front >= IR_FRONT_CRITICAL,
        "source": "ir",
    }


def detect_pose_danger(pose):
    if pose is None:
        return None

    x, y, _angle = pose

    vec = [0.0, 0.0]
    reasons = []
    critical = False
    back_first = False

    # Rectangular table boundary.
    if x < ARENA_X_MIN + BOUNDARY_TRIGGER:
        strength = (ARENA_X_MIN + BOUNDARY_TRIGGER - x) / BOUNDARY_TRIGGER
        vec[0] += 2.5 * strength
        reasons.append("left boundary")

        if x < ARENA_X_MIN + BOUNDARY_CRITICAL:
            critical = True

    if x > ARENA_X_MAX - BOUNDARY_TRIGGER:
        strength = (x - (ARENA_X_MAX - BOUNDARY_TRIGGER)) / BOUNDARY_TRIGGER
        vec[0] -= 2.5 * strength
        reasons.append("right boundary")

        if x > ARENA_X_MAX - BOUNDARY_CRITICAL:
            critical = True

    if y < ARENA_Y_MIN + BOUNDARY_TRIGGER:
        strength = (ARENA_Y_MIN + BOUNDARY_TRIGGER - y) / BOUNDARY_TRIGGER
        vec[1] += 2.5 * strength
        reasons.append("bottom boundary")

        if y < ARENA_Y_MIN + BOUNDARY_CRITICAL:
            critical = True

    if y > ARENA_Y_MAX - BOUNDARY_TRIGGER:
        strength = (y - (ARENA_Y_MAX - BOUNDARY_TRIGGER)) / BOUNDARY_TRIGGER
        vec[1] -= 2.5 * strength
        reasons.append("top boundary")

        if y > ARENA_Y_MAX - BOUNDARY_CRITICAL:
            critical = True

    # Red polygon zones.
    for name, poly in POLYGON_DANGER_ZONES:
        inside = point_in_polygon(x, y, poly)
        dist_to_poly, away_vec = polygon_distance_and_vector(x, y, poly)

        if inside:
            add_unit(vec, SAFE_CENTER_X - x, SAFE_CENTER_Y - y, 4.5)
            reasons.append("inside {}".format(name))
            critical = True

        elif dist_to_poly is not None and dist_to_poly < POLYGON_BUFFER:
            dx, dy = away_vec

            if math.sqrt(dx * dx + dy * dy) < 0.001:
                dx, dy = SAFE_CENTER_X - x, SAFE_CENTER_Y - y

            weight = 3.0 * (POLYGON_BUFFER - dist_to_poly) / POLYGON_BUFFER
            add_unit(vec, dx, dy, 1.8 + weight)

            reasons.append("near {} {:.3f}m".format(name, dist_to_poly))

            if dist_to_poly < POLYGON_CRITICAL_BUFFER:
                critical = True

    # Yellow circle.
    for name, cx, cy, radius in CIRCLE_OBSTACLES:
        dx = x - cx
        dy = y - cy
        d = math.sqrt(dx * dx + dy * dy)
        danger_d = radius + CIRCLE_BUFFER

        if d < danger_d:
            if d < 0.001:
                dx, dy = SAFE_CENTER_X - cx, SAFE_CENTER_Y - cy

            add_unit(vec, dx, dy, 3.0)
            reasons.append("near {} {:.3f}m".format(name, d))

            if d < radius + 0.015:
                critical = True

    # Other robots from MQTT.
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
            add_unit(vec, x - closest_pose[0], y - closest_pose[1], 3.0)

            reasons.append("robot {} {:.3f}m".format(closest_id, closest_d))

            if closest_d < ROBOT_CRITICAL_DISTANCE:
                critical = True
                back_first = True

    mag = math.sqrt(vec[0] * vec[0] + vec[1] * vec[1])

    if mag < 0.001:
        return None

    sh = math.atan2(vec[1], vec[0])
    current_heading = heading_from_marker(pose)
    error = norm_angle(sh - current_heading)

    return {
        "reason": ", ".join(reasons),
        "critical": critical,
        "safe_heading": sh,
        "turn_dir": 1 if error > 0 else -1,
        "back_first": back_first,
        "source": "pose",
    }


def detect_danger(pose):
    ir = detect_ir_danger()

    if ir is not None:
        return ir

    return detect_pose_danger(pose)


def start_avoidance(danger):
    global phase, phase_until, phase_started
    global turn_dir, avoid_reason, safe_heading, critical_danger

    now = time.time()

    phase_started = now
    avoid_reason = danger["reason"]
    safe_heading = danger.get("safe_heading")
    critical_danger = bool(danger.get("critical", False))
    turn_dir = int(danger.get("turn_dir", 1))

    if danger.get("back_first", False):
        phase = "back"
        phase_until = now + BACK_TIME
        return

    phase = "turn"
    phase_until = now + MAX_TURN_SECONDS


def heading_is_safe(pose):
    if safe_heading is None:
        return False

    err = abs(norm_angle(safe_heading - heading_from_marker(pose)))

    limit = HEADING_EMERGENCY_OK_DEG if critical_danger else HEADING_OK_DEG

    return math.degrees(err) <= limit


def avoidance_command(pose):
    global phase, phase_until

    now = time.time()

    if phase == "straight":
        return None

    if phase == "back":
        if now >= phase_until:
            phase = "turn"
            phase_until = now + MAX_TURN_SECONDS
        else:
            return -BACK_SPEED, -BACK_SPEED, "backup: " + avoid_reason

    if phase == "turn":
        # Pose danger: keep rotating until the marker heading actually points inward.
        if safe_heading is not None and heading_is_safe(pose):
            phase = "escape"
            phase_until = now + ESCAPE_TIME

        # IR-only danger: fixed turn because IR has no global safe heading.
        elif safe_heading is None and now - phase_started > 0.65:
            phase = "escape"
            phase_until = now + ESCAPE_TIME

        elif now >= phase_until:
            # Stop instead of driving off the table.
            return 0, 0, "turn timeout, stopping: " + avoid_reason

        else:
            signed = TURN_SIGN * turn_dir * TURN_SPEED
            return -signed, signed, "rotate until safe: " + avoid_reason

    if phase == "escape":
        # If still in pose danger and not heading safely, rotate more.
        new_danger = detect_pose_danger(pose)

        if new_danger is not None and new_danger.get("safe_heading") is not None:
            new_error = abs(norm_angle(new_danger["safe_heading"] - heading_from_marker(pose)))

            if math.degrees(new_error) > HEADING_OK_DEG:
                start_avoidance(new_danger)
                return avoidance_command(pose)

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

    cmd = avoidance_command(pose)

    if cmd is not None:
        return cmd

    danger = detect_danger(pose)

    if danger is not None:
        start_avoidance(danger)

        cmd = avoidance_command(pose)

        if cmd is not None:
            return cmd

    return CRUISE_SPEED, CRUISE_SPEED, "straight"


def send_motors_and_read_ir(pipuck, direct_io, left, right):
    if direct_io is not None and direct_io.available:
        return direct_io.exchange(left, right)

    pipuck.epuck.set_motor_speeds(left, right)

    return None


def stop_robot(pipuck, direct_io):
    if direct_io is not None and direct_io.available:
        direct_io.stop()
    else:
        pipuck.epuck.set_motor_speeds(0, 0)


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


def main():
    global last_print, latest_ir

    print("Task 2 boundary-locked safe")
    print("Robot ID:", ROBOT_ID)
    print("Own topic:", OWN_TOPIC)
    print("Position topic:", POSITION_TOPIC)
    print("Broker:", BROKER, PORT)
    print("Behavior: straight; danger -> rotate until heading is safe -> escape")
    print("Boundary trigger:", BOUNDARY_TRIGGER)
    print("Polygon buffer:", POLYGON_BUFFER)
    print("MQTT robot avoid distance:", ROBOT_AVOID_DISTANCE)
    print("IR front/side triggers:", IR_FRONT_TRIGGER, IR_SIDE_TRIGGER)
    print("If it rotates the wrong way, set TURN_SIGN = -1.")

    client = mqtt.Client(client_id="task2_boundary_locked_robot_{}".format(ROBOT_ID))
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(BROKER, PORT, 60)
    client.loop_start()

    pipuck = PiPuck(epuck_version=2)
    direct_io = EPuck2DirectIO()

    try:
        stop_robot(pipuck, direct_io)
        time.sleep(0.5)

        while True:
            pose = get_my_pose()

            if last_position_time > 0 and time.time() - last_position_time > 2.0:
                left, right, reason = 0, 0, "position stale, stopping"
            else:
                left, right, reason = choose_movement_command()

            ir = send_motors_and_read_ir(pipuck, direct_io, left, right)

            if ir is not None:
                latest_ir = ir

            send_hello_to_close_robots(client)
            update_leds(pipuck)

            now = time.time()

            if now - last_print > 1.2:
                err = heading_error_deg(pose)

                print(
                    "cmd=({}, {}) phase={} reason={} pose={} err={} ir={} sent={} received={} ids={}".format(
                        left,
                        right,
                        phase,
                        reason,
                        format_pose(pose),
                        None if err is None else round(err, 1),
                        format_ir(latest_ir),
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
            stop_robot(pipuck, direct_io)
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