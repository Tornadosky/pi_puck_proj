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

# More aware than the previous straight-reactive version,
# but not crazy sensitive.
BOUNDARY_TRIGGER = 0.18
BOUNDARY_CRITICAL = 0.10

FORBIDDEN_ZONES = [
    # name, x_min, x_max, y_min, y_max
    ("top_left_forbidden", 0.00, 0.25, 0.75, 1.00),
]

# Small buffer around forbidden zone.
# Increase to 0.13 if it still enters the forbidden zone.
FORBIDDEN_BUFFER = 0.10

# Camera/MQTT-based robot avoidance.
# IR handles very close physical objects, so this threshold stays moderate.
ROBOT_AVOID_DISTANCE = 0.17
ROBOT_CRITICAL_DISTANCE = 0.13

SAFE_CENTER_X = 1.00
SAFE_CENTER_Y = 0.50


# ============================================================
# IR SENSOR CONFIG
# ============================================================

# e-puck proximity sensor rough layout:
# ps0 front-right, ps1 right-front, ps2 right, ps3 rear-right,
# ps4 rear-left, ps5 left, ps6 left-front, ps7 front-left.
#
# Values depend on lighting and calibration.
# If it avoids too early, increase these.
# If it touches robots before avoiding, decrease these.
IR_FRONT_TRIGGER = 700
IR_FRONT_CRITICAL = 1200

IR_SIDE_TRIGGER = 900
IR_SIDE_CRITICAL = 1400


# ============================================================
# MOVEMENT CONFIG
# ============================================================

# Normal behavior: straight.
CRUISE_SPEED = 380

# Danger behavior:
# 1. optionally back up for very close IR/robot danger,
# 2. rotate in place,
# 3. drive straight away.
TURN_SPEED = 185
MIN_TURN_TIME = 0.40
MAX_TURN_TIME = 1.10

ESCAPE_SPEED = 330
ESCAPE_TIME = 2.20

BACK_SPEED = 180
BACK_TIME = 0.30

CONTROL_PERIOD = 0.10

# If it rotates the wrong way, change this to -1.
TURN_SIGN = 1

# Fallback heading conversion from camera marker angle.
ANGLE_SIGN = -1
HEADING_OFFSET_DEG = 95.0


# ============================================================
# LOW-LEVEL E-PUCK2 I2C FOR MOTORS + IR PROXIMITY
# ============================================================

class EPuck2DirectIO:
    """
    Direct e-puck2 I2C communication.

    This is used so we can read the 8 IR proximity values.
    It also sends the motor speeds in the same packet.
    """

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
            print("smbus2 is not available, IR fallback disabled")
            return

        try:
            self.bus = SMBus(self.I2C_CHANNEL)
            self.available = True
            print("Direct e-puck2 I2C opened on bus", self.I2C_CHANNEL)
            return
        except Exception:
            pass

        try:
            self.bus = SMBus(self.LEGACY_I2C_CHANNEL)
            self.available = True
            print("Direct e-puck2 I2C opened on legacy bus", self.LEGACY_I2C_CHANNEL)
            return
        except Exception as e:
            print("Could not open direct e-puck2 I2C. IR disabled:", e)

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
        """
        Send motor speeds and read IR sensors.
        Returns a list of 8 proximity values, or None on failure.
        """
        if not self.available:
            return None

        actuators = bytearray([0] * self.ACTUATORS_SIZE)

        self._put_int16_le(actuators, 0, left_speed)
        self._put_int16_le(actuators, 2, right_speed)

        # byte 4: speaker
        actuators[4] = 0

        # byte 5: normal e-puck LEDs off
        actuators[5] = 0

        # bytes 6..17: RGB LEDs off
        for i in range(6, 18):
            actuators[i] = 0

        # byte 18: settings
        actuators[18] = 0

        # byte 19: checksum
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
                value = data[i * 2 + 1] * 256 + data[i * 2]
                prox.append(value)

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


# ============================================================
# GLOBAL STATE
# ============================================================

latest_positions = {}
latest_raw = {}
last_position_time = 0.0

latest_ir = None

motion_history = []

phase = "straight"
phase_until = 0.0
turn_dir = 1
avoid_reason = ""
avoid_started = 0.0

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
    This is more reliable than only trusting the marker angle.
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


def vector_to_turn_danger(vec, reason, critical=False, back_first=False):
    mag = math.sqrt(vec[0] * vec[0] + vec[1] * vec[1])

    if mag < 0.001:
        return None

    safe_heading = math.atan2(vec[1], vec[0])

    pose = get_my_pose()
    current = recent_motion_heading(pose)
    error = norm_angle(safe_heading - current)

    needs_turn = abs(error) > math.radians(25)
    turn_time = clamp(abs(error) / math.pi * 1.15, MIN_TURN_TIME, MAX_TURN_TIME)

    return {
        "reason": reason,
        "critical": critical,
        "back_first": back_first,
        "turn_dir": 1 if error > 0 else -1,
        "turn_time": turn_time,
        "needs_turn": needs_turn,
    }


def detect_ir_danger(ir):
    """
    IR-based close obstacle avoidance.

    ps0 front-right, ps1 right-front, ps2 right,
    ps5 left, ps6 left-front, ps7 front-left.
    """
    if ir is None or len(ir) < 8:
        return None

    ps0, ps1, ps2, ps3, ps4, ps5, ps6, ps7 = ir[:8]

    front = max(ps0, ps7, int(0.75 * ps1), int(0.75 * ps6))
    left_side = max(ps5, ps6, ps7)
    right_side = max(ps0, ps1, ps2)

    front_triggered = front >= IR_FRONT_TRIGGER
    side_triggered = max(left_side, right_side) >= IR_SIDE_TRIGGER

    if not front_triggered and not side_triggered:
        return None

    left_strength = ps5 + ps6 + ps7
    right_strength = ps0 + ps1 + ps2

    # If obstacle is stronger on left, turn right.
    # If obstacle is stronger on right, turn left.
    if left_strength > right_strength:
        chosen_turn_dir = -1
    elif right_strength > left_strength:
        chosen_turn_dir = 1
    else:
        chosen_turn_dir = 1

    critical = (
        front >= IR_FRONT_CRITICAL or
        max(left_side, right_side) >= IR_SIDE_CRITICAL
    )

    reasons = []

    if front_triggered:
        reasons.append("IR front {}".format(front))

    if left_side >= IR_SIDE_TRIGGER:
        reasons.append("IR left {}".format(left_side))

    if right_side >= IR_SIDE_TRIGGER:
        reasons.append("IR right {}".format(right_side))

    return {
        "reason": ", ".join(reasons),
        "critical": critical,
        "back_first": front >= IR_FRONT_CRITICAL,
        "turn_dir": chosen_turn_dir,
        "turn_time": 0.75 if critical else 0.55,
        "needs_turn": True,
    }


def detect_pose_danger(pose):
    """
    Camera/MQTT-based danger:
      - boundary,
      - forbidden zone,
      - nearby robots.
    """
    if pose is None:
        return None

    x, y, _angle = pose

    vec = [0.0, 0.0]
    reasons = []
    critical = False
    back_first = False

    # ---------- Boundary ----------
    if x < ARENA_X_MIN + BOUNDARY_TRIGGER:
        vec[0] += 2.0
        reasons.append("left boundary")
        if x < ARENA_X_MIN + BOUNDARY_CRITICAL:
            critical = True

    if x > ARENA_X_MAX - BOUNDARY_TRIGGER:
        vec[0] -= 2.0
        reasons.append("right boundary")
        if x > ARENA_X_MAX - BOUNDARY_CRITICAL:
            critical = True

    if y < ARENA_Y_MIN + BOUNDARY_TRIGGER:
        vec[1] += 2.0
        reasons.append("bottom boundary")
        if y < ARENA_Y_MIN + BOUNDARY_CRITICAL:
            critical = True

    if y > ARENA_Y_MAX - BOUNDARY_TRIGGER:
        vec[1] -= 2.0
        reasons.append("top boundary")
        if y > ARENA_Y_MAX - BOUNDARY_CRITICAL:
            critical = True

    # ---------- Forbidden zone ----------
    for zone in FORBIDDEN_ZONES:
        name = zone[0]

        if inside_rect(x, y, zone, 0.0):
            add_vector(vec, SAFE_CENTER_X - x, SAFE_CENTER_Y - y, 3.5)
            reasons.append("inside forbidden zone: " + name)
            critical = True

        elif inside_rect(x, y, zone, FORBIDDEN_BUFFER):
            add_vector(vec, SAFE_CENTER_X - x, SAFE_CENTER_Y - y, 2.5)
            reasons.append("near forbidden zone: " + name)

    # ---------- Camera/MQTT robot collision ----------
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

            add_vector(vec, dx, dy, 2.8)
            reasons.append("robot {} at {:.3f}m".format(closest_id, closest_d))

            if closest_d < ROBOT_CRITICAL_DISTANCE:
                critical = True
                back_first = True

    if not reasons:
        return None

    return vector_to_turn_danger(
        vec,
        ", ".join(reasons),
        critical=critical,
        back_first=back_first,
    )


def detect_danger(pose):
    """
    Priority:
      1. IR close obstacle,
      2. boundary / forbidden zone / MQTT robot position.
    """
    ir_danger = detect_ir_danger(latest_ir)
    if ir_danger is not None:
        return ir_danger

    return detect_pose_danger(pose)


# ============================================================
# MOVEMENT STATE MACHINE
# ============================================================

def start_avoidance(danger):
    global phase, phase_until, turn_dir, avoid_reason, avoid_started

    now = time.time()

    avoid_reason = danger["reason"]
    avoid_started = now
    turn_dir = danger["turn_dir"]

    if danger.get("back_first", False):
        phase = "back"
        phase_until = now + BACK_TIME
        return

    if danger.get("needs_turn", True):
        phase = "turn"
        phase_until = now + danger["turn_time"]
        return

    phase = "escape"
    phase_until = now + ESCAPE_TIME


def avoidance_command():
    global phase, phase_until

    now = time.time()

    if phase == "straight":
        return None

    # Never stay in avoidance forever.
    if now - avoid_started > 5.0:
        phase = "straight"
        return None

    if phase == "back":
        if now >= phase_until:
            phase = "turn"
            phase_until = now + 0.55
        else:
            return -BACK_SPEED, -BACK_SPEED, "IR/robot backup: " + avoid_reason

    if phase == "turn":
        if now >= phase_until:
            phase = "escape"
            phase_until = now + ESCAPE_TIME
        else:
            signed = TURN_SIGN * turn_dir * TURN_SPEED
            return -signed, signed, "rotate away: " + avoid_reason

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

    # During escape, a new very close IR obstacle can interrupt.
    ir_danger = detect_ir_danger(latest_ir)
    if ir_danger is not None and ir_danger["critical"] and phase in ["straight", "escape"]:
        start_avoidance(ir_danger)

    cmd = avoidance_command()
    if cmd is not None:
        return cmd

    danger = detect_danger(pose)

    if danger is not None:
        start_avoidance(danger)

        cmd = avoidance_command()
        if cmd is not None:
            return cmd

    return CRUISE_SPEED, CRUISE_SPEED, "straight"


# ============================================================
# MOTOR COMMAND / IR READ
# ============================================================

def send_motors_and_read_ir(pipuck, direct_io, left, right):
    """
    Prefer direct I2C, because it sends motors and reads IR in one exchange.
    If that is not available, fall back to PiPuck motor API without IR.
    """
    if direct_io is not None and direct_io.available:
        return direct_io.exchange(left, right)

    pipuck.epuck.set_motor_speeds(left, right)
    return None


def stop_robot(pipuck, direct_io):
    if direct_io is not None and direct_io.available:
        direct_io.stop()
        return

    pipuck.epuck.set_motor_speeds(0, 0)


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
    global last_print, latest_ir

    print("Task 2 IR-safe")
    print("Robot ID:", ROBOT_ID)
    print("Own topic:", OWN_TOPIC)
    print("Position topic:", POSITION_TOPIC)
    print("Broker:", BROKER, PORT)
    print("Behavior: straight; IR/boundary/forbidden danger -> rotate in place -> escape")
    print("Boundary trigger:", BOUNDARY_TRIGGER)
    print("Forbidden buffer:", FORBIDDEN_BUFFER)
    print("MQTT robot avoid distance:", ROBOT_AVOID_DISTANCE)
    print("IR front trigger:", IR_FRONT_TRIGGER)
    print("IR side trigger:", IR_SIDE_TRIGGER)
    print("If avoidance rotates wrong way, set TURN_SIGN = -1.")

    client = mqtt.Client(client_id="task2_ir_safe_robot_{}".format(ROBOT_ID))
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
            # If tracking is stale, stop instead of blindly driving.
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

            if now - last_print > 1.5:
                pose = get_my_pose()

                print(
                    "cmd=({}, {}) phase={} reason={} pose={} ir={} sent={} received={} ids={}".format(
                        left,
                        right,
                        phase,
                        reason,
                        format_pose(pose),
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