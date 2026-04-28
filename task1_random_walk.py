#!/usr/bin/env python3
import json
import math
import random
import time

import paho.mqtt.client as mqtt
from pipuck.pipuck import PiPuck


# =======================
# Settings you may change
# =======================

BROKER = "192.168.178.43"
PORT = 1883
POSITION_TOPIC = "robot_pos/all"

# Your robot number
ROBOT_ID = "33"

# Arena bounds in the SAME coordinate units as the tracking dashboard / MQTT.
# If the dashboard uses different min/max values, change these.
ARENA_MIN_X = 0.0
ARENA_MAX_X = 2.0
ARENA_MIN_Y = 0.0
ARENA_MAX_Y = 1.0

WALL_MARGIN = 0.18
ROBOT_AVOID_DISTANCE = 0.35

# Optional static obstacles: add known obstacle coordinates here.
# Example: STATIC_OBSTACLES = [("box", 0.75, 0.50)]
STATIC_OBSTACLES = []
STATIC_AVOID_DISTANCE = 0.30

# Keep speeds low while testing.
BASE_SPEED = 250
SLOW_SPEED = 170
TURN_SPEED = 220
MAX_SPEED = 350

# If the robot turns the wrong way while avoiding, change this to -1.
TURN_SIGN = 1

# Safety: robot will not move until it sees its own MQTT position.
REQUIRE_POSITION_FOR_MOVEMENT = True

CONTROL_PERIOD = 0.10
RANDOM_CHANGE_MIN = 1.0
RANDOM_CHANGE_MAX = 3.0


latest_raw = {}
latest_poses = {}
last_message_time = 0.0
last_print_time = 0.0

random_left = BASE_SPEED
random_right = BASE_SPEED
next_random_change = 0.0


def norm_id(robot_id):
    s = str(robot_id).strip()
    lower = s.lower()
    for prefix in ["pi-puck", "pipuck", "robot/", "robot_", "robot"]:
        if lower.startswith(prefix):
            s = s[len(prefix):]
            break
    return s.strip()


def clamp(value, low, high):
    return max(low, min(high, value))


def to_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def dict_get(d, keys):
    if not isinstance(d, dict):
        return None

    lower_map = {}
    for k, v in d.items():
        lower_map[str(k).lower()] = v

    for key in keys:
        if key in d:
            return d[key]
        lk = key.lower()
        if lk in lower_map:
            return lower_map[lk]

    return None


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def angle_to_rad(angle):
    if angle is None:
        return None

    angle = to_float(angle)
    if angle is None:
        return None

    # If it looks like degrees, convert to radians.
    if abs(angle) > 2.0 * math.pi + 0.1:
        angle = math.radians(angle)

    return normalize_angle(angle)


def parse_pose_value(value):
    """
    Accepts several possible MQTT formats:
      [x, y, angle]
      {"x": x, "y": y, "angle": angle}
      {"position": [x, y], "angle": angle}
      {"position": {"x": x, "y": y}, "angle": angle}
    """
    x = None
    y = None
    theta = None

    if isinstance(value, (list, tuple)):
        if len(value) >= 2:
            x = to_float(value[0])
            y = to_float(value[1])
        if len(value) >= 3:
            theta = angle_to_rad(value[2])

    elif isinstance(value, dict):
        x = to_float(dict_get(value, ["x", "X", "pos_x", "position_x", "cx"]))
        y = to_float(dict_get(value, ["y", "Y", "pos_y", "position_y", "cy"]))
        theta = angle_to_rad(dict_get(value, ["angle", "theta", "a", "orientation", "yaw"]))

        pos_obj = dict_get(value, ["position", "pos", "pose", "center", "coordinates"])
        if pos_obj is not None:
            nested = parse_pose_value(pos_obj)
            if nested is not None:
                nx, ny, nt = nested
                if x is None:
                    x = nx
                if y is None:
                    y = ny
                if theta is None:
                    theta = nt

        angle_obj = dict_get(value, ["rotation", "heading"])
        if theta is None and angle_obj is not None:
            theta = angle_to_rad(angle_obj)

    if x is None or y is None:
        return None

    return (x, y, theta)


def parse_positions(data):
    """
    Converts MQTT payload into:
      {"33": (x, y, theta), "34": (...)}
    """
    poses = {}

    if isinstance(data, dict):
        for field in ["robots", "robot_pos", "positions", "data"]:
            sub = dict_get(data, [field])
            if sub is not None and sub is not data:
                poses.update(parse_positions(sub))

        rid = dict_get(data, ["id", "robot_id", "robotId", "name"])
        pose = parse_pose_value(data)
        if rid is not None and pose is not None:
            poses[norm_id(rid)] = pose

        for key, value in data.items():
            if str(key).lower() in ["time", "timestamp", "frame", "robots", "robot_pos", "positions", "data"]:
                continue

            pose = parse_pose_value(value)
            if pose is None:
                continue

            rid = key
            if isinstance(value, dict):
                inner_id = dict_get(value, ["id", "robot_id", "robotId", "name"])
                if inner_id is not None:
                    rid = inner_id

            poses[norm_id(rid)] = pose

    elif isinstance(data, list):
        for item in data:
            if not isinstance(item, dict):
                continue

            rid = dict_get(item, ["id", "robot_id", "robotId", "name"])
            pose = parse_pose_value(item)
            if rid is not None and pose is not None:
                poses[norm_id(rid)] = pose

    return poses


def add_repulsion(vec, x, y, ox, oy, avoid_distance):
    dx = x - ox
    dy = y - oy
    dist = math.sqrt(dx * dx + dy * dy)

    if dist < 0.001:
        angle = random.uniform(-math.pi, math.pi)
        dx = math.cos(angle)
        dy = math.sin(angle)
        dist = 1.0

    if dist < avoid_distance:
        strength = (avoid_distance - dist) / avoid_distance
        vec[0] += (dx / dist) * strength
        vec[1] += (dy / dist) * strength


def get_avoidance_vector(my_pose):
    x, y, _theta = my_pose
    vec = [0.0, 0.0]

    # Avoid arena boundary.
    if x < ARENA_MIN_X + WALL_MARGIN:
        vec[0] += (ARENA_MIN_X + WALL_MARGIN - x) / WALL_MARGIN

    if x > ARENA_MAX_X - WALL_MARGIN:
        vec[0] -= (x - (ARENA_MAX_X - WALL_MARGIN)) / WALL_MARGIN

    if y < ARENA_MIN_Y + WALL_MARGIN:
        vec[1] += (ARENA_MIN_Y + WALL_MARGIN - y) / WALL_MARGIN

    if y > ARENA_MAX_Y - WALL_MARGIN:
        vec[1] -= (y - (ARENA_MAX_Y - WALL_MARGIN)) / WALL_MARGIN

    # Avoid other robots using MQTT positions.
    for other_id, other_pose in latest_poses.items():
        if norm_id(other_id) == norm_id(ROBOT_ID):
            continue

        ox, oy, _ = other_pose
        add_repulsion(vec, x, y, ox, oy, ROBOT_AVOID_DISTANCE)

    # Avoid known static obstacles.
    for _name, ox, oy in STATIC_OBSTACLES:
        add_repulsion(vec, x, y, ox, oy, STATIC_AVOID_DISTANCE)

    return vec


def choose_random_speeds():
    global random_left, random_right, next_random_change

    now = time.time()

    if now >= next_random_change:
        r = random.random()

        if r < 0.60:
            random_left = BASE_SPEED
            random_right = BASE_SPEED
        elif r < 0.80:
            random_left = -TURN_SPEED
            random_right = TURN_SPEED
        else:
            random_left = TURN_SPEED
            random_right = -TURN_SPEED

        next_random_change = now + random.uniform(RANDOM_CHANGE_MIN, RANDOM_CHANGE_MAX)

    return random_left, random_right


def speeds_towards_angle(current_theta, target_theta):
    if current_theta is None:
        return -TURN_SPEED, TURN_SPEED

    error = normalize_angle(target_theta - current_theta)
    turn = TURN_SIGN * int(clamp(350.0 * error, -TURN_SPEED, TURN_SPEED))

    if abs(error) > 1.0:
        left = -turn
        right = turn
    else:
        left = SLOW_SPEED - turn
        right = SLOW_SPEED + turn

    left = int(clamp(left, -MAX_SPEED, MAX_SPEED))
    right = int(clamp(right, -MAX_SPEED, MAX_SPEED))

    return left, right


def choose_speeds():
    my_pose = latest_poses.get(norm_id(ROBOT_ID))

    if my_pose is None:
        if REQUIRE_POSITION_FOR_MOVEMENT:
            return 0, 0, "waiting for own MQTT position"

        left, right = choose_random_speeds()
        return left, right, "random walk, no position"

    avoid_vec = get_avoidance_vector(my_pose)
    avoid_strength = math.sqrt(avoid_vec[0] ** 2 + avoid_vec[1] ** 2)

    if avoid_strength > 0.01:
        target_theta = math.atan2(avoid_vec[1], avoid_vec[0])
        left, right = speeds_towards_angle(my_pose[2], target_theta)
        return left, right, "avoidance"

    left, right = choose_random_speeds()
    return left, right, "random walk"


def on_connect(client, userdata, flags, rc):
    print("Connected with result code", rc)
    client.subscribe(POSITION_TOPIC)


def on_message(client, userdata, msg):
    global latest_raw, latest_poses, last_message_time

    try:
        payload = msg.payload.decode("utf-8", "replace")
        data = json.loads(payload)
    except Exception as e:
        print("Invalid MQTT message:", e, msg.payload)
        return

    latest_raw = data
    latest_poses = parse_positions(data)
    last_message_time = time.time()


def main():
    global last_print_time

    print("Starting Task 1 random walk with avoidance")
    print("Robot ID:", ROBOT_ID)
    print("MQTT:", BROKER, PORT, POSITION_TOPIC)
    print("Arena:", ARENA_MIN_X, ARENA_MAX_X, ARENA_MIN_Y, ARENA_MAX_Y)

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
            left, right, reason = choose_speeds()
            pipuck.epuck.set_motor_speeds(left, right)

            now = time.time()
            if now - last_print_time > 2.0:
                my_pose = latest_poses.get(norm_id(ROBOT_ID))
                print("cmd=({}, {}) reason={} my_pose={} all_ids={}".format(
                    left, right, reason, my_pose, sorted(latest_poses.keys())
                ))

                if not latest_poses:
                    print("Raw MQTT payload is empty/unparsed:", latest_raw)

                last_print_time = now

            time.sleep(CONTROL_PERIOD)

    except KeyboardInterrupt:
        print("\nCtrl+C received")

    finally:
        print("Stopping motors")
        try:
            pipuck.epuck.set_motor_speeds(0, 0)
        except Exception as e:
            print("Could not stop motors:", e)

        client.loop_stop()
        client.disconnect()
        print("Done")


if __name__ == "__main__":
    main()