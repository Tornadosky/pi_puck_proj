#!/usr/bin/env python3
import json
import math
import random
import time

import paho.mqtt.client as mqtt
from pipuck.pipuck import PiPuck


# ======================
# Basic configuration
# ======================

BROKER = "192.168.178.43"
PORT = 1883
POSITION_TOPIC = "robot_pos/all"

ROBOT_ID = "33"

# Arena bounds from your previous run
ARENA_MIN_X = 0.0
ARENA_MAX_X = 2.0
ARENA_MIN_Y = 0.0
ARENA_MAX_Y = 1.0

# Smaller margins than before, so it does not avoid all the time
WALL_MARGIN = 0.12
ROBOT_AVOID_DISTANCE = 0.18

# Add static obstacles here if you know their coordinates:
# Example: STATIC_OBSTACLES = [("box", 0.8, 0.5)]
STATIC_OBSTACLES = []
STATIC_AVOID_DISTANCE = 0.18

# Safe speeds
FORWARD_SPEED = 260
CURVE_INNER_SPEED = 160
CURVE_OUTER_SPEED = 270
BACK_SPEED = -180
TURN_SPEED = 200

CONTROL_PERIOD = 0.1

latest_positions = {}
latest_raw = {}
last_print_time = 0.0

mode = "stop"
mode_until = 0.0
turn_dir = 1


def robot_key(robot_id):
    return str(robot_id).replace("pi-puck", "").replace("robot", "").strip()


def get_xy_angle_from_record(record):
    """
    Simple parser for likely MQTT formats:
      {"position": [x, y], "angle": a}
      {"x": x, "y": y, "angle": a}
      [x, y, angle]
    """

    if isinstance(record, list) or isinstance(record, tuple):
        if len(record) >= 2:
            x = float(record[0])
            y = float(record[1])
            angle = float(record[2]) if len(record) >= 3 else 0.0
            return x, y, angle

    if isinstance(record, dict):
        if "position" in record:
            pos = record["position"]

            if isinstance(pos, list) or isinstance(pos, tuple):
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
    """
    Simple parser for topic robot_pos/all.

    Expected style:
      {
        "33": {"position": [x, y], "angle": angle},
        "34": {"position": [x, y], "angle": angle}
      }

    Also supports:
      {
        "robots": [
          {"id": "33", "position": [x, y], "angle": angle}
        ]
      }
    """

    positions = {}

    if not isinstance(data, dict):
        return positions

    # Case 1: {"robots": [{"id": "33", ...}, ...]}
    if "robots" in data and isinstance(data["robots"], list):
        for record in data["robots"]:
            if not isinstance(record, dict):
                continue

            rid = record.get("id", record.get("robot_id"))
            pose = get_xy_angle_from_record(record)

            if rid is not None and pose is not None:
                positions[robot_key(rid)] = pose

        return positions

    # Case 2: {"33": {"position": [x, y], "angle": a}, ...}
    for rid, record in data.items():
        pose = get_xy_angle_from_record(record)

        if pose is not None:
            positions[robot_key(rid)] = pose

    return positions


def distance(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return math.sqrt(dx * dx + dy * dy)


def nearest_robot(my_pose):
    nearest_id = None
    nearest_dist = None

    for rid, pose in latest_positions.items():
        if robot_key(rid) == robot_key(ROBOT_ID):
            continue

        d = distance(my_pose, pose)

        if nearest_dist is None or d < nearest_dist:
            nearest_id = rid
            nearest_dist = d

    return nearest_id, nearest_dist


def danger_reason(my_pose):
    x, y, _angle = my_pose

    if x < ARENA_MIN_X + WALL_MARGIN:
        return True, "left wall"

    if x > ARENA_MAX_X - WALL_MARGIN:
        return True, "right wall"

    if y < ARENA_MIN_Y + WALL_MARGIN:
        return True, "bottom wall"

    if y > ARENA_MAX_Y - WALL_MARGIN:
        return True, "top wall"

    rid, d = nearest_robot(my_pose)
    if d is not None and d < ROBOT_AVOID_DISTANCE:
        return True, "robot {} too close: {:.3f}".format(rid, d)

    for name, ox, oy in STATIC_OBSTACLES:
        d = math.sqrt((x - ox) ** 2 + (y - oy) ** 2)
        if d < STATIC_AVOID_DISTANCE:
            return True, "static obstacle {} too close: {:.3f}".format(name, d)

    return False, "clear"


def choose_random_walk_mode():
    global mode, mode_until, turn_dir

    r = random.random()
    now = time.time()

    if r < 0.65:
        mode = "forward"
        mode_until = now + random.uniform(1.0, 2.5)

    elif r < 0.825:
        mode = "curve_left"
        mode_until = now + random.uniform(0.8, 1.8)

    else:
        mode = "curve_right"
        mode_until = now + random.uniform(0.8, 1.8)


def start_avoidance():
    global mode, mode_until, turn_dir

    turn_dir = random.choice([-1, 1])
    mode = "avoid_back"
    mode_until = time.time() + 0.45


def get_motor_command():
    global mode, mode_until

    now = time.time()
    my_pose = latest_positions.get(robot_key(ROBOT_ID))

    if my_pose is None:
        return 0, 0, "waiting for own position"

    danger, reason = danger_reason(my_pose)

    # If danger appears, do a fixed maneuver instead of constantly recalculating.
    if danger and not mode.startswith("avoid"):
        start_avoidance()

    # Avoidance phase 1: reverse
    if mode == "avoid_back":
        if now >= mode_until:
            mode = "avoid_turn"
            mode_until = now + 0.55

        return BACK_SPEED, BACK_SPEED, "avoidance: backing away from " + reason

    # Avoidance phase 2: turn briefly
    if mode == "avoid_turn":
        if now >= mode_until:
            mode = "forward"
            mode_until = now + 1.0

        return -turn_dir * TURN_SPEED, turn_dir * TURN_SPEED, "avoidance: turning away"

    # Normal random walk
    if now >= mode_until:
        choose_random_walk_mode()

    if mode == "forward":
        return FORWARD_SPEED, FORWARD_SPEED, "random walk: forward"

    if mode == "curve_left":
        return CURVE_INNER_SPEED, CURVE_OUTER_SPEED, "random walk: curve left"

    if mode == "curve_right":
        return CURVE_OUTER_SPEED, CURVE_INNER_SPEED, "random walk: curve right"

    return 0, 0, "stop"


def on_connect(client, userdata, flags, rc):
    print("Connected with result code", rc)
    client.subscribe(POSITION_TOPIC)


def on_message(client, userdata, msg):
    global latest_positions, latest_raw

    try:
        payload = msg.payload.decode("utf-8", "replace")
        data = json.loads(payload)

        latest_raw = data
        latest_positions = parse_positions(data)

    except Exception as e:
        print("Bad MQTT message:", e)


def main():
    global last_print_time

    print("Task 1 simple random walk")
    print("Robot ID:", ROBOT_ID)
    print("Broker:", BROKER)
    print("Topic:", POSITION_TOPIC)

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(BROKER, PORT, 60)
    client.loop_start()

    pipuck = PiPuck(epuck_version=2)

    try:
        pipuck.epuck.set_motor_speeds(0, 0)
        time.sleep(0.5)

        choose_random_walk_mode()

        while True:
            left, right, reason = get_motor_command()
            pipuck.epuck.set_motor_speeds(left, right)

            now = time.time()
            if now - last_print_time > 2.0:
                my_pose = latest_positions.get(robot_key(ROBOT_ID))

                nearest_id = None
                nearest_dist = None
                if my_pose is not None:
                    nearest_id, nearest_dist = nearest_robot(my_pose)

                print(
                    "cmd=({}, {}) mode={} reason={} my_pose={} nearest={} dist={} all_ids={}".format(
                        left,
                        right,
                        mode,
                        reason,
                        my_pose,
                        nearest_id,
                        nearest_dist,
                        sorted(latest_positions.keys())
                    )
                )

                if not latest_positions:
                    print("No parsed positions. Raw MQTT:", latest_raw)

                last_print_time = now

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