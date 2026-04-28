import paho.mqtt.client as mqtt
import json
import time
from pipuck.pipuck import PiPuck

# Correct exercise broker
BROKER = "192.168.178.43"
PORT = 1883

ROBOT_ID = 33
POSITION_TOPIC = "robot_pos/all"

latest_positions = {}

def on_connect(client, userdata, flags, rc):
    print("Connected with result code", rc)
    client.subscribe(POSITION_TOPIC)

def on_message(client, userdata, msg):
    global latest_positions

    try:
        data = json.loads(msg.payload.decode())
        latest_positions = data
        print(data)
    except json.JSONDecodeError:
        print("Invalid JSON:", msg.payload)

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER, PORT, 60)
client.loop_start()

pipuck = PiPuck(epuck_version=2)

try:
    # Safe while debugging: robot does not move
    pipuck.epuck.set_motor_speeds(0, 0)

    for _ in range(1000):
        # Later, replace this with your random-walk logic
        time.sleep(1)

finally:
    # Always stop the robot, even after Ctrl+C or an error
    pipuck.epuck.set_motor_speeds(0, 0)
    client.loop_stop()
    client.disconnect()
    print("Robot stopped")