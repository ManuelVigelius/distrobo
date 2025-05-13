import paho.mqtt.client as mqtt
import json
import time
from pipuck.pipuck import PiPuck
import numpy as np
import math

# Define variables and callbacks
Broker = "192.168.178.56"  # Replace with your broker address
Port = 1883 # standard MQTT port, original 1883

my_id = '2'
position = 0
orientation = 0

# function to handle connection
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("robot_pos/all")

# function to handle incoming messages
def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        if my_id in data.keys():
            position = np.asarray(data[my_id]['position'])
            orientation = angle_to_orientation(data[my_id]['angle'])
        else:
            print('Err: no position')
            print(data)
    except json.JSONDecodeError:
        print(f'invalid json: {msg.payload}')

def orient_towards(position, orientation, point, eps=0.05, speed=200):
    delta = position - point
    if delta @ orientation >= 1 - eps:
        return True
    if delta[0] > delta[0]:
        pipuck.epuck.set_motor_speeds(speed, -speed)
    else:
        pipuck.epuck.set_motor_speeds(-speed, speed)
    return False

def got_to_point(position, orientation, point, eps=0.01, speed=1000):
    delta = position - point
    if np.linalg.norm(position - point) <= eps:
        return True
    if not(orient_towards(position, orientation, point)):
        return False
    pipuck.epuck.set_motor_speeds(speed, speed)

def angle_to_orientation(angle):
    angle_radians = math.radians(angle)
    return np.asarray([math.cos(angle_radians), math.sin(angle_radians)])


# Initialize MQTT client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(Broker, Port, 60)

client.loop_start() # Start listening loop in separate thread

pipuck = PiPuck(epuck_version=2)


goal = np.asarray([0, 0])

for _ in range(30):
    time.sleep(0.1)

for _ in range(10):
    got_to_point(position, orientation, goal)
    time.sleep(1)
	
    
# Stop the MQTT client loop
pipuck.epuck.set_motor_speeds(0,0)
client.loop_stop()  