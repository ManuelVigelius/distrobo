import paho.mqtt.client as mqtt
import json
import time
from pipuck.pipuck import PiPuck
import numpy as np

# Define variables and callbacks
Broker = "192.168.178.56"  # Replace with your broker address
Port = 1883 # standard MQTT port

# function to handle connection
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("robot_pos/all")

# function to handle incoming messages
def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        print(data)
    except json.JSONDecodeError:
        print(f'invalid json: {msg.payload}')


def got_to_point(position, orientation, point, eps=0.01):
    delta = position - point
    if np.linalg.norm(position - point) <= eps:
        return True
    if delta @ orientation <  0:
        




# Initialize MQTT client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(Broker, Port, 60)

client.loop_start() # Start listening loop in separate thread

# Initialize the PiPuck
pipuck = PiPuck(epuck_version=2)

for _ in range(10):
    speed_l, speed_r = np.random.normal(size=2).clip(min=-1, max=1) * 100
    pipuck.epuck.set_motor_speeds(speed_l, speed_r)
    time.sleep(1)
	
    
# Stop the MQTT client loop
pipuck.epuck.set_motor_speeds(0,0)
client.loop_stop()  