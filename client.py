import paho.mqtt.client as mqtt
import json
import time
from pipuck.pipuck import PiPuck
import numpy as np
import math

class RobotController:
    def __init__(self, robot_id):
        self.my_id = robot_id
        self.position = None
        self.orientation = None

        self.pipuck = PiPuck(epuck_version=2)

        def on_connect(client, userdata, flags, rc):
            print("Connected with result code " + str(rc))
            client.subscribe("robot_pos/all")

        def on_message(client, userdata, msg):
            try:
                data = json.loads(msg.payload.decode())
                if self.my_id in data.keys():
                    self.position = np.asarray(data[self.my_id]['position'])
                    self.orientation = self.angle_to_orientation(data[self.my_id]['angle'])
                else:
                    print('Err: no position')
                    print(data)
            except json.JSONDecodeError:
                print(f'invalid json: {msg.payload}')



        self.client = mqtt.Client()
        self.client.on_connect = on_connect
        self.client.on_message = on_message

        Broker = "192.168.178.56"  # Replace with your broker address
        Port = 1883 # standard MQTT port
        self.client.connect(Broker, Port, 60)

    def orient_towards(self, point, eps=0.05, speed=200):
        delta = self.position - point
        inner_prod = (delta / np.linalg.norm(delta)) @ self.orientation
        l = [[0, 1], [-1, 0]] @ self.orientation
        r = [[0, -1], [1, 0]] @ self.orientation
        if inner_prod >= 1 - eps:
            return True
        if r @ delta > l @ delta:
            self.pipuck.epuck.set_motor_speeds(speed, -speed)
        else:
            self.pipuck.epuck.set_motor_speeds(-speed, speed)
        return False

    def go_to_point(self, point, eps=0.01, speed=1000):
        delta = self.position - point
        if np.linalg.norm(self.position - point) <= eps:
            return True
        if not self.orient_towards(point):
            return False
        self.pipuck.epuck.set_motor_speeds(speed, speed)
        return False

    def angle_to_orientation(self, angle):
        angle_radians = math.radians(angle) + np.pi
        return np.asarray([math.cos(angle_radians), math.sin(angle_radians)])

    def start_mqtt(self):
        self.client.loop_start()
    
    def stop_mqtt(self):
        self.client.loop_stop()

    def stop_robot(self):
        self.pipuck.epuck.set_motor_speeds(0, 0)

    def run(self):
        self.pipuck.epuck.set_motor_speeds(50, 50)


def get_random_border_point():
    scenario = np.random.randint(1, 5)
    if scenario == 1:
        return np.asarray([np.random.uniform(0.1, 1.9), 0.1])
    elif scenario == 2:
        return np.asarray([np.random.uniform(0.1, 1.9), 0.9])
    elif scenario == 3:
        return np.asarray([0.1, np.random.uniform(0.1, 0.9)])
    else:
        return np.asarray([0.9, np.random.uniform(0.1, 0.9)])

controller = RobotController('33')
controller.start_mqtt()

try:
    for i in range(100):
        time.sleep(0.1)
        if not (controller.position is None):
            break
        if i % 10 == 0:
            print('Waiting for position')

    #goal = get_random_border_point()
    goal = np.zeros(2)
    print(f'Goal is {goal}')

    for i in range(500):
        # if (controller.go_to_point(goal)):
        #     goal = get_random_border_point()
        #     print(f'Goal is {goal}')

        if(controller.orient_towards(goal)):
            controller.stop_robot()

        time.sleep(0.03)
        if i % 20 == 0:
            pos = controller.position
            # print(f'Position: {pos}')
            print(f'Orientation: {controller.orientation}')

            if (pos[0] < 0) | (pos[0] > 2) | (pos[1] < 0) | (pos[1] > 1):
                break
except Exception as e:
    print(e)

controller.stop_robot()
controller.stop_mqtt()
