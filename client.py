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
        self.robot_data = {}
        self.on_tick_callbacks = {}

        self.pipuck = PiPuck(epuck_version=2)

        def on_connect(client, userdata, flags, rc):
            print("Connected with result code " + str(rc))
            client.subscribe('robot_pos/all')
            client.subscribe(f'robot/{self.my_id}')

        def on_message(client, userdata, msg):
            try:
                data = json.loads(msg.payload.decode())
                if msg.topic == 'robot_pos/all':
                    for id in (set(self.robot_data.keys()) - set(data.keys())):
                        since = self.robot_data[id]['seen_since']
                        if (since > 10) & (id == self.my_id):
                            print('Error: No position')
                        else:
                            self.robot_data[id]['counter'] = since+1

                    for id in data.keys():
                        if not id in self.robot_data.keys():
                            self.robot_data[id] = {}
                        self.robot_data[id]['position'] = np.asarray(data[id]['position'])
                        self.robot_data[id]['orientation'] = self.angle_to_orientation(data[id]['angle'])
                        self.robot_data[id]['seen_since'] = 0

                    self.position = self.robot_data[self.my_id]['position']
                    self.orientation = self.robot_data[self.my_id]['orientation']
                elif msg.topic == 'robot/{self.my_id}':
                    print('Got message')
                    print(data)
            except json.JSONDecodeError:
                print(f'invalid json: {msg.payload}')

        self.client = mqtt.Client()
        self.client.on_connect = on_connect
        self.client.on_message = on_message

        Broker = "192.168.178.56"  # Replace with your broker address
        Port = 1883 # standard MQTT port
        self.client.connect(Broker, Port, 60)

    def orient_towards(self, point, eps=0.01, speed=200):
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

    def go_to_point(self, point, eps=0.03, speed=1000):
        delta = self.position - point
        if np.linalg.norm(delta) <= eps:
            return True
        if not self.orient_towards(point):
            return False
        self.pipuck.epuck.set_motor_speeds(speed, speed)
        return False
    
    def dist_to(self, point):
        return np.linalg.norm(self.position - point)

    def angle_to_orientation(self, angle):
        angle_radians = math.radians(angle) + np.pi
        return np.asarray([math.cos(angle_radians), math.sin(angle_radians)])

    def start_mqtt(self):
        self.client.loop_start()
    
    def stop_mqtt(self):
        self.client.loop_stop()

    def stop_robot(self):
        self.pipuck.epuck.set_motor_speeds(0, 0)

    def run(self, l=50, r=50):
        self.pipuck.epuck.set_motor_speeds(l, r)

    def on_tick(self):
        for key in self.on_tick_callbacks:
            self.on_tick_callbacks[key](self)


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
    
def greetings(controller):
    for id in controller.robot_data:
        state = controller.robot_data[id]
        if controller.dist_to(state['position']) and not('greeted' in state.keys()) and not(state['greeted']):
            controller.robot_data[id]['greeted'] = True
            controller.client.publish(id, {'Greetings': 'You are greeted!'})

def collect_position_dataset(controller, n=20):
    time.sleep(5)
    data = []

    speeds = np.random.uniform(low=-1000, high=1000, size=[n, 2]).astype(int)
    speeds = np.repeat(speeds, 2, axis=0)
    speeds[1::2] *= -1
    speeds = speeds.tolist()

    sleep_times = (np.random.uniform(size=n) + 0.5).repeat(2).tolist()

    time.sleep(5)
    for i in range(n*2):
        l, r = speeds[i]
        sleep_time = sleep_times[i]
        print(f'Run with {l}, {r}')
        current_sample = {
            'start_position': controller.position.tolist(),
            'start_orientation': controller.orientation.tolist(),
            'l': l,
            'r': r,
        }
        start_time = time.time()
        controller.run(l=l, r=r)
        time.sleep(sleep_time)
        controller.stop_robot()
        current_sample['time'] = time.time() - start_time
        time.sleep(5)
        current_sample['end_position'] = controller.position.tolist()
        current_sample['end_orientation'] = controller.orientation.tolist()
        data.append(current_sample)
    
    with open("data.json", 'w') as json_file:
        json.dump(data, json_file, indent=4)


controller = RobotController('18')
controller.start_mqtt()

#controller.on_tick_callbacks['Greetings'] = greetings

try:

    collect_position_dataset(controller, n=20)
    # for i in range(100):
    #     time.sleep(0.1)
    #     if not (controller.position is None):
    #         break
    #     if i % 10 == 0:
    #         print('Waiting for position')

    # #goal = get_random_border_point()
    # goal = np.zeros(2) + 0.1
    # print(f'Goal is {goal}')

    # for i in range(10000):
    #     # if (controller.go_to_point(goal, speed=200)):
    #     #     goal = get_random_border_point()
    #     #     print(f'Goal is {goal}')

    #     if(controller.orient_towards(goal)):
    #         controller.stop_robot()
    #     #controller.go_to_point(goal)

    #     controller.on_tick()

    #     time.sleep(0.03)
    #     if i % 20 == 0:
    #         pos = controller.position
    #         print(f'Position: {pos}')
    #         print(f'Orientation: {controller.orientation}')

    #         if (pos[0] < 0) | (pos[0] > 2) | (pos[1] < 0) | (pos[1] > 1):
    #             print('out of Bounds')
    #             break
except Exception as e:
    print(e)
finally:
    controller.stop_robot()
    controller.stop_mqtt()
