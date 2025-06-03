import numpy as np
import paho.mqtt.client as mqtt
import json
import time
from pipuck.pipuck import PiPuck
import numpy as np
import math

class Instructions(np.ndarray):
    def __new__(cls, input_array):
        if input_array.shape[1] != 3:
            raise ValueError("Input array must have 5 columns.")
        assert input_array.ndim == 2
        return np.asarray(input_array).view(cls)

    @property
    def left(self):
        return self[:, 0]

    @property
    def right(self):
        return self[:, 1]
    
    @property
    def time(self):
        return self[:, 2]

    @classmethod
    def from_components(cls, left, right, time):
        combined_array = np.hstack((left.reshape(-1, 1), right.reshape(-1, 1), time.reshape(-1, 1)))
        return cls(combined_array)

class Controller:
    def __init__(self, position, orientation, use_map_coordinates=True):
        self.theta_offset = 0.528604846447706223 if use_map_coordinates else 0
        self.wheel_size = 0.0008425527811050415
        self.axel_length = 0.34902531653642654
        self.alpha = 0.5072705149650574
        self.beta = 0.9661945104598999

        self._position = self._to_vector(position, size=2)
        self._orientation = self._to_vector(orientation)

    def _to_vector(self, v, size=1):
        v = np.asarray(v)
        if size == 1:
            if v.ndim == 0:
                return v.reshape([1])
            else:
                assert v.ndim == 1
                return v
        else:
            if v.ndim == 1:
                v = v[np.newaxis, :]
        assert v.ndim == 2
        assert v.shape[1] == size
        return v
    
    def set_position(self, position):
        self._position = self._to_vector(position, size=2)
    
    def set_orientation(self, orientation):
        self._orientation = np.mod(self._to_vector(orientation), 2 * np.pi)

    def convex_combination_of_angles(self, theta1, theta2, alpha):
        diff = theta2 - theta1
        diff = (diff + np.pi) % (2 * np.pi) - np.pi
        combined_angle = theta1 + alpha * diff
        return combined_angle

    def predict_from_instruction(self, instr, position_measurement=None, orientation_measurement=None):
        return self.get_predicted_position(
            instr.left,
            instr.right,
            instr.time,
            position_measurement=position_measurement,
            orientation_measurement=orientation_measurement
        )
     
    def get_predicted_position(
        self,
        left_speed,
        right_speed,
        time,
        position_measurement=None,
        orientation_measurement=None,
    ):
        # Boilerplate

        position_estimate = self._position
        orientation_estimate = self._orientation
        if position_measurement is None:
            position_measurement = position_estimate
        if orientation_measurement is None:
            orientation_measurement = orientation_estimate

        left_speed = self._to_vector(left_speed)
        right_speed = self._to_vector(right_speed)
        time = self._to_vector(time)

        position_estimate = self._to_vector(position_estimate, size=2)
        orientation_estimate = self._to_vector(orientation_estimate, size=1)
        position_measurement = self._to_vector(position_measurement, size=2)
        orientation_measurement = self._to_vector(orientation_measurement, size=1)

        left_speed, right_speed, time, orientation_estimate, orientation_measurement = np.broadcast_arrays(
            left_speed, right_speed, time, orientation_estimate, orientation_measurement)
        position_shape = (left_speed.shape[0], 2)
        # print(position_estimate.shape)
        position_estimate = np.broadcast_to(position_estimate, position_shape)
        position_measurement = np.broadcast_to(position_measurement, position_shape)

        # Computations
        l_m_r = left_speed - right_speed
        delta_t = - self.wheel_size * l_m_r * time / self.axel_length
        radius = np.sign(l_m_r) * self.axel_length * (left_speed + right_speed) / (4 * np.pi * np.abs(l_m_r) + 1e-5)
        delta_p = np.stack([np.sin(delta_t), (1 - np.cos(delta_t))], axis=1) * radius[:, np.newaxis]
        delta_p_equal_speed = (self.wheel_size * (left_speed + right_speed) * time / 2)[:, np.newaxis] * [[1, 0]]
        delta_p = np.where(np.abs(l_m_r[:, np.newaxis]) < 1e-5, delta_p_equal_speed, delta_p)

        delta_p = delta_p * [[1, 1]]

        orientation = self.convex_combination_of_angles(orientation_estimate, orientation_measurement, self.beta) + delta_t
        self.set_orientation(orientation)

        sin = np.sin(self.theta_offset * np.pi - orientation)
        cos = np.cos(self.theta_offset * np.pi - orientation)
        m = np.stack([np.stack([cos, -sin], axis=1),
                np.stack([sin, cos], axis=1)], axis=1)
        
        position = self.alpha * position_estimate + (1 - self.alpha) * position_measurement + (m @ delta_p[:, :, np.newaxis])[..., 0]
        self.set_position(position)

    def get_control_instructions(
            self,
            position=None,
            orientation=None, 
            target_position=None,
            target_orientation=None,
            throttle=1,
            curve_discount=0.8,
            force=None
        ):

        # Boilerplate
        position = self._position if position is None else position
        orientation = self._orientation if orientation is None else orientation
        target_position = self._position if target_position is None else target_position

        position = self._to_vector(position, size=2)
        orientation = self._to_vector(orientation)
        target_position = self._to_vector(target_position, size=2)
        if not (target_orientation is None):
            target_orientation = self._to_vector(target_orientation)
        throttle = self._to_vector(throttle)

        if target_orientation is None:
            n_trajectories = max([x.shape[0] for x in [position, orientation, target_position, throttle]])
        else:
            n_trajectories = max([x.shape[0] for x in [position, orientation, target_position, throttle, target_orientation]])
        
        position = np.broadcast_to(position, (n_trajectories, 2))
        target_position = np.broadcast_to(target_position, (n_trajectories, 2))
        orientation = np.broadcast_to(orientation, (n_trajectories,))
        throttle = np.broadcast_to(throttle, (n_trajectories,))
        if not (target_orientation is None):
            target_orientation = np.broadcast_to(target_orientation, (n_trajectories,))

        # Computations
        delta = target_position - position
        driving_direction = np.atan2(delta[:, 0], delta[:, 1])
        direct_path_instructions = [
            self.get_orientation_instruction(target_orientation=driving_direction, orientation=orientation, throttle=throttle),
            self.get_linear_driving_instruction(target_position=target_position, position=position, throttle=throttle)]
        if target_orientation:
            direct_path_instructions.append(self.get_orientation_instruction(orientation, target_orientation, throttle=throttle))

        end_direction = driving_direction * 2
        curved_path_instructions = [self.get_curved_driving_instruction(position, target_position, throttle=throttle)]
        if target_orientation:
            curved_path_instructions.append(self.get_orientation_instruction(end_direction, target_orientation, throttle=throttle))

        if force == 'direct':
            return direct_path_instructions
        elif force == 'curve':
            return curved_path_instructions

        direct_time = sum([inst.time for inst in direct_path_instructions])
        curved_time = sum([inst.time for inst in curved_path_instructions])

        if direct_time > curved_time * curve_discount:
            return direct_path_instructions
        else:
            return curved_path_instructions

    def get_orientation_instruction(self, target_orientation, orientation=None, throttle=1):
        #todo turn what direction?
        if orientation is None:
            orientation = self._orientation
        throttle = np.clip(throttle, a_min=0, a_max=1)
        delta_t = -(target_orientation + orientation - np.pi * self.theta_offset)
        max_wheel_speed = self._to_vector(1000 * throttle)
        wheel_distance = delta_t * self.axel_length / (self.wheel_size * 2)
        time = wheel_distance / max_wheel_speed
        time, max_wheel_speed = np.broadcast_arrays(time, max_wheel_speed)

        return Instructions.from_components(left=-max_wheel_speed, right=max_wheel_speed, time=time)
    
    def get_linear_driving_instruction(self, target_position, position=None, throttle=1):
        if position is None:
            position = self._position
        throttle = np.clip(throttle, a_min=0, a_max=1)
        distance = np.linalg.norm(position - target_position, axis=1)
        max_wheel_speed = 1000 * throttle
        wheel_distance = distance / self.wheel_size
        time = wheel_distance / max_wheel_speed

        time, max_wheel_speed = np.broadcast_arrays(time, max_wheel_speed)
        return Instructions.from_components(left=max_wheel_speed, right=max_wheel_speed, time=time)
    
    def get_curved_driving_instruction(self, target_position, position=None, throttle=1):
        if position is None:
            position = self._position
        throttle = np.clip(throttle, a_min=0, a_max=1)
        max_wheel_speed = 1000 * throttle[:, np.newaxis]
        delta = target_position - position

        theta = self._orientation - self.theta_offset * np.pi
        rotation_matrix = np.stack([
            np.stack([np.cos(theta), -np.sin(theta)], axis=1),
            np.stack([np.sin(theta), np.cos(theta)], axis=1)
        ], axis=2)
        delta = (rotation_matrix @ delta[:, :, np.newaxis])[..., 0]

        delta = delta + np.ones_like(delta) * 1e-5 * (np.abs(delta) < 5e-6)
        delta_x = delta[:, 0]
        delta_y = delta[:, 1]
        delta_t = np.mod(2 * np.atan(delta_x / delta_y) - self.theta_offset * np.pi * 0.01 + np.pi, 2 * np.pi) - np.pi
        radius = -np.pi * 4 * delta_y / np.sin(delta_t)

        plus = delta_t * radius * self.axel_length / self.wheel_size
        minus = delta_t * self.axel_length / self.wheel_size

        wheel_distance = np.stack([plus + minus, plus - minus], axis=1) / 2
        max_wheel_dist = np.abs(wheel_distance).max(axis=1, keepdims=True)

        wheel_speed = max_wheel_speed * wheel_distance / max_wheel_dist
        time = max_wheel_dist / max_wheel_speed
        
        return Instructions.from_components(left=-wheel_speed[:, 1], right=-wheel_speed[:, 0], time=time)

class Util:
    @staticmethod
    def on_connect(client, userdata, flags, rc):
        print("Connected with result code " + str(rc))
        client.subscribe('robot_pos/all')
        client.subscribe(f'robot/{id}')

    @staticmethod
    def on_message(client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            if msg.topic == 'robot_pos/all':
                position = np.asarray(data[id]['position'])
                orientation = data[id]['angle']
            elif msg.topic == f'robot/{id}':
                print('Got message')
                print(data)
        except json.JSONDecodeError:
            print(f'invalid json: {msg.payload}')

    @staticmethod
    def start_mqtt(client):
        client.loop_start()

    @staticmethod
    def stop_mqtt(client):
        client.loop_stop()

    @staticmethod
    def stop_robot(pipuck):
        pipuck.epuck.set_motor_speeds(0, 0)

    @staticmethod
    def run(pipuck, left, right):
        pipuck.epuck.set_motor_speeds(left, left)

id = '33'
position = None
orientation = None

pipuck = PiPuck(epuck_version=2)

client = mqtt.Client()
client.on_connect = Util.on_connect
client.on_message = Util.on_message

Broker = "192.168.178.56"  # Replace with your broker address
Port = 1883 # standard MQTT port
client.connect(Broker, Port, 60)

Util.start_mqtt(client)

ctrl = Controller([0, 0], 0)
target = [0.5, 0.5]
cycle_time = 0.1

try:
    for i in range(100):
        time.sleep(0.1)
        if not (position is None):
            break
        if i % 10 == 0:
            print('Waiting for position')
        ctrl.set_position(position)
        ctrl.set_orientation(orientation)

    instr = ctrl.get_control_instructions(target_position=target)
    instr[:, 2] = cycle_time
    for i in range(500):
        cycle_start = time.time()
        Util.run(pipuck, instr.left[0], instr.right[0])
        ctrl.predict_from_instruction(instr, position_measurement=position, orientation_measurement=orientation)
        instr = ctrl.get_control_instructions(target_position=target)
        instr[:, 2] = cycle_time

        sleep_time = cycle_time - (time.time() - cycle_start)
        if sleep_time < 0:
            print('Err: sleep time too short')
            sleep_time = 0
        time.sleep(0)

        if i % 20 == 0:
            pos = position
            # print(f'Position: {pos}')
            # print(f'Orientation: {controller.orientation}')

            if (pos[0] < 0) | (pos[0] > 2) | (pos[1] < 0) | (pos[1] > 1):
                break
except Exception as e:
    print(e)

Util.stop_robot(pipuck)
Util.stop_mqtt(client)