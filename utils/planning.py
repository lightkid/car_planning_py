import numpy as np
import collections

import gridmap

VehicleState = collections.namedtuple('VehicleState', 'x y heading v')
VehicleCmd = collections.namedtuple('VehicleCmd', 'a steer')

# np.array(4, dtype = float)

# class VehicleState:
#     def __init__(self, x=0, y=0, heading=0, v=0):
#         self.x = x
#         self.y = y
#         self.heading = heading
#         self.v = v
    
#     def __str__(self):
#         return "vehicle state: x: {:f}, y: {:f}, heading: {:f}, v: {:f}".\
#             format(self.x, self.y, self.heading, self.v)

# class VehicleCmd:
#     def __init__(self, a=0, steer=0):
#         self.a = a
#         self.steer = steer

class VehicleModel:
    def __init__(self, x=0, y=0, heading=0, v=0, a=0, steer=0, timestamp=0, length=2.85, width=1.984):
        self.x = x
        self.y = y
        self.heading = heading
        self.v = v
        self.a = a
        self.steer = steer
        self.timestamp = timestamp

        self.length = length
        self.width = width
    
    def __str__(self):
        return "vehicle state: x: {:f}, y: {:f}, heading: {:f}, v: {:f}".\
            format(self.x, self.y, self.heading, self.v)

    def dot(self, state, cmd):
        """
        state: x, y, heading, v
        cmd:   a, steer
        """
        dot_s = np.zeros(4, dtype = float)
        dot_s[0] = state[3] * np.cos(state[2])
        dot_s[1] = state[3] * np.sin(state[2])
        dot_s[2] = state[3] * np.tan(cmd[1]) / self.length
        dot_s[3] = cmd[0]
        return dot_s
        
    def forward_euler(self, state, cmd, dt):
        """
        back rear
        forward euler
        """
        # dot_x = state.v * np.cos(state.heading)
        # dot_y = state.v * np.sin(state.heading)
        # dot_heading = state.v * np.tan(cmd.steer) / self.length
        # dot_v = cmd.a
        dot_s = self.dot(state, cmd)
        new_s = state + dot_s * dt
        # x = state.x + dot_s.x * dt 
        # y = state.y + dot_s.y * dt
        # heading = state.heading + dot_s.heading * dt
        # v = state.v + dot_s.v * dt
        return new_s
    
    def forward_RK4(self, state, cmd, dt):
        k1 = self.dot(state, cmd)
        s1 = state + k1 * dt * 0.5
        k2 = self.dot(s1, cmd)
        s2 = state + k2 * dt * 0.5
        k3 = self.dot(s2, cmd)
        s3 = state + k3 * dt
        k4 = self.dot(s3, cmd)
        new_s = state + (k1 + 2 * k2 + 2 * k3 + k4) * dt / 6
        return new_s
    
    def updateRK4(self, cmd, dt):
        state = self.forward_RK4([self.x, self.y, self.heading, self.v], [cmd.a, cmd.steer], dt)
        self.x = state[0]
        self.y = state[1]
        self.heading = state[2]
        self.v = state[3]
        self.a = cmd.a
        self.steer = cmd.steer
        self.timestamp += dt
    
    def updateEuler(self, cmd, dt):
        state = self.forward_euler([self.x, self.y, self.heading, self.v], [cmd.a, cmd.steer], dt)
        self.x = state[0]
        self.y = state[1]
        self.heading = state[2]
        self.v = state[3]
        self.a = cmd.a
        self.steer = cmd.steer
        self.timestamp += dt
    
    def get_state(self):
        return [self.x, self.y, self.heading, self.v, self.a, self.steer]

