"""
PID Controller

components:
    follow attitude commands
    gps commands and yaw
    waypoint following
"""
import numpy as np
from frame_utils import euler2RM

DRONE_MASS_KG = 0.5
GRAVITY = -9.81
MOI = np.array([0.005, 0.005, 0.01])
MAX_THRUST = 10.0
MAX_TORQUE = 1.0

class NonlinearController(object):

    def __init__(self,
                k_p_z=81.0,
                k_d_z=16.2,
                k_p_x=0.0,
                k_d_x=0.0,
                k_p_y=0.0,
                k_d_y=0.0,
                k_p_roll=3.0,
                k_p_pitch=3.0,
                k_p_yaw=6.0,
                k_p_p=1.0,
                k_p_q=1.0,
                k_p_r=0.8):
        
        self.k_p_z = k_p_z
        self.k_d_z = k_d_z
        self.k_p_x = k_p_x
        self.k_d_x = k_d_x
        self.k_p_y = k_p_y
        self.k_d_y = k_d_y
        self.k_p_roll = k_p_roll
        self.k_p_pitch = k_p_pitch
        self.k_p_yaw = k_p_yaw
        self.k_p_p =  k_p_p
        self.k_p_q = k_p_q
        self.k_p_r = k_p_r
        
        self.m = DRONE_MASS_KG
        self.g = GRAVITY
        self.Ixx = MOI[0]
        self.Iyy = MOI[1]
        self.Izz = MOI[2]
        self.max_thrust = MAX_THRUST
        self.max_torque = MAX_TORQUE
        

    def trajectory_control(self, position_trajectory, yaw_trajectory, time_trajectory, current_time):
        """Generate a commanded position, velocity and yaw based on the trajectory
        
        Args:
            position_trajectory: list of 3-element numpy arrays, NED positions
            yaw_trajectory: list yaw commands in radians
            time_trajectory: list of times (in seconds) that correspond to the position and yaw commands
            current_time: float corresponding to the current time in seconds
            
        Returns: tuple (commanded position, commanded velocity, commanded yaw)
                
        """

        ind_min = np.argmin(np.abs(np.array(time_trajectory) - current_time))
        time_ref = time_trajectory[ind_min]
        
        
        if current_time < time_ref:
            position0 = position_trajectory[ind_min - 1]
            position1 = position_trajectory[ind_min]
            
            time0 = time_trajectory[ind_min - 1]
            time1 = time_trajectory[ind_min]
            yaw_cmd = yaw_trajectory[ind_min - 1]
            
        else:
            yaw_cmd = yaw_trajectory[ind_min]
            if ind_min >= len(position_trajectory) - 1:
                position0 = position_trajectory[ind_min]
                position1 = position_trajectory[ind_min]
                
                time0 = 0.0
                time1 = 1.0
            else:

                position0 = position_trajectory[ind_min]
                position1 = position_trajectory[ind_min + 1]
                time0 = time_trajectory[ind_min]
                time1 = time_trajectory[ind_min + 1]
            
        position_cmd = (position1 - position0) * \
                        (current_time - time0) / (time1 - time0) + position0
        velocity_cmd = (position1 - position0) / (time1 - time0)
        
        
        return (position_cmd, velocity_cmd, yaw_cmd)
    
    def lateral_position_control(self, local_position_cmd, local_velocity_cmd, local_position, local_velocity,
                               acceleration_ff = np.array([0.0, 0.0])):
        """Generate horizontal acceleration commands for the vehicle in the local frame

        Args:
            local_position_cmd: desired 2D position in local frame [north, east]
            local_velocity_cmd: desired 2D velocity in local frame [north_velocity, east_velocity]
            local_position: vehicle position in the local frame [north, east]
            local_velocity: vehicle velocity in the local frame [north_velocity, east_velocity]
            acceleration_cmd: feedforward acceleration command
            
        Returns: desired vehicle 2D acceleration in the local frame [north, east]
        """
        
#        return np.array([0.0, 0.0]) 
       
        error_x = local_position_cmd[0] - local_position[0]
        error_x_dot = local_velocity_cmd[0] - local_velocity[0]
        x_dot_dot_ff = acceleration_ff[0]
        
        x_dot_dot_cmd = self.k_p_x * error_x + self.k_d_x * error_x_dot + x_dot_dot_ff
        
        error_y = local_position_cmd[1] - local_position[1]
        error_y_dot = local_velocity_cmd[1] - local_velocity[1]
        y_dot_dot_ff = acceleration_ff[1]
        
        y_dot_dot_cmd = self.k_p_y * error_y + self.k_d_y * error_y_dot + y_dot_dot_ff 
        
        return np.array([x_dot_dot_cmd, y_dot_dot_cmd])
    
    def altitude_control(self, altitude_cmd, vertical_velocity_cmd, altitude, vertical_velocity, attitude, acceleration_ff=0.0):
        """Generate vertical acceleration (thrust) command

        Args:
            altitude_cmd: desired vertical position (+up)
            vertical_velocity_cmd: desired vertical velocity (+up)
            altitude: vehicle vertical position (+up)
            vertical_velocity: vehicle vertical velocity (+up)
            attitude: the vehicle's current attitude, 3 element numpy array (roll, pitch, yaw) in radians
            acceleration_ff: feedforward acceleration command (+up)
            
        Returns: thrust command for the vehicle (+up)
        """
        
#        return 0.0
        
        roll = attitude[0]
        pitch = attitude[1]
        yaw = attitude[2]
        rot_mat = euler2RM(roll, pitch, yaw)
        
        error_z = altitude_cmd - altitude
        error_z_dot = vertical_velocity_cmd - vertical_velocity
        z_dot_dot_ff = acceleration_ff
        
        ubar_1 = self.k_p_z * error_z + self.k_d_z * error_z_dot + z_dot_dot_ff
        
        c = (ubar_1 - self.g) / rot_mat[2,2]
        
        thrust = self.m * c
        
        thrust = np.clip(thrust, 0.05, self.max_thrust)          
        
        return thrust
        
    
    def roll_pitch_controller(self, acceleration_cmd, attitude, thrust_cmd):
        """ Generate the rollrate and pitchrate commands in the body frame
        
        Args:
            target_acceleration: 2-element numpy array (north_acceleration_cmd,east_acceleration_cmd) in m/s^2
            attitude: 3-element numpy array (roll, pitch, yaw) in radians
            thrust_cmd: vehicle thruts command in Newton
            
        Returns: 2-element numpy array, desired rollrate (p) and pitchrate (q) commands in radians/s
        """
       
#        return np.array([0.0, 0.0]) 
        
        if thrust_cmd > 0:
            c = thrust_cmd / self.m
            
            roll = attitude[0]
            pitch = attitude[1]
            yaw = attitude[2]
            rot_mat = euler2RM(roll, pitch, yaw) 
            
            b_x_c = acceleration_cmd[0] / c
            b_y_c = acceleration_cmd[1] / c 
        
            b_x_a = rot_mat[0, 2]
            b_y_a = rot_mat[1, 2]
    
            error_b_x = (b_x_c - b_x_a)
            error_b_y = (b_y_c - b_y_a)
            
            b_c_x_dot = self.k_p_roll * error_b_x
            b_c_y_dot = self.k_p_pitch * error_b_y
        
            b_c_dot = np.array([b_c_x_dot, b_c_y_dot])
            
            A = np.array([[rot_mat[1, 0], - rot_mat[0, 0]],
                          [rot_mat[1, 1], - rot_mat[0, 1]]])        
            
            roll_pitch_cmd = (1/rot_mat[2, 2]) * np.matmul(A, b_c_dot)    
            
            roll_cmd = roll_pitch_cmd[0]
            pitch_cmd = roll_pitch_cmd[1]
        
        else:
            roll_cmd = 0
            pitch_cmd = 0
            
        return np.array([roll_cmd, pitch_cmd])
    
    
    
    def body_rate_control(self, body_rate_cmd, body_rate):
        """ Generate the roll, pitch, yaw moment commands in the body frame
        
        Args:
            body_rate_cmd: 3-element numpy array (p_cmd,q_cmd,r_cmd) in radians/second^2
            body_rate: 3-element numpy array (p,q,r) in radians/second^2
            
        Returns: 3-element numpy array, desired roll moment, pitch moment, and yaw moment commands in Newtons*meters
        """

#        return np.array([0.0, 0.0, 0.0])
    
        error_p = body_rate_cmd[0] - body_rate[0]
        error_q = body_rate_cmd[0] - body_rate[0]
        error_r = body_rate_cmd[0] - body_rate[0]
                
        ubar_p = self.k_p_p * error_p
        ubar_q = self.k_p_q * error_q
        ubar_r = self.k_p_r * error_r
        
        roll_moment = self.Ixx * ubar_p
        pitch_moment = self.Iyy * ubar_q
        yaw_moment = self.Izz * ubar_r
        
        roll_moment = np.clip(roll_moment, -self.max_torque, self.max_torque)
        pitch_moment = np.clip(pitch_moment, -self.max_torque, self.max_torque)
        yaw_moment = np.clip(yaw_moment, -self.max_torque, self.max_torque)
        
        return np.array([roll_moment, pitch_moment, yaw_moment])

    
    def yaw_control(self, yaw_cmd, yaw):
        """ Generate the target yawrate
        
        Args:
            yaw_cmd: desired vehicle yaw in radians
            yaw: vehicle yaw in radians
        
        Returns: target yawrate in radians/sec
        """
        
#        return 0.0   

        error_yaw = yaw_cmd - yaw
        
        r_cmd = self.k_p_yaw * error_yaw
        
        return r_cmd

    
