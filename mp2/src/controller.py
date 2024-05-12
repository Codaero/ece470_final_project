import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateResponse
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import Float32MultiArray
import math
from util import euler_to_quaternion, quaternion_to_euler
import time

PLOT = False

CONTROLLER = 1 # 0 for regular and 1 for PID

interp_waypoints = []
interp_waypoints_index = 0


LD_SPEED_FACTOR = 0.8
MIN_LD = 1
MAX_LD = 20
MAX_VEL = 10


class PID:
    """ Simple PID controller """

    def __init__(self, kp, ki, kd):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.integral = 0
        self.prev_error = 0

    def update(self, error, dt):
        """ Update PID control output based on error and elapsed time dt """
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class vehicleController():
    # static variable

    def __init__(self):
        # Publisher to publish the control input to the vehicle model
        self.controlPub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size = 1)
        self.prev_vel = 0
        self.L = 0.5 # Wheelbase, can be get from gem_control.py
        self.log_acceleration = True
        self.pid = PID(0.2, 0.2, 0.2)
        self.velocity_errors = []
        

    def getModelState(self):
        # Get the current state of the vehicle
        # Input: None
        # Output: ModelState, the state of the vehicle, contain the
        #   position, orientation, linear velocity, angular velocity
        #   of the vehicle
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp = serviceResponse(model_name='ackermann_vehicle')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
            resp = GetModelStateResponse()
            resp.success = False
        return resp
    
    # Helper function
    # Gets alpha between the current position and the target (rad?)
    def get_alpha(self, pos, target, curr_yaw):
        v1 = np.array([np.cos(curr_yaw), np.sin(curr_yaw)]) # heading vector (global frame)
        v2 = target - pos # vector to target (global frame)
        v2 = v2 / np.linalg.norm(v2)

        matrix = np.vstack((v1,v2))

        dot = np.dot(v1,v2) # dot product proportional to the cosine of vector
        det = np.linalg.det(matrix) # determinant (A.K.A magnitude of cross product) proportional to sine of vector
        return np.arctan2(det, dot) # find signed angle between heading vector and position_to_target vector 


    # Tasks 1: Read the documentation https://docs.ros.org/en/fuerte/api/gazebo/html/msg/ModelState.html
    #       and extract yaw, velocity, vehicle_position_x, vehicle_position_y
    # Hint: you may use the the helper function(quaternion_to_euler()) we provide to convert from quaternion to euler
    def extract_vehicle_info(self, currentPose):

        ####################### TODO: Your TASK 1 code starts Here #######################
        pos_x = currentPose.pose.position.x
        pos_y = currentPose.pose.position.y

        vel_x = currentPose.twist.linear.x
        vel_y = currentPose.twist.linear.y

        vel = np.linalg.norm(np.array([vel_x, vel_y]))

        quat_x = currentPose.pose.orientation.x
        quat_y = currentPose.pose.orientation.y
        quat_z = currentPose.pose.orientation.z
        quat_w = currentPose.pose.orientation.w

        yaw = quaternion_to_euler(quat_x, quat_y, quat_z, quat_w)[2]

        ####################### TODO: Your Task 1 code ends Here #######################

        return pos_x, pos_y, vel, yaw # note that yaw is in radian

    # Task 2: Longtitudal Controller
    # Based on all unreached waypoints, and your current vehicle state, decide your velocity
    def longititudal_controller(self, curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints):
        # Controller Scheme 0
        
        target_velocity = None
        
        if CONTROLLER == 0:
            # Parameters
            C = 4 # Ved's number (keep target whithin C of current velocity)
            K = 110 * (np.pi / 24) # Initially chosen to be pi/24, but experimentally tested to fit in the right bounds

            ####################### TODO: Your TASK 2 code starts Here #######################

            # Use static look ahead distance of 2 waypoints
            if len(future_unreached_waypoints) >= 2:
                target_point = future_unreached_waypoints[1]
            else:
                target_point = future_unreached_waypoints[0] # Eliminate crash at the end of waypoints

            # Calculate the alpha angle
            curr_pos = np.array([curr_x, curr_y])
            alpha = self.get_alpha(curr_pos, target_point, curr_yaw)

            # Adjust the target velocity based on the angle (larger angle -> smaller target_velocity)
            speed_control = MAX_VEL - K * abs(alpha)
            speed_control = max(0, speed_control)
            speed_control = np.clip(speed_control, curr_vel - C, curr_vel + C) # Keep target velocity within +-C 

            target_velocity = speed_control / 3
            
        elif CONTROLLER == 1:
                
            # Controller Scheme #1
            target_velocity = 1
            velocity_error = target_velocity - curr_vel
            
            self.velocity_errors.append(velocity_error)
            
            pid_adjustment = self.pid.update(velocity_error, 0.01)
            
            adjusted_target_velocity = curr_vel + pid_adjustment
            
            adjusted_target_velocity = np.clip(adjusted_target_velocity, 0, MAX_VEL)
            
            target_velocity = adjusted_target_velocity
        
        ####################### TODO: Your TASK 2 code ends Here #######################
        return target_velocity


    # Task 3: Lateral Controller (Pure Pursuit)
    def lateral_controller(self, curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints):
        ####################### TODO: Your TASK 3 code starts Here #######################
        global interp_waypoints
        global interp_waypoints_index
        current_pos_point = np.array([curr_x, curr_y])
                
        # Set the lookahead distance to 5
        lookahead_distance = 5 

        # Locate the next target point
        local_target = target_point # handles when reaching end so we just set local_target to the last target point at the end
        for i in range(interp_waypoints_index, len(interp_waypoints)):
            search_point = interp_waypoints[i]
            dist = np.linalg.norm(search_point - current_pos_point)
            if dist > lookahead_distance:
                local_target = search_point
                interp_waypoints_index = i
                break

        # Calculate the alpha needed for the pure pursuit algorithm
        alpha = self.get_alpha(current_pos_point, local_target, curr_yaw)
        
        # the direction of the angle starts at the heading vector (positive alpha means target is left of heading)
        v2 = local_target - current_pos_point # vector to target (global frame)
        dist_to_target = np.linalg.norm(v2) # calculate lookahead distance
        target_steering = np.arctan2(2 * self.L * np.sin(alpha), dist_to_target) # calculate target steering

        ####################### TODO: Your TASK 3 code starts Here #######################
        return target_steering


    def execute(self, currentPose, target_point, future_unreached_waypoints):
        # Compute the control input to the vehicle according to the
        # current and reference pose of the vehicle
        # Input:
        #   currentPose: ModelState, the current state of the vehicle
        #   target_point: [target_x, target_y]
        #   future_unreached_waypoints: a list of future 29waypoints[[target_x, target_y]]
        # Output: None
        global interp_waypoints
        global interp_waypoints_index

        curr_x, curr_y, curr_vel, curr_yaw = self.extract_vehicle_info(currentPose)

        if len(interp_waypoints) == 0:
            global_waypoints = np.array(future_unreached_waypoints)

            starting_pos = np.array([ int(round(curr_x)) , int(round(curr_y))])
            global_waypoints = np.vstack([starting_pos, global_waypoints])
            
            delta_distances = np.sqrt(np.sum(np.diff(global_waypoints, axis=0)**2, axis = 1))
            cumulative_distances = np.insert(np.cumsum(delta_distances),  0, 0)

            t = cumulative_distances / cumulative_distances[-1]
            
            cs_x = CubicSpline(t, global_waypoints[:, 0])
            cs_y = CubicSpline(t, global_waypoints[:, 1])

            t_prime = np.linspace(0, 1, 1000)

            x_new = cs_x(t_prime)
            y_new = cs_y(t_prime)
            interp_waypoints = np.column_stack((x_new, y_new))

            if PLOT:            
                plt.figure()
                plt.plot(global_waypoints[:, 0], global_waypoints[:, 1],'o', label="Waypoints")
                plt.plot(x_new, y_new, '-', label="Interpolated Points")
                
                plt.grid(True)

        # Acceleration Profile
        # if self.log_acceleration:
        #     acceleration = (curr_vel- self.prev_vel) * 100 # Since we are running in 100Hz
        #     self.prev_vel = curr_vel
            
        #     if acceleration >= 5:
        #         print(f"ACCELERATION TOO FAST: {acceleration}")

        target_velocity = self.longititudal_controller(curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints)
        target_steering = self.lateral_controller(curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints)


        #Pack computed velocity and steering angle into Ackermann command
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = target_velocity
        newAckermannCmd.steering_angle = target_steering
        # print(newAckermannCmd)

        # Publish the computed control input to vehicle model
        self.controlPub.publish(newAckermannCmd)

    def stop(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = 0
        self.controlPub.publish(newAckermannCmd)
        
    def getVelocityErrors(self):
        return self.velocity_errors
