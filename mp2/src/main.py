import rospy
import numpy as np
import argparse

from gazebo_msgs.msg import  ModelState
from controller import vehicleController
import time
from waypoint_list import WayPoints
import matplotlib.pyplot as plt
from util import euler_to_quaternion, quaternion_to_euler

def run_model():
    rospy.init_node("model_dynamics")
    controller = vehicleController()

    waypoints = WayPoints()
    pos_list = waypoints.getWayPoints()
    pos_idx = 1

    target_x, target_y = pos_list[pos_idx]

    def shutdown():
        """Stop the car when this ROS node shuts down"""
        controller.stop()
        rospy.loginfo("Stop the car")

    rospy.on_shutdown(shutdown)

    rate = rospy.Rate(100)  # 100 Hz
    rospy.sleep(0.0)
    start_time = rospy.Time.now()
    prev_wp_time = start_time

    positions = np.array([])

    prev_vel = 0
    acceleration_data = []
    position_data = []
    velocity_data = []

    while not rospy.is_shutdown():
        rate.sleep()  # Wait a while before trying to get a new state

        # Get the current position and orientation of the vehicle
        currState =  controller.getModelState()
        
        if not currState.success:
            continue
        
        pos_x, pos_y, vel, yaw = controller.extract_vehicle_info(currState)
        position_data.append([pos_x, pos_y])

        accel = (vel - prev_vel) * 100
        acceleration_data.append(accel)
        velocity_data.append(vel)
        prev_vel = vel
        # print(f"Accel: {accel}, Current Velocity: {vel}, Previous Velocity: {prev_vel}")

        # Compute relative position between vehicle and waypoints
        distToTargetX = abs(target_x - currState.pose.position.x)
        distToTargetY = abs(target_y - currState.pose.position.y)

        cur_time = rospy.Time.now()
        if (cur_time - prev_wp_time).to_sec() > 4:
            print(f"failure to reach {pos_idx}-th waypoint in time")
            return False, pos_idx, (cur_time - start_time).to_sec()

        if (distToTargetX < 2 and distToTargetY < 2):
            # If the vehicle is close to the waypoint, move to the next waypoint
            prev_pos_idx = pos_idx
            pos_idx = pos_idx+1

            if pos_idx == len(pos_list): #Reached all the waypoints
                print("Reached all the waypoints")
                total_time = (cur_time - start_time).to_sec()
                print(total_time)

                plt.figure("Position Plot")
                position_data = np.array(position_data)
                plt.grid(True)
                plt.plot(position_data[:, 0], position_data[:, 1],'o', label="X-Y trajectory")
                
                plt.figure("Acceleration vs Time")
                x = np.arange(len(acceleration_data))
                plt.plot(x, acceleration_data)
                plt.ylabel("Acceleration (Meters/second^2)")
                plt.xlabel("Time (seconds)")
                
                plt.figure("Velocity vs Time")
                x = np.arange(len(velocity_data))
                plt.plot(x, velocity_data)
                plt.ylabel("Velocity (meters/second)")
                plt.xlabel("Time (seconds)")
                
                plt.figure("Velocity Error vs Time")
                velocity_errors = controller.getVelocityErrors()
                x = np.arange(len(velocity_errors))
                plt.plot(x, velocity_errors)
                plt.ylabel("Velocity (meters/second)")              
                plt.xlabel("Time (seconds)")
                

                plt.show()
                return True, pos_idx, total_time

            target_x, target_y = pos_list[pos_idx]

            time_taken = (cur_time- prev_wp_time).to_sec()
            prev_wp_time = cur_time
            print(f"Time Taken: {round(time_taken, 2)}", "reached",pos_list[prev_pos_idx][0],pos_list[prev_pos_idx][1],"next",pos_list[pos_idx][0],pos_list[pos_idx][1])

        controller.execute(currState, [target_x, target_y], pos_list[pos_idx:])

if __name__ == "__main__":
    try:
        status, num_waypoints, time_taken = run_model()
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down")
