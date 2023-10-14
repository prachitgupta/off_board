#! /usr/bin/env python
# Import ROS.
import rospy
# Import the API.
from gnc_controller import CONTROLLER
from Astar import astar

def main():
    # Initializing ROS node.
    rospy.init_node("drone_controller", anonymous=True)

    # Create an object for the API.
    drone = CONTROLLER()
    # Wait for FCU connection.
    drone.wait_for_connect()
    # Wait for the mode to be switched.
    drone.wait_for_start()

    # Request takeoff with an altitude of 5m.
    drone.takeoff(5)
    # Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish.
    rate = rospy.Rate(5)

    # Get Waypoints
    maze = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            [1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

    start = (0, 0)
    end = (9, 9)
    path = astar(maze, start, end)
    waypoints = path
    yaw = 0
    
    i = 0

    while i < len(waypoints):
        drone.set_waypoint(
            x=waypoints[i][0], y=waypoints[i][1], z = 5.0, psi=yaw)
        rate.sleep()
        if drone.check_waypoint_reached(0.3):
            i += 1
    # Land after all waypoints is reached.
    drone.land()
    rospy.loginfo("All waypoints reached landing now.")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()