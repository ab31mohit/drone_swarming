from pymavlink import mavutil
import time
from geometry_msgs.msg import Vector3
import rospy
import numpy as np

# drone local frame in world frame
x0 = 0
y0 = 15
z0 = 0

# # target in world frame
x = 0
y = 0
z = 0

def callback(target):
    global x, y, z
    x = target.x - x0
    y = target.y - y0
    z = target.z - z0

if __name__ == '__main__':
    rospy.init_node("drone11")
    # Start a connection listening to a UDP port
    the_connection = mavutil.mavlink_connection('udpin:localhost:14651')
    rospy.Subscriber("D11", Vector3, callback)

    # This sets the system and component ID of remote system for the link
    the_connection.wait_heartbeat()
    print("drone 11 :\n")
    print("Heartbeat from system (system %u component %u)" %
        (the_connection.target_system, the_connection.target_component))


    # arm
    print('Arming\n:')
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

    
    # set mode guided
    print('Guided:\n')
    mode = 'GUIDED'
    mode_id = the_connection.mode_mapping()[mode]
    the_connection.mav.set_mode_send(the_connection.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mode_id)

    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

    # takeoff
    print('Takeoff:\n')
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, z)

    while 1:
        pose = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        # Extract the position data from the message
        pos_x = pose.x  # meters
        pos_y = -pose.y  # meters
        pos_z = -pose.z  # meters (convert to upward positive convention)

        # Print the position data
        print(f"X: {pos_x + x0:.2f} m, Y: {pos_y + y0:.2f} m, Z: {pos_z + z0:.2f} m")

        # msg = the_connection.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
        

        if (abs(pos_z - z) <= 0.01):
            break

    
    print('Done taking off\n')
    time.sleep(5)


    # Movement along x : 
    print('Moving in x :\n')

    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,
    the_connection.target_system,
    the_connection.target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
    int(0b010111111000), x, 0, -z, 0, 0, 0, 0, 0, 0, 0, 0))

    while 1:
        pose = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        # Extract the position data from the message
        pos_x = pose.x  # meters
        pos_y = -pose.y  # meters
        pos_z = -pose.z  # meters (convert to upward positive convention)

        # Print the position data
        print(f"X: {pos_x + x0:.2f} m, Y: {pos_y + y0:.2f} m, Z: {pos_z + z0:.2f} m")

        # msg = the_connection.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
        
        if (sum(np.power([(pos_x-x),(pos_z-z)],2)) <= 0.01):
            break

    print('done moving in x\n')
    time.sleep(5)


    # Movement along y : 
    print('Movement in y :\n')

    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,
    the_connection.target_system,
    the_connection.target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
    int(0b010111111000), x, -y, -z, 0, 0, 0, 0, 0, 0, 0, 0))

    while 1:
        pose = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        # Extract the position data from the message
        pos_x = pose.x  # meters
        pos_y = -pose.y  # meters
        pos_z = -pose.z  # meters (convert to upward positive convention)

        # Print the position data
        print(f"X: {pos_x + x0:.2f} m, Y: {pos_y + y0:.2f} m, Z: {pos_z + z0:.2f} m")

        # msg = the_connection.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)

        if (sum(np.power([(pos_y-y),(pos_z-z)],2)) <= 0.01):
            break

    time.sleep(10)
    print('\nMission Completed..........\n')

    # landing

    # print('Land:\n')
    # the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
    #                                     mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)

    # while 1:
    #     pose = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    #     # Extract the position data from the message
    #     pos_x = pose.x  # meters
    #     pos_y = -pose.y  # meters
    #     pos_z = -pose.z  # meters (convert to upward positive convention)

    #     # Print the position data
    #     print(f"X: {pos_x + x0:.2f} m, Y: {pos_y + y0:.2f} m, Z: {pos_z + z0:.2f} m")

    #     # msg = the_connection.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)

    #     if (pos_z + z0 <= 0.01):
    #         break

    rospy.spin()

