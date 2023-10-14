from pymavlink import mavutil

# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,the_connection.target_system, the_connection.target_component,
                        mavutil.mavlink.MAV_FRAME_LOCAL_NED,int(0b110111111000),5,0,5,0,0,0,0,0,0,0,0)) #local pos
#msg = the_connection.recv_match(type = "COMMAND_ACK",blocking = True) debugging ke liye

#the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10,the_connection.target_system, the_connection.target_component,
                        #mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT ,int(0b110111111000),int(-35.3624700*10**7),int(149.1656135*10**7),10,0,0,0,0,0,0,0,0))

while True:
    msg = the_connection.recv_match(type = "NAV_CONTROLLER_OUTPUT",blocking = True)
    print(msg)