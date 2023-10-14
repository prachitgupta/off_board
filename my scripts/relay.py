from pymavlink import mavutil
import time

# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('/dev/ttyUSB0:57600',baud = 115200)

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))
for i in range(1,4):
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,mavutil.mavlink.MAV_CMD_DO_SET_RELAY,0,1,0,0,0,0,0,0)
    time.sleep(3)
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,mavutil.mavlink.MAV_CMD_DO_SET_RELAY,0,0,0,0,0,0,0,0)
msg = the_connection.recv_match(type = "COMMAND_ACK",blocking = True)
print(msg)