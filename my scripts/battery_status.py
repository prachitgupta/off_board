from pymavlink import mavutil
import time

# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))
while True:
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,512,0,147,0,0,0,0,0,0) #mav_cmd request message and 147 is mavlink message seen in mavlink inspecter
    msg = the_connection.recv_match(type = "BATTERY_STATUS",blocking = True).to_dict()
    print(msg)
    P = msg["battery_remaining"]
    print(f"battery_remaining = {P}")
    #if(battery_left <= 20):
        #the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,176,0,1,9,0,0,0,0,0)
