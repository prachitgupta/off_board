from pymavlink import mavutil
import time

# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

# Request parameter
the_connection.mav.param_request_read_send(
    the_connection.target_system, the_connection.target_component,
    b'ARMING_CHECK',
    -1
)

# Print old parameter value
message = the_connection.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
print('name: %s\tvalue: %d' %
      (message['param_id'], message['param_value']))

time.sleep(1)

the_connection.mav.param_set_send(
   the_connection.target_system, the_connection.target_component,
    b'SR0_EXTRA3',
    10,
    mavutil.mavlink.MAV_PARAM_TYPE_REAL32
)

message = the_connection.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
print('name: %s\tvalue: %d' %
      (message['param_id'], message['param_value']))

time.sleep(1)

# Request parameter value to confirm
the_connection.mav.param_request_read_send(
    the_connection.target_system, the_connection.target_component,
    b'SR0_EXTRA3',
    -1
)

# Print new value in RAM
message = the_connection.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
print('name: %s\tvalue: %d' %
      (message['param_id'], message['param_value']))

