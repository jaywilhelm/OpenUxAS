from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink
import struct
import array
import time
import os
import sys
class fifo(object):
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)

f = fifo()


mav = mavutil.mavlink_connection('udpin::14550')
#mav = mavutil.mavlink_connection("COM7",baud=115200)
print(mav.address)
print('wait')
mav.wait_heartbeat()
print('got beat')
#print('Set auto')
#mav.set_mode_auto()
#print('Done')

foo = array.array('B',[253, 25, 0, 0, 157, 1, 1, 22, 0, 0, 1, 0, 0, 0, 201, 1, 185, 1, 83, 89, 83, 95, 77, 67, 95, 69, 83, 84, 95, 71, 82, 79, 85, 80, 6, 248, 165])
number = foo[10:14]
numb = struct.unpack('I',number)
# mavl = mavlink.MAVLink(f)
# z=mavl.parse_char(foo)
#x = mavl.decode(foo)
#z = mavl.param_set_encode(param_id=42,param_type=1,param_value=69,target_component=0,target_system=4)
#z = mavlink.MAVLink_param_set_message(param_id=42,param_type=1,param_value=69,target_component=0,target_system=4)
#msg = z.pack(z)

#UDP
#array('B', [253, 25, 0, 0, 137, 1, 1, 22, 0, 0, 2, 0, 0, 0, 232, 1, 164, 0, 77, 65, 86, 95, 84, 89, 80, 69, 0, 0, 0, 0, 0, 0, 0, 0, 6, 223, 39])
#COM
#array('B', [253, 25, 0, 0, 88, 1, 1, 22, 0, 0, 2, 0, 0, 0, 232, 1, 164, 0, 77, 65, 86, 95, 84, 89, 80, 69, 0, 0, 0, 0, 0, 0, 0, 0, 6, 61, 77])


#MAV_COMPONENT
#MAV_COMP_ID_ALL
#mavlink.MAVLink_command_long_message(target_system=0,target_component=0,command=0,confirmation=0,param1=0,param2=0,param3=0,param4=0,param5=0,param6=0,param7=0)
#mav.mav.autopilot_version_send(capabilities=0, flight_sw_version=0, middleware_sw_version=0, os_sw_version=0, board_version=0, flight_custom_version=0, middleware_custom_version=0, os_custom_version=0, vendor_id=0, product_id=0, uid=0)
#mav.mav.message_interval_send(message_id=32, interval_us=0)
#mav.mav.message_interval_send(message_id=32, interval_us=0)
# MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES = 520 # Request autopilot capabilities
#
# mav.mav.command_long_send(target_system=1, target_component=1, command=MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, confirmation=0, param1=1, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0)
# while True:
#     msg = mav.recv_msg()
#     #print(type(msg))
#     if msg is not None:
#         if msg.name == 'AUTOPILOT_VERSION':
#             #print(msg)
#             #print ' '.join(format(x, '02x') for x in msg._msgbuf)
#             mj = (msg.flight_sw_version >> (8*3)) & 0xFF
#             md = (msg.flight_sw_version >> (8*2)) & 0xFF
#             mn = (msg.flight_sw_version >> (8*1)) & 0xFF
#
#             print('%i.%i.%i' % (mj,md,mn))
#             break
#         #else:
#             #print(msg)

target_system=1 #255
target_component=0 #0
mission_type=0
# 0     MAV_MISSION_TYPE_MISSION
# 255	MAV_MISSION_TYPE_ALL

#mav.mav.mission_clear_all_send(target_system, target_component, mission_type)
# http://qgroundcontrol.org/mavlink/waypoint_protocol
#mav.mav.mission_request_list_send(target_system, target_component, mission_type)
num_wp = -1
# while True:
#     msg = mav.recv_msg()
#     if msg is not None and msg.name is not "HIGHRES_IMU":
#         if msg.name is "MISSION_COUNT":
#             print("#WP:" + str(msg.count))
#             num_wp=msg.count
#             break

seq=0
#################mav.mav.mission_request_send(target_system, target_component,seq, mission_type)
mav.mav.mission_request_list_send(target_system, target_component, mission_type)
##############mav.mav.mission_request_partial_list_send(target_system,target_component,start_index=0,end_index=5,mission_type=0)
ignore_list = ["ATTITUDE_QUATERNION","HIGHRES_IMU","ATTITUDE","GLOBAL_POSITION_INT","LOCAL_POSITION_NED",
               "POSITION_TARGET_LOCAL_NED","HIGHRES_IMU","ATTITUDE_TARGET","POSITION_TARGET_GLOBAL_INT","VFR_HUD",
               "SYS_STATUS","BATTERY_STATUS","HEARTBEAT","GPS_RAW_INT","ALTITUDE","WIND_COV","EXTENDED_SYS_STATE",
               "ESTIMATOR_STATUS","VIBRATION","HOME_POSITION"]

MAV_MISSION_ACCEPTED = 1
while True:
    msg = mav.recv_msg()
    if msg is None:
        continue
    skip_msg=False
    for strnames in ignore_list:
        if(msg.name is strnames):
            skip_msg = True
    if(skip_msg is True):
        continue



    if msg.name is "MISSION_COUNT":
        num_wp=msg.count
        print("#WP:" + str(num_wp))
        mav.mav.mission_request_send(target_system, target_component, seq=0)#, mission_type=0)
    elif msg.name is "MISSION_ITEM":
        print(msg)
        if(msg.seq >= 0 and msg.seq+1 < num_wp):
            print("--> REQ # " + str(msg.seq))
            mav.mav.mission_request_send(target_system, target_component, seq=msg.seq+1)#, mission_type=0)
        elif(msg.seq+1 == num_wp):
            mav.mav.mission_ack_send(target_system, target_component, type=0)#, mission_type=0)#type=msg.seq, mission_type=0)

    elif msg.name is "MISSION_CURRENT":
        #print(msg)
        i=1
    else:
        print(msg)
#"COMMAND_ACK"













