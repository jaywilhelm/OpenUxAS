#!/usr/bin/env python3

import zmq, sys, time, copy, re, json, argparse
import xml.dom.minidom
from xml.dom.minidom import parse, parseString

# Include UAV modules for collision avoidance (https://github.com/JTEnglish/UAVHeading-CollisionAvoidance)
import sys
sys.path.append('/home/jenglish/research/UAVHeading-CollisionAvoidance/src')
from UAVHeading import UAVHeading
from TerminalColors import TerminalColors as TC

# Include LMCP Python files
sys.path.append('../../src/LMCP/py')
from lmcp import LMCPFactory
from afrl.cmasi import *

def makeUAV(factory, socket, client_id, id, position, heading, speed, label):
    ###################################################################################################################
    # Create an air vehicle by loading base configuration and state information from XML files.
    # Modify IDs and PayloadIDs to be unique.
    # -----------------------------------------------------------------------------------------------------------------

    cc3_lmcp_dom = parse("MessageLibrary/CameraConfiguration_WavelengthAnyAll_FovDiscrete_PayloadID-10001.xml")
    obj_list = factory.unpackFromXMLNode(cc3_lmcp_dom)
    cc3_obj = obj_list[0]
    cc3_obj.set_PayloadID(id + 10000)
    gc3_lmcp_dom = parse("MessageLibrary/GimbalConfiguration_ClampedFalse_PayloadID-1001.xml")
    obj_list = factory.unpackFromXMLNode(gc3_lmcp_dom)
    gc3_obj = obj_list[0]
    gc3_obj.set_PayloadID(id + 20000)
    gc3_obj.ContainedPayloadList = [id + 10000]
    avc3_lmcp_dom = parse("MessageLibrary/AirVehicleConfiguration_FlightProfileAll_LoiterAll_ID-1.xml")
    obj_list = factory.unpackFromXMLNode(avc3_lmcp_dom)
    avc3_obj = obj_list[0]
    avc3_obj.set_ID(id)
    avc3_obj.set_Label(label)
    avc3_obj.set_MaximumSpeed(speed + 100)
    avc3_obj.set_NominalSpeed(speed)
    avc3_obj.PayloadConfigurationList = [cc3_obj, gc3_obj]

    send_to_amase(avc3_obj, socket, client_id)

    cs3_lmcp_dom = parse("MessageLibrary/CameraState_PayloadID-10001.xml")
    obj_list = factory.unpackFromXMLNode(cs3_lmcp_dom)
    cs3_obj = obj_list[0]
    cs3_obj.set_PayloadID(id + 10000)
    gs3_lmcp_dom = parse("MessageLibrary/GimbalState_PayloadID-1001.xml")
    obj_list = factory.unpackFromXMLNode(gs3_lmcp_dom)
    gs3_obj = obj_list[0]
    gs3_obj.set_PayloadID(id + 20000)
    avs3_lmcp_dom = parse("MessageLibrary/AirVehicleState_FlightProfileAll_LoiterAll_ID-1.xml")
    obj_list = factory.unpackFromXMLNode(avs3_lmcp_dom)
    avs3_obj = obj_list[0]
    avs3_obj.set_Heading(heading)
    avs3_obj.set_ID(id)
    avs3_obj.set_Airspeed(speed)
    latitude = position[0]
    longitude = position[1]
    avs3_obj.get_Location().set_Longitude(longitude)
    avs3_obj.get_Location().set_Latitude(latitude)
    avs3_obj.PayloadStateList = [cs3_obj, gs3_obj]

    print(avs3_obj.__dict__)

    send_to_amase(avs3_obj, socket, client_id)

def create_and_run_amase_scenario():
    ###############################
    # ArgParse for Config Files
    # -----------------------------
    ap = argparse.ArgumentParser()
    ap.add_argument("-cfg", "--config", required=True,
        help="path to the input JSON config file")
    args = vars(ap.parse_args())

    ###################################################################################################################
    # Prepare a ZeroMQ context and socket, then connect to 5555. This is the port AMASE uses by convention.
    # The actual port is set by an XML element in the file Plugins.xml, whose location is specified with the AMASE
    # option --config <config_directory> when starting up AMASE. This XML element has the form:
    # <Plugin Class="avtas.amase.network.TcpServer">
    #    <TCPServer Port="5555"/>
    # </Plugin>
    # -----------------------------------------------------------------------------------------------------------------
    context = zmq.Context()
    socket = context.socket(zmq.STREAM)
    socket.connect("tcp://127.0.0.1:5555")
    # Store the client ID for this socket
    client_id, message = socket.recv_multipart()

    ###################################################################################################################
    # Initialize an LMCPFactory for creating LMCP messages
    # -----------------------------------------------------------------------------------------------------------------
    factory = LMCPFactory.LMCPFactory()
    

    ###############################
    # Read Input Config from JSON
    # -----------------------------
    with open(args['config']) as f:
        cfg_data = json.load(f)


    ###########################
    # Create UAV 0
    # -------------------------
    uav0_init = cfg_data['uav0']

    makeUAV(factory, socket, client_id,
            uav0_init['id'],
            (uav0_init['latitude'], uav0_init['longitude']),
            uav0_init['heading'],
            uav0_init['speed'],
            uav0_init['name'])

                                # (pos, waypt, speed, heading, tPossible)
    slave1_HEADING = UAVHeading((uav0_init['latitude'], uav0_init['longitude']),
                                (uav0_init['waypoint']['latitude'], uav0_init['waypoint']['longitude']),
                                uav0_init['speed'],
                                uav0_init['heading'],
                                uav0_init['theta_possible'])


    ###########################
    # Create UAV 1
    # -------------------------
    uav1_init = cfg_data['uav1']

    makeUAV(factory, socket, client_id,
            uav1_init['id'],
            (uav1_init['latitude'], uav1_init['longitude']),
            uav1_init['heading'],
            uav1_init['speed'],
            uav1_init['name'])

                                # (pos, waypt, speed, heading, tPossible)
    eagle5_HEADING = UAVHeading((uav1_init['latitude'], uav1_init['longitude']),
                                (uav1_init['waypoint']['latitude'], uav1_init['waypoint']['longitude']),
                                uav1_init['speed'],
                                uav1_init['heading'],
                                uav1_init['theta_possible'])


    ###############################
    # Get Vehicle State Updates
    # -----------------------------
    koz_id = 1
    koz_exists = False
    avoid_path = False
    while True:
        id_list = [1024, 2256]

        while len(id_list) > 0:
            msg_obj = get_from_amase(socket, factory)
            if msg_obj.FULL_LMCP_TYPE_NAME == 'afrl.cmasi.AirVehicleState':
                if msg_obj.get_ID() in id_list:
                    id_list.remove(msg_obj.get_ID())

                    # update uav objects
                    if msg_obj.get_ID() == 1024: #Slave 1
                        slave1_HEADING.position = (msg_obj.get_Location().get_Latitude(), msg_obj.get_Location().get_Longitude())
                        slave1_HEADING.speed = msg_obj.get_Airspeed()
                    elif msg_obj.get_ID() == 2256: #Eagle 5
                        eagle5_HEADING.position = (msg_obj.get_Location().get_Latitude(), msg_obj.get_Location().get_Longitude())
                        eagle5_HEADING.speed = msg_obj.get_Airspeed()
                        eagle5_HEADING.heading = msg_obj.get_Heading()
        
        # run UAV avoid function
        slave1_avoid_path, koz = slave1_HEADING.avoid(eagle5_HEADING)
        slave1_avoid_path = slave1_avoid_path[::-1][1:] # reverse output and remove first element

        if len(koz) > 0:
            # keepout zone for visualization
            koz_id = 23
            keepout_dom = parse("MessageLibrary/ko_zone.xml")
            ko_obj = factory.unpackFromXMLNode(keepout_dom)[0]
            ko_obj.set_ZoneID(koz_id)

            poly_obj = factory.createObjectByName("CMASI", "Polygon")
            for pt in koz:
                location3D = factory.createObjectByName("CMASI", 'Location3D')
                location3D.set_Latitude(pt[0])
                location3D.set_Longitude(pt[1])
                poly_obj.BoundaryPoints.append(location3D)

            ko_obj.set_Boundary(poly_obj)
            # print(ko_obj.__dict__)
            send_to_amase(ko_obj, socket, client_id)

        if len(slave1_avoid_path) > 0:
            print(slave1_avoid_path)
            avoid_path = True

            # make Waypoints for Slave1 and send MissionCommand
            waypoint_obj_list = []
            wpt_id = 201
            for i in range(len(slave1_avoid_path)):
                wp_obj = factory.createObjectByName("CMASI", "Waypoint")
                wp_obj.set_Number(wpt_id)
                wp_obj.set_Speed(slave1_HEADING.speed)
                wp_obj.set_Latitude(slave1_avoid_path[i][0])
                wp_obj.set_Longitude(slave1_avoid_path[i][1])
                wp_obj.set_Altitude(500.0)
                if i < (len(slave1_avoid_path) - 1):
                    wp_obj.set_NextWaypoint(wpt_id + 1)

                waypoint_obj_list.append(wp_obj)
                wpt_id += 1
            
            mc21_obj = factory.createObjectByName("CMASI", "MissionCommand")
            mc21_obj.set_FirstWaypoint(wpt_id)
            mc21_obj.set_CommandID(21)
            mc21_obj.set_VehicleID(1024)
            mc21_obj.WaypointList = waypoint_obj_list

            # time.sleep(0.1)
            send_to_amase(mc21_obj, socket, client_id)
        else:
            if avoid_path: # reset waypoints
                print(TC.OKGREEN + 'Nothing to aovid.' + TC.ENDC)
                # print('\tResetting waypoints.')
                avoid_path = False

                # ###### This doesn't quite work.
                # wpt_id = 201
                # wp_obj = factory.createObjectByName("CMASI", "Waypoint")
                # wp_obj.set_Number(wpt_id)
                # wp_obj.set_Speed(slave1_HEADING.speed)
                # wp_obj.set_Latitude(slave1_HEADING.waypoint[0])
                # wp_obj.set_Longitude(slave1_HEADING.waypoint[1])
                # wp_obj.set_Altitude(500.0)
            
                # mc21_obj = factory.createObjectByName("CMASI", "MissionCommand")
                # mc21_obj.set_FirstWaypoint(wpt_id)
                # mc21_obj.set_CommandID(22)
                # mc21_obj.set_VehicleID(1024)
                # mc21_obj.WaypointList = [wp_obj]

def send_to_amase(obj, socket, client_id):
    """
    Send an LCMP object to AMASE.

    Keyword arguments:
        obj -- an LMCP message object
        socket -- a ZMQ socket connected to AMASE
        client_id -- the client id for the AMASE socket
    """

    attributes = bytearray(str(obj.FULL_LMCP_TYPE_NAME) + "$lmcp|" + str(obj.FULL_LMCP_TYPE_NAME) + "||0|0$",
                           'ascii')
    smsg = LMCPFactory.packMessage(obj, True)

    sentinel = "+=+=+=+=";
    sentinel += str(len(attributes) + len(smsg));
    sentinel += "#@#@#@#@";

    addressedPayload = attributes;
    addressedPayload.extend(smsg);

    # sentinelized checksum
    val = 0;
    for i in range(0, len(addressedPayload)):
        val += int(addressedPayload[i] & 0xFF);

    footer = "!%!%!%!%" + str(val) + "?^?^?^?^";

    totalmsg = bytearray(sentinel, 'ascii');
    totalmsg.extend(addressedPayload);
    totalmsg.extend(bytearray(footer, 'ascii'));

    socket.send(client_id, flags=zmq.SNDMORE, copy=False)
    socket.send(totalmsg, copy=False)
    print("\n  Sent:   " + obj.FULL_LMCP_TYPE_NAME + "\n")


def get_from_amase(socket, factory):
    """
    Get an LCMP object from AMASE.

    Keyword arguments:
        socket -- a ZMQ socket connected to AMASE
        factory -- an LMCP factory
    """

    client_id, message = socket.recv_multipart()
    address, attributes, msg = message.split(bytearray('$', 'ascii'), 2)
    # msg_format, msg_type, msg_group, entityid, serviceid = attributes.split(bytearray('|', 'ascii'), 4)
    return factory.getObject(msg)


def lmcp_toprettyxml(obj):
    """
    Convert an LCMP object to a string that formats correctly with xml.dom.minidom's toprettyxml().

    Keyword arguments:
        obj -- an LMCP message object
    """

    # toprettyxml() assumes no newlines or whitespace between XML elements,
    # so remove them from what LMCP's toXML() returns.
    return re.sub('\n(\s)*', '', obj.toXML())


if __name__ == '__main__':
    create_and_run_amase_scenario()