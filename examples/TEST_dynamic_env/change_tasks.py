import zmq, sys, time, copy, re
import xml.dom.minidom
from xml.dom.minidom import parse, parseString

# Include LMCP Python files
sys.path.append('../../src/LMCP/py')
from lmcp import LMCPFactory
from afrl.cmasi import *


def give_UAV1_new_commands():
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

    ###################################################################################################################
    # Construct commands for the vehicles and send them.
    # -----------------------------------------------------------------------------------------------------------------

    # Command air vehicle 1 to follow a MissionCommand consisting of ordered waypoints that form a closed polygon.
    wp401_obj = factory.createObjectByName("CMASI", "Waypoint")
    wp401_obj.set_Number(201)
    wp401_obj.set_NextWaypoint(202)
    wp401_obj.set_Speed(20.0)
    wp401_obj.set_Latitude(1.5217)
    wp401_obj.set_Longitude(-132.5349)
    wp401_obj.set_Altitude(700.0)
    wp402_obj = factory.createObjectByName("CMASI", "Waypoint")
    wp402_obj.set_Number(202)
    wp402_obj.set_NextWaypoint(203)
    wp401_obj.set_Speed(30.0)
    wp402_obj.set_Latitude(1.5236)
    wp402_obj.set_Longitude(-132.5274)
    wp402_obj.set_Altitude(700.0)
    wp403_obj = factory.createObjectByName("CMASI", "Waypoint")
    wp403_obj.set_Number(203)
    wp403_obj.set_NextWaypoint(204)
    wp403_obj.set_Speed(20.0)
    wp403_obj.set_Latitude(1.5158)
    wp403_obj.set_Longitude(-132.5203)
    wp403_obj.set_Altitude(700.0)
    wp404_obj = factory.createObjectByName("CMASI", "Waypoint")
    wp404_obj.set_Number(204)
    wp404_obj.set_NextWaypoint(205)
    wp404_obj.set_Speed(30.0)
    wp404_obj.set_Latitude(1.5111)
    wp404_obj.set_Longitude(-132.5306)
    wp404_obj.set_Altitude(700.0)
    wp405_obj = factory.createObjectByName("CMASI", "Waypoint")
    wp405_obj.set_Number(205)
    wp405_obj.set_NextWaypoint(201)
    wp405_obj.set_Speed(20.0)
    wp405_obj.set_Latitude(1.5190)
    wp405_obj.set_Longitude(-132.5299)
    wp405_obj.set_Altitude(700.0)
    mc21_obj = factory.createObjectByName("CMASI", "MissionCommand")
    mc21_obj.set_FirstWaypoint(205)
    mc21_obj.set_CommandID(41)
    mc21_obj.set_VehicleID(1)
    mc21_obj.WaypointList = [wp401_obj, wp402_obj, wp403_obj, wp404_obj, wp405_obj]

    time.sleep(2.0)
    send_to_amase(mc21_obj, socket, client_id)

    # Command air vehicle 3 to go to a point and loiter there with a FigureEight loiter pattern.
    la_obj = factory.createObjectByName("CMASI", "LoiterAction")
    la_obj.set_LoiterType(LoiterType.LoiterType.FigureEight)
    la_obj.set_Axis(45.0)
    la_obj.set_Length(600.0)
    la_obj.set_Direction(LoiterDirection.LoiterDirection.Clockwise)
    la_obj.set_Duration(-1)
    la_obj.set_Airspeed(40.0)
    la_obj.get_Location().set_Latitude(1.5282)
    la_obj.get_Location().set_Longitude(-132.5524)
    la_obj.get_Location().set_Altitude(400.0)
    vac31_obj = factory.createObjectByName("CMASI", "VehicleActionCommand")
    vac31_obj.set_CommandID(31)
    vac31_obj.set_VehicleID(3)
    vac31_obj.VehicleActionList = [la_obj]

    time.sleep(2.0)
    send_to_amase(vac31_obj, socket, client_id)

    msg_time = 0
    while msg_time <= 7000:
        msg_obj = get_from_amase(socket, factory)
        if msg_obj.FULL_LMCP_TYPE_NAME == 'afrl.cmasi.AirVehicleState':
            msg_time = msg_obj.get_Time()


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
    print("  Sent:   " + obj.FULL_LMCP_TYPE_NAME)


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
    give_UAV1_new_commands()