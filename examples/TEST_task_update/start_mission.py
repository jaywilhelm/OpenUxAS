import zmq, sys, time, copy, re
import xml.dom.minidom
from xml.dom.minidom import parse, parseString

# Include LMCP Python files
sys.path.append('../../src/LMCP/py')
from lmcp import LMCPFactory
from afrl.cmasi import *

def create_and_run_amase_scenario():
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
    # Build Operating Region from XML file.
    # -----------------------------------------------------------------------------------------------------------------
    opr_dom = parse("MessageLibrary/OperatingRegion.xml")
    opr_obj_list = factory.unpackFromXMLNode(opr_dom)
    print(opr_obj_list[0].__dict__)
    send_to_amase(opr_obj_list[0], socket, client_id)
    
    ###################################################################################################################
    # Create an air vehicle by loading base configuration and state information from XML files.
    # Modify IDs and PayloadIDs to be unique.
    # -----------------------------------------------------------------------------------------------------------------

    cc3_lmcp_dom = parse("MessageLibrary/CameraConfiguration_WavelengthAnyAll_FovDiscrete_PayloadID-10001.xml")
    obj_list = factory.unpackFromXMLNode(cc3_lmcp_dom)
    cc3_obj = obj_list[0]
    cc3_obj.set_PayloadID(10003)
    gc3_lmcp_dom = parse("MessageLibrary/GimbalConfiguration_ClampedFalse_PayloadID-1001.xml")
    obj_list = factory.unpackFromXMLNode(gc3_lmcp_dom)
    gc3_obj = obj_list[0]
    gc3_obj.set_PayloadID(1003)
    gc3_obj.ContainedPayloadList = [10003]
    avc3_lmcp_dom = parse("MessageLibrary/AirVehicleConfiguration_FlightProfileAll_LoiterAll_ID-1.xml")
    obj_list = factory.unpackFromXMLNode(avc3_lmcp_dom)
    avc3_obj = obj_list[0]
    avc3_obj.set_ID(1024)
    avc3_obj.set_Label("Slave 1")
    avc3_obj.PayloadConfigurationList = [cc3_obj, gc3_obj]

    send_to_amase(avc3_obj, socket, client_id)

    cs3_lmcp_dom = parse("MessageLibrary/CameraState_PayloadID-10001.xml")
    obj_list = factory.unpackFromXMLNode(cs3_lmcp_dom)
    cs3_obj = obj_list[0]
    cs3_obj.set_PayloadID(10003)
    gs3_lmcp_dom = parse("MessageLibrary/GimbalState_PayloadID-1001.xml")
    obj_list = factory.unpackFromXMLNode(gs3_lmcp_dom)
    gs3_obj = obj_list[0]
    gs3_obj.set_PayloadID(1003)
    avs3_lmcp_dom = parse("MessageLibrary/AirVehicleState_FlightProfileAll_LoiterAll_ID-1.xml")
    obj_list = factory.unpackFromXMLNode(avs3_lmcp_dom)
    avs3_obj = obj_list[0]
    avs3_obj.set_Heading(90.0)
    avs3_obj.set_ID(1024)
    longitude = -132.5561
    latitude = 1.5308
    avs3_obj.get_Location().set_Longitude(longitude)
    avs3_obj.get_Location().set_Latitude(latitude)
    avs3_obj.PayloadStateList = [cs3_obj, gs3_obj]

    send_to_amase(avs3_obj, socket, client_id)

    ###################################################################################################################
    # Create new UAV 1 for UAV 0 to avoid
    # -----------------------------------------------------------------------------------------------------------------
    cc3_lmcp_dom = parse("MessageLibrary/CameraConfiguration_WavelengthAnyAll_FovDiscrete_PayloadID-10001.xml")
    obj_list = factory.unpackFromXMLNode(cc3_lmcp_dom)
    cc3_obj = obj_list[0]
    cc3_obj.set_PayloadID(10004)
    gc3_lmcp_dom = parse("MessageLibrary/GimbalConfiguration_ClampedFalse_PayloadID-1001.xml")
    obj_list = factory.unpackFromXMLNode(gc3_lmcp_dom)
    gc3_obj = obj_list[0]
    gc3_obj.set_PayloadID(1004)
    gc3_obj.ContainedPayloadList = [10004]
    avc3_lmcp_dom = parse("MessageLibrary/AirVehicleConfiguration_FlightProfileAll_LoiterAll_ID-1.xml")
    obj_list = factory.unpackFromXMLNode(avc3_lmcp_dom)
    avc3_obj = obj_list[0]
    avc3_obj.set_ID(2596)
    avc3_obj.set_Label("Eagle 5")
    avc3_obj.PayloadConfigurationList = [cc3_obj, gc3_obj]

    send_to_amase(avc3_obj, socket, client_id)

    cs3_lmcp_dom = parse("MessageLibrary/CameraState_PayloadID-10001.xml")
    obj_list = factory.unpackFromXMLNode(cs3_lmcp_dom)
    cs3_obj = obj_list[0]
    cs3_obj.set_PayloadID(10004)
    gs3_lmcp_dom = parse("MessageLibrary/GimbalState_PayloadID-1001.xml")
    obj_list = factory.unpackFromXMLNode(gs3_lmcp_dom)
    gs3_obj = obj_list[0]
    gs3_obj.set_PayloadID(1004)
    avs3_lmcp_dom = parse("MessageLibrary/AirVehicleState_FlightProfileAll_LoiterAll_ID-1.xml")
    obj_list = factory.unpackFromXMLNode(avs3_lmcp_dom)
    avs3_obj = obj_list[0]
    avs3_obj.set_Heading(45.0)
    avs3_obj.set_ID(2596)
    longitude = -132.539
    latitude = 1.5168
    avs3_obj.get_Location().set_Longitude(longitude)
    avs3_obj.get_Location().set_Latitude(latitude)
    avs3_obj.PayloadStateList = [cs3_obj, gs3_obj]

    send_to_amase(avs3_obj, socket, client_id)


    ###################################################################################################################
    # Create Line Search Task for UAV 0 from XML file.
    # -----------------------------------------------------------------------------------------------------------------
    ls_dom = parse("MessageLibrary/lst0.xml")
    ls_obj_list = factory.unpackFromXMLNode(ls_dom)
    print(ls_obj_list[0].__dict__)
    send_to_amase(ls_obj_list[0], socket, client_id)

    ###################################################################################################################
    # Assign task to UAV 0.
    # -----------------------------------------------------------------------------------------------------------------
    ar_obj = factory.createObjectByName("CMASI", "AutomationRequest")
    ar_obj.EntityList = [int(1024)]
    ar_obj.TaskList = [8675]
    # ar_obj.set_OperatingRegion(100)
    ar_obj.set_RedoAllTasks(False)
    # ar_obj.set_RedoAllTasks(True)

    print(ar_obj.__dict__)
    
    time.sleep(2.0)
    #test setting operating region after line search task is assigned
    send_to_amase(ar_obj, socket, client_id)

    ############################################################################
    #wait for a bit
    time.sleep(60)

    #create the split tasks

    # ls_dom = parse("MessageLibrary/lst0-new.xml")
    # ls_obj_list = factory.unpackFromXMLNode(ls_dom)
    # print(ls_obj_list[0].__dict__)
    # send_to_amase(ls_obj_list[0], socket, client_id)

    ls_dom = parse("MessageLibrary/lst1.xml")
    ls_obj_list = factory.unpackFromXMLNode(ls_dom)
    print(ls_obj_list[0].__dict__)
    send_to_amase(ls_obj_list[0], socket, client_id)

    ###################################################################################################################
    # Build Known KeepOut Zone from XML file.
    # -----------------------------------------------------------------------------------------------------------------
    keepout_dom = parse("MessageLibrary/ko_zone.xml")
    ko_obj_list = factory.unpackFromXMLNode(keepout_dom)
    print(ko_obj_list[0].__dict__)
    send_to_amase(ko_obj_list[0], socket, client_id)

    ###################################################################################################################
    # Build Operating Region from XML file.
    # -----------------------------------------------------------------------------------------------------------------
    opr_dom = parse("MessageLibrary/OperatingRegion.xml")
    opr_obj_list = factory.unpackFromXMLNode(opr_dom)
    print(opr_obj_list[0].__dict__)
    send_to_amase(opr_obj_list[0], socket, client_id)

    #reassign uav tasks
    ar_obj = factory.createObjectByName("CMASI", "AutomationRequest")
    ar_obj.EntityList = [int(1024)]
    ar_obj.TaskList = [8675]
    ar_obj.set_OperatingRegion(100)
    ar_obj.set_RedoAllTasks(False)
    # ar_obj.set_RedoAllTasks(True)
    time.sleep(2.0)
    send_to_amase(ar_obj, socket, client_id)

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
    create_and_run_amase_scenario()