# General Imports
import struct, array, os, zmq,time, sys, json, argparse
import pickle
import math
# For Dubins Vehicle
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import os
import subprocess
import numpy as np
from dubinsUAV import dubinsUAV

# Include LMCP Python files
lmcplocation = '../../src/LMCP/py'
sys.path.append(lmcplocation)
#from lmcp import LMCPFactory
#from afrl.cmasi import *

# sys.path.append(lmcplocation + '/afrl')
# sys.path.append(lmcplocation + '/uxas')
# from afrl.cmasi import SessionStatus
# from uxas.messages.uxnative import StartupComplete
# from afrl.cmasi import AirVehicleState
# from afrl.cmasi import Location3D
# from afrl.cmasi import EntityState
# from afrl.cmasi import AirVehicleConfiguration

# For Collision Avoidance
from lineSegmentAoE import *
sys.path.append('../UAVHeading-CollisionAvoidance/src')
from UAVHeading import UAVHeading

def get_from_uxas(socket, factory):
    """
    Get an LCMP object from uxas.

    Keyword arguments:
        socket -- a ZMQ socket connected to uxas
        factory -- an LMCP factory
    """

    client_id, message = socket.recv_multipart()
    address, attributes, msg = message.split(bytearray('$', 'ascii'), 2)
    msg_format, msg_type, msg_group, entityid, serviceid = attributes.split(bytearray('|', 'ascii'), 4)
    #print('\n\tmsg0\t' + str(msg_type) + ' ' + str(msg_format) + ' ' + str(msg_group) + ' ' + str(entityid)+ ' ' + str(serviceid))
    
    return factory.getObject(msg)

def send_to_uxas(obj, socket, client_id):
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

# def SelectDeadPoint():


def finduavbyID(uavlist,uavID):
    for uav in uavlist:
        if(uav['ID'] == uavID):
            return uav

    return None
def findotheruavs(uavlist, uavNOT):
    otherlist = []
    otherjustobj = []
    for uav in uavlist:
        if(uav['ID'] != uavNOT):
            otherlist.append(uav)
            otherjustobj.append(uav['uavobj'])
    return otherlist, otherjustobj
    

def syncAVSfromDubins(uav):
    # newloc = Location3D.Location3D()
    # newloc.set_Latitude(uav['dubins'].x)
    # newloc.set_Longitude(uav['dubins'].y)
    # newloc.set_Altitude(50)
    # uav['AVS'].set_Location(newloc)
    # uav['AVS'].set_Heading(uav['dubins'].heading)
    # msg_obj = uav['AVS']
    # uav['uavobj'] = UAVHeading(pos=[msg_obj.get_Location().get_Latitude(),msg_obj.get_Location().get_Longitude()],
    # waypt=[], speed = msg_obj.get_Airspeed(), heading=msg_obj.get_Heading(),
    # tPossible=math.radians(30))
    lat = uav['dubins'].x
    lon = uav['dubins'].y
    vel = uav['dubins'].v
    heading = uav['dubins'].heading
    avoidanceUAV = uav['avoidanceUAV']
    uav['uavobj'] = UAVHeading(pos=[lat, lon],
                               waypt=[], speed=vel, heading=heading,
                               tPossible=math.radians(45), avoidanceUAV=avoidanceUAV)
    return uav

TEST_DATA = True
if not TEST_DATA:
    print('connecting...')
    # # Connect to UxAS port and send Dubins state to AMASE using CMASI messages
    # context = zmq.Context()
    # socket = context.socket(zmq.STREAM)
    # socket.connect("tcp://localhost:9999") # a ZMQ socket connected to UxAS
    # # Store the client ID for this socket
    # client_id, message = socket.recv_multipart()
    # print("Client ID: " + str(client_id))
    # print("Message: " + str(message))
    #
    # factory = LMCPFactory.LMCPFactory()
    #
    # uavlist = []
    # dt = 0.00001
####################
####################
else:
    dt = 0.25
    v = 0.01

    #uavlist = pickle.load( open( "uavlist.p", "rb" ) )
    uavlist = []
    uav1 = {}
    uavlist.append(uav1)
    uav2 = {}
    uavlist.append(uav2)
    #fix for bad file save of degrees
    np.deg2rad(30)
    thetaRef = np.deg2rad(270)
    uavlist[0]['dubins'] = dubinsUAV(position=[45.35, -120.5], velocity=v,
                                     heading=thetaRef, dt=dt)
    deadpoint = [45.3394889, -121.2]
    uavlist[0]['ID'] = 1
    uavlist[0]['avoidanceUAV'] = True
    np.deg2rad(30)
    thetaRef = np.deg2rad(90)

    uavlist[1]['dubins'] = dubinsUAV(position=[45.32, -120.8], velocity=v,
                                     heading=thetaRef, dt=dt)
    uavlist[1]['ID'] = 4
    uavlist[1]['avoidanceUAV'] = False
    #simUAV = {'position': (uavlist[0]['AVS'].get_Location().get_Latitude(),uavlist[0]['AVS'].get_Location().get_Longitude()),
    # 'velocity': v, 'heading': uavlist[0]['uavobj'].thetaRef}

    #simUAV = {'position': (uavlist[1]['AVS'].get_Location().get_Latitude(),uavlist[1]['AVS'].get_Location().get_Longitude()),
    # 'velocity': v, 'heading': uavlist[1]['uavobj'].thetaRef}
    #uavlist[1]['dubins'] = dubinsUAV(position=simUAV['position'], velocity=simUAV['velocity'], heading=simUAV['heading'], dt=dt)
    uavlist[0] = syncAVSfromDubins(uavlist[0])
    uavlist[1] = syncAVSfromDubins(uavlist[1])
####################
####################

savePlots = []
NCuavPos = [[uavlist[1]['dubins'].x, uavlist[1]['dubins'].y]]
CASuavPos = [[uavlist[0]['dubins'].x, uavlist[0]['dubins'].y]]
previousDist = 9999999
save = []
saveDistances = []

fig, ax = plt.subplots()
TargetWPList = None
plotDetectRange = None
plotTargetWP = None
plotAstarGoalPt = None
pos = None
CASpos = None
NCpos = None
CASkoz = None
NCkoz = None
avoidArea = None
CAScone = None
NCcone = None
plotCarrot = None
wpt = None
path = None
useWPfollower = False
replanCount = 0
start_time = time.time()
avoid = []
hasPlan = False
full_path = []
step = 0
while step<175:
    # if not TEST_DATA:
        # try:
        #     msg_obj = get_from_uxas(socket, factory)
        #     #print('\n\tmsg1\t' + str(msg_obj.FULL_LMCP_TYPE_NAME))
        # except:
        #     print('\n\tProblem: Sometimes the message does not have enough stuff...not sure why\n')

        # if  (msg_obj.FULL_LMCP_TYPE_NAME == 'afrl.cmasi.AirVehicleState'):
        #
        #     uavsearch = finduavbyID(uavlist, msg_obj.get_ID())
        #     if(not uavsearch):
        #         newuav = {}
        #         newuav['ID'] = msg_obj.get_ID()
        #         newuav['uavobj'] = UAVHeading(pos=[msg_obj.get_Location().get_Latitude(),msg_obj.get_Location().get_Longitude()],
        #         waypt=[], speed = msg_obj.get_Airspeed(), heading=msg_obj.get_Heading(),
        #         tPossible=math.radians(30))
        #         newuav['AVS'] = msg_obj
        #         uavlist.append(newuav)
        #         print("new uav " + str(newuav['ID']))
        #     else:
        #         #update uav...
        #         print('Updating UAV' + str(msg_obj.get_ID()))
        #         uavsearch['AVS'] = msg_obj
        #         newuav['uavobj'] = UAVHeading(pos=[msg_obj.get_Location().get_Latitude(),msg_obj.get_Location().get_Longitude()],
        #         waypt=[], speed = msg_obj.get_Airspeed(), heading=msg_obj.get_Heading(),
        #         tPossible=math.radians(30))
    # else:
    #     time.sleep(dt)
    area_length = 0.1
    check_time = time.time()
    if(check_time - start_time > 1):
        mainUAV = finduavbyID(uavlist, 1) # IDtoWatch
        uavh_others_all, uavh_others = findotheruavs(uavlist, 1) # ID not to watch for

        if TargetWPList == None:
            ''' Generate the main path. Goal is to reconncet to this path after avoiding another UAV/obstacle'''
            usetargetPath = True
            TargetWPList = mainUAV['dubins'].makePath(pathType='Sine', numbOfPoints=20, dist=0.08)
            TargetPath, = plt.plot([pt[1] for pt in TargetWPList], [pt[0] for pt in TargetWPList], c='b', marker='.')
            uavlist[0]['dubins'].setWaypoints(TargetWPList, newradius=0.01)
            uavlist[0]['dubins'].currentWPIndex = 1
            print('TargetWPList: ' + str(TargetWPList))

        detectRange, targetWP, targetIndex, astarGoalIndex, astarGoalPt = mainUAV['dubins'].detectClosestWP(dist=0.3, theta_possible=mainUAV['uavobj'].thetaPossible, alpha=4, targetPath=TargetWPList)
        plotDetectRange, = plt.plot([pt[1] for pt in detectRange], [pt[0] for pt in detectRange], c='y')
        if len(targetWP) > 0:
            plotTargetWP, = plt.plot(targetWP[1], targetWP[0], c='r', marker = '*')
        else:
            plotTargetWP = None
        if len(astarGoalPt) > 0:
            plotAstarGoalPt, = plt.plot(astarGoalPt[1], astarGoalPt[0], c='g', marker = '*')
        else:
            plotAstarGoalPt = None

        if len(uavlist) > 1:
            #if(not hasPlan):
            replan, wplist, avoid, full_path = mainUAV['uavobj'].avoid(uavh_others, area_length=area_length, static_koz=[])
            # Comment the above line and uncomment to use a dummy wpList for testing purposes
            # replan = True
            # wp = [[  45.32, -120.74], [  45.38, -120.8 ], [  45.38, -120.96], [  45.32, -121.02]]

            # plotting in the following way breaks the .remove() function...not sure why yet
            # for i in range(len(avoid)):
            #     for pts in avoid:
            #         avoidArea, = plt.plot([pt[1] for pt in avoid[i]], [pt[0] for pt in avoid[i]])
            #         print('avoid poly: ' + str(pts))
            #         # plt.pause(dt) # check plotting fucntion

            if(replan and not hasPlan):
                hasPlan = True
            
                '''If replan is True x number of times, use the new A* wpList wp01'''
                # plt.ax([pt[1] for pt in avoid[0]], [pt[0] for pt in avoid[0]])
                closingDist = mainUAV['dubins'].distance(mainUAV['uavobj'].position, uavh_others[0].position)
                print('Distance to other UAV: ' + str(closingDist))
                print('POS: ' + str(mainUAV['uavobj'].position))
                print('WP: ' + str(wplist))
                print('DP: ' + str(deadpoint))
                #wp01 = wplist[1:]     # remove first coordinate
                # wp01 = wp01[:-1]    # Remove last waypoint
                # wp = np.append(wp01, [[mainUAV['uavobj'].waypoint[0], mainUAV['uavobj'].waypoint[1]]], axis = 0)                    
                wplist[0][0] = mainUAV['uavobj'].position[0]
                wplist[0][1] = mainUAV['uavobj'].position[1]
                #wplist = np.insert(wp01, 0, [[mainUAV['uavobj'].position[0], mainUAV['uavobj'].position[1]]], axis = 0)                         
                # wplist = np.append(wplist, [[deadpoint[0], deadpoint[1]]], axis = 0)    
                uavlist[0]['dubins'].setWaypoints(wplist, newradius=0.01)
                uavlist[0]['dubins'].currentWPIndex = 1               
                print('\nUpdated WP List: ' + str(wplist))
                path, = plt.plot([pt[1] for pt in wplist], [pt[0] for pt in wplist])
                useWPfollower = True
                usetargetPath = False
                clearedOtherUAV = False
            else:
                print("Not re-planning")
        else:
            print('\tOnly one UAV')

        #
        #run ACS here...
        #if needed, send waypoints
        for uav in uavlist:
            # lastAVS = uav['AVS']
            # plt.scatter(lastAVS.get_Location().get_Longitude(), lastAVS.get_Location().get_Latitude())
            # ax.scatter(uav['dubins'].y, uav['dubins'].x)
            # pos, = ax.plot(uav['dubins'].y, uav['dubins'].x, 'o')
            
            pts = uav['uavobj'].possibleFlightAreaStatic(area_length=area_length*1.0)
            print('points:' + str(pts))
            # Calculate new Dubins heading: 
            # ax.scatter([pt[1] for pt in pts], [pt[0] for pt in pts])

            if uav['ID'] == 1:
                print('Current WP: ' + str(uav['dubins'].currentWPIndex))
                CASpos, = ax.plot(uav['uavobj'].position[1], uav['uavobj'].position[0], 'o')
                color = '-g'
                if(replan):
                    color = '-y'
                CAScone, = ax.plot([pt[1] for pt in pts], [pt[0] for pt in pts], color)
                wpt, = ax.plot(mainUAV['uavobj'].waypoint[1], mainUAV['uavobj'].waypoint[0], 'X')
                print('Dubbs WP: ' + str(mainUAV['uavobj'].waypoint))

                if usetargetPath: 
                    print('\tUse Target Path')
                    uav['dubins'].simulateWPDubins(UseCarrotChase=True)
                    carrot = uav['dubins'].CarrotChaseWP()
                    plotCarrot, = plt.plot(carrot[1], carrot[0], c='orange', marker='^' )
                    # plotCarrot = None
                    CASuavPos = uav['dubins'].position

                elif useWPfollower == True:
                    print('\tUse Avoid Path')

                    if uav['dubins'].lastWP or clearedOtherUAV:
                        # Switch back to original path if the other UAV has moved away
                        # or if the last A* waypoint has been reached
                        print('\tRevert to original path')
                        useWPfollower = False
                        usetargetPath = True
                        uav['dubins'].lastWP = False
                        uav['dubins'].setWaypoints(TargetWPList, newradius=0.01)
                        detectRange, targetWP, targetIndex, astarGoalIndex, astarGoalPt = uav['dubins'].detectClosestWP(dist=0.3, theta_possible=uav['uavobj'].thetaPossible, alpha=4, targetPath=TargetWPList)
                        uav['dubins'].currentWPIndex = targetIndex

                    uav['dubins'].simulateWPDubins(UseCarrotChase=True)
                    carrot = uav['dubins'].CarrotChaseWP()
                    plotCarrot, = plt.plot(carrot[1], carrot[0], c='orange', marker='^' )
                    if len(avoid)>1:
                        CASkoz, = plt.plot([pt[1] for pt in avoid[0]], [pt[0] for pt in avoid[0]], '--m')
                        NCkoz, = plt.plot([pt[1] for pt in avoid[1]], [pt[0] for pt in avoid[1]], '--m')
                    else:
                        NCkoz = None
                        CASkoz = None
                    CASuavPos = uav['dubins'].position
                else:
                    uav['dubins'].update_pos_simple()
                    #wpt, = ax.plot(mainUAV['uavobj'].waypoint[1], mainUAV['uavobj'].waypoint[0], 'X')
                    CASuavPos = uav['dubins'].position

            if uav['ID'] == 4:
                NCpos, = ax.plot(uav['uavobj'].position[1], uav['uavobj'].position[0], 'o')
                NCcone, = ax.plot([pt[1] for pt in pts], [pt[0] for pt in pts], "-r")
                uav['dubins'].update_pos_simple()
                NCuavPos = uav['dubins'].position
                

            #plt.show()

            # plt.plot([pt[1] for pt in pts], [pt[0] for pt in pts])
            # print(str(lastAVS.get_Time()) +
            # '\tAVS ID: ' + str(lastAVS.get_ID()) +
            # '\tlat: ' + str(lastAVS.get_Location().get_Latitude()) +
            # '\tlon: ' + str(lastAVS.get_Location().get_Longitude()) +
            # '\tv: ' + str(lastAVS.get_Airspeed()) +
            # '\tcog: ' + str(lastAVS.get_Heading()))

            #Update Positions
            # uav['dubins'].simulateWPDubins(wpList = wp, wpRadius = 0.0005, tmax=1)

            print('ID: ' + str(uav['ID']) + '\tHeading: ' + str((uav['dubins'].heading)) + '\tlat: ' + str(uav['dubins'].x) +
            '\tlon: ' + str(uav['dubins'].y) + '\tIs CAS?: ' + str(uav['avoidanceUAV']))

            uav = syncAVSfromDubins(uav)

        dist2UAVs = uav['dubins'].distance(CASuavPos[0], NCuavPos[0])
        if dist2UAVs < previousDist:
            state = 'Moving towards'
        else:
            state = 'Moving away'
            clearedOtherUAV = True
            #plt.pause(5)

        previousDist = dist2UAVs

        save = [dist2UAVs, state]
        saveDistances.append(save)



        print('Confirm distance: ' + str(dist2UAVs) + ' ' + state)


    step += 1
    plt.axis('equal')
    plt.grid(True)
    # plt.ylim((uavlist[0]['dubins'].x - 0.1, uavlist[0]['dubins'].x + 0.1))
    # plt.xlim((uavlist[0]['dubins'].y - 0.1, uavlist[0]['dubins'].y + 0.1))
    # plt.pause(0.0000000001)

    plt.ylim(45.25,45.45)
    plt.xlim(-121.25, -120.45)
    
    # plt.pause(dt)
    time.sleep(dt) # prevents multiple blank frames at the begining of saved movie
    wd = os.getcwd()
    path=(wd + '/Movies')
    fname = '_tmp%03d.png' % step
    fname = os.path.join(path,fname)
    plt.savefig(fname)
    savePlots.append(fname)

    if pos != None:
        pos.remove()
    # if CASpos != None:
    #     CASpos.remove()
    # if NCpos != None:
    #     NCpos.remove()
    if CAScone != None:
        CAScone.remove()
    if NCcone != None:
        NCcone.remove()
    if wpt != None:
        wpt.remove()
    if plotCarrot != None:
        plotCarrot.remove()
    if avoidArea != None:
        avoidArea.remove()
    # if path != None:
    #     path.remove()

    # if uav['ID'] == 1:
    #     CAScone.remove()
    #     del CAScone
    # if uav['ID'] == 4:
    #     NCone.remove()
    #     del NCone

    if CASkoz != None:
        CASkoz.remove()
    if NCkoz != None:
        NCkoz.remove()
    if plotDetectRange != None:
        plotDetectRange.remove()
    if plotTargetWP != None:
        plotTargetWP.remove()
    if plotAstarGoalPt != None:
        plotAstarGoalPt.remove()
    # plt.pause(dt) # additional pause to make sure that .remove() fucntions are working

    # else:
    #     print('\tUse previously valid path: ')
    #     # wpList = lastWPlist  # Use the last valid wp path
    #     wpList = wpList     # Head directly for the target coordinates once the path is clear

    #time.sleep(0.1)

# anim = FuncAnimation(fig, savePlots, frames=30, interval=20, blit=True)

print('Change directory to Movies: ')
wd = os.getcwd()
path = ('Movies')
newDir = os.path.join(wd,path)
os.chdir(str(newDir))
print('Making sim movie...')
# source: https://matplotlib.org/gallery/animation/movie_demo_sgskip.html
# additional resource: https://linux.die.net/man/1/mencoder
subprocess.call("mencoder 'mf://_tmp*.png' -mf type=png:fps=10 -ovc lavc "
                "-lavcopts vcodec=mpeg4 -oac copy -o animation.mp4", shell=True)

print('Clean up...')
for fname in savePlots:
    os.remove(fname)

print('Reverting to previous Directory')
os.chdir(wd)

print('Simulation End')
print('Distance between waypoints: \n')
for pt in saveDistances:
    print(str(pt) + '\n')
