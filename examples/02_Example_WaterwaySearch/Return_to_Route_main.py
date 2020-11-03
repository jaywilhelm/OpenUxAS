# Jeremy Browne October 2020
# Main script waypoint based return-to-route script
# Procedure:
# 1) Follow refference path until collision event with 
#    a non-coopterative UAV
# 2) Avoid using A* generated collision-free replan path
# 3) Continue to avoid until the offending UAV has passed
# 4) Return to reference path using a clothoid path
#

import Return_to_Route_functions as R2R
from UAVDataHolder import uavData
# import UAVDataHolder as uavData

from dubins_Return2Route import dubinsUAV
from TerminalColors import TerminalColors as TC

import numpy as np
import sys, math, os, subprocess, time, shutil
from matplotlib import pyplot as plt

# For A* Collision Avoidance  

def del_folder_contents(folderPath):
    folder = folderPath
    for filename in os.listdir(folder):
        file_path = os.path.join(folder, filename)
        try:
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.unlink(file_path)
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)
        except Exception as e:
            print('Failed to delete %s. Reason: %s' % (file_path, e))

wd = os.getcwd()

folderPath=(wd + '/RaceTrack_AstarFormatedInput')
del_folder_contents(folderPath)

folderPath=(wd + '/RaceTrack_AstarResults')
del_folder_contents(folderPath)

folderPath=(wd + '/RaceTrack_AstarPath')
del_folder_contents(folderPath)

folderPath=(wd + '/RaceTrack_RecoveryPaths')
del_folder_contents(folderPath)

folderPath=(wd + '/RaceTrack_SelectedPaths')
del_folder_contents(folderPath)

'RaceTack points taken from Pruitt Field using Goolgle maps'
PruitTrack =[
[39.3264051914717, -82.1101289994580], [39.3263499334731, -82.1103352244684], [39.3261989661035, -82.1104861915330],
[39.3259927415369, -82.1105414491151], [39.3249489949075, -82.1105414491151],[39.3247433390045, -82.1104865192473],
[39.3245921315912, -82.1103357926091], [39.3245365454954, -82.1101296557923], [39.3245914749791, -82.1099233430367],
[39.3247422017190, -82.1097721357249], [39.3249489949075, -82.1097165492456], [39.3259927415369, -82.1097165492456],
[39.3261989665845, -82.1097718071053], [39.3263499337508, -82.1099227743733], [39.3264051914717, -82.1101289994580]]

PruitTrack_reverse = [
[39.3261989661035, -82.1104861915330], [39.3263499334731, -82.1103352244684], [39.3264051914717, -82.1101289994580], 
[39.3263499337508, -82.1099227743733], [39.3261989665845, -82.1097718071053], [39.3259927415369, -82.1097165492456], 
[39.3249489949075, -82.1097165492456], [39.3247422017190, -82.1097721357249], [39.3245914749791, -82.1099233430367], 
[39.3245365454954, -82.1101296557923], [39.3245921315912, -82.1103357926091], [39.3247433390045, -82.1104865192473], 
[39.3249489949075, -82.1105414491151], [39.3259927415369, -82.1105414491151]]

'''
Generate 2 lists of dictionary entries 
Each dictionary entry contains the racetrack waypoint
ActivePath_List - The active list the CAS UAV will follow
ReferencePath_List - Used to reset the active list
'''
Waypoint_dict = {}      # Temporary dictionary entry
ActivePath_List = []    
ReferencePath_List = [] 
for i in range(0, len(PruitTrack)):
    Waypoint_dict['pt'] = PruitTrack[i]    
    Waypoint_dict['Belongs to'] = 'Reference Path'  # Identifies which path this point belongs to
    if i == 0:
        Waypoint_dict['Is Go-To'] = True        # This is the first point in the track
    else:
        Waypoint_dict['Is Go-To'] = False

    ActivePath_List.append(Waypoint_dict.copy())
    ReferencePath_List.append(Waypoint_dict.copy())

'UAV variables/capabilities'
dt = 0.2 
# distance threshold to satisfy waypoint
wptRad = 0.0001 

# List of Collision Avoidacen UAVs
#  uav =  [uavType, ID, v, thetaRef, currentWPIndex, pathWpts]
#          uavType: 0 = Message-Based updating ; 1 = use dubins simulation updating
init_CAS = [[1, 1, 0.00005/2, np.deg2rad(270), 3, PruitTrack]]
# List of Non-cooperative/intruder UAVs
init_NC =  [[1, 4, 0.00005, np.deg2rad(135), 2, PruitTrack_reverse]]

def syncAVSfromDubins(uav):
    lat = uav['uavData'].x
    lon = uav['uavData'].y
    vel = uav['uavData'].v
    heading = uav['uavData'].heading
    IsAvoidanceUAV = uav['IsAvoidanceUAV']
    uav['uavobj'] = UAVHeading(pos=[lat, lon],
                               waypt=[], speed=vel, heading=heading,
                               tPossible=math.radians(45), IsAvoidanceUAV=IsAvoidanceUAV)
    return uav

uavlist = []
uav1 = {}
uav1['uavData'] = uavData(dt, wptRad, uavType=1, ID=1, position=[], velocity=0.00005/2, heading=np.deg2rad(270), currentWPIndex=3, pathWpts=PruitTrack)
uav1['IsAvoidance'] = True
uavlist.append(uav1)
uav4 = {}
uav4['uavData'] = uavData(dt, wptRad, uavType=1, ID=4, position=[], velocity=0.00005, heading=np.deg2rad(135), currentWPIndex=3, pathWpts=PruitTrack_reverse)
uav4['IsAvoidance'] = False

uavlist.append(uav4)

uavlist[0] = syncAVSfromDubins(uavlist[0])
uavlist[1] = syncAVSfromDubins(uavlist[1])

'Create a list of UAVs'
# uavlist = uavData.createUAVList( dt, wptRad, init_CAS, init_NC  )
lap_Counter = uavlist[0]['dubins'].lapCounter 
halfTurnRadius = uavlist[0]['dubins'].turnRadius*2

'''
Simulation Variables
'''
area_length = 0.00025
TargetWPList = None
last_targetIndex = 0
lookAhead_time = 10

lap = None
Astar_Index2Watch4 = None
Astar_Return_Index = None
Recover_Index2Watch4 = None
Recovery_Return_Index = None 

astarReplan = False
hasAstarPlan = False
hasRecoveryPlan = False
ClearedCollision = False

Show_AstarPlan = False
Show_RecoveryPlan = False
TrackUAV = False

Test = False

'Movie stuff'
savePlots = []  # stores figure frames used to make a movie
savePlots1 = []
savePlots2 = []
savePlots3 = []
savePlots4 = []
savePlots5 = []
savePlots6 = []

fig, ax = plt.subplots()
step = 0
while step <= 900:

    ''' Identify UAVs using collision avoidance '''
    mainUAV = R2R.finduavbyID(uavlist, 1)                           # ID to Watch for
    uavh_others_all, uavh_others = R2R.findotheruavs(uavlist, 1)    # ID not to watch for

    activeWP = mainUAV['dubins'].waypoints[mainUAV['dubins'].currentWPIndex]

    # detectConePts = R2R.flightProj(mainUAV['dubins'].x, mainUAV['dubins'].y, mainUAV['dubins'].heading, mainUAV['dubins'].v, mainUAV['dubins'].turnrate, dt, 5)

    ''' 
    Extract waypoint coordinates from the ReferencePath_list  
    Easier to use a list of waypoints
    '''
    if TargetWPList == None:       
        TargetWPList = []
        for i in range(0, len(ReferencePath_List)):
            TargetWPList.append(ActivePath_List[i]['pt'])

        mainUAV['dubins'].setWaypoints(TargetWPList, newradius=wptRad)
        # mainUAV['dubins'].currentWPIndex = 0

    '''
    ======================================
    === Collision Detction Starts Here ===
    ======================================
    '''

    if not hasAstarPlan and Test == True:

        detect = mainUAV['uavobj'].collisionDetector(uavh_others, mainUAV['dubins'].turnrate, dt, lookAhead_time, area_length=area_length, static_koz=[])

        if detect == True:
            lookAheadDist = 2.5
            astarGoalPoint_list = R2R.findRefPathIndex(ActivePath_List)
            astarGoalPt, targetIndex, pointList, lap = R2R.findAstarGoal(astarGoalPoint_list, ReferencePath_List, mainUAV, area_length, 
                                                                        lookAheadDist)

            # A* replan - already a function
            astarReplan, astarwpts, KOZpoints, full_path, uavID, AstarFailure = mainUAV['uavobj'].avoid(uavh_others, mainUAV['dubins'].turnrate, dt, lookAhead_time, area_length=area_length, static_koz=[], TargetPathWP=astarGoalPt, useAstarGoal=True, simStep=step)

            # some kind of waypoint list update function?
                # needs an insert point - where to put the new waypoints
                # new return to path point - both the index to switch to another point and that points index
            if (astarReplan and not hasAstarPlan):
                hasAstarPlan = True
                Show_AstarPlan = True

                indexRecall = mainUAV['dubins'].currentWPIndex-1 # used to inform Astar goal and Revocery path logic where the UAV first left the reference path

                # Current positions is saved to be used later as a possible recovery insetions point
                saveCurrentPOS = [mainUAV['dubins'].x, mainUAV['dubins'].y ] 

                ''' Insert current vehicle postion and astar goal into A* replan '''
                astarwpts[0][0] = mainUAV['uavobj'].position[0]
                astarwpts[0][1] = mainUAV['uavobj'].position[1]
                astarwpts = np.append(astarwpts, np.array([[astarGoalPt[0], astarGoalPt[1]]]), axis=0)
                numbOfAstarPts= len(astarwpts) 

                # Waypoint index to watch for - signals return to the reference path
                Astar_Index2Watch4 = mainUAV['dubins'].currentWPIndex + numbOfAstarPts
                if lap == True:
                    Astar_Return_Index = targetIndex
                else:
                    Astar_Return_Index = targetIndex + numbOfAstarPts

                # Update flight plan with A* replan waypoints - insert waypoints at current waypoint index
                R2R.UpdateWPList(ActivePath_List, mainUAV, wypts2add=astarwpts, numbOfPts=numbOfAstarPts, List_belongsTo='Astar', insertIndex=mainUAV['dubins'].currentWPIndex)

                ''' Update the List of Waypoints '''
                TargetWPList = []
                for i in range(0, len(ActivePath_List)):
                    TargetWPList.append(ActivePath_List[i]['pt'])

                mainUAV['dubins'].setWaypoints(TargetWPList, newradius=wptRad/2)
                activeWP = mainUAV['dubins'].waypoints[mainUAV['dubins'].currentWPIndex]

                print(TC.OKBLUE + 'Insert ' + str(numbOfAstarPts) + ' Astar Points at wpt index ' + str(mainUAV['dubins'].currentWPIndex) + TC.ENDC)

                TrackUAV = True
                last_dist2UAV = R2R.distance([mainUAV['dubins'].x, mainUAV['dubins'].y] , [uavlist[1]['dubins'].x, uavlist[1]['dubins'].y])

                R2R.Show_AstarSnapShot('/RaceTrack_AstarPath', step, activeWP, mainUAV, uavh_others_all, PruitTrack, astarwpts, astarGoalPt, KOZpoints, area_length, dt, lookAhead_time, fig, ax)
    
    # If collision has been detected and collision replan was successful
    # Start Distance Tracking between CAS and NC UAVs
    # Currently only useful for scenarios with only 1 non cooperative UAV
    distThreshold = area_length*1.11
    if TrackUAV == True:
        dist2UAV = R2R.distance([mainUAV['dubins'].x, mainUAV['dubins'].y] , [uavlist[1]['dubins'].x, uavlist[1]['dubins'].y])
        if dist2UAV > last_dist2UAV and dist2UAV > distThreshold:
            TrackUAV = False
            ClearedCollision = True
        else:
            last_dist2UAV = dist2UAV

    if hasAstarPlan and ClearedCollision == True:
        # Function for Generateing clothoid paths
        RecoveryPoints, referencePathPoint, InsertionPoint_list = R2R.getPotentialRecoveryPoints(ActivePath_List, ReferencePath_List, saveCurrentPOS, indexRecall, numbOfPts=5)
        
        # Generate clothoid paths and corresponding reference path indices
        clothoid_List, index_List = R2R.getRecoveryPaths(RecoveryPoints, referencePathPoint, mainUAV)

        # Select clothoid with shortest path that does not violate turn radius and report the reference path index 
        RecoveryPathFound, chosenClothoid, acceptedPaths, selectIndex, lap = R2R.selectRecoveryPath(clothoid_List, index_List, indexRecall, halfTurnRadius, 10, lap)
        R2R.RecoveryPaths_SnapShot('/RaceTrack_RecoveryPaths', step, clothoid_List, acceptedPaths, PruitTrack, astarwpts, KOZpoints, False, RecoveryPoints, activeWP, mainUAV, uavh_others_all, area_length, fig, ax)

        
        nextIndex = InsertionPoint_list[selectIndex]['Index']
        if nextIndex < indexRecall:
            lap = True
        else:
            lap = False

        if RecoveryPathFound:
            # Sample waypoints from chosen clothoid to be used as the recovery path
            RecoveryPathWPs, pltPts_List = R2R.getRecoveryPathWPs(chosenClothoid, numbOfwpts=10)
            # Waypoint update funtion
            numbOfRecoveryPts = len(RecoveryPathWPs)
            # waypoint index to watch for - Waypoint indext that identifies when to switch back to reference path
            Recover_Index2Watch4 = Astar_Index2Watch4 + numbOfRecoveryPts

            # Waypoint Index that identifies which waypoint index to change to on the reference path after completing the recovery path
            if lap == True:
                Recovery_Return_Index = InsertionPoint_list[selectIndex]['Index'] 
            else:
                Recovery_Return_Index = InsertionPoint_list[selectIndex]['Index'] + numbOfRecoveryPts 
            # Update flight plan with Recovery waypoints - insert waypoints after the A* waypoints
            R2R.UpdateWPList(ActivePath_List, mainUAV, wypts2add=RecoveryPathWPs, numbOfPts=numbOfRecoveryPts, List_belongsTo='Recovery', insertIndex=Astar_Index2Watch4)
            
            ''' Update the List of Waypoints '''
            TargetWPList = []
            for i in range(0, len(ActivePath_List)):
                TargetWPList.append(ActivePath_List[i]['pt'])

            mainUAV['dubins'].setWaypoints(TargetWPList, newradius=wptRad/2)
            mainUAV['dubins'].currentWPIndex = Astar_Index2Watch4
            activeWP = mainUAV['dubins'].waypoints[mainUAV['dubins'].currentWPIndex]

            R2R.RecoveryPath_SnapShot('/RaceTrack_SelectedPaths', step, PruitTrack, astarwpts, pltPts_List, RecoveryPathWPs, activeWP, mainUAV, uavh_others_all, area_length, fig, ax )


            hasRecoveryPlan = True
            ClearedCollision = False
            Show_RecoveryPlan = True
            Show_AstarPlan = False
            astarReplan = False

    # function to check when following Ref or Avoid path and swap between them or back to reference path
    if hasAstarPlan == True and hasRecoveryPlan == False:
        if step == 2925:
            checking = 1

        changePath, ActivePath_List, TargetWPList = R2R.check_changePath(ActivePath_List, ReferencePath_List, TargetWPList, mainUAV, Astar_Index2Watch4, lap, wptRad)        
        if changePath == True:
            hasAstarPlan = False
            Show_AstarPlan = False
            astarReplan = False
            mainUAV['dubins'].currentWPIndex = Astar_Return_Index  
            activeWP = mainUAV['dubins'].waypoints[mainUAV['dubins'].currentWPIndex]               
            print('Completed A* path')
            if lap == True:
                lap_Counter +=1 
                print('Completed a Lap')


    elif hasAstarPlan == True or hasRecoveryPlan == True:
        changePath, ActivePath_List, TargetWPList = R2R.check_changePath(ActivePath_List, ReferencePath_List, TargetWPList, mainUAV, Recover_Index2Watch4, lap, wptRad)        
        if changePath == True:
            hasAstarPlan = False
            hasRecoveryPlan = False
            Show_RecoveryPlan = False
            ClearedCollision = False
            mainUAV['dubins'].currentWPIndex = Recovery_Return_Index  
            activeWP = mainUAV['dubins'].waypoints[mainUAV['dubins'].currentWPIndex]
            print('Completed Recovery path')
            if lap == True:
                lap_Counter +=1 
                print('Completed a lap')


    # Update UAV positions - already functions
    '''
    =============================
    === Update vehicle States ===
    =============================
    '''
    uavData.updateStates(uavlist)


    'Plotting stuff'
    CASCone_pts = R2R.flightProj(mainUAV['dubins'].x, mainUAV['dubins'].y, mainUAV['dubins'].heading, mainUAV['dubins'].velocity, mainUAV['dubins'].turnrate, dt, lookAhead_time)
    color = '-g'
    if(astarReplan):
        color = '-y'
        replan = False
    plotCAScone, = plt.plot([pt[1] for pt in CASCone_pts], [pt[0] for pt in CASCone_pts], color)

    # NC_pts = uavlist[1]['uavobj'].possibleFlightAreaStatic(area_length=area_length*1.0)
    NC_pts = R2R.flightProj(uavlist[1]['dubins'].x, uavlist[1]['dubins'].y, uavlist[1]['dubins'].heading, uavlist[1]['dubins'].velocity, uavlist[1]['dubins'].turnrate, dt, lookAhead_time)
    plotNCcone, = plt.plot([pt[1] for pt in NC_pts], [pt[0] for pt in NC_pts], "-r")

    plotCASposHist, = plt.plot(uavlist[0]['dubins'].ys, uavlist[0]['dubins'].xs, 'o', c='r')
    plotCASpos, = plt.plot(uavlist[0]['dubins'].y, uavlist[0]['dubins'].x, 'o', c='k')

    plotNCpos, = plt.plot(uavlist[1]['uavobj'].position[1], uavlist[1]['uavobj'].position[0], 'o', c='orange')


    # for uav in uavlist:     
    #     pts = uav['uavobj'].possibleFlightAreaStatic(area_length=area_length*1.0)
        
    #     if uav['ID'] == 1:
    #         # dist2ncUAVs = uav['dubins'].getOtherUAVStates(uavh_others_all, uavID)

    #         plotCASpos, = plt.plot(uav['dubins'].ys, uav['dubins'].xs, 'o', c='r')
    #         plotMainUAV =  plt.plot(uav['dubins'].y, uav['dubins'].x, 'o', c='k')

    #         color = '-g'
    #         if(astarReplan):
    #             color = '-y'
    #             replan = False
    #         plotCAScone, = plt.plot([pt[1] for pt in pts], [pt[0] for pt in pts], color)
    #         # plotCurrentWypt = plt.plot(uav['dubins'].waypoints[uav['dubins'].currentWPIndex][1], uav['dubins'].waypoints[uav['dubins'].currentWPIndex][0], c='black', marker='X', markersize=10)
    #         # plotCASkoz, = plt.plot([pt[1] for pt in avoid[0]], [pt[0] for pt in avoid[0]], '--m')

    #         # cross track error measurement currently broken...   
    #         # crossError, m, b = crossTrackError(PruitTrack, [uav['dubins'].x, uav['dubins'].y]) 

    #         uav['dubins'].simulateWPDubins(UseCarrotChase=False, delta=0.01)
    #         carrot = uav['dubins'].CarrotChaseWP(delta=0.01)
    #         CASuavPos = uav['dubins'].position

    #         # clearWPList(ActivePath_List, ReferencePath_List, lap_Counter, uav, wptRad )
    #         if uav['dubins'].lapCounter > lap_Counter:
    #             # for i in range(0, len(ActivePath_List)):
    #             #     if ActivePath_List[i]['Is Go-To'] == True:
    #             #         uav['dubins'].currentWPIndex = i
    #             #         break

    #             ActivePath_List = ReferencePath_List[:]
    #             TargetWPList = []
    #             for i in range(0, len(ReferencePath_List)):
    #                 TargetWPList.append(ActivePath_List[i]['pt'])

    #             uav['dubins'].currentWPIndex = 0
    #             uav['dubins'].setWaypoints(TargetWPList, newradius=wptRad)

    #         lap_Counter= uav['dubins'].lapCounter 

    #     if uav['IsAvoidanceUAV'] == False:
 
    #         plotNCpos, = plt.plot(uav['uavobj'].position[1], uav['uavobj'].position[0], 'o', c='orange')
    #         # plotNCpos, = plt.plot(uav['dubins'].ys, uav['dubins'].xs, 'o', c='orange')
    #         NCactiveWPt = uav['dubins'].getActiveWaypoint()
    #         plt.plot( NCactiveWPt[1], NCactiveWPt[0], c='k', marker='v', markersize = 8 )

    #         plotNCcone, = plt.plot([pt[1] for pt in pts], [pt[0] for pt in pts], "-r")
    #         uav['dubins'].simulateWPDubins(UseCarrotChase=False, delta=0.01)
    #         NCuavPos = uav['dubins'].position


    #     uav = uavData.syncAVSfromDubins(uav)

    # print('\nStep: ' + str(step) + '\tUAV ID: ' + str(uav['ID']) + '\tLap:' + str(uav['dubins'].lapCounter) + '\tCurrent wpt: ' + str(uav['dubins'].currentWPIndex) + 
    #         '\tUAV Heading (deg): ' + str(round(uav['dubins'].heading,2)) + 
    #         ' (' + str(round(np.degrees(uav['dubins'].heading),2)) + ')'   )


    '''======================
       ===== Plotting =======
       ====================== '''

    if Show_AstarPlan:
        plot_AstarPlan = plt.plot([pt[1] for pt in astarwpts.tolist()], [pt[0] for pt in astarwpts.tolist()], c = 'k', marker='*', markersize=8)
        for pts in astarwpts.tolist():
                wptCircle = plt.Circle((pts[1], pts[0]), wptRad, color='green', alpha=0.2)
                plt.gca().add_artist(wptCircle)
                plt.scatter(pts[1],pts[0])

    if Show_RecoveryPlan :
        plot_RecoveryPlan = plt.plot([pt[1] for pt in RecoveryPathWPs], [pt[0] for pt in RecoveryPathWPs], c = 'g', marker='o', markersize=8)
        for pts in RecoveryPathWPs:
                wptCircle = plt.Circle((pts[1], pts[0]), wptRad, color='green', alpha=0.2)
                plt.gca().add_artist(wptCircle)
                plt.scatter(pts[1],pts[0])

    if Show_RecoveryPlan :
        plot_RecoveryPlan_Full = plt.plot([pt[1] for pt in pltPts_List], [pt[0] for pt in pltPts_List], c = 'b', marker='.', markersize=2)



    # Plot the A* goal and interpolated points used for distance calcualtion
    # if detect:
    #     plt.plot([pt[1] for pt in pointList], [pt[0] for pt in pointList], c='k', marker='.', markersize = 8)
    #     plt.plot(astarGoalPt[1], astarGoalPt[0], c='r', marker='*', markersize = 12)


    plt.plot([pt[1] for pt in PruitTrack], [pt[0] for pt in PruitTrack], c='b', marker='.', markersize=8)
    plt.plot([pt[1] for pt in PruitTrack_reverse], [pt[0] for pt in PruitTrack_reverse], c='r', marker='o', markersize=8)

    for pts in PruitTrack:
            wptCircle = plt.Circle((pts[1], pts[0]), wptRad, color='green', alpha=0.2)
            plt.gca().add_artist(wptCircle)
            plt.scatter(pts[1],pts[0])

    plotCurrentWypt = plt.plot(mainUAV['dubins'].waypoints[mainUAV['dubins'].currentWPIndex][1], mainUAV['dubins'].waypoints[mainUAV['dubins'].currentWPIndex][0], c='black', marker='X', markersize=10)
    NCactiveWPt = uavlist[1]['dubins'].getActiveWaypoint()
    plt.plot( NCactiveWPt[1], NCactiveWPt[0], c='k', marker='v', markersize = 10 )

    text = ("Simulation Step: " + str(step) + "  Lap: " + str(lap_Counter) + "\nCAS Target wypt (Total): " + str(uavlist[0]['dubins'].currentWPIndex) + '(' + str(len(ActivePath_List)) + ')' +
            "\nA* Index2Watch: " + str(Astar_Index2Watch4) + "\nA* Return Index: " + str(Astar_Return_Index) + "\nRecovery Index2Watch: " + str(Recover_Index2Watch4) + 
            "\nRecovery Return Index: " + str(Recovery_Return_Index)) + "\nMaking A Lap: " + str(lap)

    if hasAstarPlan == False and hasRecoveryPlan == False:
        text1 = ("Currently Following: Ref Path")
    elif hasAstarPlan == True and hasRecoveryPlan == False:
        text1 = ("Currently Following: A* path")    
    elif hasAstarPlan == False and hasRecoveryPlan == True:
        text1 = ("Currently Following: Recovery Path")


    plt.text(0.7, 0.05, text, transform=ax.transAxes)
    plt.text(0.7, 0.9, text1, transform=ax.transAxes)

    ax.axis('equal')
    plt.ylim((39.32450, 39.32657))
    plt.xlim((-82.11100, -82.1090))

    fig.set_size_inches((12, 10))  
    plt.grid(True)
    plt.pause(0.01)
    # plt.show()

    makeMovie = True
    if makeMovie:
        ''' 
        Save frames for a movie 
        '''
        wd = os.getcwd()
        path=(wd + '/Movies')
        # fname = '_tmp%03d.png' % step
        # fname = os.path.join(path,fname)
        # plt.savefig(fname)

        if step < 500:
            fname = '_tmpz%03d.png' % step
            fname = os.path.join(path,fname)
            plt.savefig(fname)
            savePlots.append(fname)

        elif step >= 500 and step < 1000:
            fname1 = '_tmpa%03d.png' % step
            fname1 = os.path.join(path,fname1)
            plt.savefig(fname1)        
            savePlots1.append(fname1)

        elif step >= 1000 and step < 1500:
            fname2 = '_tmpb%03d.png' % step
            fname2 = os.path.join(path,fname2)
            plt.savefig(fname2)
            savePlots2.append(fname2)  

        elif step >= 1500 and step < 2000:
            fname3 = '_tmpc%03d.png' % step
            fname3 = os.path.join(path,fname3)
            plt.savefig(fname3)
            savePlots3.append(fname3)  

        elif step >= 2000 and step < 2500:
            fname4 = '_tmpd%03d.png' % step
            fname4 = os.path.join(path,fname4)
            plt.savefig(fname4)
            savePlots4.append(fname4)  

        elif step >= 2500 and step < 3000:
            fname5 = '_tmpe%03d.png' % step
            fname5 = os.path.join(path,fname5)
            plt.savefig(fname5)
            savePlots5.append(fname5)  

        elif step >= 3000 and step < 3500:
            fname6 = '_tmpf%03d.png' % step
            fname6 = os.path.join(path,fname6)
            plt.savefig(fname6)
            savePlots6.append(fname6)  




    plt.clf()
    step+=1


if makeMovie:

    '''
    Make a Movie
    '''
    print('Change directory to Movies... ')
    wd = os.getcwd()
    path = ('Movies')
    newDir = os.path.join(wd,path)
    os.chdir(str(newDir))
    print('Making sim movie...')
    # source: https://matplotlib.org/gallery/animation/movie_demo_sgskip.html
    # additional resource: https://linux.die.net/man/1/mencoder
    subprocess.call("mencoder 'mf://_tmpz*.png' -mf type=png:fps=10 -ovc lavc "
                    "-lavcopts vcodec=mpeg4 -oac copy -o animation.mp4", shell=True)

    subprocess.call("mencoder 'mf://_tmpa*.png' -mf type=png:fps=10 -ovc lavc "
                    "-lavcopts vcodec=mpeg4 -oac copy -o animation1.mp4", shell=True)

    subprocess.call("mencoder 'mf://_tmpb*.png' -mf type=png:fps=10 -ovc lavc "
                    "-lavcopts vcodec=mpeg4 -oac copy -o animation2.mp4", shell=True)

    subprocess.call("mencoder 'mf://_tmpc*.png' -mf type=png:fps=10 -ovc lavc "
                    "-lavcopts vcodec=mpeg4 -oac copy -o animation3.mp4", shell=True)

    subprocess.call("mencoder 'mf://_tmpd*.png' -mf type=png:fps=10 -ovc lavc "
                    "-lavcopts vcodec=mpeg4 -oac copy -o animation4.mp4", shell=True)

    subprocess.call("mencoder 'mf://_tmpe*.png' -mf type=png:fps=10 -ovc lavc "
                    "-lavcopts vcodec=mpeg4 -oac copy -o animation5.mp4", shell=True)

    subprocess.call("mencoder 'mf://_tmpf*.png' -mf type=png:fps=10 -ovc lavc "
                    "-lavcopts vcodec=mpeg4 -oac copy -o animation6.mp4", shell=True)