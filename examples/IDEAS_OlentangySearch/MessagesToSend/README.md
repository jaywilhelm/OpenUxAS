# Waterway Search Example

This is an example of running UxAS service that communicates to the Amase simulation in order to generate plans and an assignment for one of two UAVs to monitor a waterway. For more background see the file 'doc/UxAS_UserManual.pdf'

## Files:

* `cfg_OlentangySearch.xml` - 
* `runUxAS_OlentagySearch.sh` - 
* `runAMASE_OlentangySearch.sh` - 
* `Scenario_OlentangySearch.xml` -
* `MessagesToSend/` - most of the messages in this directory are explained in the document, `WaterwayExample_MessageFLow.pdf`. A few are explained below:
* `MessagesToSend/LINE_Waterway_Deschutes.kml` - a 'kml' file from Google Earth, used to define the line search task.
* `MessagesToSend/KmlToBoundariesTasks.WaterwaySearch.py` -


## Running the Example:
1. Open QGC
2. Open terminal and change directory to the px4 firmware 
3. enter command to change location: 
	export PX4_HOME_LAT=40.07928367570902
	export PX4_HOME_LON=-83.07235656422958
	export PX4_HOME_ALT=1000

3. enter command: 'make px4_sitl_defualt gazebo'
3. Open another termianl and run 'mavproxy.py --master=udp:localhost:14550 --out=udp:localhost:14551 --out=udp:localhost:14552' 
2. open a ternimal window in the directory: "examples/IDEAS_OlentantgySearch/"
2. enter the command: `./runAMASE_OlentangySearch.sh`
3. start the Amase simulation (i.e. push the play button)
4. open another ternimal window in the directory: "examples/IDEAS_OlentantgySearch/"
5. enter the command: `./runUxAS_OlentangySearch.sh`
6. In QGC go to the plan tab > plan > download > upload
7. Switch to the flight tab and start the mission


### What Happens?
* When the Amase simulation starts, two UAVs will be initialized and begin loitering about to different loactions. Note: Amase uses NASA Worldwind for background imagery. If no imagery is available, Amase's background will be black.
* .3 seconds after UxAS starts a line representing the LineSearchTask will appear in Amase
* 5 seconds after UxAS start an AutomationRequest is sent to UxAS which kick off the mission
* Once the plans have been calculated and a UAV is assigned to perform the LineSearchTask, waypoints will be displayed in Amase and the UAV will start following them.
* When the UAV reaches the first waypoint of the LineSearchTask, its sensor will move to follow the river.

### Things to Try:
1. Edit the file, 'Scenario_WaterwaySearch.xml' and change locations of the UAV so the assignment calculation produces different results.


