Modified fork of OpenUxAS to communicate with the PX4/Pixhawk Autopilot.

Developed by: Jay Wilhelm (Assistant Professor, Mechanical Engineering, Ohio University)

Tested using SITL PX4 and JMAVSIM using built-in mission of waterway search for UxAS that removed second aircraft.

Successfully interprets  UxAS style waypoints, converts to PX4 format, and queues up MAVLINK messages that are a chain of waypoints. Also, reads the MAVLINK messages to determine aircraft status and formulates aircraft vehicle states message for UxAS.


See github.com/afrl-rq/openuxas for full project details.
