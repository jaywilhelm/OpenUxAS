'''
    wp201_obj = factory.createObjectByName("CMASI", "Waypoint")
    wp201_obj.set_Number(201)
    wp201_obj.set_NextWaypoint(202)
    wp201_obj.set_Speed(20.0)
    wp201_obj.set_Latitude(1.5217)
    wp201_obj.set_Longitude(-132.5349)
    wp201_obj.set_Altitude(600.0)
'''

wpts = [
    (1.5306, -132.5521),
    (1.5306, -132.5320),
    (1.5306, -132.5119),
    (1.5176, -132.5119),
    (1.5047, -132.5119),
    (1.5047, -132.5320),
    (1.5047, -132.5521)
]

for i in range(len(wpts)):
    print('wp' + str(i) + '_obj = factory.createObjectByName("CMASI", "Waypoint")')
    print('wp' + str(i) + '_obj.set_Number(' + str(i) + ')')
    if (i + 1 < len(wpts)):
        print('wp' + str(i) + '_obj.set_NextWaypoint(' + str(i+ 1) + ')')
    print('wp' + str(i) + '_obj.set_Speed(20.0)')
    print('wp' + str(i) + '_obj.set_Latitude(' + str(wpts[i][0]) + ')')
    print('wp' + str(i) + '_obj.set_Longitude(' + str(wpts[i][1]) + ')')
    print('wp' + str(i) + '_obj.set_Altitude(600.0)\n')

    
