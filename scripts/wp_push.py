#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from mavros_msgs.srv import WaypointPush, WaypointClear
from mavros_msgs.msg import Waypoint

TARGET_LAT1 = -35.3631847
TARGET_LONG1 = 149.1652344
TARGET_ATT1 = 3.0

TARGET_LAT2 = -35.3631941
TARGET_LONG2 = 149.1652928
TARGET_ATT2 = 3.3

TARGET_LAT3 = -35.3631928
TARGET_LONG3 = 149.1651780
TARGET_ATT3 = 3.5

TARGET1 = (TARGET_LAT1, TARGET_LONG1, TARGET_ATT1)
TARGET2 = (TARGET_LAT2, TARGET_LONG2, TARGET_ATT2)
TARGET3 = (TARGET_LAT3, TARGET_LONG3, TARGET_ATT3)

# TARGET_LIST = TARGET1, TARGET2, TARGET3


def push_waypoint(wl):
    try:
        service = rospy.ServiceProxy('mavros/mission/push', WaypointPush, persistent=True)
        service(start_index=0, waypoints=wl)

        if (service.call(0, wl).success):
            print ('write mission success')
        else:
            print ('write mission error')

    except rospy.ServiceException:
        print ("Service call failed" )


def create_waypoint(delay=1.2):

    waypoint_clear_client()
    wl = []
    
    # Waypoint Initializer
    wp = Waypoint()
    wp.frame = 0
    wp.command = 22  # takeoff
    wp.is_current = False
    wp.autocontinue = False
    wp.param1 = 0  # takeoff altitude
    wp.param2 = 0
    wp.param3 = 0
    wp.param4 = 0
    wp.x_lat = 0
    wp.y_long = 0
    wp.z_alt = 0
    wl.append(wp)

    wp = Waypoint()
    wp.frame = 3
    wp.command = 16  # takeoff
    wp.is_current = True
    wp.autocontinue = False
    wp.param1 = delay  # delay
    wp.param2 = 0
    wp.param3 = 0
    wp.param4 = 0
    wp.x_lat = TARGET1[0]
    wp.y_long = TARGET1[1]
    wp.z_alt = TARGET1[2]
    wl.append(wp)

    wp = Waypoint()
    wp.frame = 3
    wp.command = 16  # takeoff
    wp.is_current = True
    wp.autocontinue = False
    wp.param1 = delay  # delay
    wp.param2 = 0
    wp.param3 = 0
    wp.param4 = 0
    wp.x_lat = TARGET2[0]
    wp.y_long = TARGET2[1]
    wp.z_alt = TARGET2[2]
    wl.append(wp)

    wp = Waypoint()
    wp.frame = 3
    wp.command = 16  # takeoff
    wp.is_current = True
    wp.autocontinue = False
    wp.param1 = delay  # delay
    wp.param2 = 0
    wp.param3 = 0
    wp.param4 = 0
    wp.x_lat = TARGET3[0]
    wp.y_long = TARGET3[1]
    wp.z_alt = TARGET3[2]
    wl.append(wp)

    print(wl)

    try:
        service = rospy.ServiceProxy('mavros/mission/push', WaypointPush, persistent=True)
        service(start_index=0, waypoints=wl)

        if (service.call(0, wl).success):
            print ('write mission success')
        else:
            print ('write mission error')

    except rospy.ServiceException:
        print ("Service call failed" )


def waypoint_clear_client():
    try:
        response = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
        return response.call().success
    except rospy.ServiceException:
        print ("Service call failed")
    return False


if __name__ == '__main__':
    rospy.init_node('waypoint_node', anonymous=True)
    pub = rospy.Publisher('global',String,queue_size=10)
    msg = "Pushing waypoint" # to be used later
    pub.publish(msg)
    create_waypoint()
