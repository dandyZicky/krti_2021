#!/usr/bin/env python3

import rospy
import time
import math
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State, WaypointReached, PositionTarget
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped
from std_msgs.msg import Header, Bool, Int32, Float64
from mechanism.serial_servo import Mechanism 

MISI = 4

"""
Centering constants for new dropping mechanism
1: Kiri
2: Tengah
3: Kanan
"""

# CENTER_X_DROP1 = 396
# CENTER_Y_DROP1 = 200

# CENTER_X_DROP2 = 426
# CENTER_Y_DROP2 = 200

# CENTER_X_DROP3 = 456
# CENTER_Y_DROP3 = 200

# Constant for centering (landing)
CENTER_X = 425
CENTER_Y = 235           
CENTERING_LIMIT = 200           # centering tolerance to get lower

# Constant for loop
LOOP_RATE = 4               # used for 4hz loop
LOOP_RATE_FASTER = 10        # used for faster 5hz loop

X_GAIN = 0.001               # x axis gain for centering-land velocity (East X+)
Y_GAIN = 0.001                # y axis gain for centering-land velocity (North Y+)
Z_GAIN = -0.8                   # z axis gain for centering-land velocity(Up Z+)
Z_GAIN_UP = 1.60                 # z axis gain for centering-land safety mode velocity (Up Z+)
X_GAIN_DROP = 0.0020            # x axis gain for centering dropping velocity (East X+)
Y_GAIN_DROP = 0.0020           # y axis gain for centering dropping velocity (North Y+)
CENTERING_TIME_LIMIT = 12        # copter will go up if not detected landing pad when centering
CENTERING_DROP_TIME_LIMIT = 12   # copter will force drop if not detected landing pad when centering
ALT_SAFETY_LIMIT = 4            # copter will go to land mode if not detected ELP until ALT_SAFETY_LIMIT
DELAY_AFTER_DROP = 0.2            # copter remains centering after drop for DELAY_AFTER_DROP second

# Constant for navigation
WP_FIRST = 1                # waypoint for the first gedung and so forth
# WP_SECOND = 2               
# WP_THIRD = 3               
WP_CENTERING = 2            # waypoint for landing
ALT_TAKEOFF = 2.0             # target altitude for takeoff
ALT_FOR_LAND = 1.5         # altitude threshold to change mode to LAND
KD = 0.000008

# def servo(misi):
#     if misi == 1:
#         return ('1', '2', '3', CENTER_X_DROP1, CENTER_Y_DROP1, CENTER_X_DROP2, CENTER_Y_DROP2, CENTER_X_DROP3, CENTER_Y_DROP3)
#     elif misi == 2:
#         return ('1', '3', '2', CENTER_X_DROP1, CENTER_Y_DROP1, CENTER_X_DROP3, CENTER_Y_DROP3, CENTER_X_DROP2, CENTER_Y_DROP2)
#     elif misi == 3:
#         return ('2', '1', '3', CENTER_X_DROP2, CENTER_Y_DROP2, CENTER_X_DROP1, CENTER_Y_DROP1, CENTER_X_DROP3, CENTER_Y_DROP3)
#     elif misi == 4:
#         return ('2', '3', '1', CENTER_X_DROP2, CENTER_Y_DROP2, CENTER_X_DROP3, CENTER_Y_DROP3, CENTER_X_DROP1, CENTER_Y_DROP1)
#     elif misi == 5:
#         return ('3', '1', '2', CENTER_X_DROP3, CENTER_Y_DROP3, CENTER_X_DROP1, CENTER_Y_DROP1, CENTER_X_DROP2, CENTER_Y_DROP2)
#     elif misi == 6:
#         return ('3', '2', '1', CENTER_X_DROP3, CENTER_Y_DROP3, CENTER_X_DROP2, CENTER_Y_DROP2, CENTER_X_DROP1, CENTER_Y_DROP1)
#     else:
#         return 0 

def set_drop(cmd):
    global drop_counter
    drop_counter_pub = rospy.Publisher("/gmfc/drop_counter", Int32, queue_size=1)
    drop_counter.data -= 1
    if drop_counter.data < 0:
        drop_counter.data = 0
    
    drop_mech.drop_now(cmd)

    while drop_counter_pub.get_num_connections() < 2:
        rate_fast.sleep()
    # wait until there's connection
    print(drop_counter)
    drop_counter_pub.publish(drop_counter)

def set_arm():
    arm_client = rospy.ServiceProxy("/mavros/cmd/arming",CommandBool)
    arm_client(True)
    rospy.loginfo("ARMING..")

def set_disarm():
    arm_client = rospy.ServiceProxy("/mavros/cmd/arming",CommandBool)
    arm_client(False)
    rospy.loginfo("DISARMING..")

def set_land():
    mode_client = rospy.ServiceProxy("/mavros/set_mode",SetMode)
    mode_client(0,'LAND')
    rospy.loginfo("Setting mode to LAND..")

def set_auto():
    mode_client = rospy.ServiceProxy("/mavros/set_mode",SetMode)
    mode_client(0,'AUTO')
    rospy.loginfo("Setting mode to AUTO..")

def set_guided():
    mode_client = rospy.ServiceProxy("/mavros/set_mode",SetMode)
    mode_client(0,'GUIDED')
    rospy.loginfo("Setting mode to GUIDED..")

def set_guided_nogps():
    mode_client = rospy.ServiceProxy("/mavros/set_mode",SetMode)
    mode_client(0,'GUIDED_NOGPS')
    rospy.loginfo("Setting mode to Guided No GPS..")

def set_takeoff(alt_target):
    takeoff_client = rospy.ServiceProxy("/mavros/cmd/takeoff",CommandTOL)
    takeoff_client(0,0,0,0,alt_target)
    rospy.loginfo(f"Taking off... Takeoff altitude target = {alt_target}")

def set_rtl():
    mode_client = rospy.ServiceProxy("/mavros/set_mode",SetMode)
    mode_client(0,'RTL')
    rospy.loginfo("Setting mode to Guided No GPS..")

def center_and_drop():
    rospy.logwarn("STARTED CENTERING FOR DROPPING!")
    previous_time = rospy.Time.now()

    cmd_vel_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=1)
    while cmd_vel_pub.get_num_connections() < 1:
        rate_fast.sleep()
        
    cmd_vel_data = PositionTarget()
    cmd_vel_data.coordinate_frame = 8
    cmd_vel_data.type_mask = 0

    # if wp_reached == WP_FIRST:
    #     CenterX = urutan[3]
    #     CenterY = urutan[4]
    # elif wp_reached == WP_SECOND:
    #     CenterX = urutan[5]
    #     CenterY = urutan[6]
    # elif wp_reached == WP_THIRD:
    #     CenterX = urutan[7]
    #     CenterY = urutan[8]
    # else:
    CenterX = CENTER_X
    CenterY = CENTER_Y

    delta_previous_x = delta_previous_y = 10.0

    while not rospy.is_shutdown():

        if is_dropzone_detected:

            # delta.flat = [target_x_dropzone-CenterX, -(target_y_dropzone-CenterY)]
            delta_x = CenterY - target_y_dropzone  # from ENU to NWU
            delta_y = CenterX - target_x_dropzone

            vel_x = delta_x * X_GAIN_DROP + KD*(delta_x-delta_previous_x)
            vel_y = delta_y * Y_GAIN_DROP + KD*(delta_y-delta_previous_y)

            delta_previous_x = delta_x
            delta_previous_y = delta_y

            if(-centering_limit_drop< (target_x_dropzone - CenterX) < centering_limit_drop \
                and -centering_limit_drop < (target_y_dropzone - CenterY) < centering_limit_drop):
                # if wp_reached == WP_FIRST:
                #     set_drop(urutan[0])
                # elif wp_reached == WP_SECOND:
                #     set_drop(urutan[1])
                # elif wp_reached == WP_THIRD:
                #     set_drop(urutan[2])
                set_drop('1')
                previous_time_afdrop = rospy.Time.now()
                current_time_afdrop = previous_time_afdrop
                delta_time_afdrop = current_time_afdrop - previous_time_afdrop
                rospy.logwarn("DELAY AFTER DROP, START.....")
                while delta_time_afdrop.secs < DELAY_AFTER_DROP:
                    current_time_afdrop = rospy.Time.now()
                    delta_time_afdrop = current_time_afdrop - previous_time_afdrop
                    delta_x = CenterY - target_y_dropzone
                    delta_y = CenterX - target_x_dropzone
                    vel_x = delta_x * X_GAIN_DROP + KD*(delta_x-delta_previous_x)
                    vel_y = delta_y * Y_GAIN_DROP + KD*(delta_y-delta_previous_y) 
                    cmd_vel_data.velocity.x = vel_x
                    cmd_vel_data.velocity.y = vel_y
                    cmd_vel_data.velocity.z = 0
                    cmd_vel_pub.publish(cmd_vel_data)
                    rate.sleep()
                break
            else:
                vel_z = 0
            previous_time = rospy.Time.now()
        else:
            current_time = rospy.Time.now()
            delta_time = current_time - previous_time
            vel_x = vel_y = 0
            vel_z = -0.10
            if delta_time.secs > CENTERING_DROP_TIME_LIMIT:
                # if wp_reached == WP_FIRST:
                #     set_drop(urutan[0])
                # elif wp_reached == WP_SECOND:
                #     set_drop(urutan[1])
                # elif wp_reached == WP_THIRD:
                #     set_drop(urutan[2])
                previous_time_afdrop = rospy.Time.now()
                current_time_afdrop = previous_time_afdrop
                delta_time_afdrop = current_time_afdrop - previous_time_afdrop
                rospy.logwarn("DELAY AFTER FORCED DROP, START....")
                while delta_time_afdrop.secs < DELAY_AFTER_DROP:
                    current_time_afdrop = rospy.Time.now()
                    delta_time_afdrop = current_time_afdrop - previous_time_afdrop
                    cmd_vel_data.velocity.x = 0
                    cmd_vel_data.velocity.y = 0
                    cmd_vel_data.velocity.z = 0
                    cmd_vel_pub.publish(cmd_vel_data)
                    rate.sleep()
                break
        
        cmd_vel_data.velocity.x = vel_x
        cmd_vel_data.velocity.y = vel_y
        cmd_vel_data.velocity.z = vel_z


        rospy.logwarn(f'\n------------------\nVelocity Value\n{cmd_vel_data.velocity}\n----------------------\n')
        cmd_vel_pub.publish(cmd_vel_data)

        rate.sleep()


def center_and_land():
    rospy.loginfo("Centering Start!")
    # Constant for centering-land safety mode
    previous_time = rospy.Time.now()

    cmd_vel_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=1)
    while cmd_vel_pub.get_num_connections() < 1:
        # wait until there's connection
        rate_fast.sleep()
    cmd_vel_data = PositionTarget()
    cmd_vel_data.coordinate_frame = 8
    cmd_vel_data.type_mask = 0

    while not rospy.is_shutdown():

        if is_elp_detected:
            delta_x = CENTER_Y - target_y
            delta_y = CENTER_X - target_x

            vel_x = delta_x * X_GAIN
            vel_y = delta_y * Y_GAIN # invert

            if(-CENTERING_LIMIT< (target_x - CENTER_X) < CENTERING_LIMIT \
                and -CENTERING_LIMIT < (target_y - CENTER_Y) < CENTERING_LIMIT):
                vel_z = Z_GAIN
            else:
                vel_z = 0
            previous_time = rospy.Time.now()
        else:
            current_time = rospy.Time.now()
            delta_time = current_time - previous_time
            if delta_time.secs > CENTERING_TIME_LIMIT:
                vel_x = 0
                vel_y = 0
                vel_z = Z_GAIN_UP
                if alt > ALT_SAFETY_LIMIT:
                    while(fcu_state.mode!='LAND'):
                        set_land()
                        rate.sleep()
                    break
            else:
                vel_x = 0
                vel_y = 0
                vel_z = 0
        
        cmd_vel_data.velocity.x = vel_x
        cmd_vel_data.velocity.y = vel_y
        cmd_vel_data.velocity.z = vel_z
        rospy.loginfo(f'{cmd_vel_data}')
        cmd_vel_pub.publish(cmd_vel_data)

        if(alt < ALT_FOR_LAND):
            set_land()
            break

        rate.sleep()

def guided_and_arm():

    while(fcu_state.mode!='GUIDED'):
       set_guided()
       rate.sleep()

    time.sleep(0.25)

    rospy.loginfo("Arming...")
    while(not fcu_state.armed):
        set_arm()
        rate.sleep()

    time.sleep(0.5)

def main_mission_alpha(alt_takeoff=ALT_TAKEOFF, wp_centering=WP_FIRST):
# Takeoff-WP1-Centering-Land
    guided_and_arm()

    set_takeoff(alt_target=alt_takeoff)

    while alt < (alt_takeoff*0.8):
        rospy.loginfo(alt)
        rate.sleep()

    rospy.loginfo("Done takeoff")

    time.sleep(0.5)
    while(fcu_state.mode!='AUTO'):
        set_auto()
        rate.sleep()

    while(wp_reached < wp_centering):
        rospy.loginfo(wp_reached)
        rate.sleep()

    while(fcu_state.mode!='GUIDED'):
        set_guided()
        rate.sleep()
    time.sleep(0.5)

    center_and_land()

def main_mission_beta(alt_takeoff=ALT_TAKEOFF,wp_one=WP_FIRST, wp_centering=WP_CENTERING):
# Takeoff-WP1-Drop-WP2-Drop-WP3-Drop-WP4-Centering-Land

    mission_status=[
            False, # 0:Takeoff
            False, # 1:Waypoint 1
            False, # 2:Delay 1
            False, # 3:Waypoint 2
            False, # 4:Delay 2
            False, # 5:Waypoint 3
            False, # 6:Delay 3
            False, # 7:Waypoint ELP
            False, # 8:Delay ELP
            ]

    guided_and_arm()

    set_takeoff(alt_target=alt_takeoff)

    while alt < (alt_takeoff*0.8):
        rospy.loginfo(alt)
        rate.sleep()

    rospy.loginfo("Done takeoff")
    while(fcu_state.mode!='AUTO'):
        set_auto()
        rate.sleep()

    while(wp_reached < wp_centering):
        if wp_reached == wp_one and (not mission_status[2]):
            # set_drop(urutan[0])
            mission_status[2]=True
        if wp_reached == wp_two and (not mission_status[4]):
            # set_drop(urutan[1])
            mission_status[4]=True
        if wp_reached == wp_three and (not mission_status[6]):
            # set_drop(urutan[2])
            mission_status[6]=True
        rate_fast.sleep()

    while(fcu_state.mode!='GUIDED'):
        set_guided()
        rate.sleep()

    center_and_land()

def main_mission_gamma(alt_takeoff=ALT_TAKEOFF,wp_one=WP_FIRST, wp_centering=WP_CENTERING):
# Takeoff-WP1-Drop-WP2-Drop-WP3-Drop-WP4-Centering-Land
    mission_status=[
            False, # 0:Takeoff
            False, # 1:Waypoint 1
            False, # 2:Delay 1
            False, # 3:Waypoint 2
            False, # 4:Delay 2
            False, # 5:Waypoint 3
            False, # 6:Delay 3
            False, # 7:Waypoint ELP
            False, # 8:Delay ELP
            ]

    rospy.loginfo(mission_status)

    guided_and_arm()

    set_takeoff(alt_target=alt_takeoff)

    while alt < (alt_takeoff*0.8):
        rospy.loginfo(alt)
        rate.sleep()

    rospy.loginfo("Done takeoff")
    while(fcu_state.mode!='AUTO'):
        set_auto()
        rate.sleep()

    while(wp_reached < wp_centering):
        
        if wp_reached == wp_one and (not mission_status[2]):
            while(fcu_state.mode!='GUIDED'):
                set_guided()
                rate.sleep()
            center_and_drop()
            set_drop('2')
            rospy.loginfo('WP REACHED 1')
            while(fcu_state.mode!='AUTO'):
                set_auto()
                rate.sleep()
            mission_status[2]=True

        # if wp_reached == wp_two and (not mission_status[4]):
        #     while(fcu_state.mode!='GUIDED'):
        #         set_guided()
        #         rate.sleep()
        #     center_and_drop()
        #     rospy.loginfo('WP REACHED 2')
        #     while(fcu_state.mode!='AUTO'):
        #         set_auto()
        #         rate.sleep()
        #     mission_status[4]=True

        # if wp_reached == wp_three and (not mission_status[6]):
        #     while(fcu_state.mode!='GUIDED'):
        #         set_guided()
        #         rate.sleep()
        #     center_and_drop()
        #     rospy.loginfo('WP REACHED 3')
        #     while(fcu_state.mode!='AUTO'):
        #         set_auto()
        #         rate.sleep()
        #     mission_status[6]=True
        rate_fast.sleep()

    while(fcu_state.mode!='GUIDED'):
        set_guided()
        rate.sleep()

    center_and_land()    

if __name__=="__main__":
    rospy.init_node("control_vtol",anonymous=True)
    start = time.perf_counter()
    rospy.loginfo("Initialize control node")

    drop_mech = Mechanism()
    # urutan = servo(MISI)

    rate = rospy.Rate(LOOP_RATE)
    rate_fast = rospy.Rate(LOOP_RATE_FASTER)

    # Initialize Subscriber
    fcu_state = State()
    def fcu_state_cb(data):
        global fcu_state
        fcu_state = data
    rospy.Subscriber("/mavros/state",State,fcu_state_cb)

    wp_reached = 0
    def wp_reached_cb(data):
        global wp_reached
        wp_reached = data.wp_seq
    rospy.Subscriber("/mavros/mission/reached", WaypointReached, wp_reached_cb)

    alt = 0
    def alt_cb(data):
        global alt
        alt = data.data
    rospy.Subscriber('/mavros/global_position/rel_alt', Float64, alt_cb)

    heading = 0


    vision_header = Header()
    vision_pose = Pose()
    vision_orientation = vision_pose.orientation
    def vision_pose_cb(data):
        global vision_pose, vision_header, vision_orientation
        vision_header = data.header
        vision_pose = data.pose.position
        vision_orientation = data.pose.orientation
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, vision_pose_cb)

    is_elp_detected = False
    def is_elp_detected_cb(data):
        global is_elp_detected
        is_elp_detected = data.data
    rospy.Subscriber("/gmfc/vision/elp_is_detected", Bool, is_elp_detected_cb)    

    target_x = 0
    def target_x_cb(data):
        global target_x
        target_x = data.data
    rospy.Subscriber("/gmfc/vision/elp_target_x", Int32, target_x_cb)

    target_y = 0
    def target_y_cb(data):
        global target_y
        target_y = data.data
    rospy.Subscriber("/gmfc/vision/elp_target_y", Int32, target_y_cb)

    is_dropzone_detected = False
    def is_dropzone_detected_cb(data):
        global is_dropzone_detected
        is_dropzone_detected = data.data
    rospy.Subscriber("/gmfc/vision/dropzone_is_detected", Bool, is_dropzone_detected_cb)    

    target_x_dropzone = 0
    def target_x_dropzone_cb(data):
        global target_x_dropzone
        target_x_dropzone = data.data
    rospy.Subscriber("/gmfc/vision/dropzone_target_x", Int32, target_x_dropzone_cb)

    target_y_dropzone = 0
    def target_y_dropzone_cb(data):
        global target_y_dropzone
        target_y_dropzone = data.data
    rospy.Subscriber("/gmfc/vision/dropzone_target_y", Int32, target_y_dropzone_cb)

    centering_limit_drop = 0
    def centering_limit_drop_cb(data):
        global centering_limit_drop
        centering_limit_drop = data.data
    rospy.Subscriber("/gmfc/centering_drop_limit", Int32, centering_limit_drop_cb)

    centering_limit = 0
    def centering_limit_cb(data):
        global centering_limit
        centering_limit = data.data
    rospy.Subscriber("/gmfc/centering_land_limit", Int32, centering_limit_cb)

    drop_counter = Int32()
    def drop_counter_cb(data):
        global drop_counter
        drop_counter = data
    rospy.Subscriber("/gmfc/drop_counter", Int32, drop_counter_cb)

    # Initialize Publisher
    drop_counter_pub = rospy.Publisher("/gmfc/drop_counter", Int32,queue_size=1)
    drop_counter = 3

    reset_wp = rospy.Publisher("/mavros/mission/reached", WaypointReached, queue_size=1)
    null_wp = WaypointReached()
    null_wp.wp_seq = 0

    while drop_counter_pub.get_num_connections() < 1:
     
    # wait until there's connection
        rate_fast.sleep()

    drop_counter_pub.publish(drop_counter)
    reset_wp.publish(null_wp)

    # Main control
    main_mission_gamma()

    while(fcu_state.armed):
        set_disarm()
        rate.sleep()
    rospy.loginfo('Mission Accomplished!')
    end = time.perf_counter()
    rospy.loginfo(f'Flight Time: {end-start} seconds')