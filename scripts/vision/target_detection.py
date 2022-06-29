#!/usr/bin/env python3
from sensor_msgs.msg import Image
import imagezmq
import rospy
import cv2
import time
import numpy as np
from datetime import datetime
import imagezmq
import simplejpeg
import socket
import os
import sys
from imutils.video import WebcamVideoStream


from mavros_msgs.msg import State, WaypointReached, PositionTarget
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped, Vector3
from std_msgs.msg import Bool, Float64, Int32, Header
from cv_bridge import CvBridge

sys.path.append(os.path.join(os.path.dirname(sys.path[0]), 'mechanism'))
from camera_initialize import init_camera
from ellipse_detector import detect_max_ellipse
from preprocessing import mask_img
from dropzone_detector import get_dropzone

# MISI = 4

# WP FOR GEDUNG B
WP_DROP_B = 2
IS_DROP_B_BEHIND = False
DELTA_CENTERING_B = 0.55
RECORD_PORT_EFFECT = False
ENCODED_FPS = 30
LOOP_RATE = 30
LIVE_STREAMING = True
# STREAMING_RECEIVER_IP = 'tcp://192.168.1.98:5555'
STREAMING_RECEIVER_IP = 'tcp://127.0.0.1:5555'
JPEG_QUALITY = 50
# RED DROPZONE COLOR RANGE
BLUR_KERNEL_WIDTH = 7
# LOW_H1 = 75
# HIGH_H1 = 179
# LOW_H2 = 168
# HIGH_H2 = 185
# LOW_S1 = 255
# HIGH_S1 = 255
# LOW_S2 = 255
# HIGH_S2 = 255
# LOW_V1 = 175
# HIGH_V1 = 208
# LOW_V2 = 175
# HIGH_V2 = 220
OPENING_KERNEL_WIDTH = 1
OPENING_APPLICATION_COUNT = 0
CLOSING_KERNEL_WIDTH = 1
CLOSING_APPLICATION_COUNT = 0
# Centering Constant for dropping
# CENTER_X_DROP1 = 396
# CENTER_Y_DROP1 = 200

# CENTER_X_DROP2 = 426
# CENTER_Y_DROP2 = 200

# CENTER_X_DROP3 = 456
# CENTER_Y_DROP3 = 200

# Constant for centering (landing)
CENTER_X = 320
CENTER_Y = 240           
CENTERING_LIMIT = 200


def drawPlus(img,x,y,len,color):
    cv2.line(img,(x, y-len),(x, y+len), color, 2)
    cv2.line(img,(x-len, y),(x+len, y), color, 2)

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

        
if __name__ == "__main__":
    rospy.init_node("vision_node",anonymous=True)
    rospy.loginfo("Vision Node created")
    # urutan = servo(MISI)
    time.sleep(0.5)

    br = CvBridge()
    img = None
    def img_cb(data):
        global img
        img = br.imgmsg_to_cv2(data, desired_encoding="bgr8")
    rospy.Subscriber("/webcam2/image_raw2", Image, img_cb)


    rate = rospy.Rate(LOOP_RATE)
    
    rpi_name = socket.gethostname() # send RPi hostname with each image
    rospy.loginfo(rpi_name)

    # initialize camera channel
    try:
        while img is None:
            cap = img
    except TypeError:
        print("wait....")
    rospy.loginfo("Intialize camera")
    # frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    # frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # initialize video recording
    fourcc = cv2.VideoWriter_fourcc('M','J','P','G')
    # out = cv2.VideoWriter('/home/vtol/record/'+datetime.now().strftime('%Y-%m-%d_%H:%M:%S') + '.avi', fourcc, ENCODED_FPS, (frame_width, frame_height))
    # rospy.loginfo(f'Camera initialized, dimensions: {frame_width}x{frame_height}')


    # initializing subscribers
    rospy.loginfo('Initializing subscribers...')

    alt = 0
    def alt_cb(data):
        global alt
        alt = data.data
    rospy.Subscriber('/mavros/global_position/rel_alt', Float64, alt_cb)

    fcu_state = State()
    def fcu_state_cb(state):
        global fcu_state
        fcu_state = state
    rospy.Subscriber('/mavros/state', State, fcu_state_cb)

    wp_reached = 0
    def wp_reached_cb(data):
        global wp_reached
        wp_reached = data.wp_seq
    rospy.Subscriber('/mavros/mission/reached',WaypointReached, wp_reached_cb)

    local_header = Header()
    local_pose = Pose()
    def local_pose_cb(data):
        global local_pose, local_header
        local_header = data.header
        local_pose = data.pose
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, local_pose_cb)

    drop_counter = Int32()
    def drop_counter_cb(data):
        global drop_counter
        drop_counter = data
    rospy.Subscriber("/gmfc/drop_counter", Int32, drop_counter_cb)

    cmd_vel = Vector3()
    def cmd_vel_cb(data):
        global cmd_vel
        cmd_vel = data.velocity
    rospy.Subscriber("/mavros/setpoint_raw/local", PositionTarget, cmd_vel_cb)
    
    is_dropping = 1
    def is_dropping_cb(data):
        global is_dropping
        is_dropping = data.data
    rospy.Subscriber("/gmfc/control/isDropping", Bool, is_dropping_cb)
    
    is_landing = 1
    def is_landing_cb(data):
        global is_landing
        is_landing = data.data
    rospy.Subscriber("/gmfc/control/isLanding", Bool, is_landing_cb)


    # initalize publisher
    rospy.loginfo("Initialize publisher...")

    elp_is_detected_pub = rospy.Publisher("/gmfc/vision/elp_is_detected", Bool, queue_size=1)
    elp_is_detected = Bool()
    elp_is_detected.data = False
    elp_is_detected_pub.publish(elp_is_detected)

    elp_target_x_pub = rospy.Publisher("/gmfc/vision/elp_target_x", Int32, queue_size=1)
    elp_target_x = Int32()
    elp_target_x = 0
    elp_target_x_pub.publish(elp_target_x)

    elp_target_y_pub = rospy.Publisher("/gmfc/vision/elp_target_y", Int32, queue_size=1)
    elp_target_y = Int32()
    elp_target_y = 0
    elp_target_y_pub.publish(elp_target_y)

    dropzone_is_detected_pub = rospy.Publisher("/gmfc/vision/dropzone_is_detected", Bool, queue_size=1)
    dropzone_is_detected = Bool()
    dropzone_is_detected.data = False
    dropzone_is_detected_pub.publish(dropzone_is_detected)

    dropzone_target_x_pub = rospy.Publisher("/gmfc/vision/dropzone_target_x", Int32, queue_size=1)
    dropzone_target_x = Int32()
    dropzone_target_x = 0
    dropzone_target_x_pub.publish(dropzone_target_x)

    dropzone_target_y_pub = rospy.Publisher("/gmfc/vision/dropzone_target_y", Int32, queue_size=1)
    dropzone_target_y = Int32()
    dropzone_target_y = 0
    dropzone_target_y_pub.publish(dropzone_target_y)    

    centering_drop_pub = rospy.Publisher("/gmfc/centering_drop_limit", Int32, queue_size=1)
    centering_limit_drop = Int32()
    centering_limit_drop = 0
    centering_drop_pub.publish(centering_limit_drop)

    centering_land_pub = rospy.Publisher("/gmfc/centering_land_limit", Int32, queue_size=1)
    centering_limit = Int32()
    centering_limit = 0
    centering_land_pub.publish(centering_limit)

    drop_counter_pub = rospy.Publisher("/gmfc/drop_counter", Int32, queue_size=1)
    drop_counter.data = 3
    drop_counter_pub.publish(drop_counter)

    previous_time = 0
    frame_count = 0

    with imagezmq.ImageSender(connect_to=STREAMING_RECEIVER_IP, REQ_REP=False) as sender:
        while not rospy.is_shutdown():
            frame = img.copy()
            mask = mask_img(frame)
            ellipse = detect_max_ellipse(mask)
            dropzone = get_dropzone(frame)

            if is_dropping:
                centering_limit_drop = int(dropzone[3]/3) # from control.py
                if dropzone[2]:
                    dropzone_is_detected.data = True
                    dropzone_is_detected_pub.publish(dropzone_is_detected)
                    dropzone_target_x = dropzone[0]
                    dropzone_target_x_pub.publish(dropzone_target_x)
                    dropzone_target_y = dropzone[1]
                    dropzone_target_y_pub.publish(dropzone_target_y)
                    centering_drop_pub.publish(centering_limit_drop)
                    cv2.polylines(frame, dropzone[2], True, (255,0,255),3)
                    drawPlus(frame, dropzone_target_x, dropzone_target_y, 15, (0,0,255))
                    drawPlus(frame, CENTER_X,CENTER_Y, 20, (255,0,0))       # plus in Copter's center
                    cv2.circle(frame, (dropzone_target_x,dropzone_target_y), centering_limit_drop, (0,0,0))

                else:
                    dropzone_is_detected.data = False
                    dropzone_is_detected_pub.publish(dropzone_is_detected)

            if is_landing:
                if ellipse:
                    elp_is_detected.data = True
                    elp_is_detected_pub.publish(elp_is_detected)
                    x, y, w, h, a = ellipse
                    ellipse_half_width = int(w // 2)
                    ellipse_half_height = int(h // 2)
                    # centering_limit = int((ellipse_half_height + ellipse_half_width) / 4) # from control.py
                    elp_target_x = x
                    elp_target_x_pub.publish(elp_target_x)
                    elp_target_y = y
                    elp_target_y_pub.publish(elp_target_y)
                    # centering_land_pub.publish(centering_limit)
                    cv2.ellipse(frame, (x, y), (ellipse_half_width, ellipse_half_height), a, 0, 360, (0, 0, 255), 5)
                    drawPlus(frame, elp_target_x, elp_target_y, 10, (0,0,0)) # plus in ELP
                    drawPlus(frame, CENTER_X, CENTER_Y, 20, (255,0,0))       # plus in Copter's center
                    cv2.circle(frame, (elp_target_x,elp_target_y), CENTERING_LIMIT, (0,0,0))

                else:
                    elp_is_detected.data = False
                    elp_is_detected_pub.publish(elp_is_detected)

            
            # calculate fps

            time_diff = time.time() - previous_time
            if time_diff > 0:
              fps = 1//time_diff
              previous_time = time.time()
            else:
              fps = '-'

            # on every frame
            # write other additional informations
            if fcu_state.connected:
                cur_mode = fcu_state.mode
                fcu_status_color = (0, 255, 0)
            else:
                cur_mode = 'DISCONNECTED'
                fcu_status_color = (0, 0, 255)
            if fcu_state.armed:
                arm_status_color = (0, 255, 0)
            else:
                arm_status_color = (0, 0, 255)
            if elp_is_detected.data:
                elp_status_color = (0, 255, 0)
            else:
                elp_status_color = (0, 0, 255)


            cv2.putText(frame,
                        f'ARM: {fcu_state.armed}' ,
                        (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        arm_status_color,
                        2)
            cv2.putText(frame, 
                        f'MODE: {cur_mode}',
                        (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        0.5, 
                        fcu_status_color,
                        2) 
            cv2.putText(frame,
                        'ALTITUDE: %.2f' % alt,
                        (10, 80),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2)
            cv2.putText(frame,
                        f'POS X: {local_pose.position.x:.2f} Y: {local_pose.position.y:.2f} Z: {local_pose.position.z:.2f}',
                        (10, 100),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2)
            cv2.putText(frame,
                        f'VEL X: {cmd_vel.x:.2f} Y: {cmd_vel.y:.2f} Z: {cmd_vel.z:.2f}',
                        (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2)            
            cv2.putText(frame,
                        f'ELP: {elp_is_detected.data}',
                        (10, 140),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        elp_status_color,
                        2)
            cv2.putText(frame,
                        f'PAYLOAD COUNT: {drop_counter.data}',
                        (10, 160),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2)
            cv2.putText(frame,
                        f'WP: {wp_reached}',
                        (10,180),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0,255,0),
                        2)
            cv2.putText(frame,
                        f'FPS: {fps}',
                        (10, 230),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 255, 0),
                        2)

            # if RECORD_PORT_EFFECT:
            #     out.write(frame)

            # rospy.loginfo_once('Live streaming started!')
            # if LIVE_STREAMING:
            #     jpg_buffer     = simplejpeg.encode_jpeg(frame, quality=JPEG_QUALITY,
            #                                             colorspace='BGR')
            #     sender.send_jpg(rpi_name, jpg_buffer)

            cv2.imshow("Result", frame)
            if cv2.waitKey(1) & 0xff == 27:
                break
            try:
                rate.sleep()
            except:
                rospy.logwarn('rate.sleep() interrupted by shutdown.') 
                rospy.loginfo('Shutting down elp_detection.py...')

    rospy.loginfo('Releasing resources...')
    cap.release()
    # out.release()
    rospy.loginfo('target_detection.py is shut down')
