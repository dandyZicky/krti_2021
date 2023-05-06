#!/usr/bin/env python3
from sensor_msgs.msg import Image
import imagezmq
import rospy
import cv2
import time
import imagezmq
import simplejpeg
import socket

from mavros_msgs.msg import State, WaypointReached
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped, Vector3
from std_msgs.msg import Bool, Float64, Int32, Header
from cv_bridge import CvBridge

from ellipse_detector import detect_max_ellipse
from preprocessing import mask_img
from dropzone_detector import get_dropzone



RECORD_PORT_EFFECT = False
ENCODED_FPS = 26
LOOP_RATE = 60
LIVE_STREAMING = True
STREAMING_RECEIVER_IP = 'tcp://*:5555'
JPEG_QUALITY = 30


# Centering Constant for dropping and picking
CENTER_X_DROP = 320
CENTER_Y_DROP = 240

# Centering Constant for landing
CENTER_X = 320
CENTER_Y = 240

CENTERING_LIMIT_DROP = 120

def drawPlus(img,x,y,len,color):
    cv2.line(img,(x, y-len),(x, y+len), color, 2)
    cv2.line(img,(x-len, y),(x+len, y), color, 2)

        
if __name__ == "__main__":

    
    rospy.init_node("vision_node",anonymous=True)
    rospy.loginfo("Vision Node created")

    br = CvBridge()
    img = None
    def img_cb(data):
        global img
        img = br.imgmsg_to_cv2(data, desired_encoding="bgr8")
    rospy.Subscriber("/webcam2/image_raw2", Image, img_cb)


    rate = rospy.Rate(LOOP_RATE)
    
    rpi_name = socket.gethostname() # send RPi hostname with each image

    # initialize camera channel
    try:
        while img is None:
            cap = img
    except TypeError:
        print("wait....")
    rospy.loginfo("Intialize camera")
    # initialize video recording
    fourcc = cv2.VideoWriter_fourcc('m','p','4','v')
    rospy.loginfo(r'Camera initialized')


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

    cmd_vel = Vector3()
    def cmd_vel_cb(data):
        global cmd_vel
        cmd_vel = data.twist.linear
    rospy.Subscriber("/mavros/setpoint_velocity/cmd_vel", TwistStamped, cmd_vel_cb)

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

    drop_color_pub = rospy.Publisher("/gmfc/drop_color", Int32, queue_size=1)
    drop_color = Int32()

    previous_time = 0
    frame_count = 0

    with imagezmq.ImageSender(connect_to=STREAMING_RECEIVER_IP, REQ_REP=False) as sender:
        while not rospy.is_shutdown():
            frame = img.copy()
            drop_color = 0
            
            mask = mask_img(frame)
            ellipse = detect_max_ellipse(mask)    
            dropzone = get_dropzone(frame, 1)

            if dropzone[2]:
                dropzone_is_detected.data = True
                dropzone_is_detected_pub.publish(dropzone_is_detected)
                dropzone_target_x = dropzone[0]
                dropzone_target_y = dropzone[1]
                # centering_limit_drop = int(dropzone[3]/3) # from control.py
                centering_limit_drop = CENTERING_LIMIT_DROP # from control.py
                dropzone_target_x_pub.publish(dropzone_target_x)
                dropzone_target_y_pub.publish(dropzone_target_y)
                # centering_drop_pub.publish(centering_limit_drop)
                delta_x = CENTER_X - dropzone_target_x
                delta_y = CENTER_Y - dropzone_target_y
                cv2.polylines(frame, dropzone[2], True, (255,0,255),3)
                drawPlus(frame, dropzone_target_x, dropzone_target_y, 15, (0,0,255))
                drawPlus(frame, CENTER_X, CENTER_Y, 20, (255,0,0))       # plus in Copter's center
                cv2.circle(frame, (dropzone_target_x,dropzone_target_y), centering_limit_drop, (0,0,0))                    

            else:
                pass
                dropzone_is_detected.data = False
                dropzone_is_detected_pub.publish(dropzone_is_detected)

            # if ellipse:
            #     elp_is_detected.data = True
            #     elp_is_detected_pub.publish(elp_is_detected)
            #     x, y, w, h, a = ellipse
            #     ellipse_half_width = int(w // 2)
            #     ellipse_half_height = int(h // 2)
            #     centering_limit = 120 # from control.py
            #     elp_target_x = x
            #     elp_target_y = y
            #     elp_target_x_pub.publish(elp_target_x)
            #     elp_target_y_pub.publish(elp_target_y)
            #     # centering_land_pub.publish(centering_limit)
            #     cv2.ellipse(frame, (x, y), (ellipse_half_width, ellipse_half_height), a, 0, 360, (0, 0, 255), 5)
            #     drawPlus(frame, elp_target_x, elp_target_y, 10, (0,0,0)) # plus in ELP
            #     drawPlus(frame, CENTER_X, CENTER_Y, 20, (255,0,0))       # plus in Copter's center
            #     cv2.circle(frame, (elp_target_x,elp_target_y), centering_limit, (0,0,0))

            # else:
            #     pass
            #     elp_is_detected.data = False
            #     elp_is_detected_pub.publish(elp_is_detected)

            
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
                        f'WP: {wp_reached}',
                        (10,160),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0,255,0),
                        2)
            
            cv2.putText(frame,
                        f'FPS: {fps}',
                        (10, 200),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 255, 0),
                        2)

            if LIVE_STREAMING:
                jpg_buffer     = simplejpeg.encode_jpeg(frame, quality=JPEG_QUALITY,
                                                        colorspace='BGR')
                sender.send_jpg(rpi_name, jpg_buffer)

            cv2.imshow("Result", frame)
            if cv2.waitKey(10) & 0xff == 27:
                break
            
            rate.sleep()

        rospy.loginfo('Releasing resources...')
        cap.release()
        rospy.loginfo('target_detection.py is shut down')