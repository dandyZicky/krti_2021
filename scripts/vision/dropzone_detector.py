import cv2
import numpy as np
from math import sqrt

font = cv2.FONT_HERSHEY_COMPLEX

BLUR_KERNEL_WIDTH = 3
LOW_H1 = 0
HIGH_H1 = 4
LOW_H2 = 138
HIGH_H2 = 255
LOW_S = 74
HIGH_S = 180
LOW_V = 0
HIGH_V = 255
OPENING_KERNEL_WIDTH = 3
OPENING_APPLICATION_COUNT = 1
CLOSING_KERNEL_WIDTH = 13
CLOSING_APPLICATION_COUNT = 1
VERTICES = 4
MIN_AREA = 200

# euclidian distance formula
def calc_distance(p1, p2):
    return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def get_dropzone(img, blur_kernel_width=BLUR_KERNEL_WIDTH, 
            low_hsv1=np.array([LOW_H1, LOW_S, LOW_V], np.uint8), 
            high_hsv1=np.array([HIGH_H1, HIGH_S, HIGH_V], np.uint8), 
            # low_hsv2=np.array([LOW_H2, LOW_S2, LOW_V2], np.uint8), 
            # high_hsv2=np.array([HIGH_H2, HIGH_S2, HIGH_V2], np.uint8), 
            opening_kernel_width=OPENING_KERNEL_WIDTH,
            opening_application_count=OPENING_APPLICATION_COUNT,
            closing_kernel_width=CLOSING_KERNEL_WIDTH,
            closing_application_count=CLOSING_APPLICATION_COUNT,
            vertices=VERTICES,
            min_area=MIN_AREA):
    cx = -1
    cy = -1
    approx_arr = []

    contour_corner = []
    sides_lenght_arr = []

    contour_width = -1

    if blur_kernel_width > 1:
        blur = cv2.medianBlur(img, blur_kernel_width)
    else:
        blur = img
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    # threshold image with the given hsv range
    # use two mask (hsv range may wrap around, e.g. for red)
    mask = cv2.inRange(hsv, low_hsv1, high_hsv1)
    # mask2 = cv2.inRange(hsv, low_hsv2, high_hsv2)
    # mask = cv2.bitwise_or(mask1, mask2)

    # apply morphological transform to reduce noise in mask
    # opening transform to reduce the outer noise
    if(opening_kernel_width > 1):
        for i in range(opening_application_count):
            opening_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (opening_kernel_width, opening_kernel_width))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, opening_kernel)
    # closing transform to reduce the inner noise
    if(closing_kernel_width > 1):
        for i in range(closing_application_count):
            closing_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (closing_kernel_width, closing_kernel_width))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, closing_kernel)

    # Contours detection
    contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    biggest_area = -1
    cx_target = -1
    cy_target = -1
    approx_arr_target = []
    contour_width_target = -1

    if contours :
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if (min_area < area):

                approx = cv2.approxPolyDP(cnt, 0.1*cv2.arcLength(cnt,True),True)

                if len(approx) == vertices:
                    M = cv2.moments(cnt)
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    for i in range(vertices):
                        contour_corner.append([approx.ravel()[2*i], approx.ravel()[2*i+1]])
                        # print(f'CONTOUR CORNER: {contour_corner}')
                    
                    # print("\n\n")
                    # calculate length of each side
                    for j in range(vertices):
                        sides_lenght_arr.append(calc_distance(contour_corner[j], contour_corner[(j+1)%vertices]))
                        # print(f'SIDES_LENGTH: {sides_lenght_arr}')

                    # print("\n\n")

                    # get the max length side
                    contour_width = max(sides_lenght_arr)
                    if area > biggest_area:
                        biggest_area = area
                        cx_target = cx
                        cy_target = cy
                        approx_arr_target = [approx]
                        contour_width_target = contour_width
                        # print(f'LEBAR: {contour_width_target}')
    return (cx_target,cy_target,approx_arr_target, contour_width_target, mask)
    # return the center of dropzone, the array of points, and the length

if __name__ == '__main__':
    cap = cv2.VideoCapture(0)

    while True:
        _,frame = cap.read()
        target = get_dropzone(frame)
        mask = target[4]
        cv2.polylines(frame, target[2], True, (0,255,0),3)
        cv2.circle(frame, (target[0],target[1]), 10, (255,0,255),-1)
        cv2.imshow("frame",frame)
        cv2.imshow("mask", mask)
        if(cv2.waitKey(10) & 0xFF==27):
            break
    
    cv2.destroyAllWindows()
    cap.release()