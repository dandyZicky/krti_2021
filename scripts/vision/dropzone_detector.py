import cv2
from numpy import array, uint8
from math import sqrt


VERTICES = 4
MIN_AREA = 650
"""
HSV_LISTS
    0: Hijau
"""
LOW_HSV1 = [array([0, 127, 132], uint8),]

HIGH_HSV1 = [array([15, 227, 232], uint8),]

LOW_HSV2 = [array([138, 255, 255], uint8),]

HIGH_HSV2 = [array([255, 0, 0], uint8),]

# euclidian distance formula
def calc_distance(p1, p2):
    return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def get_dropzone(img, color):

    if color == 0:
        return (-1, -1, [], -1)

    color_params = {
        1: (LOW_HSV1[0], HIGH_HSV1[0], LOW_HSV2[0], HIGH_HSV2[0], 5, 1, 0, 1, 0),
    }

    low_hsv1, high_hsv1, low_hsv2, high_hsv2, blur_kernel_width, opening_kernel_width, opening_application_count, closing_kernel_width, closing_application_count = color_params[color]

    cx = -1
    cy = -1

    contour_corner = []
    sides_lenght_arr = []

    contour_width = -1

    if blur_kernel_width > 1:
        blur = cv2.medianBlur(img, blur_kernel_width)
    else:
        blur = img
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    if(low_hsv2[0] == 255 and high_hsv2[0] == 0):
        mask = cv2.inRange(hsv, low_hsv1, high_hsv1)

    else:
        mask1 = cv2.inRange(hsv, low_hsv1, high_hsv1)
        mask2 = cv2.inRange(hsv, low_hsv2, high_hsv2)
        mask = cv2.bitwise_or(mask1, mask2)

    if(opening_kernel_width > 1):
        for i in range(opening_application_count):
            opening_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (opening_kernel_width, opening_kernel_width))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, opening_kernel)
    
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
            if (MIN_AREA < area):

                approx = cv2.approxPolyDP(cnt, 0.1*cv2.arcLength(cnt,True),True)

                if len(approx) == VERTICES:
                    M = cv2.moments(cnt)
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    for i in range(VERTICES):
                        contour_corner.append([approx.ravel()[2*i], approx.ravel()[2*i+1]])
                    
                    # calculate length of each side
                    for j in range(VERTICES):
                        sides_lenght_arr.append(calc_distance(contour_corner[j], contour_corner[(j+1)%VERTICES]))

                    # get the max length side
                    contour_width = max(sides_lenght_arr)
                    if area > biggest_area:
                        biggest_area = area
                        cx_target = cx
                        cy_target = cy
                        approx_arr_target = [approx]
                        contour_width_target = contour_width
    return (cx_target,cy_target,approx_arr_target, contour_width_target)

if __name__ == '__main__':
    cap = cv2.VideoCapture(0)

    while True:
        _,frame = cap.read()
        target = get_dropzone(frame)
        cv2.polylines(frame, target[2], True, (0,255,0),3)
        cv2.circle(frame, (target[0],target[1]), 10, (255,0,255),-1)
        cv2.imshow("frame",frame)
        if(cv2.waitKey(10) & 0xFF==27):
            break
    
    cv2.destroyAllWindows()
    cap.release()
