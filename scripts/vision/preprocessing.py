import cv2
import numpy as np

# values obtained through calibration
# ELP
BLUR_KERNEL_WIDTH = 1
LOW_H1 = 20
HIGH_H1 = 36
LOW_H2 = 0
HIGH_H2 = 255
LOW_S = 80
HIGH_S = 255
LOW_V = 112
HIGH_V = 255
OPENING_KERNEL_WIDTH = 1
OPENING_APPLICATION_COUNT = 0
CLOSING_KERNEL_WIDTH = 1
CLOSING_APPLICATION_COUNT = 0

# mask the image for detection
# use calibration values by default
def mask_img(img, blur_kernel_width=BLUR_KERNEL_WIDTH, 
            low_hsv1=np.array([LOW_H1, LOW_S, LOW_V], np.uint8), 
            high_hsv1=np.array([HIGH_H1, HIGH_S, HIGH_V], np.uint8), 
            low_hsv2=np.array([LOW_H2, LOW_S, LOW_V], np.uint8), 
            high_hsv2=np.array([HIGH_H2, HIGH_S, HIGH_V], np.uint8), 
            opening_kernel_width=OPENING_KERNEL_WIDTH,
            opening_application_count=OPENING_APPLICATION_COUNT,
            closing_kernel_width=CLOSING_KERNEL_WIDTH,
            closing_application_count=CLOSING_APPLICATION_COUNT):
  # preprocess before masking
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
    
  return mask

# masking calibration
if __name__ == '__main__':
  # define placeholder callback
  def nothing(val):
    pass

  # create window with trackbar
  cv2.namedWindow('preprocessing_calibration')
  cv2.createTrackbar('BLUR_KERNEL_WIDTH', 'preprocessing_calibration', (BLUR_KERNEL_WIDTH - 1)//2, 6, nothing)
  cv2.createTrackbar('LOW_H1', 'preprocessing_calibration', LOW_H1, 255, nothing)
  cv2.createTrackbar('HIGH_H1', 'preprocessing_calibration', HIGH_H1, 255, nothing)
  cv2.createTrackbar('LOW_H2', 'preprocessing_calibration', LOW_H2, 255, nothing)
  cv2.createTrackbar('HIGH_H2', 'preprocessing_calibration', HIGH_H2, 255, nothing)
  cv2.createTrackbar('LOW_S1', 'preprocessing_calibration', LOW_S, 255, nothing)
  cv2.createTrackbar('HIGH_S1', 'preprocessing_calibration', HIGH_S, 255, nothing)
  # cv2.createTrackbar('LOW_S2', 'preprocessing_calibration', LOW_S2, 255, nothing)
  # cv2.createTrackbar('HIGH_S2', 'preprocessing_calibration', HIGH_S2, 255, nothing)
  cv2.createTrackbar('LOW_V1', 'preprocessing_calibration', LOW_V, 255, nothing)
  cv2.createTrackbar('HIGH_V1', 'preprocessing_calibration', HIGH_V, 255, nothing)
  # cv2.createTrackbar('LOW_V2', 'preprocessing_calibration', LOW_V2, 255, nothing)
  # cv2.createTrackbar('HIGH_V2', 'preprocessing_calibration', HIGH_V2, 255, nothing)
  cv2.createTrackbar('OPENING_KERNEL_WIDTH', 'preprocessing_calibration', (OPENING_KERNEL_WIDTH - 1)//2, 6, nothing)
  cv2.createTrackbar('OPENING_APPLICATION_COUNT', 'preprocessing_calibration', OPENING_APPLICATION_COUNT, 10, nothing)
  cv2.createTrackbar('CLOSING_KERNEL_WIDTH', 'preprocessing_calibration', (CLOSING_KERNEL_WIDTH - 1)//2, 6, nothing)
  cv2.createTrackbar('CLOSING_APPLICATION_COUNT', 'preprocessing_calibration', CLOSING_APPLICATION_COUNT, 10, nothing)

  # read the image for calibration
  cap = cv2.VideoCapture(1)
  i = 1
  while not cap.isOpened():
    if i==10:
      i=1
    cap = cv2.VideoCapture(i)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    i+=1
  # img = cv2.imread('calibration_data/test.png')
  while True:
    blur_kernel_width = 2 * cv2.getTrackbarPos('BLUR_KERNEL_WIDTH', 'preprocessing_calibration') + 1
    low_h1 = cv2.getTrackbarPos('LOW_H1', 'preprocessing_calibration')
    high_h1 = cv2.getTrackbarPos('HIGH_H1', 'preprocessing_calibration')
    low_h2 = cv2.getTrackbarPos('LOW_H2', 'preprocessing_calibration')
    high_h2 = cv2.getTrackbarPos('HIGH_H2', 'preprocessing_calibration')
    low_s1 = cv2.getTrackbarPos('LOW_S1', 'preprocessing_calibration')
    high_s1 = cv2.getTrackbarPos('HIGH_S1', 'preprocessing_calibration')
    low_s2 = cv2.getTrackbarPos('LOW_S2', 'preprocessing_calibration')
    high_s2 = cv2.getTrackbarPos('HIGH_S2', 'preprocessing_calibration')
    low_v1 = cv2.getTrackbarPos('LOW_V1', 'preprocessing_calibration')
    high_v1 = cv2.getTrackbarPos('HIGH_V1', 'preprocessing_calibration')
    low_v2 = cv2.getTrackbarPos('LOW_V2', 'preprocessing_calibration')
    high_v2 = cv2.getTrackbarPos('HIGH_V2', 'preprocessing_calibration')
    opening_kernel_width = 2 * cv2.getTrackbarPos('OPENING_KERNEL_WIDTH', 'preprocessing_calibration') + 1
    opening_application_count = cv2.getTrackbarPos('OPENING_APPLICATION_COUNT', 'preprocessing_calibration')
    closing_kernel_width = 2 * cv2.getTrackbarPos('CLOSING_KERNEL_WIDTH', 'preprocessing_calibration') + 1
    closing_application_count = cv2.getTrackbarPos('CLOSING_APPLICATION_COUNT', 'preprocessing_calibration')


    ret, frame = cap.read()
    if not ret:
      # video ends, loop back
      cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
      continue
    frame = cv2.resize(frame, (640, 480))
    # frame = img.copy()

    mask = mask_img(frame, blur_kernel_width, 
                    np.array([low_h1, low_s1, low_v1], np.uint8),
                    np.array([high_h1, high_s1, high_v1], np.uint8), 
                    np.array([low_h2, low_s2, low_v2], np.uint8),
                    np.array([high_h2, high_s2, high_v2], np.uint8), 
                    opening_kernel_width,
                    opening_application_count,
                    closing_kernel_width,
                    closing_application_count)
    processed_img = cv2.bitwise_and(frame, frame, mask=mask)
    cv2.imshow('preprocessing_calibration', processed_img)
    cv2.imshow('frame', frame)

    # check if esc key is pressed
    if cv2.waitKey(10) & 0xff == 27:
      # print the calibration result
      print(f'BLUR_KERNEL_WIDTH = {blur_kernel_width}')
      print(f'LOW_H1 = {low_h1}')
      print(f'HIGH_H1 = {high_h1}')
      print(f'LOW_H2 = {low_h2}')
      print(f'HIGH_H2 = {high_h2}')
      print(f'LOW_S1 = {low_s1}')
      print(f'HIGH_S1 = {high_s1}')
      print(f'LOW_S2 = {low_s2}')
      print(f'HIGH_S2 = {high_s2}')
      print(f'LOW_V1 = {low_v1}')
      print(f'HIGH_V1 = {high_v1}')
      print(f'LOW_V2 = {low_v2}')
      print(f'HIGH_V2 = {high_v2}')
      print(f'OPENING_KERNEL_WIDTH = {opening_kernel_width}')
      print(f'OPENING_APPLICATION_COUNT = {opening_application_count}')
      print(f'CLOSING_KERNEL_WIDTH = {closing_kernel_width}')
      print(f'CLOSING_APPLICATION_COUNT = {closing_application_count}')

      # exit
      break 