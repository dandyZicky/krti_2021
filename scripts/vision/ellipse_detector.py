import math
import cv2

# values obtained through calibration
MAX_ERROR = 0.51
MIN_VIABLE_AREA = 858

# calculate how much the contour deviate from an ellipse
def calc_ellipse_error(cont, a, b, min_viable_area):
  # calculate the ideal ellipse area
  ideal_area = math.pi * a * b
  # automatically discard contour with area that is too small (noise)
  if ideal_area <= min_viable_area:
    return 999
  # calculate the ideal ellipse perimeter
  # use the ramanujan approximation formula
  h = math.pow(a - b, 2)/math.pow(a + b, 2)
  ideal_perimeter = math.pi * (a+b) * (1 + 3*h / (10 + math.sqrt(4 - 3*h)))

  # calculate the contour area
  real_area = cv2.contourArea(cont)
  # calculate the ellipse area error
  area_err = abs(real_area - ideal_area) / ideal_area

  # calculate the contour perimeter
  real_perimeter = cv2.arcLength(cont, True)
  # calculate the ellipse perimeter error
  perimeter_err = abs(real_perimeter - ideal_perimeter) / ideal_perimeter

  # the error is the average of the two errors
  return (area_err + perimeter_err)/2


# return an the biggest ellipses detected on an image
# ellipse format: (x_center, y_center, radius1, radius2, angle)
def detect_max_ellipse(img, max_error=MAX_ERROR, min_viable_area=MIN_VIABLE_AREA):
  conts, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  max_ellipse = None
  max_ellipse_area = 0
  min_error = max_error
  for cont in conts:
    # use minAreaRectrather than boundingRect because of more consistent results
    (x, y), (w, h), a = cv2.minAreaRect(cont)
    area = w * h
    error = calc_ellipse_error(cont, h/2, w/2, min_viable_area)
    # store only if error is lower than a certain threshold
    # if there are multiple object with lower error,
    # take the one with lowest error
    if error <= min_error:
      # store the ellipse into the array
      max_ellipse = (int(x), int(y), w, h, a)
      max_ellipse_area = area
      min_error = error

  return max_ellipse


# ellipse detection calibration
if __name__ == '__main__':
  import sys
  sys.path.append('..')
  from preprocessing import mask_img
  # from img_proc.object_localization import calc_obj_horizontal_distance

  # define placeholder callback
  def nothing(val):
    pass

  # create window with trackbar
  cv2.namedWindow('ellipse_detection_calibration')
  cv2.createTrackbar('MAX_ERROR', 'ellipse_detection_calibration', int(MAX_ERROR * 100), 100, nothing)
  cv2.createTrackbar('MIN_VIABLE_AREA', 'ellipse_detection_calibration', MIN_VIABLE_AREA, 2000, nothing)

  # read the image for calibration
  cap = cv2.VideoCapture(0)
  # img = cv2.imread('calibration_data/test3.png')
  while True:
    max_error = cv2.getTrackbarPos('MAX_ERROR', 'ellipse_detection_calibration') / 100
    min_viable_area = cv2.getTrackbarPos('MIN_VIABLE_AREA', 'ellipse_detection_calibration')

    ret, frame = cap.read()
    if not ret:
      # video ends, loop back
      cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
      continue
    frame = cv2.resize(frame, (640, 480))
    # frame = img.copy()

    mask = mask_img(frame)
    ellipse = detect_max_ellipse(mask, max_error, min_viable_area)
    if ellipse != None:
      x, y, w, h, a = ellipse
      major_axis = max(w, h)
      target_dist = 999
      print('=')
      print(f'major axis = {major_axis}')
      print(f'target dist = {target_dist}')
      cv2.ellipse(frame, (x, y), (int(w//2), int(h//2)), a, 0, 360, (0, 0, 255), 5)
      cv2.putText(frame, f'{target_dist} m', (x, y + int(h//2) + 20),
                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    cv2.imshow(f'ellipse_detection_calibration', frame)

    # check if esc key is pressed
    if cv2.waitKey(10) & 0xff == 27:
      # print the calibration result
      print('===')
      print(f'MAX_ERROR = {max_error}')
      print(f'MIN_VIABLE_AREA = {min_viable_area}')

      # exit
      break