import cv2
# Check camera port by run "ls /dev/video*"
def init_camera():
  cap = cv2.VideoCapture(0)
  while not cap.isOpened():
    cap = cv2.VideoCapture(0)
  return cap