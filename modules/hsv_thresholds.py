import cv2
import modules.color_detection as col
import modules.crop_color as crop
from time import sleep

WIDTH = 640
HEIGHT = 360

win_name = "HSV Configuration"

def empty(a):
    pass

def get_trackbar_pos():
    lower = [0, 0, 0]
    upper = [0, 0, 0]
    lower[0] = cv2.getTrackbarPos("Hue Min", win_name)
    lower[1] = cv2.getTrackbarPos("Sat Min", win_name)
    lower[2] = cv2.getTrackbarPos("Val Min", win_name)
    upper[0] = cv2.getTrackbarPos("Hue Max", win_name)
    upper[1] = cv2.getTrackbarPos("Sat Max", win_name)
    upper[2] = cv2.getTrackbarPos("Val Max", win_name)
    return lower, upper

def cap_shot():
    cap = cv2.VideoCapture(cam.gstreamer_pipeline())
    while True:
        valid, frame = cap.read()
        if not valid:
            break
        image = cv2.resize(frame, (WIDTH, HEIGHT))
        cv2.imshow("Video", image)
        key = cv2.waitKey(1) & 0xFF
        # if key == 27:
        #     break    
        if key == ord('s'):
            break
    cv2.destroyAllWindows()
    return image

def init_trackbars(lower, upper):
    cv2.namedWindow(win_name)
    cv2.resizeWindow(win_name, WIDTH, 260)
    cv2.createTrackbar("Hue Min",win_name,lower[0],179,empty)
    cv2.createTrackbar("Hue Max",win_name,upper[0],179,empty)
    cv2.createTrackbar("Sat Min",win_name,lower[1],255,empty)
    cv2.createTrackbar("Sat Max",win_name,upper[1],255,empty)
    cv2.createTrackbar("Val Min",win_name,lower[2],255,empty)
    cv2.createTrackbar("Val Max",win_name,upper[2],255,empty)

def get_thresholds(image=None):
    if image is None:
        image = cap_shot()
    lower, upper = crop.crop_color(image)
    init_trackbars(lower, upper)
    while True:
        i = image.copy()
        lower, upper = get_trackbar_pos()
        #print(lower, upper)
        mask, img_color = col.findColor(i, lower, upper)
        cv2.imshow("Image", img_color)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            print("break")
            break
    cv2.destroyAllWindows()
    # print(lower, upper)
    return image, lower, upper

