import cv2
import numpy as np
import enum

res = None
pixel = (0, 0, 0)


# smoothing filters enum
class SMOOTHING(enum.Enum):
    NORMAL = 0
    FILTER2D = 1
    GAUSSIANBLUR = 2
    MEDIANBLUR = 3
    BILATERAL = 4


# Morphological filters enum
class Morphological(enum.Enum):
    NORMAL = 0
    EROSION = 1
    DILATION = 2
    OPENING = 3
    CLOSING = 4


# set initial smoothing and morphological filters' states (no filter)
current_smoothing_filter = SMOOTHING.NORMAL
current_morphological_filter = Morphological.NORMAL


# this fn will be called whenever the value of the trackbar changes
def trackbarCallBack(x):
    pass


# Smoothing the frame
def smooth():
    global res
    # apply filter2D to the frame if the current smoothing filter flag of the filter2d is raised
    if current_smoothing_filter == SMOOTHING.FILTER2D:
        kernel = np.ones((15, 15), np.float32) / 25
        smoothed = cv2.filter2D(res, -1, kernel)
        print("filter2d")
        return smoothed

    # apply GaussianBlur to the frame if the current smoothing filter flag of the gaussian blur is raised
    elif current_smoothing_filter == SMOOTHING.GAUSSIANBLUR:
        blur = cv2.GaussianBlur(res, (15, 15), 0)
        print("gaussian")
        return blur

    # apply medianBlur to the frame if the current smoothing filter flag of the median blur is raised
    elif current_smoothing_filter == SMOOTHING.MEDIANBLUR:
        median = cv2.medianBlur(res, 15)
        print("medianblur")
        return median

    # apply bilateralFilter to the frame if the current smoothing filter flag of the bilateral filter is raised
    elif current_smoothing_filter == SMOOTHING.BILATERAL:
        bilateral = cv2.bilateralFilter(res, 15, 75, 75)
        print("bilateral")
        return bilateral
    print("normal")
    return


def apply_morphological_filter():
    gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    kernel = np.ones((5, 5), np.uint8)

    if current_morphological_filter == Morphological.EROSION:
        eroded = cv2.erode(res, kernel, iterations=2)
        return eroded

    elif current_morphological_filter == Morphological.DILATION:
        dilated = cv2.dilate(res, kernel, iterations=2)
        return dilated

    elif current_morphological_filter == Morphological.OPENING:
        opening_frame = cv2.morphologyEx(res, cv2.MORPH_OPEN, kernel, iterations=2)
        return opening_frame

    elif current_morphological_filter == Morphological.CLOSING:
        closing_frame = cv2.morphologyEx(res, cv2.MORPH_CLOSE, kernel, iterations=2)
        return closing_frame

    return


def moveTrackbarsAccordingly():
    # triggers the current position of the trackbar
    # [100, 120, 50] -> [120, 255, 255]
    l_h = cv2.getTrackbarPos("LH", "Tracking")
    l_s = cv2.getTrackbarPos("LS", "Tracking")
    l_v = cv2.getTrackbarPos("LV", "Tracking")

    u_h = cv2.getTrackbarPos("UH", "Tracking")
    u_s = cv2.getTrackbarPos("US", "Tracking")
    u_v = cv2.getTrackbarPos("UV", "Tracking")

    l_c = np.array([l_h, l_s, l_v])
    u_c = np.array([u_h, u_s, u_v])

    mask = cv2.inRange(hsv, l_c, u_c)

    new_frame = cv2.bitwise_and(frame, frame, mask=mask)
    return new_frame


def pick_color(event, x, y, flags, params):
    global pixel, res
    if event == cv2.EVENT_LBUTTONDOWN:
        pixel = hsv[y, x]

        lower_h = pixel[0] - 40
        lower_s = pixel[1] - 45
        lower_v = pixel[2] - 45
        upper_h = pixel[0] + 40
        upper_s = pixel[1] + 45
        upper_v = pixel[2] + 45

        lower_color = np.array([lower_h, lower_s, lower_v])
        upper_color = np.array([upper_h, upper_s, upper_v])
        print(pixel, lower_color, upper_color)

        cv2.setTrackbarPos("LH", "Tracking", lower_h)
        cv2.setTrackbarPos("LS", "Tracking", lower_s)
        cv2.setTrackbarPos("LV", "Tracking", lower_v)
        cv2.setTrackbarPos("UH", "Tracking", upper_h)
        cv2.setTrackbarPos("US", "Tracking", upper_s)
        cv2.setTrackbarPos("UV", "Tracking", upper_v)

        frame_mask = cv2.inRange(hsv, lower_color, upper_color)

        res = cv2.bitwise_and(frame, frame, mask=frame_mask)


if __name__ == "__main__":

    cap = cv2.VideoCapture(0)

    # create a new window
    cv2.namedWindow('Tracking')

    # resize your window
    cv2.resizeWindow('Tracking', 500, 100)

    # create a black background image for the trackbars
    img = np.ones((300, 512, 3), np.uint8)

    # callbackfn that will be called when u move the trackbar
    # lazem yeb2a fih callback fn even if we don't need it
    cv2.createTrackbar("LH", "Tracking", 0, 255, trackbarCallBack)
    cv2.createTrackbar("LS", "Tracking", 0, 255, trackbarCallBack)
    cv2.createTrackbar("LV", "Tracking", 0, 255, trackbarCallBack)
    cv2.createTrackbar("UH", "Tracking", 255, 180, trackbarCallBack)
    cv2.createTrackbar("US", "Tracking", 255, 255, trackbarCallBack)
    cv2.createTrackbar("UV", "Tracking", 255, 255, trackbarCallBack)

    while True:
        # read frame
        _, frame = cap.read()

        # convert frame from bgr to hsv
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        if current_smoothing_filter != SMOOTHING.NORMAL:
            res = smooth()

        if current_morphological_filter != Morphological.NORMAL:
            res = apply_morphological_filter()

        cv2.imshow("Tracking", img)

        res = moveTrackbarsAccordingly()

        # wait for keystroke
        k = cv2.waitKey(1) & 0xFF

        # if 'esc' or 'q' is pressed, exit and close window
        if (k == 27) or (k == ord('q')):
            break

        if k == ord('n'):
            current_smoothing_filter = SMOOTHING.FILTER2D

        if k == ord('g'):
            current_smoothing_filter = SMOOTHING.GAUSSIANBLUR

        if k == ord('m'):
            current_smoothing_filter = SMOOTHING.MEDIANBLUR

        if k == ord('b'):
            current_smoothing_filter = SMOOTHING.BILATERAL

        if k == ord('e'):
            current_morphological_filter = Morphological.EROSION

        if k == ord('d'):
            current_morphological_filter = Morphological.DILATION

        if k == ord('o'):
            current_morphological_filter = Morphological.OPENING

        if k == ord('c'):
            current_morphological_filter = Morphological.CLOSING

        # print(current_smoothing_filter)
        print(current_morphological_filter)
        cv2.imshow("frame", res)
        cv2.setMouseCallback('frame', pick_color)
    cap.release()
    cv2.destroyAllWindows()
