import cv2
import numpy as np
from sys import argv

H_MIN, H_MAX = 0, 255
S_MIN, S_MAX = 0, 255
V_MIN, V_MAX = 0, 255

class Ranger:
    def __init__(self, path):
        # set default values
        self.h_low, self.h_high = H_MIN, H_MAX
        self.s_low, self.s_high = S_MIN, S_MAX
        self.v_low, self.v_high = V_MIN, V_MAX

        # load image from path
        self.bgr_img = cv2.imread(path)

        # convert colorspace to hsv
        if self.bgr_img is not None:
            self.hsv_img = cv2.cvtColor(self.bgr_img, cv2.COLOR_BGR2HSV)
    
    def is_legal(self):
        return self.bgr_img is not None

    def update_h_low(self, x):
        self.h_low = x
        self.refresh_img()

    def update_h_high(self, x):
        self.h_high = x
        self.refresh_img()

    def update_s_low(self, x):
        self.s_low = x
        self.refresh_img()

    def update_s_high(self, x):
        self.s_high = x
        self.refresh_img()

    def update_v_low(self, x):
        self.v_low = x
        self.refresh_img()

    def update_v_high(self, x):
        self.v_high = x
        self.refresh_img()

    def show(self):
        # build window
        cv2.namedWindow("HSV ranger")
        cv2.createTrackbar("H min", "HSV ranger", H_MIN, H_MAX, self.update_h_low)
        cv2.createTrackbar("H max", "HSV ranger", H_MAX, H_MAX, self.update_h_high)
        cv2.createTrackbar("S min", "HSV ranger", S_MIN, S_MAX, self.update_s_low)
        cv2.createTrackbar("S max", "HSV ranger", S_MAX, S_MAX, self.update_s_high)
        cv2.createTrackbar("V min", "HSV ranger", V_MIN, V_MAX, self.update_v_low)
        cv2.createTrackbar("V max", "HSV ranger", V_MAX, V_MAX, self.update_v_high)

        cv2.imshow("HSV ranger", self.bgr_img)
        while(True):
            cv2.imshow("HSV ranger", self.bgr_img)
            k = cv2.waitKey(100)  & 0xFF
            if k == 27:
                break

    def refresh_img(self):
        # range image
        low_th = np.array([self.h_low, self.s_low, self.v_low])
        high_th = np.array([self.h_high, self.s_high, self.v_high])
        mask = cv2.inRange(self.hsv_img, low_th, high_th)
        masked_hsv = cv2.bitwise_and(self.hsv_img, self.hsv_img, mask=mask)
        self.bgr_img = cv2.cvtColor(masked_hsv, cv2.COLOR_HSV2BGR)    

if __name__ == "__main__":
    if len(argv) < 2:
        print "format: python hsvrange.py <image-path>"
        exit(-1)
    
    ranger = Ranger(argv[1])

    if not ranger.is_legal():
        print "'%s' is either not an image or dosn't exist!" % argv[1]
        exit(-1)

    ranger.show()