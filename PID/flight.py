import cv2
import numpy as np

from pidSolver import PID_Solver

img_h = 1000
img_w = 600
h_ground = 200  # the height of ground
target_height = 0.0

u = 0.5
dt = 0.05
ratio = 10

class Flight():
    def __init__(self, h=0, m=1, flight=[40, 7], paddle=[10, 2], bracket=[2, 3]):
        self.f_w = flight[0]        # the width of the flight
        self.f_h = flight[1]        # the height of the flight
        self.p_w = paddle[0]        # the width of the paddle
        self.p_h = paddle[1]        # the height of the paddle
        self.b_w = bracket[0]        # the width of the bracket
        self.b_h = bracket[1]        # the height of the bracket

        self.h = h
        self.m = m
        self.v = 0
        self.a = 0

    def resolve(self, force, dt):
        total_force = force - self.m * 10 - u * self.v
        self.a = total_force / self.m
        self.v += self.a * dt
        self.h += self.v * dt

        if self.h < 0:
            self.h = 0
            self.v = 0
            self.a = 0
        return

    def draw(self, img):
        img_h, img_w, _ = img.shape

        cv2.rectangle(img, (int(img_w / 2 - self.f_w / 2), int(img_h * 0.8 - self.h * ratio - self.f_h - self.b_h)),
                    (int(img_w / 2 + self.f_w / 2), int(img_h * 0.8 - self.h * ratio -self.b_h)),
                    (200, 200, 200), -1)
        cv2.line(img, (int(img_w / 2 - self.f_w / 2 - self.p_w / 2), int(img_h * 0.8 - self.h * ratio - self.f_h - self.b_h)),
                (int(img_w / 2 - self.f_w / 2 + self.p_w / 2), int(img_h * 0.8 - self.h * ratio - self.f_h - self.b_h)),
                (255, 255, 255), self.p_h)
        cv2.line(img, (int(img_w / 2 + self.f_w / 2 - self.p_w / 2), int(img_h * 0.8 - self.h * ratio - self.f_h - self.b_h)),
                (int(img_w / 2 + self.f_w / 2 + self.p_w / 2), int(img_h * 0.8 - self.h * ratio - self.f_h - self.b_h)),
                (255, 255, 255), self.p_h)
        cv2.line(img, (int(img_w / 2 - self.f_w / 2), int(img_h * 0.8 - self.h * ratio - self.b_h)),
                (int(img_w / 2 - self.f_w / 2), int(img_h * 0.8 - self.h * ratio)),
                (200, 200, 200), self.b_w)
        cv2.line(img, (int(img_w / 2 + self.f_w / 2), int(img_h * 0.8 - self.h * ratio - self.b_h)),
                (int(img_w / 2 + self.f_w / 2), int(img_h * 0.8 - self.h * ratio)),
                (200, 200, 200), self.b_w)
        return img

def nothing(x):
    pass

def mouseEvent(event, x, y, flags, ustc):
    if event == cv2.EVENT_LBUTTONDOWN:
        global target_height
        target_height = (img_h * 0.8 - y) / ratio

def main():
    cv2.namedWindow("img", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("img", mouseEvent)
    cv2.createTrackbar("Kp", "img", 0, 1000, nothing)
    cv2.createTrackbar("Ki", "img", 0, 1000, nothing)
    cv2.createTrackbar("Kd", "img", 0, 5000, nothing)

    flight = Flight()
    pidsolver = PID_Solver()

    while(True):
        background = np.zeros((img_h, img_w, 3))
        cv2.line(background, (0, int(img_h * 0.8) + 3), (img_w, int(img_h * 0.8) + 3), (100, 100, 100), 6)
        cv2.line(background, (0, int(img_h * 0.8 - target_height * ratio)), (img_w, int(img_h * 0.8 - target_height * ratio)), (255, 255, 0), 2)
        cv2.line(background, (0, int(img_h * 0.8 - flight.h * ratio)), (img_w, int(img_h * 0.8 - flight.h * ratio)), (0, 255, 255), 2)
        cv2.putText(background, "Target Height: " + str(target_height), (int(img_w / 2), int(img_h * 0.8 - target_height * ratio + 30)),
                     cv2.FONT_HERSHEY_COMPLEX, 0.6, (255, 255, 0), 1, 8)
        cv2.putText(background, "Current Height: " + str(flight.h), (int(img_w / 2), int(img_h * 0.8 - flight.h * ratio + 30)),
                     cv2.FONT_HERSHEY_COMPLEX, 0.6, (0, 255, 255), 1, 8)
        cv2.putText(background, "Kp: " + str(pidsolver.Kp), (20, 50),
                     cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 255), 1, 8)
        cv2.putText(background, "Ki: " + str(pidsolver.Ki), (20, 100),
                     cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 255), 1, 8)
        cv2.putText(background, "Kd: " + str(pidsolver.Kd), (20, 150),
                     cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 255), 1, 8)
        
        pidsolver.change_param(cv2.getTrackbarPos("Kp", "img") / 1000, cv2.getTrackbarPos("Ki", "img") / 1000, cv2.getTrackbarPos("Kd", "img") / 50)
        force = pidsolver.solve(target_height - flight.h)
        flight.resolve(force, dt)

        flight.draw(background)
        cv2.imshow("img", background)
        c = cv2.waitKey(10)
        if c == ord('q'):
            break
    return

if __name__ == "__main__":
    main()