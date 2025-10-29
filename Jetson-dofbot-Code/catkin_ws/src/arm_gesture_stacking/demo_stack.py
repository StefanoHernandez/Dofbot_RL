#!/usr/bin/env python
# coding: utf-8
import cv2
from gesture_stack import gesture_stack

if __name__ == '__main__':
    gesture = gesture_stack()
    capture = cv2.VideoCapture(0)
    capture.set(3, 640)
    capture.set(4, 480)
    capture.set(5, 30)
    try:
        while capture.isOpened():
            _, img = capture.read()
            img = gesture.start_gesture(img)
            cv2.imshow("img", img)
            action = cv2.waitKey(10) & 0xff
            if action == ord('q') or action == 27:
                cv2.destroyAllWindows()
                capture.release()
                break
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        capture.release()
        print(" Program closed! ")
        pass
    cv2.destroyAllWindows()
    capture.release()