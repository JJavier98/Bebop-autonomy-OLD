from threading import Thread
import cv2
import numpy as np
import time
import Queue

class VideoViewer:

    def __init__(self, shape=(1280,720), caption="Viewer"):
        self.caption = caption
        self.stopped = False
        self.shape = shape

        self.frame = np.zeros(shape+(3,), np.uint8)
        self.color = (0, 0, 0)
        self.text_color = (255,255,255)
        self.text_thickness = 1
        self.thickness = 1

    def run(self, update_fun):
        terminate = update_fun(self.stopped)
        while not terminate and not self.stopped:
            if cv2.waitKey(1) == ord("q"):
                self.stopped = True
            cv2.imshow(self.caption, self.frame)
            terminate = update_fun(self.stopped)
        cv2.waitKey(1)
        #cv2.destroyWindow(self.caption)

    def set_frame(self, frame):
        self.frame = frame

    def start(self):
        Thread(target=self.run, args=()).start()
        return self

    def show(self):
        while not self.stopped:
            cv2.imshow(self.caption, self.frame)
            if cv2.waitKey(1) == ord("q"):
                self.stopped = True

        self.output_video.release()
        self.frame[:] = 0
        cv2.destroyWindow(self.caption)
        cv2.waitKey(1)
        cv2.imshow(self.caption, self.frame)

    def stop(self):
        self.stopped = True

    def draw_rectangle(self, x, y, w, h, label=None):
        x, y, w, h = int(x), int(y), int(w), int(h)
        cv2.rectangle(self.frame, (x, y), (x + w, y + h), self.color, self.thickness)
        if label is not None:
            text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_PLAIN, 1, self.thickness)
            center = x + 5, y - text_size[0][1] + 5
            pt2 = x + 10 + text_size[0][0], y - 10 - text_size[0][1]
            cv2.rectangle(self.frame, (x, y), pt2, self.color, -1)
            cv2.putText(self.frame, label, center, cv2.FONT_HERSHEY_PLAIN, 1, self.text_color, self.text_thickness)
