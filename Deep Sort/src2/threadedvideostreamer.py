import cv2 as cv
from Queue import *
import threading


class ThreadedVideoStreamer(object):

    def __init__(self, src, res=(1280, 720), queueSize = 128):
        self.path = src
        self.res = res
        self.stream = cv.VideoCapture(src)
        self.fps = self.stream.get(cv.CAP_PROP_FPS)
        self.Q = Queue(maxsize=queueSize)
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False

    def start(self):
        # start a thread to read frames from the file video stream
        self.t = threading.Thread(target=self.update, args=())
        self.t.daemon = True
        self.t.start()
        return self

    def update(self):
    	count2 = 0
        # keep looping infinitely
        while True:
            # if the thread indicator variable is set, stop the
            # thread
            if self.stopped:
                break

            # otherwise, ensure the queue has room in it
            if not self.Q.full():
				print(self.path)
				(grabbed, frame) = self.stream.read()
				if self.path != 0:
					# read the next frame from the file
					count = self.stream.get(1)

					# if the `grabbed` boolean is `False`, then we have
					# reached the end of the video file
					if not grabbed:
						self.stop()
						return
					# add the frame to the queue
					self.Q.put((cv.resize(frame, self.res), int(count-1)))
				else:
					self.Q.put((cv.resize(frame, self.res), int(count2-1)))
					count2+=1

    def read(self):
        # return next (n_frame, frame) in the queue
        value = self.Q.get()
        return value

    def more(self):
        # return True if there are still frames in the queue
        res=self.Q.qsize() > 0
        return res

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True
        #self.t.join()
