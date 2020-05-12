from __future__ import print_function
import cv2 as cv
from Queue import *
import threading
import time
import roslib
try:
	roslib.load_manifest('msgs_to_cv2')
except:
	pass
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

go_read=False

class ThreadedVideoStreamer(object):
	
	def callback2(self,data):
		if not go_read:
			try:
				self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
			except CvBridgeError as e:
				print(e)
			
	def callback1(self):
		self.image_sub = rospy.Subscriber("/bebop/image_raw",Image,self.callback2)

	def __init__(self, src, res=(1280, 720), queueSize = 128, refresh=0.025):
		self.refresh = refresh #tiempo de escaneo de imagenes. No puede ser muy cercano a 0.0
		self.path = src
		self.res = res
		self.frame=[]
		if self.path=='bebop_cam':
			self.bridge = CvBridge()
			self.stream = None
			self.fps = 30.0
			self.grabbed = True
			self.n_frame=0
		else:
			self.stream = cv.VideoCapture(src)
			self.fps = self.stream.get(cv.CAP_PROP_FPS)
			(self.grabbed, self.frame) = self.stream.read()
		self.Q = Queue(maxsize=queueSize)
		self.stopped = False

	def start(self):
		if self.path == 'bebop_cam':
			# start a thread to read frames from the file video stream
			self.t = threading.Thread(target=self.callback1, args=())
			self.t.daemon = True
			self.t.start()
			
			rospy.init_node('image_converter', anonymous=True)
			
		else:
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
				if self.path!='bebop_cam':
					(self.grabbed, self.frame) = self.stream.read()
				if self.path != 0 and self.path != 'bebop_cam':
					# read the next frame from the file
					count = self.stream.get(1)

					# if the `grabbed` boolean is `False`, then we have
					# reached the end of the video file
					if not self.grabbed:
						self.stop()
						return
					# add the frame to the queue
					self.Q.put((cv.resize(self.frame, self.res), int(count-1)))
				else:
					self.Q.put((cv.resize(self.frame, self.res), int(count2)))
					count2+=1

	def read(self):
		# return next (n_frame, frame) in the queue
		go_read=True
		if self.path=='bebop_cam':
			#time.sleep(self.refresh) # tiempo de escaneo
			value=[self.frame, self.n_frame]
			self.n_frame+=1
		else:
			value = self.Q.get()
			
		go_read=False
		return value

	def more(self):
		# return True if there are still frames in the queue
		res=self.Q.qsize() > 0
		return res

	def stop(self):
		# indicate that the thread should be stopped
		self.stopped = True
		#self.t.join()
        
        
        
        
        
        
