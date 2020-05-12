from threadedvideostreamer import ThreadedVideoStreamer
from humanclassifier import HumanClassifier
from imgtracker import ImgTracker
from objtracker import ObjTracker
from visualizer import Visualizer, NoViewerVisualizer
from extractfeatures import FeatureExtractor
import os
import cv2
import numpy as np
import time

# args
prototxt = '../detector_data/MobileNetSSD_deploy.prototxt'
caffemodel = '../detector_data/MobileNetSSD_deploy.caffemodel'
thresshold = 0.2
start_time = 0
n_frame = 0

# Labels of Network.
classNames = { 0: 'background',
    1: 'aeroplane', 2: 'bicycle', 3: 'bird', 4: 'boat',
    5: 'bottle', 6: 'bus', 7: 'car', 8: 'cat', 9: 'chair',
    10: 'cow', 11: 'diningtable', 12: 'dog', 13: 'horse',
    14: 'motorbike', 15: 'person', 16: 'pottedplant',
    17: 'sheep', 18: 'sofa', 19: 'train', 20: 'tvmonitor' }

#Load the Caffe model 
net = cv2.dnn.readNetFromCaffe(prototxt, caffemodel)

def get_video_infomation(path, fps, res, str):
   
    video_info = {
        "video_name": os.path.basename(path),
        "image_size": res,
        "fps": fps,
        "streamer": str
    }

    return video_info


def run(path, res, track_interval=5, display=True):

	video = ThreadedVideoStreamer(path, res)
	video_info = get_video_infomation(str(path), video.fps, video.res, video)

	# Inicializamos el tracker y el extractor de caracteristicas
	obj_tracker = ObjTracker()
	feat_extractor = FeatureExtractor("../encoder/mars-small128.pb")

	video.start()
	time.sleep(1)

	def frame_callback(vis):

		detection_centroids = []
		detection_bboxes = []
		
		frame, n_frame = video.read()
		(detection_centroids, detection_bboxes) = detection(frame)

		features = feat_extractor.extract_features(frame, np.copy(detection_bboxes))
		obj_tracker.update(detection_bboxes, detection_centroids, features)

		if display:
			vis.set_image(frame)
			vis.draw_tracks(obj_tracker.get_confirmed_tracks())
			vis.draw_detections(detection_bboxes)
	
		#print(obj_tracker.get_confirmed_tracks())
		#print("Tiempo total " + str(n_frame) + ": " + str(time.time()-start_time))
	
	
	vis = Visualizer(video_info) if display else NoViewerVisualizer(video_info)
	vis.run(frame_callback)


def detection(frame):
    centroids = []
    bboxes = []
    
    frame_resized = cv2.resize(frame,(300,300)) # resize frame for prediction

    # MobileNet requires fixed dimensions for input image(s)
    # so we have to ensure that it is resized to 300x300 pixels.
    # set a scale factor to image because network the objects has differents size. 
    # We perform a mean subtraction (127.5, 127.5, 127.5) to normalize the input;
    # after executing this command our "blob" now has the shape:
    # (1, 3, 300, 300)
    blob = cv2.dnn.blobFromImage(frame_resized, 0.007843, (300, 300), (127.5, 127.5, 127.5), False)
    #Set to network the input blob 
    net.setInput(blob)
    #Prediction of network
    detections = net.forward()

    #Size of frame resize (300x300)
    cols = frame_resized.shape[1] 
    rows = frame_resized.shape[0]

    #For get the class and location of object detected, 
    # There is a fix index for class, location and confidence
    # value in @detections array .
    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2] #Confidence of prediction 
        if confidence > thresshold: # Filter prediction 

            class_id = int(detections[0, 0, i, 1]) # Class label

            # In order to only detect people
            if class_id == 15:

                # Object location 
                xLeftBottom = int(detections[0, 0, i, 3] * cols) 
                yLeftBottom = int(detections[0, 0, i, 4] * rows)
                xRightTop   = int(detections[0, 0, i, 5] * cols)
                yRightTop   = int(detections[0, 0, i, 6] * rows)
                
                # Factor for scale to original size of frame
                heightFactor = frame.shape[0]/300.0  
                widthFactor = frame.shape[1]/300.0 
                # Scale object detection to frame
                xLeftBottom = int(widthFactor * xLeftBottom) 
                yLeftBottom = int(heightFactor * yLeftBottom)
                xRightTop   = int(widthFactor * xRightTop)
                yRightTop   = int(heightFactor * yRightTop)

                w = xRightTop - xLeftBottom
                h = yRightTop - yLeftBottom
                cx = xLeftBottom + w/2
                cy = yLeftBottom + h/2

                centroids.append([cx,cy])
                bboxes.append([xLeftBottom, yLeftBottom, w, h])
                # Draw location of object  
                #cv2.rectangle(frame, (xLeftBottom, yLeftBottom), (xRightTop, yRightTop),
                #            (0, 255, 0))

                # Draw label and confidence of prediction in frame resized
                #if class_id in classNames:
                #    label = classNames[class_id] + ": " + str(confidence)
                #    labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)

                #    yLeftBottom = max(yLeftBottom, labelSize[1])
                #    cv2.rectangle(frame, (xLeftBottom, yLeftBottom - labelSize[1]),
                #                        (xLeftBottom + labelSize[0], yLeftBottom + baseLine),
                #                        (255, 255, 255), cv2.FILLED)
                #    cv2.putText(frame, label, (xLeftBottom, yLeftBottom),
                #                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))

                #    print(label) #print class and confidence

    #cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
    #cv2.imshow("frame", frame)

    return (centroids, bboxes)

