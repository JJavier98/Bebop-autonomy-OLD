import cv2

class ImgTracker:

    def __init__(self, type):
        self.type = type

    def init_tracker(self, frame, bboxes):
        multiTracker = cv2.MultiTracker_create()

        for bbox in bboxes:
            multiTracker.add(self.get_tracker_by_type(), frame, tuple(bbox))

        self.multitracker = multiTracker


    def update_tracker(self, frame):
        status, tracks = self.multitracker.update(frame)
        centroids = [(x+int(w/2),y+int(h/2)) for (x,y,w,h) in tracks]
        return centroids, tracks, status


    def get_tracker_by_type(self):
        tracking_algorithms = {
            'BOOSTING': cv2.TrackerBoosting_create(),
            'MIL': cv2.TrackerMIL_create(),
            'KCF': cv2.TrackerKCF_create(),
            'TLD': cv2.TrackerTLD_create(),
            'MEDIANFLOW': cv2.TrackerMedianFlow_create(),
            'GOTURN': cv2.TrackerGOTURN_create(),
            'MOSSE': cv2.TrackerMOSSE_create(),
            'CSRT': cv2.TrackerCSRT_create(),
        }

        return tracking_algorithms.get(self.type)