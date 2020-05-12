from viewer import VideoViewer
import time
import numpy as np
import  cv2

class Visualizer:

    def __init__(self, seq_info):
        image_shape = seq_info["image_size"][::-1]
        aspect_ratio = float(image_shape[1]) / image_shape[0]
        image_shape = int(aspect_ratio * 360), 360
        self.viewer = VideoViewer(image_shape, seq_info["video_name"])
        self.viewer.thickness = 2
        self.str = seq_info["streamer"]

    def run(self, frame_callback):
        self.viewer.start()
        start_time = time.time()
        self.viewer.run(lambda stop=False: self.update_fun(frame_callback, stop))
        print("Tiempo total de la secuencia:" + str(time.time() - start_time))

    def update_fun(self, frame_callback, stop):
        if self.str.stopped and not self.str.more() or stop:
            self.str.stop()
            return True
        frame_callback(self)
        return False

    def set_image(self, frame):
        self.viewer.set_frame(frame)

    def draw_groundtruth(self, bboxes, track_ids):
        self.viewer.thickness = 2
        for track_id, box in zip(track_ids, bboxes):
            self.viewer.draw_rectangle(bboxes, label=str(track_id))

    def draw_detections(self, detections):
        self.viewer.thickness = 3
        self.viewer.color = 0, 0, 255
        for detection in detections:
            (x,y,w,h) = detection
            self.viewer.draw_rectangle(x,y,w,h)

    def draw_tracks(self, tracks):
        self.viewer.text_thickness = 2
        self.viewer.thickness = 2
        self.viewer.color = 0, 255, 0
        for track in tracks:
            (x,y,w,h) = track.bbox
            self.viewer.draw_rectangle(x, y, w, h, label=str(track.id))


class NoViewerVisualizer:

    def __init__(self, seq_info):
        self.str = seq_info["streamer"]

    def run(self, frame_callback):
        start_time = time.time()
        count = 0
        while not self.str.stopped or self.str.more():
            frame_callback(self)
            count += 1

        fps = count/(time.time()-start_time)
        print("FPS medio: " + str(fps))


    def set_image(self, frame):
        pass

    def draw_groundtruth(self, bboxes, track_ids):
        pass

    def draw_detections(self, detections):
        pass

    def draw_tracks(self, tracks):
        pass
