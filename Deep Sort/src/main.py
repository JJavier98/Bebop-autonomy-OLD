import app
#import treatdataset
#import trainhumanclassifiermodels
#from threadedvideostreamer import ThreadedVideoStreamer
import cv2
import os

if __name__ == '__main__':

    #path_videos = '/put/your/path/to/videos/'
    #path_videos = '../video/motor_bike.mp4'
    path_videos = 0

    #for file in os.listdir(path_videos):

    #   if file.startswith('.') or os.path.isdir(os.path.join(path_videos,file)):
    #       continue

    app.run(path_videos, res=(1280, 720), track_interval=7, display=True)
