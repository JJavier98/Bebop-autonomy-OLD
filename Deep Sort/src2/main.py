import app
#import treatdataset
#import trainhumanclassifiermodels
#from threadedvideostreamer import ThreadedVideoStreamer
import cv2
import os
import argparse

# cosntruimos los argumentos
parser = argparse.ArgumentParser(description='Script to run Deep-SORT')

parser.add_argument('--path',default=0, help='path del video a usar. Si se deja vacio se tomara /dev/video0')
parser.add_argument("--res", default=(1280, 720), help='resolucion del video indicado')
parser.add_argument("--interval", default="3", help='cada cuantos frames se aplica la deteccion')
parser.add_argument("--display", default=True, help="Si True mostramos por pantalla los resultados del tracking")
args = parser.parse_args()

if __name__ == '__main__':

    path = args.path
    res = args.res
    interval = args.interval
    display = args.display

    app.run(path=path, res=res, track_interval=interval, display=display)

