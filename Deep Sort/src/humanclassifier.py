import cPickle as cPickle
import cv2
import time
import numpy as np
from itertools import compress
#from tensorflow.contrib import keras
import keras

import tensorflow as tf

config = tf.compat.v1.ConfigProto()
config.gpu_options.allow_growth = True
sess = tf.compat.v1.Session(config=config)

class HumanClassifier:

    def __init__(self, path, alg):
        self.histogram_equalization = cv2.createCLAHE(clipLimit=5, tileGridSize=(8, 8))
        self.descriptor = cv2.HOGDescriptor()
        self.alg = alg
        self.get_model_by_alg(path)

    def get_model_by_alg(self, path):

        if self.alg == 'DNN':
            self.classifier, self.train_mean, self.train_std = load_keras_model(path)

        if self.alg == 'SVM':
            self.classifier = load_pkl_model(path)

    def preprocess_by_alg(self, frame, bbs):

        if self.alg == 'DNN':
            return self.dnn_img_preprocess(frame, bbs)

        if self.alg == 'SVM':
            return self.svm_img_preprocess(frame, bbs)

    def predict_by_alg(self, data):

        if self.alg == 'DNN':
            return self.classifier.predict(data, verbose=1, batch_size=128).argmax(axis=-1)

        if self.alg == 'SVM':
            return self.classifier.predict(data)

    def filter_human_bb(self, frame, bbs):
        start_time = time.time()
        if len(bbs) == 0:
            return []
        data = self.preprocess_by_alg(frame, bbs)
        print("Filter preprocess:" + str(time.time() - start_time))
        if len(bbs) == 1:
            data.reshape(1, -1)
        prediction = self.predict_by_alg(data)
        # Return filtered detections (Human detections)
        print("Filter prediction:" + str(time.time()-start_time))
        return list(compress(bbs, prediction))

    def svm_img_preprocess(self, frame, bbs):

        data = []
        for bb in bbs:
            # get bounding box coordinates
            x, y, w, h = bb
            # crop image to get detection
            img = frame[y:y + h, x:x + w]
            # Histogram equalization and resizing
            img = self.histogram_equalization.apply(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY))
            img = cv2.resize(img, (64, 128))
            # get image descriptor
            img_d = self.descriptor.compute(img)
            data.append(np.concatenate(img_d))
        data = np.array(data)
        return data

    def dnn_img_preprocess(self, frame, bbs):
        data = []
        for bb in bbs:
            # get bounding box coordinates
            x, y, w, h = bb
            # crop image to get detection
            img = frame[y:y + h, x:x + w]
            # cv2.imwrite("../datasets/detections/" + str(uuid.uuid4()) + ".png", img)

            img = cv2.resize(img, (64, 128))

            # Histogram equalization and resizing
            img_YCrCb = cv2.cvtColor(img, cv2.COLOR_BGR2YCrCb)

            #cl1 = self.histogram_equalization.apply(img_YCrCb[:, :, 0])

            #img_YCrCb[:, :, 0] = cl1

            img_RGB = cv2.cvtColor(img_YCrCb, cv2.COLOR_YCrCb2RGB)

            img_prep = cv2.copyMakeBorder(img_RGB, 0, 0, 32, 32, cv2.BORDER_CONSTANT)

            data.append(img_prep)
        data = np.array(data)
        data = (data - self.train_mean) / (self.train_std + 0.000001)
        return data

def load_pkl_model(path):
    with open(path, 'rb') as fid:
        return cPickle.load(fid)

def load_keras_model(path):
    model = keras.models.load_model(path)
    file = open(path + '_norm', "r")
    for line in file:
        fields = line.split(",")
        train_mean = fields[0]
        train_std = fields[1]

    return model, float(train_mean), float(train_std)

