import tensorflow as tf
import numpy as np
import cv2

class ImageEncoder(object):

    def __init__(self, checkpoint_filename, input_name="images",
                 output_name="features"):
        self.session = tf.compat.v1.Session()
        with tf.io.gfile.GFile(checkpoint_filename, "rb") as file_handle:
            graph_def = tf.compat.v1.GraphDef()
            graph_def.ParseFromString(file_handle.read())
        tf.import_graph_def(graph_def, name="net")
        self.input_var = tf.compat.v1.get_default_graph().get_tensor_by_name(
            "net/%s:0" % input_name)
        self.output_var = tf.compat.v1.get_default_graph().get_tensor_by_name(
            "net/%s:0" % output_name)

        assert len(self.output_var.get_shape()) == 2
        assert len(self.input_var.get_shape()) == 4
        self.feature_dim = self.output_var.get_shape().as_list()[-1]
        self.image_shape = self.input_var.get_shape().as_list()[1:]

    def encode(self, data_x, batch_size=32):
        out = np.zeros((len(data_x), self.feature_dim), np.float32)
        _run_in_batches(
            lambda x: self.session.run(self.output_var, feed_dict=x),
            {self.input_var: data_x}, out, batch_size)
        return out


def _run_in_batches(function, data, out, batch_size):
    data_len = len(out)
    num_batches = int(data_len/batch_size)
    e = 0
    s = 0
    for i in range(0,num_batches):
        s = i*batch_size
        e = (i+1)*batch_size
        batch_dict = {k: v[s:e] for k,v in data.items()}
        out[s:e] = function(batch_dict)

    if e < len(out):
        batch_dict = {k: v[e:] for k, v in data.items()}
        out[e:] = function(batch_dict)


class FeatureExtractor:

    def __init__(self, model):
        self.encoder = ImageEncoder(model)
        self.image_shape = self.encoder.image_shape

    def extract_features(self, frame, bboxes):
        bbox_images = []
        bbox_shape = self.image_shape[:2]
        # correct aspect ratio to patch shape
        target_aspect = float(bbox_shape[1]) / bbox_shape[0]
        for bbox in bboxes:
            new_width = target_aspect * bbox[3]
            bbox[0] = bbox[0] - ((new_width - bbox[2])/2)
            bbox[2] = new_width
            bbox[2:] = [sum(x) for x in zip(bbox[2:], bbox[:2])]
            bbox = np.array(bbox, dtype=int)
            bbox[:2] = np.maximum(0, bbox[:2])
            bbox[2:] = np.minimum(np.asarray(frame.shape[:2][::-1]) - 1, bbox[2:])
            if np.any(bbox[:2] >= bbox[2:]):
                return None
            sx, sy, ex, ey = bbox
            image = frame[sy:ey, sx:ex]
            image = cv2.resize(image, tuple(bbox_shape[::-1]))

            bbox_images.append(image)

        bbox_images = np.asarray(bbox_images)
        return self.encoder.encode(bbox_images, batch_size=32)




