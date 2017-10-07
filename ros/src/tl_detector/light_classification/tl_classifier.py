from styx_msgs.msg import TrafficLight

import numpy as np
import tensorflow as tf
from tensorflow.python.platform import gfile
import cv2


class TLClassifier(object):
    def __init__(self, model_file):
        # Setup Tensorflow with JIT
        config = tf.ConfigProto()
        config.graph_options.optimizer_options.global_jit_level = tf.OptimizerOptions.ON_1

        # Tensorflow session
        self.session = tf.Session(config=config)
        # Load graph
        with gfile.FastGFile(model_file, 'rb') as f:
            graph_def = tf.GraphDef()
            graph_def.ParseFromString(f.read())
            self.session.graph.as_default()
            tf.import_graph_def(graph_def, name='')
        # Recover tensors
        tensors = {}
        tensors['image_input'] = self.session.graph.get_tensor_by_name('input_1:0')
        tensors['image_shape'] = self.session.graph.get_tensor_by_name('image_shape:0')
        tensors['learning'] = self.session.graph.get_tensor_by_name('batch_normalization_1/keras_learning_phase:0')
        tensors['boxes'] = self.session.graph.get_tensor_by_name('Gather:0')
        tensors['scores'] = self.session.graph.get_tensor_by_name('Gather_1:0')
        tensors['classes'] = self.session.graph.get_tensor_by_name('Gather_2:0')
        image_size = self.session.graph.get_tensor_by_name('config_size:0')
        self.tensors = tensors
        # Get input image size from model_file
        self.model_input_size = self.session.run(image_size)

    def close(self):
        # Close Tensorflow session
        self.session.close()

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Resize and reverse colour channels (BGR to RGB)
        resized_image = cv2.resize(image[:,:,-1::-1],
                                   (self.model_input_size[1], self.model_input_size[0]))
        image_data = np.array(resized_image, dtype='float32')

        image_data /= 255.
        image_data = np.expand_dims(image_data, 0)  # Add batch dimension.

        out_boxes, out_scores, out_classes = self.session.run(
            [self.tensors['boxes'], self.tensors['scores'], self.tensors['classes']],
            feed_dict={
                self.tensors['image_input']: image_data,
                self.tensors['image_shape']: [image.shape[0], image.shape[1]],
                self.tensors['learning']: 0
            })

        num_lights = len(out_boxes)
        if num_lights == 0:
            TrafficLight.UNKNOWN
        else:
            # Compute probability weighted average of light colour
            scores = {TrafficLight.UNKNOWN: 0.0,
                      TrafficLight.RED: 0.0}
            annotated_image = image.copy()
            for box, score in zip(out_boxes, out_scores):
                colour = self.light_colour(image, box)
                scores[colour] = scores[colour] + score
            # Return colour with highest score
            return max(scores, key=scores.get)

    def light_colour(self, img, box):
        # Crop around the box
        top, left, bottom, right = box
        light_img = img[int(top):int(bottom), int(left):int(right), :]
        if light_img.size == 0:
            return TrafficLight.UNKNOWN
        # Convert to HSV
        hsv = cv2.cvtColor(light_img, cv2.COLOR_BGR2HSV)
        # Crop edges
        hsv = hsv[int(0.05 * hsv.shape[0]):int(0.95 * hsv.shape[0]), int(0.05 * hsv.shape[1]):int(0.95 * hsv.shape[1]), :]
        # Get bright pixels
        h = hsv[:, :, 0]
        red_pixels = (np.sum(cv2.inRange(hsv, (0, 120, 220), (10, 255, 255))) + \
                      np.sum(cv2.inRange(hsv, (170, 120, 220), (180, 255, 255)))) / 255
        # Proportion of red pixels in box
        prop_red = red_pixels / (hsv.shape[0] * hsv.shape[1])
        if (prop_red > 0.03):
            return TrafficLight.RED
        else:
            return TrafficLight.UNKNOWN
