from __future__ import print_function

import numpy as np
import tensorflow as tf
from tensorflow.python.platform import gfile
import cv2

import argparse
from os.path import isfile, join, exists
from os import listdir, makedirs
import time


if __name__ == '__main__':
    parser = argparse.ArgumentParser('Test YOLO model on a folder of images')
    parser.add_argument('model_file', type=str,
                        help='Tensorflow .pb file')
    parser.add_argument('--input_path', type=str, default='test_images',
                        help='path containing images to analyse')
    parser.add_argument('--output_path', type=str, default='test_images/out',
                        help='location in which annotated images will be saved')
    args = parser.parse_args()

    # Create output path if required
    if not exists(args.output_path):
        makedirs(args.output_path)

    # Setup Tensorflow with JIT
    config = tf.ConfigProto()
    config.graph_options.optimizer_options.global_jit_level = tf.OptimizerOptions.ON_1

    with tf.Session(config=config) as sess:
        # Load graph
        with gfile.FastGFile(args.model_file, 'rb') as f:
            graph_def = tf.GraphDef()
            graph_def.ParseFromString(f.read())
            sess.graph.as_default()
            tf.import_graph_def(graph_def, name='')
        # Recover tensors
        image_input = sess.graph.get_tensor_by_name('input_1:0')
        image_shape = sess.graph.get_tensor_by_name('image_shape:0')
        learning = sess.graph.get_tensor_by_name('batch_normalization_1/keras_learning_phase:0')
        boxes = sess.graph.get_tensor_by_name('Gather:0')
        scores = sess.graph.get_tensor_by_name('Gather_1:0')
        classes = sess.graph.get_tensor_by_name('Gather_2:0')
        image_size = sess.graph.get_tensor_by_name('config_size:0')

        model_input_size = sess.run(image_size)

        input_files = [f for f in listdir(args.input_path) if isfile(join(args.input_path, f))]
        for file_name in input_files:
            if file_name[0] == '.':
                # Skip hidden DS_Store files
                continue
            print('Running inference on {0}'.format(file_name))

            image = cv2.imread(join(args.input_path, file_name))
            # Reverse colour channels (BGR to RGB)
            resized_image = cv2.resize(image[:,:,-1::-1],
                                       (model_input_size[1], model_input_size[0]))
            image_data = np.array(resized_image, dtype='float32')

            image_data /= 255.
            image_data = np.expand_dims(image_data, 0)  # Add batch dimension.

            start = time.time()
            out_boxes, out_scores, out_classes = sess.run(
                [boxes, scores, classes],
                feed_dict={
                    image_input: image_data,
                    image_shape: [image.shape[0], image.shape[1]],
                    learning: 0
                })
            finish = time.time()
            print('Found {0} traffic lights in {1}s'.format(len(out_boxes), finish - start))

            # Annotate the image and save
            annotated_image = image.copy()
            for box in out_boxes:
                top, left, bottom, right = box
                top = max(0, np.floor(top + 0.5).astype('int32'))
                left = max(0, np.floor(left + 0.5).astype('int32'))
                bottom = min(image.shape[0], np.floor(bottom + 0.5).astype('int32'))
                right = min(image.shape[1], np.floor(right + 0.5).astype('int32'))
                cv2.rectangle(annotated_image, (left, top), (right, bottom), (0, 0, 255), 6)
            cv2.imwrite(join(args.output_path, file_name), annotated_image)
