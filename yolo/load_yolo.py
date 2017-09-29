from __future__ import print_function

import tensorflow as tf
from tensorflow.python.framework import graph_util
from tensorflow.python.framework import graph_io
from tensorflow.python.tools import optimize_for_inference_lib
from tensorflow.python.platform import gfile
from keras import backend as K
from yolo import create_yolo

import argparse
import os.path


if __name__ == '__main__':
    parser = argparse.ArgumentParser('Creates YOLO model and exports Tensorflow protobuf file')
    parser.add_argument('input_file', type=str,
                        help='input file (h5 Keras model)')
    parser.add_argument('output_path', type=str,
                        help='folder to save the protobuf file')
    parser.add_argument('--min-score', dest='score', type=float, default=0.3,
                        help='threshold for a positive detection')
    parser.add_argument('--print-names', dest='names', action='store_true',
                        help='print out tensor names')
    args = parser.parse_args()

    print('Loading {0}'.format(args.input_file))

    model, b, s, c = create_yolo(args.input_file, args.score)

    sess = K.get_session()

    # Print out tensor names
    if args.names:
        print(model.input.name)
        print(K.learning_phase().name)
        print(b.name)
        print(s.name)
        print(c.name)

    # Save required image size as constant
    K.constant(model.layers[0].input_shape[1:3], dtype='int32', shape=(2,), name='config_size')

    # Freeze weights as the model will now only be used for inference
    output_graph_def = graph_util.convert_variables_to_constants(
                        sess,
                        sess.graph.as_graph_def(),
                        ['Gather', 'Gather_1', 'Gather_2', 'config_size'])
    file_name = os.path.splitext(os.path.basename(args.input_file))[0]
    graph_io.write_graph(output_graph_def,
                         args.output_path,
                         file_name + '.pb',
                         as_text=False)
