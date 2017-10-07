# YOLO network for traffic light detection

This folder contains code to generate an implementation of [YOLO](https://pjreddie.com/darknet/yolo/) in Tensorflow for use as a traffic light detector.  We selected YOLO because it performs fast on a GPU, is able to detect bounding boxes of objects in an image, and has pre-trained weights available for download.

Two versions of YOLO are available:
* Standard YOLO, which detects bounding boxes accurately on both the simulator and the real-world data, taking around 3.5s per frame on a CPU;
* Tiny YOLO, a faster version of the model that is still good enough for the simulator data and takes around 0.4s per frame on a CPU.

We are using YOLO trained against the [COCO dataset](http://cocodataset.org) as this includes traffic lights as an object class.

## Preparation

Install Keras version 2.0.3 before running the `get_yolo` script.

Run `./get_yolo.sh`.  This will do the following:
1. Download the weights and model definition files for YOLO;
2. Download [YAD2K](https://github.com/allanzelener/YAD2K), which converts these files to Keras models;
3. Runs the `load_yolo.py` script to modify the models to detect traffic lights only, and exports them to Tensorflow protobuf files.
